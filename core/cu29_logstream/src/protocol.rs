//! Native capture records exchanged between a generated runtime and a live twin.
//!
//! A protocol instance serves one session. It owns packetization, integrity,
//! reassembly and duplicate suppression. Both peers use the matching application
//! and capture encoding. Each byte slice is one complete native payload.

use crate::{ApplicationSchema, StreamIdentity};
use bincode::{Decode, Encode};
use cu29_clock::CuTime;
use cu29_traits::{CuError, CuResult};
use cu29_unifiedlog::SectionContext;

/// Identifies the robot construction, active mission and selected output slots.
#[derive(Clone, Debug, PartialEq, Eq, Encode, Decode)]
pub struct CuTelemetrySession {
    pub identity: StreamIdentity,
    pub context: SectionContext,
    pub schema: ApplicationSchema,
}

/// Borrowed for one synchronous handoff. No packet or FEC header is included.
#[derive(Clone, Copy, Debug)]
pub enum CuTelemetryRecord<'a> {
    Session(&'a CuTelemetrySession),
    CopperList { id: u64, capture: &'a [u8] },
    KeyFrame { copperlist_id: u64, state: &'a [u8] },
    StructuredLog { sequence: u64, entry: &'a [u8] },
}

/// Output worker lifecycle. During drain the worker enforces a finite deadline.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TxMode {
    Running,
    Draining,
}

/// Robot-side protocol, driven by a host worker or explicitly between firmware iterations. The first record opens
/// the session. A retained payload must be moved into protocol-owned storage.
pub trait CuTelemetryTx: Send {
    /// Accept one borrowed record with bounded, nonblocking work.
    fn send(&mut self, record: CuTelemetryRecord<'_>) -> CuResult<()>;

    /// Perform bounded, nonblocking work and return the next absolute wakeup
    /// on the transport clock supplied at setup.
    /// The worker may also poll early after receiving a record. In draining
    /// mode, stop periodic repetition and return None when pending work is done.
    fn poll(&mut self, _mode: TxMode) -> CuResult<Option<CuTime>> {
        Ok(None)
    }
}

/// Run one bounded service pass. Always poll after processing the available records.
#[cfg(any(feature = "std", feature = "telemetry"))]
pub(crate) fn service_tx<T: CuTelemetryTx + ?Sized>(
    tx: &mut T,
    mode: TxMode,
    budget: usize,
    mut send_next: impl FnMut(&mut T) -> CuResult<bool>,
) -> CuResult<(usize, Option<CuTime>)> {
    let mut sent = 0;
    while sent < budget && send_next(tx)? {
        sent += 1;
    }
    Ok((sent, tx.poll(mode)?))
}

/// Deliver each borrowed record to both protocols on the same output worker.
/// Both children are called on error. The first error in tuple order propagates.
pub struct Fanout<A, B>(pub A, pub B);

impl<A: CuTelemetryTx, B: CuTelemetryTx> CuTelemetryTx for Fanout<A, B> {
    fn send(&mut self, record: CuTelemetryRecord<'_>) -> CuResult<()> {
        let first = self.0.send(record);
        let second = self.1.send(record);
        first.and(second)
    }

    fn poll(&mut self, mode: TxMode) -> CuResult<Option<CuTime>> {
        let first = self.0.poll(mode);
        let second = self.1.poll(mode);
        Ok(first?.into_iter().chain(second?).min())
    }
}

/// Disable an optional output and report its first send or poll error on the worker.
/// Both the output and its error callback must be nonblocking.
pub struct BestEffort<T, F>(Option<(T, F)>);

impl<T: CuTelemetryTx, F: FnOnce(CuError) + Send> BestEffort<T, F> {
    pub fn new(tx: T, on_error: F) -> Self {
        Self(Some((tx, on_error)))
    }

    fn attempt<R: Default>(&mut self, op: impl FnOnce(&mut T) -> CuResult<R>) -> R {
        if let Some((tx, _)) = &mut self.0 {
            match op(tx) {
                Ok(value) => return value,
                Err(error) => {
                    if let Some((_, report)) = self.0.take() {
                        report(error);
                    }
                }
            }
        }
        R::default()
    }
}

impl<T: CuTelemetryTx, F: FnOnce(CuError) + Send> CuTelemetryTx for BestEffort<T, F> {
    fn send(&mut self, record: CuTelemetryRecord<'_>) -> CuResult<()> {
        self.attempt(|tx| tx.send(record));
        Ok(())
    }

    fn poll(&mut self, mode: TxMode) -> CuResult<Option<CuTime>> {
        Ok(self.attempt(|tx| tx.poll(mode)))
    }
}

/// Delivery from a single receiver. Report missing CopperLists in order,
/// immediately before the next delivered list. Gap bounds are inclusive.
#[derive(Clone, Copy, Debug)]
pub enum CuTelemetryEvent<'a> {
    Record(CuTelemetryRecord<'a>),
    Gap {
        first_id: u64,
        last_id: u64,
        reason: cu29_runtime::continuity::SourceGapReason,
    },
}

/// Outcome of one bounded nonblocking receive operation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CuTelemetryPoll {
    /// No input is available yet.
    Pending,
    /// Input was processed, possibly yielding zero or multiple events.
    Progress,
    /// All known input and final gap events have been delivered.
    End,
}

/// Ground-side protocol. Deliver one session header before its records,
/// CopperLists in ID order starting at zero (or report an initial gap), and
/// snapshots at their original boundaries. The callback may borrow directly
/// from reassembly storage and its first error must terminate delivery.
pub trait CuTelemetryRx: Send {
    fn try_recv(
        &mut self,
        deliver: impl FnMut(CuTelemetryEvent<'_>) -> CuResult<()>,
    ) -> CuResult<CuTelemetryPoll>;
}
