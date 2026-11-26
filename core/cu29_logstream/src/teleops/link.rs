//! Session-bound CUOP datagrams on an integrity-protected return carrier.
use super::*;
use crate::CuStreamRx;
use crate::protocol::{CuTelemetryRecord, CuTelemetryTx, TxMode};
use crate::{CuStreamRxError, CuStreamTx};
use cu29_clock::RobotClock;
use cu29_clock::{CuDuration, CuTime};

const PREFIX: &[u8; 5] = b"CUOP\x03";
const MTU: usize = 1200;
/// Console sender for a single robot session.
pub struct ReturnTx<T> {
    tx: T,
    session: [u8; 16],
    sequence: u64,
}
impl<T: CuStreamTx> ReturnTx<T> {
    /// Drive carriers that retain bytes after accepting a complete packet.
    pub fn poll_pending(&mut self) -> Result<bool, CuStreamTxError> {
        self.tx.poll_pending()
    }
    pub fn transport(&self) -> &T {
        &self.tx
    }
    pub fn new(tx: T, session: [u8; 16]) -> Self {
        Self {
            tx,
            session,
            sequence: 0,
        }
    }
}
impl<T: CuStreamTx> Submit for ReturnTx<T> {
    fn submit<U: Encode>(
        &mut self,
        schema: &[u8; 32],
        observed: u64,
        update: &U,
    ) -> Result<(), CuStreamTxError> {
        let sequence = self
            .sequence
            .checked_add(1)
            .ok_or(CuStreamTxError::Failed("teleops sequence exhausted"))?;
        let mut packet = [0; MTU];
        packet[..PREFIX.len()].copy_from_slice(PREFIX);
        let len = bincode::encode_into_slice(
            (self.session, schema, observed, sequence, update),
            &mut packet[PREFIX.len()..],
            bincode::config::standard(),
        )
        .map_err(|_| {
            CuStreamTxError::Failed("teleops update exceeds packet budget or cannot be encoded")
        })?;
        self.tx.try_send(&packet[..PREFIX.len() + len])?;
        self.sequence = sequence;
        Ok(())
    }
}
/// A bounded history of application-captured robot ticks, indexed by CopperList ID.
pub struct TickHistory<const N: usize> {
    ticks: [Option<(u64, CuTime)>; N],
}
impl<const N: usize> Default for TickHistory<N> {
    fn default() -> Self {
        Self { ticks: [None; N] }
    }
}
impl<const N: usize> TickHistory<N> {
    pub fn record(&mut self, id: u64, tick: CuTime) {
        if N > 0 {
            self.ticks[(id % N as u64) as usize] = Some((id, tick));
        }
    }
    pub fn get(&self, id: u64) -> Option<CuTime> {
        if N == 0 {
            return None;
        }
        self.ticks[(id % N as u64) as usize]
            .filter(|(key, _)| *key == id)
            .map(|(_, tick)| tick)
    }
}
/// Decoder owned by the robot's sole return reader.
pub struct Admissions<P> {
    publish: P,
    session: [u8; 16],
}
impl<P: Dispatch> Admissions<P> {
    pub fn new(publish: P, session: [u8; 16]) -> Self {
        Self { publish, session }
    }
    pub fn receive(
        &mut self,
        packet: &[u8],
        lookup: impl FnOnce(u64) -> Option<CuTime>,
    ) -> Result<u64, (Option<u64>, Rejected)> {
        let bytes = packet
            .strip_prefix(PREFIX)
            .filter(|_| packet.len() <= MTU)
            .ok_or((None, Rejected::Framing))?;
        let ((session, schema, cl, sequence), len): (([u8; 16], [u8; 32], u64, u64), _) =
            bincode::decode_from_slice(bytes, bincode::config::standard())
                .map_err(|_| (None, Rejected::Framing))?;
        let reject = |why| (Some(cl), why);
        if session != self.session {
            return Err(reject(Rejected::Session));
        }
        if &schema != self.publish.schema() {
            return Err(reject(Rejected::Schema));
        }
        let tick = lookup(cl).ok_or(reject(Rejected::UnknownFrame))?;
        self.publish
            .dispatch(cl, tick, sequence, &bytes[len..])
            .map_err(reject)?;
        Ok(cl)
    }
}

/// Attach the sole return reader to the bounded telemetry service. The session header binds
/// admissions to the robot boot. Extract ticks from captured application data in robot time.
pub fn with_teleops<T, R, P, H, A>(
    inner: T,
    rx: R,
    publish: P,
    clock: RobotClock,
    policy: ReturnPolicy,
    tick: H,
    audit: A,
) -> ReturnLink<T, R, P, H, A>
where
    T: CuTelemetryTx,
    R: CuStreamRx,
    P: Dispatch + Send,
    H: FnMut(&[u8]) -> Option<CuTime> + Send,
    A: FnMut(Result<u64, (Option<u64>, Rejected)>) + Send,
{
    ReturnLink {
        inner,
        rx,
        clock,
        policy,
        history: TickHistory::<256>::default(),
        admissions: Admissions::new(publish, [0; 16]),
        bound: false,
        tick,
        audit,
    }
}

/// Optional shared feedback endpoint.
pub struct ReturnPolicy {
    pub feedback: Option<FeedbackInbox>,
}

/// Robot return-link service, shared by host and threadless telemetry drivers.
pub struct ReturnLink<T, R, P, H, A> {
    inner: T,
    rx: R,
    clock: RobotClock,
    policy: ReturnPolicy,
    history: TickHistory<256>,
    admissions: Admissions<P>,
    bound: bool,
    tick: H,
    audit: A,
}

impl<T, R, P, H, A> CuTelemetryTx for ReturnLink<T, R, P, H, A>
where
    T: CuTelemetryTx,
    R: CuStreamRx,
    P: Dispatch + Send,
    H: FnMut(&[u8]) -> Option<CuTime> + Send,
    A: FnMut(Result<u64, (Option<u64>, Rejected)>) + Send,
{
    fn send(&mut self, record: CuTelemetryRecord<'_>) -> CuResult<()> {
        match record {
            CuTelemetryRecord::Session(s) => {
                if self.bound && self.admissions.session != s.identity.session_id {
                    return Err("return reader already bound to another session".into());
                }
                self.admissions.session = s.identity.session_id;
                self.bound = true;
            }
            CuTelemetryRecord::CopperList { id, capture } => {
                if let Some(t) = (self.tick)(capture) {
                    self.history.record(id, t)
                }
            }
            _ => {}
        }
        self.inner.send(record)
    }
    fn poll(&mut self, mode: TxMode) -> CuResult<Option<CuTime>> {
        if mode == TxMode::Draining {
            return self.inner.poll(mode);
        }
        let mut bytes = [0; MTU];
        for _ in 0..8 {
            match self.rx.try_recv(&mut bytes) {
                Ok(Some(n)) if n <= MTU => {
                    let packet = &bytes[..n];
                    if self
                        .policy
                        .feedback
                        .as_ref()
                        .is_some_and(|f| f.route(packet))
                    {
                        continue;
                    }
                    if self.bound {
                        (self.audit)(self.admissions.receive(packet, |id| self.history.get(id)))
                    }
                }
                Ok(None) => break,
                Err(CuStreamRxError::Failed(reason)) => return Err(reason.into()),
                _ => continue,
            }
        }
        Ok(Some(
            self.inner
                .poll(mode)?
                .unwrap_or(CuTime::MAX)
                .min(self.clock.now() + CuDuration::from_millis(1)),
        ))
    }
}
/// Latest complete CUFB report for the FEC controller sharing the CUOP reader.
#[derive(Clone, Debug)]
pub struct FeedbackInbox(Arc<ArrayQueue<FeedbackPacket>>);
#[derive(Debug)]
struct FeedbackPacket {
    bytes: [u8; crate::feedback::FEEDBACK_BUFFER_BYTES],
    len: usize,
}
impl Default for FeedbackInbox {
    fn default() -> Self {
        Self(Arc::new(ArrayQueue::new(1)))
    }
}
impl FeedbackInbox {
    /// Returns true for feedback, including oversized reports discarded here.
    pub fn route(&self, bytes: &[u8]) -> bool {
        if !bytes.starts_with(b"CUFB") {
            return false;
        }
        let mut packet = FeedbackPacket {
            len: bytes.len(),
            bytes: [0; crate::feedback::FEEDBACK_BUFFER_BYTES],
        };
        if let Some(dst) = packet.bytes.get_mut(..bytes.len()) {
            dst.copy_from_slice(bytes);
            self.0.force_push(packet);
        }
        true
    }
}
impl crate::CuFeedbackRx for FeedbackInbox {
    fn try_recv_feedback(&mut self, out: &mut [u8]) -> Result<Option<usize>, CuStreamRxError> {
        let Some(packet) = self.0.pop() else {
            return Ok(None);
        };
        let dst = out
            .get_mut(..packet.len)
            .ok_or(CuStreamRxError::BufferTooSmall { needed: packet.len })?;
        dst.copy_from_slice(&packet.bytes[..packet.len]);
        Ok(Some(packet.len))
    }
}
