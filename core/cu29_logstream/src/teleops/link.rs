//! Session-bound CUOP datagrams on an integrity-protected return carrier.
use super::*;
use crate::CuStreamRx;
use crate::protocol::{CuTelemetryRecord, CuTelemetryTx, TxMode};
use crate::{CuStreamRxError, CuStreamTx};
use cu29_clock::RobotClock;
use cu29_clock::{CuDuration, CuTime};

const PREFIX: &[u8; 5] = b"CUOP\x05";
/// Console sender for a single robot session.
pub struct ReturnTx<T, const MAX_PACKET_BYTES: usize> {
    tx: T,
    packet: [u8; MAX_PACKET_BYTES],
    session: [u8; 16],
    sequence: u64,
}
impl<T: CuStreamTx, const MAX_PACKET_BYTES: usize> ReturnTx<T, MAX_PACKET_BYTES> {
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
            packet: [0; MAX_PACKET_BYTES],
            session,
            sequence: 0,
        }
    }
}
impl<T: CuStreamTx, const MAX_PACKET_BYTES: usize> Submit for ReturnTx<T, MAX_PACKET_BYTES> {
    fn submit<U: Encode>(&mut self, observed: u64, update: &U) -> Result<(), CuStreamTxError> {
        let sequence = self
            .sequence
            .checked_add(1)
            .ok_or(CuStreamTxError::Failed("teleops sequence exhausted"))?;
        let (prefix, payload) =
            self.packet
                .split_at_mut_checked(PREFIX.len())
                .ok_or(CuStreamTxError::Failed(
                    "teleops packet budget is smaller than its prefix",
                ))?;
        prefix.copy_from_slice(PREFIX);
        let len = bincode::encode_into_slice(
            (self.session, observed, sequence, update),
            payload,
            bincode::config::standard(),
        )
        .map_err(|_| {
            CuStreamTxError::Failed("teleops update exceeds packet budget or cannot be encoded")
        })?;
        self.tx.try_send(&self.packet[..PREFIX.len() + len])?;
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
pub struct Admissions<
    P,
    const MAX_PACKET_BYTES: usize,
    const DECODE_BYTES: usize = MAX_PACKET_BYTES,
> {
    publish: P,
    session: [u8; 16],
}
impl<P: Dispatch, const MAX_PACKET_BYTES: usize, const DECODE_BYTES: usize>
    Admissions<P, MAX_PACKET_BYTES, DECODE_BYTES>
{
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
            .filter(|_| packet.len() <= MAX_PACKET_BYTES)
            .ok_or((None, Rejected::Framing))?;
        let ((session, cl, sequence), len): (([u8; 16], u64, u64), _) =
            bincode::decode_from_slice(bytes, bincode::config::standard())
                .map_err(|_| (None, Rejected::Framing))?;
        let reject = |why| (Some(cl), why);
        if session != self.session {
            return Err(reject(Rejected::Session));
        }
        let tick = lookup(cl).ok_or(reject(Rejected::UnknownFrame))?;
        self.publish
            .dispatch::<DECODE_BYTES>(cl, tick, sequence, &bytes[len..])
            .map_err(reject)?;
        Ok(cl)
    }
}

/// Attach the sole return reader to the bounded telemetry service. The session header binds
/// admissions to the robot boot. Extract ticks from captured application data in robot time.
pub fn with_teleops<T, R, P, H, A, const MAX_PACKET_BYTES: usize, const DECODE_BYTES: usize>(
    inner: T,
    rx: R,
    publish: P,
    clock: RobotClock,
    policy: ReturnPolicy<MAX_PACKET_BYTES, DECODE_BYTES>,
    tick: H,
    audit: A,
) -> ReturnLink<T, R, P, H, A, MAX_PACKET_BYTES, DECODE_BYTES>
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
        packet: [0; MAX_PACKET_BYTES],
        history: TickHistory::<256>::default(),
        admissions: Admissions::new(publish, [0; 16]),
        bound: false,
        tick,
        audit,
    }
}

/// Maximum complete return packet size for this carrier, including the CUOP header.
/// When feedback shares the endpoint, reserve room for its reports as well.
/// The optional second bound limits bincode decoding, including space claimed for
/// collections. Increase it for payloads whose decoded representation is larger.
pub struct ReturnPolicy<const MAX_PACKET_BYTES: usize, const DECODE_BYTES: usize = MAX_PACKET_BYTES>
{
    pub feedback: Option<FeedbackInbox>,
}

/// Robot return-link service, shared by host and threadless telemetry drivers.
pub struct ReturnLink<T, R, P, H, A, const MAX_PACKET_BYTES: usize, const DECODE_BYTES: usize> {
    inner: T,
    rx: R,
    clock: RobotClock,
    policy: ReturnPolicy<MAX_PACKET_BYTES, DECODE_BYTES>,
    packet: [u8; MAX_PACKET_BYTES],
    history: TickHistory<256>,
    admissions: Admissions<P, MAX_PACKET_BYTES, DECODE_BYTES>,
    bound: bool,
    tick: H,
    audit: A,
}

impl<T, R, P, H, A, const MAX_PACKET_BYTES: usize, const DECODE_BYTES: usize> CuTelemetryTx
    for ReturnLink<T, R, P, H, A, MAX_PACKET_BYTES, DECODE_BYTES>
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
        for _ in 0..8 {
            match self.rx.try_recv(&mut self.packet) {
                Ok(Some(n)) if n <= MAX_PACKET_BYTES => {
                    let packet = &self.packet[..n];
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
