//! Session-bound CUOP datagrams on an integrity-protected return carrier.
use super::*;
use crate::CuStreamRx;
use crate::protocol::{CuTelemetryRecord, CuTelemetryTx, TxMode};
use crate::{CuStreamRxError, CuStreamTx};
use cu29_clock::RobotClock;
use cu29_clock::{CuDuration, CuTime};

const PREFIX: &[u8; 5] = b"CUOP\x01";
const MTU: usize = 1200;
/// Console sender for a single robot session and authorized control epoch.
pub struct ReturnTx<T> {
    tx: T,
    session: [u8; 16],
    epoch: u64,
    sequence: u64,
}
impl<T: CuStreamTx> ReturnTx<T> {
    pub fn transport(&self) -> &T {
        &self.tx
    }
    pub fn new(tx: T, session: [u8; 16], epoch: u64) -> Self {
        Self {
            tx,
            session,
            epoch,
            sequence: 0,
        }
    }
}
impl<T: CuStreamTx> Submit for ReturnTx<T> {
    fn submit<V: Encode>(
        &mut self,
        field: u16,
        observed: Option<u64>,
        validity: Validity,
        value: &V,
    ) -> Result<(), CuStreamTxError> {
        let sequence = self
            .sequence
            .checked_add(1)
            .ok_or(CuStreamTxError::Failed("control sequence exhausted"))?;
        let id = CommandId {
            session: self.session,
            epoch: self.epoch,
            field,
            sequence,
            observed_copperlist_id: observed,
        };
        let mut packet = [0; MTU];
        packet[..PREFIX.len()].copy_from_slice(PREFIX);
        let len = bincode::encode_into_slice(
            (id, validity, value),
            &mut packet[PREFIX.len()..],
            bincode::config::standard(),
        )
        .map_err(|_| CuStreamTxError::Failed("teleops encoding failed"))?;
        self.tx.try_send(&packet[..len + PREFIX.len()])?;
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
    epoch: u64,
    max_age: CuDuration,
}
impl<P: Dispatch> Admissions<P> {
    pub fn new(publish: P, session: [u8; 16], epoch: u64, max_age: CuDuration) -> Self {
        Self {
            publish,
            session,
            epoch,
            max_age,
        }
    }
    /// Call while graph admissions are quiescent. Apply revocation before accepting the new epoch.
    pub fn takeover(&mut self, epoch: u64, revoke: impl FnOnce()) -> CuResult<()> {
        if epoch <= self.epoch {
            return Err("control epoch must increase".into());
        }
        self.publish.reset();
        revoke();
        self.epoch = epoch;
        Ok(())
    }
    pub fn receive(
        &mut self,
        packet: &[u8],
        now: CuTime,
        lookup: impl FnOnce(u64) -> Option<CuTime>,
    ) -> Result<CommandId, (Option<CommandId>, Rejected)> {
        let bytes = packet
            .strip_prefix(PREFIX)
            .filter(|_| packet.len() <= MTU)
            .ok_or((None, Rejected::Framing))?;
        let ((id, validity), len): ((CommandId, Validity), _) =
            bincode::decode_from_slice(bytes, bincode::config::standard())
                .map_err(|_| (None, Rejected::Framing))?;
        let reject = |why| (Some(id), why);
        if id.session != self.session {
            return Err(reject(Rejected::Session));
        }
        if id.epoch != self.epoch {
            return Err(reject(Rejected::Epoch));
        }
        let tov = match validity {
            Validity::Robot(tov) => tov,
            Validity::Observed => match id.observed_copperlist_id {
                None => Tov::None,
                Some(frame) => {
                    let tick = lookup(frame).ok_or(reject(Rejected::Freshness))?;
                    if tick > now || now - tick > self.max_age {
                        return Err(reject(Rejected::Freshness));
                    }
                    Tov::Time(tick)
                }
            },
        };
        self.publish
            .dispatch(id, tov, &bytes[len..])
            .map_err(reject)?;
        Ok(id)
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
) -> impl CuTelemetryTx
where
    T: CuTelemetryTx,
    R: CuStreamRx,
    P: Dispatch + Send,
    H: FnMut(&[u8]) -> Option<CuTime> + Send,
    A: FnMut(Result<CommandId, (Option<CommandId>, Rejected)>) + Send,
{
    ReturnLink {
        inner,
        rx,
        clock,
        policy,
        history: TickHistory::<256>::default(),
        admissions: Admissions::new(publish, [0; 16], 0, CuDuration::default()),
        bound: false,
        tick,
        audit,
    }
}

/// Parameters for a trusted control epoch and its shared feedback endpoint.
pub struct ReturnPolicy {
    pub epoch: u64,
    pub max_age: CuDuration,
    pub feedback: Option<FeedbackInbox>,
}

struct ReturnLink<T, R, P, H, A> {
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
    A: FnMut(Result<CommandId, (Option<CommandId>, Rejected)>) + Send,
{
    fn send(&mut self, record: CuTelemetryRecord<'_>) -> CuResult<()> {
        match record {
            CuTelemetryRecord::Session(s) => {
                if self.bound && self.admissions.session != s.identity.session_id {
                    return Err("return reader already bound to another session".into());
                }
                self.admissions.session = s.identity.session_id;
                self.admissions.epoch = self.policy.epoch;
                self.admissions.max_age = self.policy.max_age;
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
                        (self.audit)(
                            self.admissions
                                .receive(packet, self.clock.now(), |id| self.history.get(id)),
                        )
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
