//! Host driver for the autonomous sender. Only semantic output workers call these sinks.

use crate::feedback::{
    FEEDBACK_BUFFER_BYTES, FeedbackController, FeedbackSnapshot, ReceiverReport,
};
use crate::protocol::{CuTelemetryRecord, CuTelemetrySession, CuTelemetryTx, TxMode};
use crate::{
    CuFeedbackRx, CuStreamRxError, CuStreamTx, LogStreamSenderConfig, OneWay, SenderCore,
    SenderStats,
};
use cu29_clock::{CuDuration, CuTime, RobotClock};
#[allow(unused_imports)]
use cu29_log::{ANONYMOUS, CuLogEntry, CuLogLevel};
#[allow(unused_imports)]
use cu29_log_runtime::log;
#[cfg(debug_assertions)]
#[allow(unused_imports)]
use cu29_log_runtime::log_debug_mode;
use cu29_runtime::{copperlist::CopperList, curuntime::KeyFrame};
use cu29_traits::{CopperListTuple, CuError, CuResult, WriteStream};
#[allow(unused_imports)]
use cu29_value::to_value;
use std::{
    fmt::{Debug, Formatter},
    marker::PhantomData,
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, AtomicU64, Ordering},
        mpsc::{self, Receiver, SyncSender, TryRecvError},
    },
    thread::{JoinHandle, Thread},
};

use crate::TelemetryBufferConfig;

// Limit work between protocol polls independently of the configured queue capacities.
const MAX_RECORDS_PER_POLL: usize = 6;
const MAX_LOGS_PER_POLL: usize = 4;

struct StructuredRecord {
    bytes: Box<[u8]>,
    len: usize,
}

struct StructuredInbox {
    free: crossbeam_queue::ArrayQueue<StructuredRecord>,
    pending: crossbeam_queue::ArrayQueue<StructuredRecord>,
}

/// A bounded destination for bytes produced by the local structured-log encoder.
/// It does not own the sender thread: dropping the CL and keyframe sinks still
/// drains and joins that worker, even while the global logger holds this sink.
pub struct ScheduledStructuredLogSink {
    inbox: Arc<StructuredInbox>,
    status: Arc<Shared>,
    thread: Thread,
    current: Option<StructuredRecord>,
    oversized: bool,
}

impl Debug for ScheduledStructuredLogSink {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ScheduledStructuredLogSink")
            .finish_non_exhaustive()
    }
}

impl crate::structured::StructuredLogOutput for ScheduledStructuredLogSink {
    fn begin(&mut self) {
        if self.current.is_none()
            && !self.status.stop.load(Ordering::Acquire)
            && !self.status.failed.load(Ordering::Acquire)
        {
            self.current = self.inbox.free.pop();
        }
        if let Some(record) = &mut self.current {
            record.len = 0;
        }
        self.oversized = false;
    }

    fn write(&mut self, bytes: &[u8]) {
        if let Some(record) = &mut self.current
            && !self.oversized
        {
            if bytes.len() > record.bytes.len() - record.len {
                self.oversized = true;
            } else {
                record.bytes[record.len..record.len + bytes.len()].copy_from_slice(bytes);
                record.len += bytes.len();
            }
        }
    }

    fn finish(&mut self, success: bool) {
        let record = self.current.take();
        if !success
            || self.oversized
            || self.status.stop.load(Ordering::Acquire)
            || self.status.failed.load(Ordering::Acquire)
        {
            if let Some(record) = record {
                let _ = self.inbox.free.push(record);
            }
            if success {
                self.status.inbox_drops.fetch_add(1, Ordering::Relaxed);
            }
        } else if let Some(record) = record {
            // Every owned buffer has a slot in the bounded pending queue.
            if let Err(record) = self.inbox.pending.push(record) {
                let _ = self.inbox.free.push(record);
                self.status.inbox_drops.fetch_add(1, Ordering::Relaxed);
            } else {
                self.thread.unpark();
            }
        } else {
            self.status.inbox_drops.fetch_add(1, Ordering::Relaxed);
        }
    }
}

struct Record {
    bytes: Box<[u8]>,
    len: usize,
    keyframe: bool,
    id: u64,
}

/// Live worker snapshot. Publication uses try_lock at 10 Hz, never on the RT path.
#[derive(Clone, Copy, Debug, Default)]
pub struct SenderSnapshot {
    pub sampled_at: CuTime,
    pub stats: SenderStats,
    pub inbox_drops: u64,
    pub stopped: bool,
    pub failed: bool,
    pub feedback_failed: bool,
    pub feedback: Option<FeedbackSnapshot>,
}

/// Final counters remain readable after both output sinks have been dropped.
#[derive(Clone, Debug)]
pub struct SenderMonitor(Arc<Shared>);

#[derive(Debug, Default)]
struct Shared {
    stop: AtomicBool,
    failed: AtomicBool,
    inbox_drops: AtomicU64,
    final_stats: Mutex<Option<SenderStats>>,
    snapshot: Mutex<SenderSnapshot>,
}

impl SenderMonitor {
    pub fn snapshot(&self) -> SenderSnapshot {
        let mut snapshot = *self.0.snapshot.lock().expect("sender snapshot poisoned");
        snapshot.failed |= self.failed();
        snapshot
    }

    pub fn inbox_drops(&self) -> u64 {
        self.0.inbox_drops.load(Ordering::Relaxed)
    }
    pub fn failed(&self) -> bool {
        self.0.failed.load(Ordering::Acquire)
    }
    pub fn final_stats(&self) -> Option<SenderStats> {
        *self.0.final_stats.lock().expect("sender stats poisoned")
    }
}

#[derive(Debug)]
struct Worker {
    shared: Arc<Shared>,
    thread: Thread,
    handle: Option<JoinHandle<()>>,
}

impl Drop for Worker {
    fn drop(&mut self) {
        self.shared.stop.store(true, Ordering::Release);
        self.thread.unpark();
        if self
            .handle
            .take()
            .is_some_and(|handle| handle.join().is_err())
        {
            self.shared.failed.store(true, Ordering::Release);
        }
    }
}

#[derive(Debug)]
struct Inbox {
    worker: Arc<Worker>,
    pending: SyncSender<Record>,
    // Mutex supplies Sync for WriteStream; exclusive log() uses get_mut, never lock().
    free: Mutex<Receiver<Box<[u8]>>>,
    recycled: SyncSender<Box<[u8]>>,
}

impl Inbox {
    fn submit(
        &mut self,
        keyframe: bool,
        id: u64,
        encode: impl FnOnce(&mut [u8]) -> CuResult<usize>,
    ) -> CuResult<()> {
        if self.worker.shared.failed.load(Ordering::Acquire) {
            return Err(CuError::from("Logstream sender worker failed"));
        }
        let mut bytes = match self.free.get_mut().expect("exclusive inbox").try_recv() {
            Ok(bytes) => bytes,
            Err(TryRecvError::Empty) => {
                self.worker
                    .shared
                    .inbox_drops
                    .fetch_add(1, Ordering::Relaxed);
                return Ok(());
            }
            Err(TryRecvError::Disconnected) => {
                return Err(CuError::from("Logstream sender worker stopped"));
            }
        };
        let len = match encode(&mut bytes) {
            Ok(len) => len,
            Err(error) => {
                let _ = self.recycled.try_send(bytes);
                return Err(error);
            }
        };
        // Capacity equals the total buffer count, so an owned free buffer always
        // has a corresponding inbox slot. Neither this nor pool exhaustion waits.
        self.pending
            .try_send(Record {
                bytes,
                len,
                keyframe,
                id,
            })
            .map_err(|_| CuError::from("Logstream sender inbox disconnected"))?;
        self.worker.thread.unpark();
        Ok(())
    }
}

/// Encodes directly into a sender-owned buffer on the existing CL output worker.
pub struct ScheduledCopperListSink<P: CopperListTuple> {
    inbox: Inbox,
    structured: Option<ScheduledStructuredLogSink>,
    encoder: fn(&CopperList<P>, &mut [u8]) -> crate::Result<usize>,
    _payload: PhantomData<fn() -> P>,
}
impl<P: CopperListTuple> ScheduledCopperListSink<P> {
    /// Take the optional structured-log handoff before installing the CL sink.
    pub fn take_structured_log_sink(&mut self) -> Option<ScheduledStructuredLogSink> {
        self.structured.take()
    }

    /// Select the generated capture encoder before handing the sink to its output worker.
    pub fn with_payload_encoder(
        mut self,
        encoder: fn(&CopperList<P>, &mut [u8]) -> crate::Result<usize>,
    ) -> Self
    where
        P: CopperListTuple,
    {
        self.encoder = encoder;
        self
    }
}
impl<P: CopperListTuple> Debug for ScheduledCopperListSink<P> {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ScheduledCopperListSink")
            .finish_non_exhaustive()
    }
}
impl<P: CopperListTuple + Send + Sync> WriteStream<CopperList<P>> for ScheduledCopperListSink<P> {
    fn log(&mut self, record: &CopperList<P>) -> CuResult<()> {
        self.inbox.submit(false, record.id, |bytes| {
            (self.encoder)(record, bytes).map_err(|error| CuError::from(error.to_string()))
        })
    }
}

/// Bounded keyframe encoding; capture objects are released before pacing begins.
#[derive(Debug)]
pub struct ScheduledKeyFrameSink {
    inbox: Inbox,
    interval: u32,
}
impl WriteStream<KeyFrame> for ScheduledKeyFrameSink {
    fn log(&mut self, keyframe: &KeyFrame) -> CuResult<()> {
        if !keyframe.culistid.is_multiple_of(u64::from(self.interval)) {
            return Ok(());
        }
        self.inbox.submit(true, keyframe.culistid, |bytes| {
            bincode::encode_into_slice(keyframe, bytes, bincode::config::standard())
                .map_err(|error| CuError::from(error.to_string()))
        })
    }
}

type TxFactory = Box<dyn FnOnce(RobotClock) -> CuResult<Box<dyn CuTelemetryTx>> + Send>;

/// Injected destination used by generated runtime builders.
#[doc(hidden)]
pub enum TelemetryDestination {
    Fec(Box<dyn CuStreamTx>, Box<LogStreamSenderConfig>),
    Native {
        make_tx: TxFactory,
        buffers: TelemetryBufferConfig,
    },
}

impl TelemetryDestination {
    pub fn start<P: crate::capture::CaptureDataSet + Send + Sync>(
        self,
        context: cu29_unifiedlog::SectionContext,
        clock: RobotClock,
    ) -> CuResult<(
        ScheduledCopperListSink<P>,
        ScheduledKeyFrameSink,
        Option<cu29_runtime::monitoring::LogStreamMonitor>,
    )> {
        let schema = P::stream_schema();
        let selective = !schema.reconstruction.is_empty();
        let clock = if clock.is_mock() {
            RobotClock::new()
        } else {
            clock
        };
        let (lists, frames, monitor) = match self {
            Self::Native { make_tx, buffers } => {
                buffers.payload_bytes()?;
                let tx = make_tx(clock.clone())?;
                let session = CuTelemetrySession {
                    identity: crate::StreamIdentity {
                        session_id: crate::new_session_id(),
                        sender_id: context.instance_id,
                    },
                    context,
                    schema,
                };
                let (lists, frames, _) = spawn_worker(
                    CustomerDriver(tx),
                    clock,
                    buffers,
                    buffers.structured_bytes(),
                    1,
                    CuDuration::from_millis(50),
                    Some(session),
                )?;
                (lists, frames, None)
            }
            Self::Fec(transport, config) => {
                let mut config = *config;
                let mut manifest =
                    crate::SessionManifest::decode_record(&config.recovery.manifest_record)
                        .map_err(|e| CuError::from(e.to_string()))?;
                if manifest.application_schema != schema {
                    return Err("injected sender must use the generated application schema".into());
                }
                config.continuous.identity.sender_id = context.instance_id;
                config.recovery.finite.identity.sender_id = context.instance_id;
                manifest.identity = config.continuous.identity;
                manifest.context = context;
                config.recovery.manifest_record = manifest
                    .encode_record()
                    .map_err(|e| CuError::from(e.to_string()))?;
                let bitrate = config.pacing.bitrate_bps;
                let baseline = config.continuous.repair_every_source_symbols;
                let (lists, frames, monitor) = scheduled_sinks(transport, config, clock)?;
                (
                    lists,
                    frames,
                    Some(monitor.into_runtime_monitor("injected", bitrate, baseline)),
                )
            }
        };
        let lists = if selective {
            lists.with_payload_encoder(crate::capture::encode_capture_into)
        } else {
            lists
        };
        Ok((lists, frames, monitor))
    }
}

/// Start one autonomous worker with an explicitly selected local RobotClock.
/// A real carrier must use a running clock even when application time is mocked.
/// Drops of both sinks stop repetition, then drain at most max_latency in local
/// clock time. No application object, serialization, or wait is added to the RT path.
pub fn scheduled_sinks<P, T>(
    transport: T,
    config: LogStreamSenderConfig,
    clock: RobotClock,
) -> CuResult<(
    ScheduledCopperListSink<P>,
    ScheduledKeyFrameSink,
    SenderMonitor,
)>
where
    P: CopperListTuple + Send + Sync,
    T: CuStreamTx + 'static,
{
    scheduled_worker(OneWay::new(transport), config, clock, false)
}

/// Explicitly enable the advisory receive direction; the manifest must advertise it.
pub fn scheduled_feedback_sinks<P, T>(
    transport: T,
    config: LogStreamSenderConfig,
    clock: RobotClock,
) -> CuResult<(
    ScheduledCopperListSink<P>,
    ScheduledKeyFrameSink,
    SenderMonitor,
)>
where
    P: CopperListTuple + Send + Sync,
    T: CuStreamTx + CuFeedbackRx + 'static,
{
    scheduled_worker(transport, config, clock, true)
}

fn scheduled_worker<P, T>(
    transport: T,
    config: LogStreamSenderConfig,
    clock: RobotClock,
    feedback_enabled: bool,
) -> CuResult<(
    ScheduledCopperListSink<P>,
    ScheduledKeyFrameSink,
    SenderMonitor,
)>
where
    P: CopperListTuple + Send + Sync,
    T: CuStreamTx + CuFeedbackRx + 'static,
{
    let manifest = crate::SessionManifest::decode_record(&config.recovery.manifest_record);
    let cl_bytes = config.continuous.max_record_bytes;
    let kf_bytes = usize::try_from(config.recovery.finite.max_object_bytes)
        .map_err(|_| CuError::from("Keyframe limit exceeds usize"))?;
    let framed_buffers = TelemetryBufferConfig::new(cl_bytes, kf_bytes);
    let structured_bytes = framed_buffers.structured_bytes();
    let reserved = framed_buffers
        .payload_bytes()?
        .checked_add(
            usize::from(framed_buffers.structured_log_count) * 2 * size_of::<StructuredRecord>(),
        )
        .ok_or_else(|| CuError::from("Sender pool size overflow"))?;
    let header = crate::record::RECORD_HEADER_LEN;
    if cl_bytes <= header || kf_bytes <= header {
        return Err("FEC record budgets must leave room for payloads".into());
    }
    let interval = config.recovery.recovery_interval;
    let drain_time = config.pacing.max_latency;
    let state = FecState::new(
        config,
        manifest.as_ref().ok(),
        feedback_enabled,
        clock.now(),
        reserved,
    )?;
    let driver = FecTelemetryTx {
        clock: clock.clone(),
        transport,
        session: FecSession::Active(Box::new(state)),
    };
    let session = manifest.ok().map(|m| CuTelemetrySession {
        identity: m.identity,
        schema: m.application_schema,
        context: m.context,
    });
    spawn_worker(
        driver,
        clock,
        TelemetryBufferConfig::new(cl_bytes - header, kf_bytes - header),
        structured_bytes - header,
        interval,
        drain_time,
        session,
    )
}

trait Driver: CuTelemetryTx {
    fn discard_pending(&mut self) {}
    fn stats(&self) -> Option<SenderStats> {
        None
    }
    fn feedback(&self) -> Option<FeedbackSnapshot> {
        None
    }
    fn feedback_failed(&self) -> bool {
        false
    }
}

struct CustomerDriver(Box<dyn CuTelemetryTx>);
impl CuTelemetryTx for CustomerDriver {
    fn send(&mut self, record: CuTelemetryRecord<'_>) -> CuResult<()> {
        self.0.send(record)
    }
    fn poll(&mut self, mode: TxMode) -> CuResult<Option<CuTime>> {
        self.0.poll(mode)
    }
}
impl Driver for CustomerDriver {}

/// Record-level FEC sender driven by the telemetry worker. The opening session
/// supplies the shared identity, schema and mission for the repeated FEC manifest.
pub struct FecTelemetryTx<T> {
    clock: RobotClock,
    transport: T,
    session: FecSession,
}

enum FecSession {
    Waiting(crate::LogStreamPlan),
    Active(Box<FecState>),
}

impl<T: CuStreamTx> FecTelemetryTx<OneWay<T>> {
    /// Create a one-way FEC sender. The plan budgets this branch's FEC buffers;
    /// the native worker's shared record pool is sized separately by the app builder.
    pub fn new(transport: T, plan: crate::LogStreamPlan, clock: RobotClock) -> CuResult<Self> {
        Self::waiting(OneWay::new(transport), plan, clock, false)
    }
}

impl<T: CuStreamTx + CuFeedbackRx> FecTelemetryTx<T> {
    /// Use the transport's return channel for advisory receiver feedback.
    pub fn new_with_feedback(
        transport: T,
        plan: crate::LogStreamPlan,
        clock: RobotClock,
    ) -> CuResult<Self> {
        Self::waiting(transport, plan, clock, true)
    }

    fn waiting(
        transport: T,
        plan: crate::LogStreamPlan,
        clock: RobotClock,
        feedback: bool,
    ) -> CuResult<Self> {
        plan.validate().map_err(|e| CuError::from(e.to_string()))?;
        if plan.feedback.is_some() != feedback {
            return Err("feedback policy must match the transport wiring".into());
        }
        Ok(Self {
            clock,
            transport,
            session: FecSession::Waiting(plan),
        })
    }
}

impl<T: CuStreamTx + CuFeedbackRx> CuTelemetryTx for FecTelemetryTx<T> {
    fn send(&mut self, record: CuTelemetryRecord<'_>) -> CuResult<()> {
        let now = self.clock.now();
        match &mut self.session {
            FecSession::Active(state) => state.send(now, record),
            FecSession::Waiting(plan) => {
                let CuTelemetryRecord::Session(session) = record else {
                    return Err("FEC sender requires an opening session".into());
                };
                let config = plan
                    .sender_config(session.identity, session.schema.clone(), session.context)
                    .map_err(|e| CuError::from(e.to_string()))?;
                let manifest =
                    crate::SessionManifest::decode_record(&config.recovery.manifest_record)
                        .map_err(|e| CuError::from(e.to_string()))?;
                let state =
                    FecState::new(config, Some(&manifest), plan.feedback.is_some(), now, 0)?;
                self.session = FecSession::Active(Box::new(state));
                Ok(())
            }
        }
    }

    fn poll(&mut self, mode: TxMode) -> CuResult<Option<CuTime>> {
        match &mut self.session {
            FecSession::Active(state) => state.poll(&mut self.transport, self.clock.now(), mode),
            FecSession::Waiting(_) => Ok(None),
        }
    }
}

struct FecState {
    feedback: Option<FeedbackController>,
    feedback_failed: bool,
    core: SenderCore,
    staging: Vec<u8>,
}
impl FecState {
    fn new(
        config: LogStreamSenderConfig,
        manifest: Option<&crate::SessionManifest>,
        feedback_enabled: bool,
        now: CuTime,
        reserved: usize,
    ) -> CuResult<Self> {
        let policy = config.feedback;
        let advertised = manifest.and_then(|manifest| manifest.requirements.feedback);
        if policy.is_some() != feedback_enabled || advertised.is_some() != feedback_enabled {
            return Err(CuError::from(
                "Feedback transport wiring must match manifest capability",
            ));
        }
        if let (Some(policy), Some(advertised)) = (policy, advertised)
            && (manifest.expect("validated feedback manifest").identity
                != config.continuous.identity
                || advertised.report_interval_ms != policy.report_interval_ms)
        {
            return Err(CuError::from(
                "Feedback manifest identity and cadence must match the sender",
            ));
        }
        let feedback = policy
            .map(|policy| {
                let baseline = u16::try_from(config.continuous.repair_every_source_symbols)
                    .map_err(|_| crate::Error::InvalidConfig("feedback baseline exceeds u16"))?;
                FeedbackController::new(
                    policy,
                    config.continuous.identity,
                    advertised
                        .expect("validated feedback capability")
                        .destination,
                    baseline,
                )
            })
            .transpose()
            .map_err(|error| CuError::from(error.to_string()))?;
        let staging = config.continuous.max_record_bytes.max(
            usize::try_from(config.recovery.finite.max_object_bytes)
                .map_err(|_| CuError::from("Keyframe limit exceeds usize"))?,
        );
        let reserved = reserved
            .checked_add(staging)
            .ok_or_else(|| CuError::from("sender pool size overflow"))?;
        let core =
            SenderCore::new(config, now, reserved).map_err(|e| CuError::from(e.to_string()))?;
        Ok(Self {
            feedback,
            core,
            staging: vec![0; staging],
            feedback_failed: false,
        })
    }

    fn send(&mut self, now: CuTime, record: CuTelemetryRecord<'_>) -> CuResult<()> {
        let (kind, id, bytes) = match record {
            CuTelemetryRecord::CopperList { id, capture } => {
                (crate::RecordKind::CopperList, id, capture)
            }
            CuTelemetryRecord::KeyFrame {
                copperlist_id,
                state,
            } => (crate::RecordKind::KeyFrame, copperlist_id, state),
            CuTelemetryRecord::StructuredLog { sequence, entry } => {
                (crate::RecordKind::StructuredLog, sequence, entry)
            }
            CuTelemetryRecord::Session(session) => {
                // The pacing core has the repeated manifest from setup.
                return self
                    .core
                    .check_session(session)
                    .map_err(|e| CuError::from(e.to_string()));
            }
        };
        let (header, payload) = self.staging.split_at_mut(crate::record::RECORD_HEADER_LEN);
        let out = payload
            .get_mut(..bytes.len())
            .ok_or_else(|| CuError::from("FEC record exceeds configured bound"))?;
        out.copy_from_slice(bytes);
        crate::record::encode_record_header(kind, id, out, header)
            .map_err(|e| CuError::from(e.to_string()))?;
        self.core
            .accept_record(
                &self.staging[..crate::record::RECORD_HEADER_LEN + bytes.len()],
                now,
            )
            .map_err(|e| CuError::from(e.to_string()))
    }
    fn poll(
        &mut self,
        transport: &mut (impl CuStreamTx + CuFeedbackRx),
        now: CuTime,
        mode: TxMode,
    ) -> CuResult<Option<CuTime>> {
        if mode == TxMode::Draining {
            self.core.begin_shutdown();
        }
        let mut feedback_packet = [0; FEEDBACK_BUFFER_BYTES];
        if let Some(controller) = &mut self.feedback {
            for _ in 0..8 {
                if self.feedback_failed {
                    break;
                }
                match transport.try_recv_feedback(&mut feedback_packet) {
                    Ok(Some(len)) => match feedback_packet
                        .get(..len)
                        .and_then(|p| ReceiverReport::decode(p).ok())
                    {
                        Some(report) => {
                            if controller.receive(report, now) {
                                self.core.request_recovery();
                            }
                        }
                        None => controller.invalid_report(),
                    },
                    Ok(None) => break,
                    Err(CuStreamRxError::BufferTooSmall { .. }) => controller.invalid_report(),
                    Err(CuStreamRxError::Failed(_)) => {
                        self.feedback_failed = true;
                    }
                }
            }
            controller.tick(now);
            self.core
                .set_repair_interval(controller.snapshot().effective_repair_every_source_symbols)
                .map_err(|e| CuError::from(e.to_string()))?;
        }

        self.core
            .poll(now, transport)
            .map_err(|e| CuError::from(e.to_string()))
    }
}
impl<T: CuStreamTx + CuFeedbackRx> Driver for FecTelemetryTx<T> {
    fn discard_pending(&mut self) {
        if let FecSession::Active(state) = &mut self.session {
            state.core.discard_pending();
        }
    }
    fn stats(&self) -> Option<SenderStats> {
        match &self.session {
            FecSession::Active(state) => Some(state.core.stats()),
            FecSession::Waiting(_) => None,
        }
    }
    fn feedback(&self) -> Option<FeedbackSnapshot> {
        match &self.session {
            FecSession::Active(state) => state.feedback.as_ref().map(FeedbackController::snapshot),
            FecSession::Waiting(_) => None,
        }
    }
    fn feedback_failed(&self) -> bool {
        matches!(&self.session, FecSession::Active(state) if state.feedback_failed)
    }
}

struct SenderInbox {
    records: Receiver<Record>,
    cl_return: SyncSender<Box<[u8]>>,
    kf_return: SyncSender<Box<[u8]>>,
    structured: Arc<StructuredInbox>,
    structured_id: u64,
}

impl SenderInbox {
    fn service(
        &mut self,
        driver: &mut impl Driver,
        mode: TxMode,
    ) -> crate::Result<(usize, Option<CuTime>)> {
        let mut captures = 0;
        let mut logs = 0;
        crate::protocol::service_tx(
            driver,
            mode,
            MAX_RECORDS_PER_POLL + MAX_LOGS_PER_POLL,
            |driver| {
                if captures < MAX_RECORDS_PER_POLL {
                    if let Ok(record) = self.records.try_recv() {
                        let bytes = &record.bytes[..record.len];
                        let (value, returned) = if record.keyframe {
                            (
                                CuTelemetryRecord::KeyFrame {
                                    copperlist_id: record.id,
                                    state: bytes,
                                },
                                &self.kf_return,
                            )
                        } else {
                            (
                                CuTelemetryRecord::CopperList {
                                    id: record.id,
                                    capture: bytes,
                                },
                                &self.cl_return,
                            )
                        };
                        let result = driver.send(value);
                        let _ = returned.try_send(record.bytes);
                        result?;
                        captures += 1;
                        return Ok(true);
                    }
                    captures = MAX_RECORDS_PER_POLL;
                }
                if logs == MAX_LOGS_PER_POLL {
                    return Ok(false);
                }
                let Some(record) = self.structured.pending.pop() else {
                    return Ok(false);
                };
                let result = driver.send(CuTelemetryRecord::StructuredLog {
                    sequence: self.structured_id,
                    entry: &record.bytes[..record.len],
                });
                let _ = self.structured.free.push(record);
                result?;
                self.structured_id = self
                    .structured_id
                    .checked_add(1)
                    .ok_or_else(|| CuError::from("structured log sequence exhausted"))?;
                logs += 1;
                Ok(true)
            },
        )
        .map_err(|error| crate::Error::Codec(error.to_string()))
    }
}

fn spawn_worker<P: CopperListTuple + Send + Sync>(
    mut driver: impl Driver + 'static,
    clock: RobotClock,
    buffers: TelemetryBufferConfig,
    structured_bytes: usize,
    interval: u32,
    drain_time: CuDuration,
    session: Option<CuTelemetrySession>,
) -> CuResult<(
    ScheduledCopperListSink<P>,
    ScheduledKeyFrameSink,
    SenderMonitor,
)> {
    let shutdown_clock = if clock.is_mock() {
        RobotClock::new()
    } else {
        clock.clone()
    };
    let capture_count = usize::from(buffers.capture_count);
    let keyframe_count = usize::from(buffers.keyframe_count);
    let structured_count = usize::from(buffers.structured_log_count);
    let record_count = capture_count + keyframe_count;
    let (pending, records) = mpsc::sync_channel::<Record>(record_count);
    let (cl_return, cl_free) = mpsc::sync_channel(capture_count);
    let (kf_return, kf_free) = mpsc::sync_channel(keyframe_count);
    for _ in 0..capture_count {
        cl_return
            .try_send(vec![0; buffers.max_capture_bytes].into_boxed_slice())
            .expect("new pool");
    }
    for _ in 0..keyframe_count {
        kf_return
            .try_send(vec![0; buffers.max_keyframe_bytes].into_boxed_slice())
            .expect("new pool");
    }
    let structured = Arc::new(StructuredInbox {
        free: crossbeam_queue::ArrayQueue::new(structured_count),
        pending: crossbeam_queue::ArrayQueue::new(structured_count),
    });
    for _ in 0..structured_count {
        let _ = structured.free.push(StructuredRecord {
            bytes: vec![0; structured_bytes].into_boxed_slice(),
            len: 0,
        });
    }
    let structured_worker = structured.clone();
    let shared = Arc::new(Shared::default());
    *shared.snapshot.lock().expect("new snapshot") = SenderSnapshot {
        sampled_at: clock.now(),
        feedback: driver.feedback(),
        ..Default::default()
    };
    let status = shared.clone();
    let cl_recycled = cl_return.clone();
    let kf_recycled = kf_return.clone();
    let handle = std::thread::Builder::new()
        .name("cu-logstream".into())
        .spawn(move || {
            let mut next_snapshot = clock.now();
            let mut stop_at = None;
            let mut inbox = SenderInbox { records, cl_return, kf_return, structured: structured_worker.clone(), structured_id: 0 };
            // Teardown remains finite even for deliberately frozen scheduler test clocks.
            let result = (|| -> crate::Result<()> {
                if let Some(session) = &session { driver.send(CuTelemetryRecord::Session(session)).map_err(|e| crate::Error::Codec(e.to_string()))?; }
                loop {
                    if status.stop.load(Ordering::Acquire) && stop_at.is_none() {
                        stop_at = Some(shutdown_clock.now() + drain_time);
                    }
                    let mode = if stop_at.is_some() { TxMode::Draining } else { TxMode::Running };
                    let (received, next) = inbox.service(&mut driver, mode)?;
                    let now = clock.now();
                    if now >= next_snapshot {
                        if let Ok(mut snapshot) = status.snapshot.try_lock() {
                            *snapshot = SenderSnapshot { sampled_at: now, stats: driver.stats().unwrap_or_default(),
                                inbox_drops: status.inbox_drops.load(Ordering::Relaxed),
                                feedback: driver.feedback(), feedback_failed: driver.feedback_failed(),
                                ..Default::default() };
                        }
                        next_snapshot = now + CuDuration::from_millis(100);
                    }
                    if stop_at.is_some_and(|deadline| shutdown_clock.now() >= deadline)
                        || (stop_at.is_some() && received == 0 && next.is_none())
                    {
                        break;
                    }
                    if received > 0 {
                        continue;
                    }
                    if let Some(deadline) = next {
                        let now = clock.now();
                        if deadline > now {
                            let duration = (deadline - now).min(CuDuration::from_millis(50));
                            std::thread::park_timeout(std::time::Duration::from_nanos(
                                duration.as_nanos(),
                            ));
                        }
                    } else {
                        std::thread::park_timeout(std::time::Duration::from_millis(50));
                    }
                }
                Ok(())
            })();
            status.failed.store(result.is_err(), Ordering::Release);
            driver.discard_pending();
            while let Some(record) = structured_worker.pending.pop() {
                status.inbox_drops.fetch_add(1, Ordering::Relaxed);
                let _ = structured_worker.free.push(record);
            }
            let stats = driver.stats();
            *status.final_stats.lock().expect("sender stats") = stats;
            *status.snapshot.lock().expect("sender snapshot") = SenderSnapshot {
                sampled_at: clock.now(), stats: stats.unwrap_or_default(), inbox_drops: status.inbox_drops.load(Ordering::Relaxed),
                stopped: true, failed: result.is_err(), feedback_failed: driver.feedback_failed(),
                feedback: driver.feedback(),
            };
            if let Some(stats) = stats {
                cu29_log_derive::info!("Logstream sender stopped: sent={} bytes={} queue_drops={} expired={} transport_drops={} inbox_drops={} shutdown_drops={}",
                stats.packets_sent, stats.bytes_sent, stats.queue_drops, stats.expired_packets, stats.transport_drops,
                status.inbox_drops.load(Ordering::Relaxed), stats.shutdown_drops);
            } else {
                cu29_log_derive::info!("Telemetry sender stopped: inbox_drops={}", status.inbox_drops.load(Ordering::Relaxed));
            }
            if let Err(error) = result {
                status.failed.store(true, Ordering::Release);
                cu29_log_derive::error!("Logstream sender failed: {}", error.to_string());
            }
        })
        .map_err(|error| CuError::new_with_cause("Start logstream sender", error))?;
    let worker = Arc::new(Worker {
        shared: shared.clone(),
        thread: handle.thread().clone(),
        handle: Some(handle),
    });
    Ok((
        ScheduledCopperListSink {
            structured: Some(ScheduledStructuredLogSink {
                inbox: structured,
                status: shared.clone(),
                thread: worker.thread.clone(),
                current: None,
                oversized: false,
            }),
            encoder: |list, bytes| {
                bincode::encode_into_slice(list, bytes, bincode::config::standard())
                    .map_err(|e| crate::Error::Codec(e.to_string()))
            },
            inbox: Inbox {
                worker: worker.clone(),
                pending: pending.clone(),
                free: Mutex::new(cl_free),
                recycled: cl_recycled,
            },
            _payload: PhantomData,
        },
        ScheduledKeyFrameSink {
            inbox: Inbox {
                worker,
                pending,
                free: Mutex::new(kf_free),
                recycled: kf_recycled,
            },
            interval,
        },
        SenderMonitor(shared),
    ))
}

impl SenderMonitor {
    /// Transfer a read-only worker handle into generated monitoring metadata.
    pub fn into_runtime_monitor(
        self,
        destination: &str,
        bitrate_bps: u64,
        baseline: usize,
    ) -> cu29_runtime::monitoring::LogStreamMonitor {
        cu29_runtime::monitoring::LogStreamMonitor::new(destination, bitrate_bps, baseline, self)
    }
}
impl cu29_runtime::monitoring::LogStreamStatsSource for SenderMonitor {
    fn snapshot(&self) -> cu29_runtime::monitoring::LogStreamStats {
        use cu29_runtime::monitoring::{
            LogStreamFeedbackState, LogStreamFeedbackStats, LogStreamStats,
        };
        let snapshot = SenderMonitor::snapshot(self);
        let stats = snapshot.stats;
        LogStreamStats {
            sampled_at: snapshot.sampled_at,
            packets_sent: stats.packets_sent,
            bytes_sent: stats.bytes_sent,
            queue_drops: stats.queue_drops,
            expired_packets: stats.expired_packets,
            transport_drops: stats.transport_drops,
            inbox_drops: snapshot.inbox_drops,
            shutdown_drops: stats.shutdown_drops,
            recovery_rounds: stats.recovery_rounds,
            recovery_superseded: stats.recovery_superseded,
            queue_peak: stats.queue_peak,
            stopped: snapshot.stopped,
            failed: snapshot.failed,
            feedback: snapshot.feedback.map(|feedback| {
                let report = feedback.report.unwrap_or_default();
                LogStreamFeedbackStats {
                    state: match feedback.state {
                        crate::feedback::FeedbackState::Waiting => LogStreamFeedbackState::Waiting,
                        crate::feedback::FeedbackState::Active => LogStreamFeedbackState::Active,
                        crate::feedback::FeedbackState::Stale => LogStreamFeedbackState::Stale,
                    },
                    age: feedback.last_received.map(|last| {
                        CuDuration::from_nanos(
                            snapshot
                                .sampled_at
                                .as_nanos()
                                .saturating_sub(last.as_nanos()),
                        )
                    }),
                    failed: snapshot.feedback_failed,
                    reports: feedback.accepted_reports,
                    rejected_reports: feedback.rejected_reports,
                    invalid_reports: feedback.invalid_reports,
                    rates_available: feedback.receiver_rates_available,
                    source_metrics_available: feedback.source_metrics_available,
                    bytes_per_second: feedback.receiver_bytes_per_second,
                    packets_per_second: feedback.receiver_packets_per_second,
                    finalized_symbols: report.sources.finalized,
                    loss_basis_points: feedback.source_loss_basis_points,
                    recovery_basis_points: feedback.source_recovery_basis_points,
                    buffered_records: report.buffered_records,
                    record_capacity: report.record_capacity,
                    latest_copperlist: report.latest_copperlist,
                    effective_repair_every_source_symbols: feedback
                        .effective_repair_every_source_symbols,
                    invalid_packets: report.invalid_packets,
                    duplicate_packets: report.duplicate_packets,
                    expired_records: report.expired_records,
                }
            }),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn service_yields_with_records_still_queued() {
        #[derive(Default)]
        struct Probe(usize);
        impl CuTelemetryTx for Probe {
            fn send(&mut self, record: CuTelemetryRecord<'_>) -> CuResult<()> {
                assert!(
                    matches!(record, CuTelemetryRecord::CopperList { id, .. } if id == self.0 as u64)
                );
                self.0 += 1;
                Ok(())
            }
        }
        impl Driver for Probe {}

        let capacity = MAX_RECORDS_PER_POLL + 1;
        let (pending, records) = mpsc::sync_channel(capacity);
        let (cl_return, recycled) = mpsc::sync_channel(capacity);
        let (kf_return, _) = mpsc::sync_channel(1);
        let mut inbox = SenderInbox {
            records,
            cl_return,
            kf_return,
            structured: Arc::new(StructuredInbox {
                free: crossbeam_queue::ArrayQueue::new(1),
                pending: crossbeam_queue::ArrayQueue::new(1),
            }),
            structured_id: 0,
        };
        for id in 0..capacity {
            assert!(
                pending
                    .try_send(Record {
                        bytes: Box::new([0]),
                        len: 1,
                        keyframe: false,
                        id: id as u64
                    })
                    .is_ok()
            );
        }
        let mut driver = Probe::default();
        assert_eq!(
            inbox.service(&mut driver, TxMode::Running).unwrap(),
            (MAX_RECORDS_PER_POLL, None)
        );
        assert_eq!(recycled.try_iter().count(), MAX_RECORDS_PER_POLL);
        assert_eq!(
            inbox.service(&mut driver, TxMode::Draining).unwrap(),
            (1, None)
        );
        assert_eq!(driver.0, capacity);
    }

    #[test]
    fn native_pool_payload_budget_uses_configured_sizes_and_counts() {
        let defaults = TelemetryBufferConfig::new(1171, 1171);
        assert_eq!(defaults.payload_bytes().unwrap(), 11_710);
        let small = TelemetryBufferConfig {
            capture_count: 1,
            keyframe_count: 1,
            structured_log_count: 1,
            ..defaults
        };
        assert_eq!(small.payload_bytes().unwrap(), 3513);
        assert_eq!(
            TelemetryBufferConfig::new(1, 8192).payload_bytes().unwrap(),
            4 + 2 * 8192 + 4 * 4096
        );
        for invalid in [
            TelemetryBufferConfig {
                capture_count: 0,
                ..small
            },
            TelemetryBufferConfig {
                keyframe_count: 0,
                ..small
            },
            TelemetryBufferConfig {
                structured_log_count: 0,
                ..small
            },
            TelemetryBufferConfig::new(usize::MAX, 1),
        ] {
            assert!(invalid.payload_bytes().is_err());
        }
    }
}
