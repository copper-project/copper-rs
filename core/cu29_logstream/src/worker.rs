//! Host driver for the autonomous sender. Only semantic output workers call these sinks.

use crate::feedback::{
    FEEDBACK_BUFFER_BYTES, FeedbackController, FeedbackSnapshot, ReceiverReport,
};
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

const CL_BUFFERS: usize = 4;
const KF_BUFFERS: usize = 2;
const STRUCTURED_BUFFERS: usize = 4;
use crate::pacing::STRUCTURED_RECORD_BYTES;

struct StructuredRecord {
    bytes: Box<[u8]>,
    len: usize,
    queued_at: CuTime,
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
    clock: RobotClock,
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
            record.len = crate::record::RECORD_HEADER_LEN;
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
        } else if let Some(mut record) = record {
            record.queued_at = self.clock.now();
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
    queued_at: CuTime,
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
    clock: RobotClock,
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
        let queued_at = self.clock.now();
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
                queued_at,
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
    pub fn with_encoder(
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
        self.inbox.submit(false, |bytes| {
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
        self.inbox.submit(true, |bytes| {
            if bytes.len() < crate::record::RECORD_HEADER_LEN {
                return Err(CuError::from("Keyframe buffer too small"));
            }
            let (header, payload) = bytes.split_at_mut(crate::record::RECORD_HEADER_LEN);
            let len = bincode::encode_into_slice(keyframe, payload, bincode::config::standard())
                .map_err(|error| CuError::from(error.to_string()))?;
            crate::record::encode_record_header(
                crate::RecordKind::KeyFrame,
                keyframe.culistid,
                &payload[..len],
                header,
            )
            .map_err(|error| CuError::from(error.to_string()))?;
            Ok(header.len() + len)
        })
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
    let policy = config.feedback;
    let advertised = manifest
        .as_ref()
        .ok()
        .and_then(|manifest| manifest.requirements.feedback);
    if policy.is_some() != feedback_enabled || advertised.is_some() != feedback_enabled {
        return Err(CuError::from(
            "Feedback transport wiring must match manifest capability",
        ));
    }
    if let (Some(policy), Some(advertised)) = (policy, advertised)
        && (manifest
            .as_ref()
            .expect("validated feedback manifest")
            .identity
            != config.continuous.identity
            || advertised.report_interval_ms != policy.report_interval_ms)
    {
        return Err(CuError::from(
            "Feedback manifest identity and cadence must match the sender",
        ));
    }
    let mut feedback = policy
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
    let cl_bytes = config.continuous.max_record_bytes;
    let kf_bytes = usize::try_from(config.recovery.finite.max_object_bytes)
        .map_err(|_| CuError::from("Keyframe limit exceeds usize"))?;
    let structured_bytes = kf_bytes.min(STRUCTURED_RECORD_BYTES);
    let reserved = cl_bytes
        .checked_mul(CL_BUFFERS)
        .and_then(|v| {
            kf_bytes
                .checked_mul(KF_BUFFERS)
                .and_then(|k| v.checked_add(k))
        })
        .and_then(|v| {
            v.checked_add(
                STRUCTURED_BUFFERS * (structured_bytes + 2 * size_of::<StructuredRecord>()),
            )
        })
        .ok_or_else(|| CuError::from("Sender pool size overflow"))?;
    let shutdown_clock = if clock.is_mock() {
        RobotClock::new()
    } else {
        clock.clone()
    };
    let interval = config.recovery.recovery_interval;
    let drain_time = config.pacing.max_latency;
    let mut core = SenderCore::new(config, clock.now(), reserved)
        .map_err(|error| CuError::from(error.to_string()))?;
    let (pending, records) = mpsc::sync_channel::<Record>(CL_BUFFERS + KF_BUFFERS);
    let (cl_return, cl_free) = mpsc::sync_channel(CL_BUFFERS);
    let (kf_return, kf_free) = mpsc::sync_channel(KF_BUFFERS);
    for _ in 0..CL_BUFFERS {
        cl_return
            .try_send(vec![0; cl_bytes].into_boxed_slice())
            .expect("new pool");
    }
    for _ in 0..KF_BUFFERS {
        kf_return
            .try_send(vec![0; kf_bytes].into_boxed_slice())
            .expect("new pool");
    }
    let structured = Arc::new(StructuredInbox {
        free: crossbeam_queue::ArrayQueue::new(STRUCTURED_BUFFERS),
        pending: crossbeam_queue::ArrayQueue::new(STRUCTURED_BUFFERS),
    });
    for _ in 0..STRUCTURED_BUFFERS {
        let _ = structured.free.push(StructuredRecord {
            bytes: vec![0; structured_bytes].into_boxed_slice(),
            len: 0,
            queued_at: CuTime::default(),
        });
    }
    let structured_worker = structured.clone();
    let structured_clock = clock.clone();
    let cl_clock = clock.clone();
    let kf_clock = clock.clone();
    let shared = Arc::new(Shared::default());
    *shared.snapshot.lock().expect("new snapshot") = SenderSnapshot {
        sampled_at: clock.now(),
        feedback: feedback.as_ref().map(FeedbackController::snapshot),
        ..Default::default()
    };
    let status = shared.clone();
    let cl_recycled = cl_return.clone();
    let kf_recycled = kf_return.clone();
    let handle = std::thread::Builder::new()
        .name("cu-logstream".into())
        .spawn(move || {
            let mut transport = transport;
            let mut feedback_packet = [0; FEEDBACK_BUFFER_BYTES];
            let mut feedback_failed = false;
            let mut next_snapshot = clock.now();
            let mut stop_at = None;
            let mut structured_id = 0u64;
            // Teardown remains finite even for deliberately frozen scheduler test clocks.
            let result = (|| -> crate::Result<()> {
                loop {
                    if status.stop.load(Ordering::Acquire) && stop_at.is_none() {
                        core.begin_shutdown();
                        stop_at = Some(shutdown_clock.now() + drain_time);
                    }
                    if let Some(controller) = &mut feedback {
                        for _ in 0..8 {
                            if feedback_failed { break; }
                            match transport.try_recv_feedback(&mut feedback_packet) {
                                Ok(Some(len)) => match feedback_packet.get(..len).and_then(|p| ReceiverReport::decode(p).ok()) {
                                    Some(report) => { if controller.receive(report, clock.now()) { core.request_recovery(); } }
                                    None => controller.invalid_report(),
                                },
                                Ok(None) => break,
                                Err(CuStreamRxError::BufferTooSmall { .. }) => controller.invalid_report(),
                                Err(CuStreamRxError::Failed(_)) => { feedback_failed = true; }
                            }
                        }
                        controller.tick(clock.now());
                        core.set_repair_interval(controller.snapshot().effective_repair_every_source_symbols)?;
                    }
                    let mut received = 0;
                    for _ in 0..CL_BUFFERS + KF_BUFFERS {
                        let Ok(record) = records.try_recv() else {
                            break;
                        };
                        let result =
                            core.accept_record(&record.bytes[..record.len], record.queued_at);
                        let returned = if record.keyframe {
                            &kf_return
                        } else {
                            &cl_return
                        };
                        let _ = returned.try_send(record.bytes);
                        result?;
                        received += 1;
                    }
                    for _ in 0..STRUCTURED_BUFFERS {
                        let Some(mut record) = structured_worker.pending.pop() else { break; };
                        let (header, payload) = record.bytes[..record.len].split_at_mut(crate::record::RECORD_HEADER_LEN);
                        let result = crate::record::encode_record_header(
                            crate::RecordKind::StructuredLog, structured_id, payload, header,
                        ).and_then(|()| core.accept_record(&record.bytes[..record.len], record.queued_at));
                        let _ = structured_worker.free.push(record);
                        result?;
                        structured_id = structured_id.checked_add(1)
                            .ok_or(crate::Error::InvalidConfig("structured log sequence exhausted"))?;
                        received += 1;
                    }
                    let next = core.poll(clock.now(), &mut transport)?;
                    let now = clock.now();
                    if now >= next_snapshot {
                        if let Ok(mut snapshot) = status.snapshot.try_lock() {
                            *snapshot = SenderSnapshot { sampled_at: now, stats: core.stats(),
                                inbox_drops: status.inbox_drops.load(Ordering::Relaxed),
                                feedback: feedback.as_ref().map(FeedbackController::snapshot), feedback_failed,
                                ..Default::default() };
                        }
                        next_snapshot = now + CuDuration::from_millis(100);
                    }
                    if stop_at.is_some_and(|deadline| shutdown_clock.now() >= deadline)
                        || (stop_at.is_some() && received == 0 && core.is_idle())
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
            core.discard_pending();
            while let Some(record) = structured_worker.pending.pop() {
                status.inbox_drops.fetch_add(1, Ordering::Relaxed);
                let _ = structured_worker.free.push(record);
            }
            let stats = core.stats();
            *status.final_stats.lock().expect("sender stats") = Some(stats);
            *status.snapshot.lock().expect("sender snapshot") = SenderSnapshot {
                sampled_at: clock.now(), stats, inbox_drops: status.inbox_drops.load(Ordering::Relaxed),
                stopped: true, failed: result.is_err(), feedback_failed,
                feedback: feedback.as_ref().map(FeedbackController::snapshot),
            };
            cu29_log_derive::info!("Logstream sender stopped: sent={} bytes={} queue_drops={} expired={} transport_drops={} inbox_drops={} shutdown_drops={}",
                stats.packets_sent, stats.bytes_sent, stats.queue_drops, stats.expired_packets, stats.transport_drops,
                status.inbox_drops.load(Ordering::Relaxed), stats.shutdown_drops);
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
                clock: structured_clock,
                current: None,
                oversized: false,
            }),
            encoder: crate::encode_copperlist_record_into,
            inbox: Inbox {
                clock: cl_clock,
                worker: worker.clone(),
                pending: pending.clone(),
                free: Mutex::new(cl_free),
                recycled: cl_recycled,
            },
            _payload: PhantomData,
        },
        ScheduledKeyFrameSink {
            inbox: Inbox {
                clock: kf_clock,
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
