//! Copper-owned receive, archive, and reconstruction lifecycle for one robot session.

use crate::telemetry::{TelemetryReader, telemetry_channel};
use crate::twin::{LiveReplay, TwinFrame, TwinReceiverStatus, TwinStatus, TwinWorker};
use crate::{
    CaptureArchive, CuStreamRx, SessionEvent, SessionRouter, SessionRouterLimits, StreamIdentity,
};
use cu29_traits::{CuError, CuResult};
use std::marker::PhantomData;
use std::num::NonZeroUsize;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

const REPLAY_CAPACITY: NonZeroUsize = NonZeroUsize::new(32).unwrap();
const FRAME_CAPACITY: NonZeroUsize = NonZeroUsize::new(64).unwrap();
const SLAB_BYTES: usize = 16 * 1024 * 1024;
// Includes the native section header and continuity envelope around a finite record.
const SECTION_BYTES: usize = 128 * 1024;
const CONTINUITY_OVERHEAD: usize = 32;
const POLL_INTERVAL: Duration = Duration::from_millis(1);

/// Recording state is independent of reconstruction and display consumption.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum CuTwinRecordingState {
    #[default]
    Waiting,
    Recording,
    Closed,
    Failed,
}

/// Receiver and reconstruction health, available even when the view is paused.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CuTwinStatus {
    pub latest: Option<u64>,
    pub recovery_point: Option<u64>,
    pub gaps: usize,
    pub packets: usize,
    pub archived: u64,
    pub structured_logs: u64,
    pub identity: Option<StreamIdentity>,
    pub last_packet: Option<Instant>,
    pub state: CuTwinRecordingState,
    pub twin: TwinStatus,
    pub feedback_reports_sent: u64,
    pub feedback_reports_dropped: u64,
    pub feedback_failed: bool,
}

impl TwinReceiverStatus for CuTwinStatus {
    fn with_twin(mut self, status: TwinStatus) -> Self {
        self.twin = status;
        self
    }
}

/// Builder for a generated application's live twin. Copper owns all worker,
/// routing, archive, and recovery plumbing; callers supply a packet transport.
/// Receiver bounds match the default 1200-byte-MTU, 64-symbol streaming profile.
/// One handle accepts one sender session, with a fresh archive path.
pub struct CuTwinBuilder<A, R> {
    rx: R,
    feedback_tx: Option<Box<dyn crate::CuFeedbackTx>>,
    log_base: Option<PathBuf>,
    frame_capacity: NonZeroUsize,
    log_capacity: NonZeroUsize,
    replay_capacity: NonZeroUsize,
    slab_bytes: usize,
    section_bytes: usize,
    receiver_limits: SessionRouterLimits,
    reconstruct: bool,
    app: PhantomData<fn() -> A>,
}

impl<A: LiveReplay, R: CuStreamRx + 'static> CuTwinBuilder<A, R> {
    /// Enable feedback only when the sender advertises support. The caller owns endpoint selection.
    pub fn with_feedback<T: crate::CuFeedbackTx + 'static>(mut self, tx: T) -> Self {
        self.feedback_tx = Some(Box::new(tx));
        self
    }

    pub fn with_log_path(mut self, path: impl AsRef<Path>) -> Self {
        self.log_base = Some(path.as_ref().to_path_buf());
        self
    }

    /// Sets the size of each archive backing file (slab), in bytes. Defaults to 16 MiB.
    ///
    /// The archive adds another slab when the current one fills; this is not a
    /// limit on total recording size. A slab must fit the largest section used
    /// to store CopperLists, keyframes, or structured logs. Prefer a slab roughly
    /// 10–100 times larger than a section to hold many sections per file and
    /// reduce file turnover. Larger slabs reserve more disk space at a time.
    /// Both slab and section sizes must be multiples of 512 bytes.
    pub fn with_slab_size(mut self, bytes: usize) -> Self {
        self.slab_bytes = bytes;
        self
    }

    /// Sets the size of an archive section, in bytes. Defaults to 128 KiB.
    ///
    /// Sections group records such as CopperLists, keyframes, and structured log
    /// entries inside a slab. A complete record must fit within one section.
    /// Choose at least the larger of `max_record_bytes` and
    /// `finite_objects.max_object_bytes` from [`Self::with_receiver_limits`],
    /// plus 512 bytes for the section header and 32 bytes for the archive's
    /// continuity envelope. Round up to a multiple of 512 bytes.
    ///
    /// Increase this alongside receive bounds when captures or task-state
    /// snapshots grow. The section must fit in [`Self::with_slab_size`]; keeping
    /// slabs much larger lets each file hold many sections.
    pub fn with_section_size(mut self, bytes: usize) -> Self {
        self.section_bytes = bytes;
        self
    }

    /// Sets receive size and buffering limits for one sender session.
    /// Defaults to [`SessionRouterLimits::default`].
    ///
    /// `max_record_bytes` bounds one complete serialized CopperList record
    /// (default: 4 KiB). `finite_objects.max_object_bytes` bounds one manifest,
    /// keyframe, or structured log record (default: 64 KiB). Match or exceed
    /// the sender's advertised bounds so its captures and recovery snapshots
    /// can be received, and increase [`Self::with_section_size`] to fit them.
    ///
    /// The remaining limits bound records, packets, recovery data, and objects
    /// buffered during decoding. Larger sizes and concurrency limits increase
    /// receiver memory use; they are not a total process memory budget.
    /// `max_sessions` must be 1. The packet profile remains a 1200-byte MTU with
    /// at most 1128-byte symbols and 64 FEC equations.
    pub fn with_receiver_limits(mut self, limits: SessionRouterLimits) -> Self {
        self.receiver_limits = limits;
        self
    }

    /// Sets buffering between recording and task replay. Defaults to 32.
    ///
    /// This bounds both the queued replay events (captured CopperLists and
    /// recovery keyframes) and the pending CopperLists waiting for matching
    /// replay state. One recovery keyframe and one executing CopperList are
    /// retained in addition to these buffers.
    ///
    /// Increase this to absorb short replay slowdowns; reduce it to retain fewer
    /// payloads in memory. Overflow interrupts reconstruction until a matching
    /// recovery point is available. Received data continues to be recorded in
    /// the archive. This setting is unused with [`Self::archive_only`].
    pub fn with_replay_capacity(mut self, capacity: NonZeroUsize) -> Self {
        self.replay_capacity = capacity;
        self
    }

    /// Sets the number of unread structured log entries buffered for
    /// [`CuTwin::take_log_reader`]. Defaults to 64 entries.
    ///
    /// These are the robot's structured logging calls (such as `info!` and
    /// `debug!`), published after being saved in the archive. Increase this to
    /// let the log reader pause longer at the cost of retaining more entries in
    /// memory. When full, the buffer replaces the oldest unread entry. Reading
    /// slowly never blocks recording; overwritten entries remain in the archive.
    pub fn with_log_capacity(mut self, capacity: NonZeroUsize) -> Self {
        self.log_capacity = capacity;
        self
    }

    /// Sets the number of unread reconstructed CopperLists buffered for the
    /// returned [`CuTwinReader`]. Defaults to 64.
    ///
    /// Each [`TwinFrame`] contains one CopperList with its captured inputs and
    /// locally reconstructed outputs, plus sender identity and receive time.
    /// Increase this to absorb pauses in a UI or analysis reader; reduce it to
    /// retain fewer message payloads in memory. When full, the buffer replaces
    /// the oldest unread CopperList and the reader reports how many it missed.
    /// Recording and task replay continue independently of reader consumption.
    /// Network packets and recovery keyframes have separate receive/replay
    /// buffers; this capacity counts completed CopperLists.
    pub fn with_frame_capacity(mut self, capacity: NonZeroUsize) -> Self {
        self.frame_capacity = capacity;
        self
    }

    /// Record the native capture without starting task reconstruction.
    pub fn archive_only(mut self) -> Self {
        self.reconstruct = false;
        self
    }

    pub fn spawn(self) -> CuResult<(CuTwin<A>, CuTwinReader<A>)> {
        validate_sizes(self.slab_bytes, self.section_bytes, self.receiver_limits)?;
        // Validate and construct routing before creating directories or starting workers.
        let mut router =
            SessionRouter::<1128, 64, 64>::new(self.receiver_limits).map_err(stream_error)?;
        let path = self
            .log_base
            .ok_or_else(|| CuError::from("Copper twin requires a log path"))?;
        if cu29_runtime::replay::first_slab_path(&path)?.exists() || path.exists() {
            return Err(CuError::from(
                "Copper twin archive already exists; choose a fresh log path",
            ));
        }
        if let Some(parent) = path
            .parent()
            .filter(|parent| !parent.as_os_str().is_empty())
        {
            std::fs::create_dir_all(parent)
                .map_err(|e| CuError::new_with_cause("Create twin log directory", e))?;
        }
        let (publisher, reader) = telemetry_channel(self.frame_capacity, CuTwinStatus::default());
        let (mut log_publisher, log_reader) = telemetry_channel(self.log_capacity, ());
        let stop = Arc::new(AtomicBool::new(false));
        let stopping = stop.clone();
        let (ready_tx, ready_rx) = std::sync::mpsc::sync_channel(1);
        let worker = thread::Builder::new()
            .name("copper-twin-receiver".into())
            .spawn(move || {
                let mut status = CuTwinStatus::default();
                let (replay, mut publisher) = if self.reconstruct {
                    match TwinWorker::spawn::<A>(self.replay_capacity, publisher) {
                        Ok(replay) => (Some(replay), None),
                        Err(error) => {
                            let _ = ready_tx.send(Err(error.clone()));
                            return Err(error);
                        }
                    }
                } else {
                    (None, Some(publisher))
                };
                let mut publish_status = |status| {
                    if let Some(replay) = &replay {
                        replay.set_status(status);
                    }
                    if let Some(publisher) = &mut publisher {
                        publisher.set_status(status);
                    }
                };
                let mut archive = None;
                let result = (|| {
                    let _ = ready_tx.send(Ok(()));
                    let mut rx = self.rx;
                    let mut feedback_tx = self.feedback_tx;
                    let feedback_clock = cu29_clock::RobotClock::new();
                    let receiver_id = crate::new_session_id();
                    let mut reporter = None;
                    let mut feedback_packet = [0; crate::feedback::FEEDBACK_BUFFER_BYTES];
                    let mut packet = [0; 1200];
                    while !stopping.load(Ordering::Acquire) {
                        if let Some(len) = rx
                            .try_recv(&mut packet)
                            .map_err(|e| CuError::from(format!("Twin receive: {e:?}")))?
                        {
                            let bytes = packet.get(..len).ok_or_else(|| {
                                CuError::from("Transport returned an invalid packet length")
                            })?;
                            status.last_packet = Some(Instant::now());
                            router
                                .receive_datagram(bytes, |event| {
                                    // Recovery objects can arrive before the manifest.
                                    // The router retains them and emits VerifiedRecoveryPoint
                                    // once their manifest/keyframe references agree.
                                    if matches!(&event, SessionEvent::Object { record, .. } if record.decoded().kind != crate::RecordKind::StructuredLog) {
                                        return Ok(());
                                    }
                                    if let SessionEvent::Manifest(manifest) = &event {
                                        status.identity = Some(manifest.manifest().identity);
                                        if feedback_tx.is_some() {
                                            reporter = crate::feedback::FeedbackReporter::new(
                                                manifest.manifest(),
                                                receiver_id,
                                                feedback_clock.now(),
                                            );
                                        }
                                        archive = Some(CaptureArchive::<A::DataSet>::new(
                                            &path,
                                            manifest,
                                            self.slab_bytes,
                                            self.section_bytes,
                                        )?);
                                    }
                                    let writer =
                                        archive.as_mut().ok_or(crate::Error::InvalidConfig(
                                            "capture arrived before its manifest",
                                        ))?;
                                    let capture = writer.accept(&event)?;
                                    if let Some(entry) = writer.take_structured_log() {
                                        status.structured_logs += 1;
                                        if let SessionEvent::Object { identity, record } = &event {
                                            log_publisher.publish(ReceivedStructuredLog {
                                                identity: *identity, sequence: record.decoded().object_id, entry,
                                            });
                                        }
                                    }
                                    if let Some(capture) = &capture {
                                        status.latest = Some(capture.copperlist.id);
                                        status.archived += 1;
                                        status.state = CuTwinRecordingState::Recording;
                                    }
                                    if let Some(replay) = &replay {
                                        replay.accept(&event, capture)?;
                                    }
                                    match event {
                                        SessionEvent::Gap { .. } => status.gaps += 1,
                                        SessionEvent::VerifiedRecoveryPoint {
                                            recovery_point,
                                            ..
                                        } => {
                                            status.recovery_point =
                                                Some(recovery_point.copperlist_id)
                                        }
                                        _ => {}
                                    }
                                    Ok::<(), crate::Error>(())
                                })
                                .map_err(|e| CuError::from(format!("Twin receive: {e}")))?;
                            status.packets = router.stats().datagrams_seen;
                            publish_status(status);
                        } else {
                            thread::park_timeout(POLL_INTERVAL);
                        }
                        if let (Some(reporter), Some(tx), Some(identity)) =
                            (&mut reporter, &mut feedback_tx, status.identity)
                            && let Some(mut counters) = router.feedback_counters(identity)
                        {
                            counters.latest_copperlist = status.latest;
                            if let Some(report) = reporter.report(feedback_clock.now(), counters) {
                                let len = report
                                    .encode_into(&mut feedback_packet)
                                    .map_err(stream_error)?;
                                match tx.try_send_feedback(&feedback_packet[..len]) {
                                    Ok(()) => status.feedback_reports_sent += 1,
                                    Err(crate::CuStreamTxError::WouldBlock) => {
                                        status.feedback_reports_dropped += 1
                                    }
                                    Err(crate::CuStreamTxError::Failed(_)) => {
                                        feedback_tx = None;
                                        status.feedback_failed = true;
                                    }
                                }
                                publish_status(status);
                            }
                        }
                    }
                    if let Some(archive) = archive.take() {
                        archive.finish().map_err(stream_error)?;
                    }
                    Ok(())
                })();
                status.state = if result.is_ok() {
                    CuTwinRecordingState::Closed
                } else {
                    CuTwinRecordingState::Failed
                };
                publish_status(status);
                // Drain admitted replay before returning the final receiver status.
                if let Some(replay) = replay {
                    status.twin = replay.finish()?;
                }
                result.map(|()| status)
            })
            .map_err(|e| CuError::new_with_cause("Start Copper twin", e))?;
        if let Err(error) = ready_rx
            .recv()
            .map_err(|_| CuError::from("Copper twin initialization failed"))
            .and_then(|ready| ready)
        {
            stop.store(true, Ordering::Release);
            let _ = worker.join();
            return Err(error);
        }
        Ok((
            CuTwin {
                stop,
                worker: Some(worker),
                final_status: None,
                log_reader: Some(log_reader),
                app: PhantomData,
            },
            reader,
        ))
    }
}

fn stream_error(error: crate::Error) -> CuError {
    CuError::from(error.to_string())
}

/// Reader for a generated graph's reconstructed frames and recording status.
pub type CuTwinReader<A> = TelemetryReader<TwinFrame<<A as LiveReplay>::DataSet>, CuTwinStatus>;

/// One original robot log entry, published only after native archival succeeds.
#[derive(Debug)]
pub struct ReceivedStructuredLog {
    pub identity: StreamIdentity,
    pub sequence: u64,
    pub entry: cu29_log::CuLogEntry,
}

/// Independent bounded log reader; a stalled reader never delays native recording.
pub type CuTwinLogReader = TelemetryReader<ReceivedStructuredLog, ()>;

fn validate_sizes(
    slab_bytes: usize,
    section_bytes: usize,
    limits: SessionRouterLimits,
) -> CuResult<()> {
    let header = usize::from(cu29_unifiedlog::SECTION_HEADER_COMPACT_SIZE);
    if slab_bytes < 1024 || !slab_bytes.is_multiple_of(header) {
        return Err(CuError::from(
            "Twin slab size must be at least 1024 bytes and a multiple of 512",
        ));
    }
    if section_bytes <= header
        || !section_bytes.is_multiple_of(header)
        || section_bytes > slab_bytes
    {
        return Err(CuError::from(
            "Twin section size must exceed 512 bytes, be a multiple of 512, and fit in its slab",
        ));
    }
    if section_bytes - header > u32::MAX as usize {
        return Err(CuError::from("Twin section payload size exceeds u32"));
    }
    if limits.max_sessions != 1 {
        return Err(CuError::from(
            "Twin receiver limits must allow exactly one sender session",
        ));
    }
    if limits.equation_capacity > 64 || usize::from(limits.finite_objects.max_symbol_size) > 1128 {
        return Err(CuError::from(
            "Twin receiver limits exceed the 1128-byte-symbol, 64-equation FEC profile",
        ));
    }
    crate::object::validate_limits(limits.finite_objects).map_err(stream_error)?;
    if limits.max_record_bytes > u32::MAX as usize {
        return Err(CuError::from("Twin maximum record size exceeds u32"));
    }
    let object_bytes = usize::try_from(limits.finite_objects.max_object_bytes)
        .map_err(|_| CuError::from("Twin maximum object size exceeds usize"))?;
    let required = limits
        .max_record_bytes
        .max(object_bytes)
        .checked_add(CONTINUITY_OVERHEAD)
        .and_then(|bytes| bytes.checked_add(header))
        .ok_or_else(|| CuError::from("Twin section size requirement overflow"))?;
    if section_bytes < required {
        return Err(CuError::from(format!(
            "Twin section size is {section_bytes} bytes; configured record/object bounds require at least {required} bytes including header and continuity envelope"
        )));
    }
    Ok(())
}

/// Running Copper twin. The separately owned reader can be paused or dropped
/// without affecting recording. `stop()` closes the archive and drains admitted
/// replay; dropping the handle also stops and joins Copper's workers.
pub struct CuTwin<A: LiveReplay> {
    stop: Arc<AtomicBool>,
    worker: Option<JoinHandle<CuResult<CuTwinStatus>>>,
    final_status: Option<CuTwinStatus>,
    log_reader: Option<CuTwinLogReader>,
    app: PhantomData<fn() -> A>,
}

impl<A: LiveReplay> CuTwin<A> {
    pub fn builder<R: CuStreamRx + 'static>(rx: R) -> CuTwinBuilder<A, R> {
        CuTwinBuilder {
            rx,
            feedback_tx: None,
            log_base: None,
            frame_capacity: FRAME_CAPACITY,
            log_capacity: FRAME_CAPACITY,
            replay_capacity: REPLAY_CAPACITY,
            slab_bytes: SLAB_BYTES,
            section_bytes: SECTION_BYTES,
            receiver_limits: SessionRouterLimits::default(),
            reconstruct: true,
            app: PhantomData,
        }
    }

    /// Take the independent bounded log view once (64 entries by default).
    /// Entries retain sender string IDs; use the producing application's string index to render text.
    pub fn take_log_reader(&mut self) -> Option<CuTwinLogReader> {
        self.log_reader.take()
    }

    pub fn stop(&mut self) -> CuResult<CuTwinStatus> {
        self.stop.store(true, Ordering::Release);
        if let Some(worker) = self.worker.take() {
            worker.thread().unpark();
            self.final_status = Some(
                worker
                    .join()
                    .map_err(|_| CuError::from("Copper twin receiver panicked"))??,
            );
        }
        self.final_status
            .ok_or_else(|| CuError::from("Copper twin stopped after a worker failure"))
    }
}

impl<A: LiveReplay> Drop for CuTwin<A> {
    fn drop(&mut self) {
        let _ = self.stop();
    }
}
