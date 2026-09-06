//! Copper-owned receive, archive, and reconstruction lifecycle for one robot session.

use crate::telemetry::{TelemetryReader, telemetry_channel};
use crate::twin::{LiveReplay, TwinFrame, TwinReceiverStatus, TwinStatus, TwinWorker};
use crate::{
    CaptureArchive, CuStreamRx, FiniteObjectLimits, SessionEvent, SessionRouter,
    SessionRouterLimits, StreamIdentity,
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
const SECTION_BYTES: usize = 65536;
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
    pub identity: Option<StreamIdentity>,
    pub last_packet: Option<Instant>,
    pub state: CuTwinRecordingState,
    pub twin: TwinStatus,
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
    log_base: Option<PathBuf>,
    frame_capacity: NonZeroUsize,
    reconstruct: bool,
    app: PhantomData<fn() -> A>,
}

impl<A: LiveReplay, R: CuStreamRx + 'static> CuTwinBuilder<A, R> {
    pub fn with_log_path(mut self, path: impl AsRef<Path>) -> Self {
        self.log_base = Some(path.as_ref().to_path_buf());
        self
    }

    /// Presentation retention only; slowing the reader never delays recording.
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
        let stop = Arc::new(AtomicBool::new(false));
        let stopping = stop.clone();
        let (ready_tx, ready_rx) = std::sync::mpsc::sync_channel(1);
        let worker = thread::Builder::new()
            .name("copper-twin-receiver".into())
            .spawn(move || {
                let mut status = CuTwinStatus::default();
                let (replay, mut publisher) = if self.reconstruct {
                    match TwinWorker::spawn::<A>(REPLAY_CAPACITY, publisher) {
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
                    let mut router = SessionRouter::<1128, 64, 64>::new(SessionRouterLimits {
                        max_sessions: 1,
                        max_startup_packets: 64,
                        max_recovery_records: 8,
                        max_pending_events: 64,
                        max_record_bytes: 4096,
                        max_buffered_records: 64,
                        equation_capacity: 64,
                        finite_objects: FiniteObjectLimits::new(65536, 1128, 4),
                    })
                    .map_err(stream_error)?;
                    let _ = ready_tx.send(Ok(()));
                    let mut rx = self.rx;
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
                                    if matches!(event, SessionEvent::Object { .. }) {
                                        return Ok(());
                                    }
                                    if let SessionEvent::Manifest(manifest) = &event {
                                        status.identity = Some(manifest.manifest().identity);
                                        archive = Some(CaptureArchive::<A::DataSet>::new(
                                            &path,
                                            manifest,
                                            SLAB_BYTES,
                                            SECTION_BYTES,
                                        )?);
                                    }
                                    let writer =
                                        archive.as_mut().ok_or(crate::Error::InvalidConfig(
                                            "capture arrived before its manifest",
                                        ))?;
                                    let capture = writer.accept(&event)?;
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
                                .map_err(|e| CuError::from(format!("Twin receive: {e:?}")))?;
                            status.packets = router.stats().datagrams_seen;
                            publish_status(status);
                        } else {
                            thread::park_timeout(POLL_INTERVAL);
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

/// Running Copper twin. The separately owned reader can be paused or dropped
/// without affecting recording. `stop()` closes the archive and drains admitted
/// replay; dropping the handle also stops and joins Copper's workers.
pub struct CuTwin<A: LiveReplay> {
    stop: Arc<AtomicBool>,
    worker: Option<JoinHandle<CuResult<CuTwinStatus>>>,
    final_status: Option<CuTwinStatus>,
    app: PhantomData<fn() -> A>,
}

impl<A: LiveReplay> CuTwin<A> {
    pub fn builder<R: CuStreamRx + 'static>(rx: R) -> CuTwinBuilder<A, R> {
        CuTwinBuilder {
            rx,
            log_base: None,
            frame_capacity: FRAME_CAPACITY,
            reconstruct: true,
            app: PhantomData,
        }
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
