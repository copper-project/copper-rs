//! Ordered, bounded live replay, independent of archive writes and presentation loss.
use crate::{
    StreamIdentity,
    capture::{CaptureDataSet, CapturedList},
    telemetry::TelemetryPublisher,
};
use crossbeam_queue::ArrayQueue;
use cu29_clock::RobotClockMock;
use cu29_runtime::{copperlist::CopperList, curuntime::KeyFrame};
use cu29_traits::{CopperListTuple, CuResult};
use std::{
    num::NonZeroUsize,
    sync::{
        Arc,
        atomic::{AtomicU64, Ordering},
        mpsc::{self, SyncSender, TrySendError},
    },
    thread::{self, JoinHandle},
    time::{Duration, Instant},
};

/// Implemented by `#[copper_runtime(..., sim_mode = true)]` with logstream enabled.
pub trait LiveReplay: Sized + 'static {
    type DataSet: CaptureDataSet + Send + 'static;
    fn build_twin() -> CuResult<(Self, RobotClockMock)>;
    fn stop_twin(&mut self) -> CuResult<()> {
        Ok(())
    }
    fn replay_capture(
        &mut self,
        clock: &RobotClockMock,
        capture: &mut CopperList<Self::DataSet>,
        keyframe: Option<&KeyFrame>,
    ) -> CuResult<()>;
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ReconstructionState {
    #[default]
    Waiting,
    Recovering,
    Reconstructed,
    Verified,
    Diverged,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TwinStatus {
    pub state: ReconstructionState,
    pub reconstructed: u64,
    pub verified: u64,
    pub divergences: u64,
    pub queue_overflows: u64,
}

/// Compose receiver/archive status with independently available reconstruction health.
pub trait TwinReceiverStatus: Copy + PartialEq + Send + 'static {
    fn with_twin(self, status: TwinStatus) -> Self;
}

pub struct TwinFrame<P: CopperListTuple> {
    pub identity: StreamIdentity,
    pub received_at: Instant,
    pub copperlist: CopperList<P>,
}

/// Synchronous replay engine, also usable for ordinary offline capture replay.
pub struct LiveTwin<A: LiveReplay> {
    app: A,
    clock: RobotClockMock,
    next: Option<u64>,
    pub status: TwinStatus,
}
impl<A: LiveReplay> LiveTwin<A> {
    pub fn new() -> CuResult<Self> {
        let (app, clock) = A::build_twin()?;
        Ok(Self {
            app,
            clock,
            next: None,
            status: TwinStatus::default(),
        })
    }
    pub fn recover(&mut self) {
        self.next = None;
        self.status.state = ReconstructionState::Recovering;
    }
    pub fn reconstruct(
        &mut self,
        mut capture: CapturedList<A::DataSet>,
        keyframe: Option<&KeyFrame>,
    ) -> CuResult<Option<CopperList<A::DataSet>>> {
        let id = capture.copperlist.id;
        if self.next != Some(id) && keyframe.is_none_or(|k| k.culistid != id) {
            if self.status.state != ReconstructionState::Diverged {
                self.recover();
            }
            return Ok(None);
        }
        let result = self
            .app
            .replay_capture(&self.clock, &mut capture.copperlist, keyframe);
        #[cfg(feature = "verify-reconstruction")]
        let result = result.and_then(|()| {
            capture
                .verify()
                .map_err(|e| cu29_traits::CuError::from(e.to_string()))
        });
        if let Err(error) = result {
            self.next = None;
            self.status.state = ReconstructionState::Diverged;
            self.status.divergences += 1;
            return Err(error);
        }
        self.next = id.checked_add(1);
        self.status.state = ReconstructionState::Reconstructed;
        #[cfg(feature = "verify-reconstruction")]
        if capture.digest.is_some() {
            self.status.state = ReconstructionState::Verified;
        }
        self.status.reconstructed += 1;
        if self.status.state == ReconstructionState::Verified {
            self.status.verified += 1;
        }
        Ok(Some(capture.copperlist))
    }
}

impl<A: LiveReplay> Drop for LiveTwin<A> {
    fn drop(&mut self) {
        let _ = self.app.stop_twin();
    }
}

enum Input<P: CopperListTuple> {
    Capture {
        capture: CapturedList<P>,
        identity: StreamIdentity,
        received_at: Instant,
    },
    Anchor(KeyFrame),
}
struct Work<P: CopperListTuple> {
    input: Input<P>,
    generation: u64,
}
struct Pending<P: CopperListTuple> {
    capture: CapturedList<P>,
    identity: StreamIdentity,
    received_at: Instant,
}

/// Nonblocking archive-to-replay handoff. The worker consumes every admitted
/// iteration; only successfully reconstructed results enter the UI ring.
/// Payload verification is opt-in through `verify-reconstruction`.
/// Storage bounds are `capacity` queued events, `capacity` pending captures,
/// one pending anchor and one executing frame, in addition to the telemetry ring.
/// Overflow invalidates replay until an admitted matching anchor, never archival.
#[doc(hidden)]
pub struct TwinWorker<P: CopperListTuple, S> {
    sender: Option<SyncSender<Work<P>>>,
    statuses: Arc<ArrayQueue<S>>,
    generation: Arc<AtomicU64>,
    overflows: Arc<AtomicU64>,
    worker: Option<JoinHandle<TwinStatus>>,
}

impl<P: CaptureDataSet + Send + 'static, S: TwinReceiverStatus> TwinWorker<P, S> {
    pub fn spawn<A: LiveReplay<DataSet = P>>(
        capacity: NonZeroUsize,
        mut publisher: TelemetryPublisher<TwinFrame<P>, S>,
    ) -> CuResult<Self> {
        let (sender, receiver) = mpsc::sync_channel::<Work<P>>(capacity.get());
        let statuses = Arc::new(ArrayQueue::new(1));
        let pending_status = statuses.clone();
        let generation = Arc::new(AtomicU64::new(0));
        let epoch = generation.clone();
        let overflows = Arc::new(AtomicU64::new(0));
        let drops = overflows.clone();
        let (ready_tx, ready_rx) = mpsc::sync_channel(1);
        let worker = thread::Builder::new()
            .name("copper-live-twin".into())
            .spawn(move || {
                let mut twin = match LiveTwin::<A>::new() {
                    Ok(twin) => {
                        let _ = ready_tx.send(Ok(()));
                        twin
                    }
                    Err(e) => {
                        let _ = ready_tx.send(Err(e));
                        return TwinStatus::default();
                    }
                };
                let mut current_generation = 0;
                let mut pending: std::collections::VecDeque<Pending<P>> =
                    std::collections::VecDeque::with_capacity(capacity.get());
                let mut anchor: Option<KeyFrame> = None;
                let mut receiver_status = publisher.status();
                loop {
                    if let Some(status) = pending_status.pop() {
                        receiver_status = status;
                    }
                    let latest_generation = epoch.load(Ordering::Acquire);
                    if current_generation != latest_generation {
                        twin.recover();
                        pending.clear();
                        anchor = None;
                        current_generation = latest_generation;
                    }
                    twin.status.queue_overflows = drops.load(Ordering::Relaxed);
                    publisher.set_status(receiver_status.with_twin(twin.status));
                    match receiver.recv_timeout(Duration::from_millis(10)) {
                        Ok(work) => {
                            let latest = epoch.load(Ordering::Acquire);
                            if current_generation != latest {
                                twin.recover();
                                pending.clear();
                                anchor = None;
                                current_generation = latest;
                            }
                            if work.generation != current_generation {
                                continue;
                            }
                            match work.input {
                                Input::Capture {
                                    capture,
                                    identity,
                                    received_at,
                                } => {
                                    if pending.len() == capacity.get() {
                                        pending.pop_front();
                                        drops.fetch_add(1, Ordering::Relaxed);
                                        twin.recover();
                                    }
                                    pending.push_back(Pending {
                                        capture,
                                        identity,
                                        received_at,
                                    });
                                }
                                Input::Anchor(frame) => {
                                    if anchor
                                        .as_ref()
                                        .is_none_or(|old| old.culistid < frame.culistid)
                                    {
                                        anchor = Some(frame);
                                    }
                                }
                            }
                            while let Some(first) = pending.front() {
                                if twin.next != Some(first.capture.copperlist.id) {
                                    let Some(frame) = &anchor else {
                                        break;
                                    };
                                    let Some(position) = pending.iter().position(|item| {
                                        item.capture.copperlist.id == frame.culistid
                                    }) else {
                                        break;
                                    };
                                    pending.drain(..position);
                                }
                                let item = pending.pop_front().unwrap();
                                let keyframe = if anchor.as_ref().is_some_and(|frame| {
                                    frame.culistid == item.capture.copperlist.id
                                }) {
                                    anchor.take()
                                } else {
                                    None
                                };
                                if let Ok(Some(copperlist)) =
                                    twin.reconstruct(item.capture, keyframe.as_ref())
                                    && epoch.load(Ordering::Acquire) == current_generation
                                {
                                    publisher.publish(TwinFrame {
                                        identity: item.identity,
                                        received_at: item.received_at,
                                        copperlist,
                                    });
                                }
                            }
                        }
                        Err(mpsc::RecvTimeoutError::Timeout) => {}
                        Err(mpsc::RecvTimeoutError::Disconnected) => break,
                    }
                }
                if let Some(status) = pending_status.pop() {
                    receiver_status = status;
                }
                publisher.set_status(receiver_status.with_twin(twin.status));
                twin.status
            })
            .map_err(|e| cu29_traits::CuError::new_with_cause("spawn live twin", e))?;
        ready_rx
            .recv()
            .map_err(|_| cu29_traits::CuError::from("live twin initialization failed"))??;
        Ok(Self {
            sender: Some(sender),
            statuses,
            generation,
            overflows,
            worker: Some(worker),
        })
    }
    pub fn finish(mut self) -> CuResult<TwinStatus> {
        self.sender.take();
        self.worker
            .take()
            .ok_or_else(|| cu29_traits::CuError::from("Twin worker already joined"))?
            .join()
            .map_err(|_| cu29_traits::CuError::from("Twin replay worker panicked"))
    }

    /// Route an already archived event into live replay. This is the usual
    /// ground-station integration: `twin.accept(event, archive.accept(event)?)`.
    /// Archival must succeed first; replay never retries or delays that writer.
    pub fn accept(
        &self,
        event: &crate::SessionEvent,
        capture: Option<CapturedList<P>>,
    ) -> crate::Result<()> {
        match event {
            crate::SessionEvent::Manifest(_) | crate::SessionEvent::Gap { .. } => self.recover(),
            crate::SessionEvent::VerifiedAnchor { keyframe, .. } => {
                self.anchor(crate::decode_keyframe(keyframe.decoded()?.payload)?)
            }
            crate::SessionEvent::ContinuousRecord { identity, .. } => {
                if let Some(capture) = capture {
                    self.submit(capture, *identity, Instant::now());
                }
            }
            _ => {}
        }
        Ok(())
    }
    pub fn set_status(&self, status: S) {
        self.statuses.force_push(status);
    }
    pub fn recover(&self) {
        self.generation.fetch_add(1, Ordering::AcqRel);
    }
    pub fn submit(&self, capture: CapturedList<P>, identity: StreamIdentity, received_at: Instant) {
        self.send(Input::Capture {
            capture,
            identity,
            received_at,
        });
    }
    /// Accept only keyframes from the session router's VerifiedAnchor event.
    /// Control objects may arrive before or after their captured boundary.
    pub fn anchor(&self, frame: KeyFrame) {
        self.send(Input::Anchor(frame));
    }
    fn send(&self, input: Input<P>) {
        let work = Work {
            input,
            generation: self.generation.load(Ordering::Acquire),
        };
        if let Err(TrySendError::Full(_)) = self.sender.as_ref().unwrap().try_send(work) {
            self.overflows.fetch_add(1, Ordering::Relaxed);
            self.recover();
        }
    }
}
impl<P: CopperListTuple, S> Drop for TwinWorker<P, S> {
    fn drop(&mut self) {
        self.sender.take();
        if let Some(worker) = self.worker.take() {
            let _ = worker.join();
        }
    }
}
