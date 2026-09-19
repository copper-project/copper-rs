//! Parallel runtime scheduler state for concurrent CopperList execution.
//!
//! The proc macro emits exact ordered workers and their cross-worker
//! dependencies. Workers publish one cache-isolated monotonic progress word;
//! the dispatcher derives CopperList completion from generated terminal
//! thresholds and commits in `clid` order.

use crate::config::NodeId;
pub use crate::curuntime::{ProcessStepOutcome, ProcessStepResult};
use crate::monitoring::ComponentId;
use core::fmt::{Debug, Formatter, Result as FmtResult};
use core::ops::{Deref, DerefMut};
use core::sync::atomic::{AtomicU64, Ordering};

/// Scheduler-facing category for one process-stage checkpoint.
///
/// A stage maps to one node in the generated execution plan. Future worker
/// threads will use this to pick the correct shared mutable lane:
/// - `Task`: a normal Copper task instance
/// - `BridgeRx`: a bridge receive channel
/// - `BridgeTx`: a bridge send channel
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ParallelRtStageKind {
    Task,
    BridgeRx,
    BridgeTx,
}

/// Static metadata describing one ordered process stage in the generated plan.
///
/// Field meanings:
/// - `label`: stable human-readable identifier used in diagnostics and tests.
/// - `kind`: whether the stage targets a task, bridge receive lane, or bridge
///   send lane.
/// - `plan_node_id`: node identifier inside the build-time execution plan.
/// - `component_id`: monitor component id attached to this stage.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ParallelRtStageMetadata {
    pub label: &'static str,
    pub kind: ParallelRtStageKind,
    pub plan_node_id: NodeId,
    pub component_id: ComponentId,
}

impl ParallelRtStageMetadata {
    pub const fn new(
        label: &'static str,
        kind: ParallelRtStageKind,
        plan_node_id: NodeId,
        component_id: ComponentId,
    ) -> Self {
        Self {
            label,
            kind,
            plan_node_id,
            component_id,
        }
    }
}

/// Immutable scheduler layout shared by every runtime instance of a mission.
///
/// `stages` is in the exact order emitted by the proc macro for the per-CL
/// process path. The generated schedule maps those stages onto exact workers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ParallelRtMetadata {
    pub stages: &'static [ParallelRtStageMetadata],
    pub max_in_flight: usize,
}

impl ParallelRtMetadata {
    pub const fn new(stages: &'static [ParallelRtStageMetadata], max_in_flight: usize) -> Self {
        Self {
            stages,
            max_in_flight,
        }
    }

    #[inline]
    pub const fn process_stage_count(self) -> usize {
        self.stages.len()
    }
}

/// Empty metadata used by tests and by code paths that do not generate any
/// process-stage parallel layout.
pub const DISABLED_PARALLEL_RT_METADATA: ParallelRtMetadata = ParallelRtMetadata::new(&[], 1);

/// Cache isolation used for independently written scheduler state.
#[repr(align(128))]
pub struct CachePadded<T>(pub T);

impl<T> CachePadded<T> {
    pub const fn new(value: T) -> Self {
        Self(value)
    }

    pub fn into_inner(self) -> T {
        self.0
    }
}

impl<T> Deref for CachePadded<T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<T> DerefMut for CachePadded<T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl<T: Debug> Debug for CachePadded<T> {
    fn fmt(&self, f: &mut Formatter<'_>) -> FmtResult {
        self.0.fmt(f)
    }
}

/// Monotonic authorization cursor used by ordered commit.
///
/// `next_clid` is the smallest CopperList id the serial commit path may accept.
#[derive(Debug)]
pub struct CausalityCheckpoint {
    pub next_clid: AtomicU64,
}

impl CausalityCheckpoint {
    pub const fn new(initial_clid: u64) -> Self {
        Self {
            next_clid: AtomicU64::new(initial_clid),
        }
    }

    #[inline]
    pub fn current_clid(&self) -> u64 {
        self.next_clid.load(Ordering::Acquire)
    }

    #[inline]
    pub fn is_authorized_for(&self, clid: u64) -> bool {
        self.current_clid() == clid
    }

    #[inline]
    pub fn authorize_next(&self, next_clid: u64) {
        self.next_clid.store(next_clid, Ordering::Release);
    }
}

#[cfg(all(feature = "std", feature = "parallel-rt"))]
mod imp {
    use super::{CachePadded, CausalityCheckpoint, ParallelRtMetadata};
    use cu29_traits::CuResult;

    /// Feature-enabled runtime state shared by the generated stage pipeline.
    pub struct ParallelRt<const NBCL: usize> {
        /// Static process-stage layout emitted by the proc macro.
        metadata: &'static ParallelRtMetadata,
        /// Ordered commit cursor for monitor/keyframe/log handoff.
        commit_checkpoint: CachePadded<CausalityCheckpoint>,
        /// Maximum number of CopperLists intended to be in flight at once.
        in_flight_limit: usize,
    }

    impl<const NBCL: usize> ParallelRt<NBCL> {
        pub fn new(metadata: &'static ParallelRtMetadata) -> CuResult<Self> {
            if metadata.max_in_flight == 0 || metadata.max_in_flight > NBCL {
                return Err(cu29_traits::CuError::from(format!(
                    "Pipeline max_in_flight ({}) must be within 1..={NBCL}",
                    metadata.max_in_flight,
                )));
            }
            Ok(Self {
                metadata,
                commit_checkpoint: CachePadded::new(CausalityCheckpoint::new(0)),
                in_flight_limit: metadata.max_in_flight,
            })
        }

        #[inline]
        pub fn enabled(&self) -> bool {
            !self.metadata.stages.is_empty()
        }

        #[inline]
        pub const fn metadata(&self) -> &'static ParallelRtMetadata {
            self.metadata
        }

        #[inline]
        pub const fn commit_checkpoint(&self) -> &CachePadded<CausalityCheckpoint> {
            &self.commit_checkpoint
        }

        #[inline]
        pub const fn in_flight_limit(&self) -> usize {
            self.in_flight_limit
        }

        /// Reinitializes the ordered commit cursor to the next CopperList id
        /// that will be dispatched by a fresh parallel run loop.
        pub fn reset_cursors(&self, next_clid: u64) {
            self.commit_checkpoint.authorize_next(next_clid);
        }

        #[inline]
        pub fn current_commit_clid(&self) -> u64 {
            self.commit_checkpoint.current_clid()
        }

        #[inline]
        pub fn release_commit(&self, next_clid: u64) {
            self.commit_checkpoint.authorize_next(next_clid);
        }
    }
}

#[cfg(all(feature = "std", feature = "parallel-rt"))]
mod lanes {
    use super::{CachePadded, ProcessStepOutcome, ProcessStepResult};
    use alloc::vec::Vec;
    use core::cell::UnsafeCell;
    use core::sync::atomic::AtomicBool;
    use core::sync::atomic::AtomicPtr;
    use core::sync::atomic::AtomicU8;
    use core::sync::atomic::AtomicU64;
    use core::sync::atomic::AtomicUsize;
    use core::sync::atomic::Ordering;
    use cu29_traits::CuError;
    use std::sync::OnceLock;
    use std::thread::Thread;

    const SPIN_ROUNDS: u32 = 256;

    const ERROR_EMPTY: u8 = 0;
    const ERROR_WRITING: u8 = 1;
    const ERROR_READY: u8 = 2;
    const ERROR_TAKEN: u8 = 3;

    #[repr(align(128))]
    struct LaneSlot {
        culist: AtomicPtr<u8>,
        keyframe: AtomicPtr<u8>,
        keyframe_len: AtomicUsize,
        aborted: AtomicBool,
        clid: AtomicU64,
    }

    struct FailureSlot {
        state: AtomicU8,
        clid: AtomicU64,
        error: UnsafeCell<Option<CuError>>,
    }

    // One worker claims the cell with ERROR_WRITING. The dispatcher reads it
    // only after the release store of ERROR_READY. Admission stops immediately,
    // while workers finish CopperLists older than the failing one.
    unsafe impl Sync for FailureSlot {}

    /// Preallocated admission, dependency, and completion state for workers.
    pub struct LaneExecutor {
        first_clid: u64,
        admitted: CachePadded<AtomicU64>,
        stopping: AtomicBool,
        shutdown: AtomicBool,
        worker_progress: Vec<CachePadded<AtomicU64>>,
        slots: Vec<LaneSlot>,
        copperlists_per_cycle: u64,
        dispatcher_generation: CachePadded<AtomicU64>,
        dispatcher: Thread,
        worker_threads: Vec<OnceLock<Thread>>,
        failure: CachePadded<FailureSlot>,
    }

    impl LaneExecutor {
        pub fn new(
            first_clid: u64,
            max_in_flight: usize,
            copperlists_per_cycle: u32,
            workers: usize,
        ) -> Self {
            Self {
                first_clid,
                admitted: CachePadded::new(AtomicU64::new(first_clid)),
                stopping: AtomicBool::new(false),
                shutdown: AtomicBool::new(false),
                worker_progress: (0..workers)
                    .map(|_| CachePadded::new(AtomicU64::new(0)))
                    .collect(),
                slots: (0..max_in_flight.max(1))
                    .map(|_| LaneSlot {
                        culist: AtomicPtr::new(core::ptr::null_mut()),
                        keyframe: AtomicPtr::new(core::ptr::null_mut()),
                        keyframe_len: AtomicUsize::new(0),
                        aborted: AtomicBool::new(false),
                        clid: AtomicU64::new(u64::MAX),
                    })
                    .collect(),
                copperlists_per_cycle: u64::from(copperlists_per_cycle.max(1)),
                dispatcher_generation: CachePadded::new(AtomicU64::new(0)),
                dispatcher: std::thread::current(),
                worker_threads: (0..workers).map(|_| OnceLock::new()).collect(),
                failure: CachePadded::new(FailureSlot {
                    state: AtomicU8::new(ERROR_EMPTY),
                    clid: AtomicU64::new(u64::MAX),
                    error: UnsafeCell::new(None),
                }),
            }
        }

        fn slot(&self, clid: u64) -> &LaneSlot {
            &self.slots[(clid % self.slots.len() as u64) as usize]
        }

        fn wake_dispatcher(&self) {
            self.dispatcher_generation.fetch_add(1, Ordering::AcqRel);
            self.dispatcher.unpark();
        }

        fn wake_all_workers(&self) {
            for worker in &self.worker_threads {
                if let Some(worker) = worker.get() {
                    worker.unpark();
                }
            }
        }

        fn wait_until(&self, mut condition: impl FnMut() -> bool) -> bool {
            loop {
                for _ in 0..SPIN_ROUNDS {
                    if condition() {
                        return true;
                    }
                    if self.shutdown.load(Ordering::Acquire) {
                        return false;
                    }
                    core::hint::spin_loop();
                }
                if condition() {
                    return true;
                }
                if self.shutdown.load(Ordering::Acquire) {
                    return false;
                }
                std::thread::park();
            }
        }

        pub fn register_worker(&self, worker: usize) {
            self.worker_threads[worker]
                .set(std::thread::current())
                .unwrap_or_else(|_| panic!("lane worker {worker} registered twice"));
        }

        pub fn progress_generation(&self) -> u64 {
            self.dispatcher_generation.load(Ordering::Acquire)
        }

        pub fn wait_for_progress(&self, observed: u64, timeout: Option<std::time::Duration>) {
            if self.dispatcher_generation.load(Ordering::Acquire) != observed {
                return;
            }
            if let Some(timeout) = timeout {
                std::thread::park_timeout(timeout);
            } else {
                std::thread::park();
            }
        }

        pub fn first_clid(&self) -> u64 {
            self.first_clid
        }

        pub fn cycle_of(&self, clid: u64) -> u64 {
            (clid - self.first_clid) / self.copperlists_per_cycle
        }

        pub fn keyframe_capture(&self, clid: u64) -> (*mut u8, usize) {
            let slot = self.slot(clid);
            (
                slot.keyframe.load(Ordering::Acquire),
                slot.keyframe_len.load(Ordering::Acquire),
            )
        }

        pub fn admit_with_keyframe(
            &self,
            clid: u64,
            culist: *mut u8,
            keyframe: *mut u8,
            keyframe_len: usize,
        ) {
            debug_assert_eq!(self.admitted.load(Ordering::Acquire), clid);
            let slot = self.slot(clid);
            slot.aborted.store(false, Ordering::Relaxed);
            slot.clid.store(clid, Ordering::Relaxed);
            slot.keyframe.store(keyframe, Ordering::Relaxed);
            slot.keyframe_len.store(keyframe_len, Ordering::Relaxed);
            slot.culist.store(culist, Ordering::Release);
            self.admitted.store(clid + 1, Ordering::Release);
            self.wake_all_workers();
        }

        pub fn next_admission(&self) -> u64 {
            self.admitted.load(Ordering::Acquire)
        }

        pub fn stop_admitting(&self) {
            self.stopping.store(true, Ordering::Release);
            self.wake_all_workers();
        }

        pub fn request_shutdown(&self) {
            self.shutdown.store(true, Ordering::Release);
            self.wake_dispatcher();
            self.wake_all_workers();
        }

        pub fn is_shut_down(&self) -> bool {
            self.shutdown.load(Ordering::Acquire)
        }

        pub fn wait_admitted(&self, clid: u64) -> Option<*mut u8> {
            let ready = self.wait_until(|| {
                self.admitted.load(Ordering::Acquire) > clid
                    || self.stopping.load(Ordering::Acquire)
            });
            if !ready || self.admitted.load(Ordering::Acquire) <= clid {
                return None;
            }
            Some(self.slot(clid).culist.load(Ordering::Acquire))
        }

        pub fn wait_progress(&self, worker: usize, target: u64) -> bool {
            let progress = &self.worker_progress[worker];
            self.wait_until(|| progress.load(Ordering::Acquire) >= target)
        }

        pub fn worker_reached(&self, worker: usize, target: u64) -> bool {
            self.worker_progress[worker].load(Ordering::Acquire) >= target
        }

        pub fn publish_progress(
            &self,
            worker: usize,
            progress: u64,
            dependent_workers: &[usize],
            terminal: bool,
        ) {
            self.worker_progress[worker].store(progress, Ordering::Release);
            for &dependent in dependent_workers {
                if let Some(thread) = self.worker_threads[dependent].get() {
                    thread.unpark();
                }
            }
            if terminal {
                self.wake_dispatcher();
            }
        }

        pub fn publish_error(&self, clid: u64, error: CuError) {
            if self
                .failure
                .state
                .compare_exchange(
                    ERROR_EMPTY,
                    ERROR_WRITING,
                    Ordering::AcqRel,
                    Ordering::Acquire,
                )
                .is_ok()
            {
                // SAFETY: this worker exclusively claimed the error cell above.
                unsafe { *self.failure.error.get() = Some(error) };
                self.failure.clid.store(clid, Ordering::Relaxed);
                self.failure.state.store(ERROR_READY, Ordering::Release);
            }
            self.stop_admitting();
            self.wake_dispatcher();
        }

        pub fn completion_outcome(&self, clid: u64) -> ProcessStepResult {
            let slot = self.slot(clid);
            debug_assert_eq!(slot.clid.load(Ordering::Acquire), clid);
            if slot.aborted.load(Ordering::Acquire) {
                Ok(ProcessStepOutcome::AbortCopperList)
            } else {
                Ok(ProcessStepOutcome::Continue)
            }
        }

        pub fn take_failure(&self) -> Option<(u64, CuError)> {
            if self.failure.state.load(Ordering::Acquire) != ERROR_READY {
                return None;
            }
            let clid = self.failure.clid.load(Ordering::Acquire);
            // SAFETY: ERROR_READY publishes the value, and only the dispatcher
            // calls this method.
            let error = unsafe { &mut *self.failure.error.get() }
                .take()
                .expect("lane error state without an error");
            self.failure.state.store(ERROR_TAKEN, Ordering::Release);
            Some((clid, error))
        }

        pub fn abort(&self, clid: u64) {
            self.slot(clid).aborted.store(true, Ordering::Release);
        }

        pub fn is_aborted(&self, clid: u64) -> bool {
            self.slot(clid).aborted.load(Ordering::Acquire)
        }
    }
}

#[cfg(all(feature = "std", feature = "parallel-rt"))]
pub use lanes::LaneExecutor;

#[cfg(not(all(feature = "std", feature = "parallel-rt")))]
mod imp {
    use super::{CachePadded, CausalityCheckpoint, ParallelRtMetadata};
    use cu29_traits::CuResult;

    /// Feature-disabled placeholder.
    ///
    /// Keeping the type available lets the rest of the runtime compose against a
    /// single API while the actual executor remains behind the `parallel-rt`
    /// feature.
    pub struct ParallelRt<const NBCL: usize> {
        metadata: &'static ParallelRtMetadata,
        commit_checkpoint: CachePadded<CausalityCheckpoint>,
    }

    impl<const NBCL: usize> ParallelRt<NBCL> {
        pub fn new(metadata: &'static ParallelRtMetadata) -> CuResult<Self> {
            Ok(Self {
                metadata,
                commit_checkpoint: CachePadded::new(CausalityCheckpoint::new(0)),
            })
        }

        #[inline]
        pub const fn enabled(&self) -> bool {
            false
        }

        #[inline]
        pub const fn metadata(&self) -> &'static ParallelRtMetadata {
            self.metadata
        }

        #[inline]
        pub const fn commit_checkpoint(&self) -> &CachePadded<CausalityCheckpoint> {
            &self.commit_checkpoint
        }

        #[inline]
        pub const fn in_flight_limit(&self) -> usize {
            NBCL
        }

        #[inline]
        pub fn reset_cursors(&self, next_clid: u64) {
            self.commit_checkpoint.authorize_next(next_clid);
        }

        #[inline]
        pub fn current_commit_clid(&self) -> u64 {
            self.commit_checkpoint.current_clid()
        }

        #[inline]
        pub fn release_commit(&self, next_clid: u64) {
            self.commit_checkpoint.authorize_next(next_clid);
        }
    }
}

pub use imp::ParallelRt;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::monitoring::ComponentId;

    #[test]
    fn checkpoint_advances_monotonically() {
        let checkpoint = CausalityCheckpoint::new(0);
        assert!(checkpoint.is_authorized_for(0));
        checkpoint.authorize_next(1);
        assert!(!checkpoint.is_authorized_for(0));
        assert!(checkpoint.is_authorized_for(1));
    }

    #[test]
    fn hot_scheduler_words_are_cache_isolated() {
        assert_eq!(core::mem::align_of::<CachePadded<AtomicU64>>(), 128);
        assert_eq!(core::mem::size_of::<CachePadded<AtomicU64>>(), 128);
    }

    #[test]
    fn disabled_metadata_is_empty() {
        assert_eq!(DISABLED_PARALLEL_RT_METADATA.process_stage_count(), 0);
    }

    #[test]
    fn parallel_rt_stage_metadata_is_const_constructible() {
        const STAGES: &[ParallelRtStageMetadata] = &[ParallelRtStageMetadata::new(
            "demo",
            ParallelRtStageKind::Task,
            7,
            ComponentId::new(3),
        )];
        const METADATA: ParallelRtMetadata = ParallelRtMetadata::new(STAGES, 4);
        assert_eq!(METADATA.process_stage_count(), 1);
        assert_eq!(METADATA.stages[0].label, "demo");
    }

    #[cfg(all(feature = "std", feature = "parallel-rt"))]
    #[test]
    fn enabled_parallel_rt_tracks_metadata_and_limit() {
        const STAGES: &[ParallelRtStageMetadata] = &[
            ParallelRtStageMetadata::new("a", ParallelRtStageKind::Task, 0, ComponentId::new(0)),
            ParallelRtStageMetadata::new("b", ParallelRtStageKind::Task, 1, ComponentId::new(1)),
        ];
        const METADATA: ParallelRtMetadata = ParallelRtMetadata::new(STAGES, 4);

        let rt = ParallelRt::<4>::new(&METADATA).expect("parallel rt should build");
        assert!(rt.enabled());
        assert_eq!(rt.metadata().process_stage_count(), 2);
        assert_eq!(rt.in_flight_limit(), 4);
    }

    #[cfg(all(feature = "std", feature = "parallel-rt"))]
    #[test]
    fn parallel_rt_rejects_invalid_capacity() {
        const EMPTY: &[ParallelRtStageMetadata] = &[];
        const ZERO: ParallelRtMetadata = ParallelRtMetadata::new(EMPTY, 0);
        const EXCESS: ParallelRtMetadata = ParallelRtMetadata::new(EMPTY, 5);

        assert!(ParallelRt::<4>::new(&ZERO).is_err());
        assert!(ParallelRt::<4>::new(&EXCESS).is_err());
    }

    #[cfg(all(feature = "std", feature = "parallel-rt"))]
    #[test]
    fn lane_progress_wakes_after_release_publication() {
        use std::sync::Arc;
        use std::sync::mpsc;

        let lanes = Arc::new(LaneExecutor::new(10, 2, 1, 2));
        let (registered_tx, registered_rx) = mpsc::channel();
        let waiter = {
            let lanes = Arc::clone(&lanes);
            std::thread::spawn(move || {
                lanes.register_worker(1);
                registered_tx.send(()).unwrap();
                lanes.wait_progress(0, 7)
            })
        };
        registered_rx.recv().unwrap();
        assert!(!lanes.worker_reached(0, 7));
        lanes.publish_progress(0, 7, &[1], true);
        assert!(waiter.join().unwrap());
        assert!(lanes.worker_reached(0, 7));
        assert_ne!(lanes.progress_generation(), 0);
    }

    #[cfg(all(feature = "std", feature = "parallel-rt"))]
    #[test]
    fn shutdown_releases_waiters_and_admission_reinitializes_slot_state() {
        use std::sync::Arc;
        use std::sync::mpsc;

        let lanes = Arc::new(LaneExecutor::new(4, 1, 2, 1));
        let (registered_tx, registered_rx) = mpsc::channel();
        let waiter = {
            let lanes = Arc::clone(&lanes);
            std::thread::spawn(move || {
                lanes.register_worker(0);
                registered_tx.send(()).unwrap();
                lanes.wait_admitted(4).is_none()
            })
        };
        registered_rx.recv().unwrap();
        lanes.request_shutdown();
        assert!(waiter.join().unwrap());

        let lanes = LaneExecutor::new(4, 1, 2, 0);
        let mut culist = 0u8;
        let mut keyframe = [0u8; 8];
        lanes.admit_with_keyframe(4, &mut culist, keyframe.as_mut_ptr(), keyframe.len());
        assert_eq!(lanes.wait_admitted(4), Some(&mut culist as *mut u8));
        assert_eq!(lanes.keyframe_capture(4), (keyframe.as_mut_ptr(), 8));
        lanes.abort(4);
        assert!(lanes.is_aborted(4));
        lanes.admit_with_keyframe(5, &mut culist, core::ptr::null_mut(), 0);
        assert!(!lanes.is_aborted(5));
        assert_eq!(lanes.cycle_of(5), 0);
        assert_eq!(lanes.cycle_of(6), 1);
    }

    #[cfg(all(feature = "std", feature = "parallel-rt"))]
    #[test]
    fn worker_error_stops_admission_without_interrupting_older_work() {
        use cu29_traits::CuError;

        let lanes = LaneExecutor::new(4, 1, 1, 0);
        lanes.publish_error(7, CuError::from("worker failed"));

        assert!(!lanes.is_shut_down());
        assert!(lanes.wait_admitted(4).is_none());
        let (clid, error) = lanes.take_failure().expect("published failure");
        assert_eq!(clid, 7);
        assert!(error.to_string().contains("worker failed"));
    }

    #[cfg(not(all(feature = "std", feature = "parallel-rt")))]
    #[test]
    fn disabled_parallel_rt_preserves_metadata() {
        const STAGES: &[ParallelRtStageMetadata] = &[ParallelRtStageMetadata::new(
            "a",
            ParallelRtStageKind::Task,
            0,
            ComponentId::new(0),
        )];
        const METADATA: ParallelRtMetadata = ParallelRtMetadata::new(STAGES, 4);

        let rt = ParallelRt::<4>::new(&METADATA).expect("parallel rt placeholder should build");
        assert!(!rt.enabled());
        assert_eq!(rt.metadata().process_stage_count(), 1);
    }
}
