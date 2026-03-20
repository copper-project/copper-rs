//! Parallel runtime scaffolding for future concurrent CopperList execution.
//!
//! This module intentionally separates:
//! - static execution-plan metadata (`ParallelRtMetadata`)
//! - hot synchronization cursors (`CausalityCheckpoint`)
//! - in-flight work ownership (`IterationTicket`)
//! - feature-gated runtime state (`ParallelRt`)
//!
//! The current implementation only wires the data structures and the clean
//! feature split. The generated runtime still executes CopperLists
//! synchronously. The goal is to let the proc-macro and runtime agree on a
//! stable parallel execution model before the worker/commit pipeline is
//! switched on.

use crate::config::NodeId;
use crate::copperlist::{CopperList, CuListZeroedInit};
use crate::monitoring::ComponentId;
use alloc::boxed::Box;
use alloc::vec::Vec;
use core::fmt::{Debug, Formatter, Result as FmtResult};
use core::ops::{Deref, DerefMut};
use core::sync::atomic::{AtomicU64, Ordering};
use cu29_clock::CuTime;
use cu29_traits::CopperListTuple;

/// Conservative default for busy-spin handoff attempts before yielding.
///
/// The parallel runtime is not active yet, but the setting is part of the
/// public scheduler contract because the checkpoint wait strategy is a core
/// performance tradeoff.
pub const DEFAULT_PARALLEL_RT_SPIN_ITERS: u32 = 64;

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
/// process path. Future parallel execution will allocate one causality
/// checkpoint per entry.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ParallelRtMetadata {
    pub stages: &'static [ParallelRtStageMetadata],
}

impl ParallelRtMetadata {
    pub const fn new(stages: &'static [ParallelRtStageMetadata]) -> Self {
        Self { stages }
    }

    #[inline]
    pub const fn process_stage_count(self) -> usize {
        self.stages.len()
    }
}

/// Empty metadata used by tests and by code paths that do not generate any
/// process-stage parallel layout.
pub const DISABLED_PARALLEL_RT_METADATA: ParallelRtMetadata = ParallelRtMetadata::new(&[]);

/// Minimal cache-line padding wrapper used for hot scheduler cursors.
///
/// Checkpoints are expected to be updated by different cores at a high rate,
/// so they should not share cache lines with each other or with colder
/// metadata. A dedicated wrapper keeps that policy explicit without pulling in
/// another dependency.
#[repr(align(64))]
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

/// Monotonic authorization cursor for one causality stage.
///
/// `next_clid` is the smallest CopperList id allowed to enter the guarded
/// stage. When a worker finishes stage `S` for CopperList `n`, it releases
/// `n + 1` for the next worker waiting on the same stage.
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

/// Runtime-tunable scheduler settings for the future parallel executor.
///
/// Field meanings:
/// - `executor_threads`: number of dedicated process workers intended to run
///   CopperList stages.
/// - `spin_iterations`: how many times a worker should hot-spin on a checkpoint
///   before yielding/parking.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ParallelRtSettings {
    pub executor_threads: usize,
    pub spin_iterations: u32,
}

#[cfg(all(feature = "std", feature = "parallel-rt"))]
impl Default for ParallelRtSettings {
    fn default() -> Self {
        let executor_threads = std::thread::available_parallelism()
            .map(usize::from)
            .unwrap_or(1)
            .max(1);
        Self {
            executor_threads,
            spin_iterations: DEFAULT_PARALLEL_RT_SPIN_ITERS,
        }
    }
}

#[cfg(not(all(feature = "std", feature = "parallel-rt")))]
impl Default for ParallelRtSettings {
    fn default() -> Self {
        Self {
            executor_threads: 1,
            spin_iterations: 0,
        }
    }
}

/// Per-CopperList scratch state that a future parallel executor will own while
/// the list is in flight.
///
/// This mirrors the current single-threaded keyframe lifecycle but makes the
/// ownership explicit so multiple CopperLists can carry independent snapshots at
/// the same time.
#[derive(Debug, Clone, Default)]
pub struct ParallelKeyFrameScratch {
    pub culistid: u64,
    pub timestamp: CuTime,
    pub serialized_tasks: Vec<u8>,
}

/// Ownership container for one in-flight CopperList.
///
/// Field meanings:
/// - `clid`: globally ordered CopperList id.
/// - `culist`: the boxed CopperList buffer currently being filled or committed.
/// - `keyframe`: optional per-CL snapshot scratch space.
/// - `raw_payload_bytes`: payload bytes observed before serialization.
/// - `handle_bytes`: handle-backed payload accounting for monitor/log I/O stats.
#[derive(Debug)]
pub struct IterationTicket<P: CopperListTuple> {
    pub clid: u64,
    pub culist: Box<CopperList<P>>,
    pub keyframe: Option<Box<ParallelKeyFrameScratch>>,
    pub raw_payload_bytes: u64,
    pub handle_bytes: u64,
}

impl<P> IterationTicket<P>
where
    P: CopperListTuple + CuListZeroedInit,
{
    pub fn new(clid: u64, mut culist: Box<CopperList<P>>) -> Self {
        culist.id = clid;
        culist.msgs.init_zeroed();
        Self {
            clid,
            culist,
            keyframe: None,
            raw_payload_bytes: 0,
            handle_bytes: 0,
        }
    }
}

#[cfg(all(feature = "std", feature = "parallel-rt"))]
mod imp {
    use super::{CachePadded, CausalityCheckpoint, ParallelRtMetadata, ParallelRtSettings};
    use alloc::boxed::Box;
    use alloc::vec::Vec;
    use cu29_traits::CuResult;

    /// Feature-enabled parallel runtime state.
    ///
    /// The runtime is not executing CopperLists concurrently yet, but this
    /// struct already reserves the hot scheduler state in the shape the future
    /// worker pipeline expects.
    pub struct ParallelRt<const NBCL: usize> {
        /// Static process-stage layout emitted by the proc macro.
        metadata: &'static ParallelRtMetadata,
        /// Scheduler tuning knobs used to shape worker handoff behavior.
        settings: ParallelRtSettings,
        /// One authorization cursor per generated process stage.
        process_checkpoints: Box<[CachePadded<CausalityCheckpoint>]>,
        /// Ordered commit cursor for monitor/keyframe/log handoff.
        commit_checkpoint: CachePadded<CausalityCheckpoint>,
        /// Maximum number of CopperLists intended to be in flight at once.
        in_flight_limit: usize,
    }

    impl<const NBCL: usize> ParallelRt<NBCL> {
        pub fn new(metadata: &'static ParallelRtMetadata) -> CuResult<Self> {
            Self::new_with_settings(metadata, ParallelRtSettings::default())
        }

        pub fn new_with_settings(
            metadata: &'static ParallelRtMetadata,
            settings: ParallelRtSettings,
        ) -> CuResult<Self> {
            let mut process_checkpoints = Vec::with_capacity(metadata.process_stage_count());
            for _ in 0..metadata.process_stage_count() {
                process_checkpoints.push(CachePadded::new(CausalityCheckpoint::new(0)));
            }

            Ok(Self {
                metadata,
                settings,
                process_checkpoints: process_checkpoints.into_boxed_slice(),
                commit_checkpoint: CachePadded::new(CausalityCheckpoint::new(0)),
                in_flight_limit: NBCL,
            })
        }

        #[inline]
        pub const fn enabled(&self) -> bool {
            true
        }

        #[inline]
        pub const fn metadata(&self) -> &'static ParallelRtMetadata {
            self.metadata
        }

        #[inline]
        pub const fn settings(&self) -> ParallelRtSettings {
            self.settings
        }

        #[inline]
        pub fn process_checkpoints(&self) -> &[CachePadded<CausalityCheckpoint>] {
            &self.process_checkpoints
        }

        #[inline]
        pub const fn commit_checkpoint(&self) -> &CachePadded<CausalityCheckpoint> {
            &self.commit_checkpoint
        }

        #[inline]
        pub const fn in_flight_limit(&self) -> usize {
            self.in_flight_limit
        }
    }
}

#[cfg(not(all(feature = "std", feature = "parallel-rt")))]
mod imp {
    use super::{CachePadded, CausalityCheckpoint, ParallelRtMetadata, ParallelRtSettings};
    use cu29_traits::CuResult;

    /// Feature-disabled placeholder.
    ///
    /// Keeping the type available lets the rest of the runtime compose against a
    /// single API while the actual executor remains behind the `parallel-rt`
    /// feature.
    pub struct ParallelRt<const NBCL: usize> {
        metadata: &'static ParallelRtMetadata,
        settings: ParallelRtSettings,
        commit_checkpoint: CachePadded<CausalityCheckpoint>,
    }

    impl<const NBCL: usize> ParallelRt<NBCL> {
        pub fn new(metadata: &'static ParallelRtMetadata) -> CuResult<Self> {
            Self::new_with_settings(metadata, ParallelRtSettings::default())
        }

        pub fn new_with_settings(
            metadata: &'static ParallelRtMetadata,
            settings: ParallelRtSettings,
        ) -> CuResult<Self> {
            Ok(Self {
                metadata,
                settings,
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
        pub const fn settings(&self) -> ParallelRtSettings {
            self.settings
        }

        #[inline]
        pub fn process_checkpoints(&self) -> &[CachePadded<CausalityCheckpoint>] {
            &[]
        }

        #[inline]
        pub const fn commit_checkpoint(&self) -> &CachePadded<CausalityCheckpoint> {
            &self.commit_checkpoint
        }

        #[inline]
        pub const fn in_flight_limit(&self) -> usize {
            NBCL
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
        const METADATA: ParallelRtMetadata = ParallelRtMetadata::new(STAGES);
        assert_eq!(METADATA.process_stage_count(), 1);
        assert_eq!(METADATA.stages[0].label, "demo");
    }

    #[cfg(all(feature = "std", feature = "parallel-rt"))]
    #[test]
    fn enabled_parallel_rt_allocates_one_checkpoint_per_stage() {
        const STAGES: &[ParallelRtStageMetadata] = &[
            ParallelRtStageMetadata::new("a", ParallelRtStageKind::Task, 0, ComponentId::new(0)),
            ParallelRtStageMetadata::new("b", ParallelRtStageKind::Task, 1, ComponentId::new(1)),
        ];
        const METADATA: ParallelRtMetadata = ParallelRtMetadata::new(STAGES);

        let rt = ParallelRt::<4>::new(&METADATA).expect("parallel rt should build");
        assert!(rt.enabled());
        assert_eq!(rt.process_checkpoints().len(), 2);
        assert_eq!(rt.in_flight_limit(), 4);
    }

    #[cfg(not(all(feature = "std", feature = "parallel-rt")))]
    #[test]
    fn disabled_parallel_rt_keeps_process_checkpoints_empty() {
        const STAGES: &[ParallelRtStageMetadata] = &[ParallelRtStageMetadata::new(
            "a",
            ParallelRtStageKind::Task,
            0,
            ComponentId::new(0),
        )];
        const METADATA: ParallelRtMetadata = ParallelRtMetadata::new(STAGES);

        let rt = ParallelRt::<4>::new(&METADATA).expect("parallel rt placeholder should build");
        assert!(!rt.enabled());
        assert!(rt.process_checkpoints().is_empty());
    }
}
