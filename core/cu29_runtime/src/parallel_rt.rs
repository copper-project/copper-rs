//! Parallel runtime scheduler state for concurrent CopperList execution.
//!
//! The proc macro emits one ordered process-stage entry per generated runtime
//! plan node. The feature-enabled runtime executes those stages as a FIFO
//! pipeline: each stage worker drains CopperLists in ascending `clid` order and
//! forwards them to the next stage. Determinism therefore comes from queue
//! order, while commit/log handoff is still protected by an explicit ordered
//! cursor.

use crate::config::NodeId;
use crate::copperlist::{CopperList, CuListZeroedInit};
use crate::monitoring::ComponentId;
use alloc::boxed::Box;
use alloc::vec::Vec;
use core::fmt::{Debug, Formatter, Result as FmtResult};
use core::ops::{Deref, DerefMut};
use core::sync::atomic::{AtomicU64, Ordering};
use cu29_clock::CuTime;
use cu29_traits::{CopperListTuple, CuResult};

/// Control-flow result returned by one generated process stage.
///
/// `AbortCopperList` preserves the current runtime semantics for monitor
/// decisions that abort the current CopperList without shutting the runtime
/// down. The outer driver remains responsible for ordered cleanup and log
/// handoff.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ProcessStepOutcome {
    Continue,
    AbortCopperList,
}

/// Result type used by generated process-step functions.
pub type ProcessStepResult = CuResult<ProcessStepOutcome>;

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
/// process path. The stage-affine executor spawns one FIFO worker lane per
/// entry and hands ownership of each in-flight CopperList from one lane to the
/// next.
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

/// Per-CopperList scratch state carried while a list is in flight.
///
/// The current parallel executor still serializes keyframe capture at commit
/// time, but the ownership model is explicit so future work can move keyframe
/// accumulation fully into the in-flight ticket.
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
            Ok(Self {
                metadata,
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
    fn enabled_parallel_rt_tracks_metadata_and_limit() {
        const STAGES: &[ParallelRtStageMetadata] = &[
            ParallelRtStageMetadata::new("a", ParallelRtStageKind::Task, 0, ComponentId::new(0)),
            ParallelRtStageMetadata::new("b", ParallelRtStageKind::Task, 1, ComponentId::new(1)),
        ];
        const METADATA: ParallelRtMetadata = ParallelRtMetadata::new(STAGES);

        let rt = ParallelRt::<4>::new(&METADATA).expect("parallel rt should build");
        assert!(rt.enabled());
        assert_eq!(rt.metadata().process_stage_count(), 2);
        assert_eq!(rt.in_flight_limit(), 4);
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
        const METADATA: ParallelRtMetadata = ParallelRtMetadata::new(STAGES);

        let rt = ParallelRt::<4>::new(&METADATA).expect("parallel rt placeholder should build");
        assert!(!rt.enabled());
        assert_eq!(rt.metadata().process_stage_count(), 1);
    }
}
