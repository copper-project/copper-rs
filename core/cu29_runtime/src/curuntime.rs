//! CuRuntime is the heart of what copper is running on the robot.
//! It is exposed to the user via the `copper_runtime` macro injecting it as a field in their application struct.
//!

use crate::app::Subsystem;
use crate::config::{ComponentConfig, DEFAULT_KEYFRAME_INTERVAL, Node, TaskKind};
use crate::config::{
    CuConfig, CuGraph, MAX_RATE_TARGET_HZ, NodeId, RuntimeConfig, resolve_task_kind_for_id,
};
use crate::copperlist::{CopperList, CopperListState, CuListZeroedInit, CuListsManager};
use crate::cutask::{BincodeAdapter, Freezable};
#[cfg(feature = "std")]
use crate::monitoring::ExecutionProbeHandle;
#[cfg(feature = "std")]
use crate::monitoring::MonitorExecutionProbe;
use crate::monitoring::{
    ComponentId, CopperListInfo, CuMonitor, CuMonitoringMetadata, CuMonitoringRuntime,
    ExecutionMarker, MonitorComponentMetadata, RuntimeExecutionProbe, build_monitor_topology,
    take_last_completed_handle_bytes,
};
#[cfg(all(feature = "std", feature = "parallel-rt"))]
use crate::parallel_rt::{ParallelRt, ParallelRtMetadata};
use crate::planner::{CuPlanner, Linearity, check_order, plan_from_order};
use crate::resource::ResourceManager;
#[cfg(feature = "std")]
use alloc::sync::Arc;
use compact_str::CompactString;
use cu29_clock::{ClockProvider, CuDuration, CuTime, RobotClock};
use cu29_traits::CuResult;
use cu29_traits::WriteStream;
use cu29_traits::{CopperListTuple, CuError};
#[cfg(feature = "std")]
use rayon::ThreadPool;

#[cfg(target_os = "none")]
#[allow(unused_imports)]
use cu29_log::{ANONYMOUS, CuLogEntry, CuLogLevel};
#[cfg(target_os = "none")]
#[allow(unused_imports)]
use cu29_log_derive::info;
#[cfg(target_os = "none")]
#[allow(unused_imports)]
use cu29_log_runtime::log;
#[cfg(all(target_os = "none", debug_assertions))]
#[allow(unused_imports)]
use cu29_log_runtime::log_debug_mode;
#[cfg(target_os = "none")]
#[allow(unused_imports)]
use cu29_value::to_value;

#[cfg(all(feature = "std", any(feature = "async-cl-io", feature = "parallel-rt")))]
use alloc::alloc::{alloc_zeroed, handle_alloc_error};
use alloc::boxed::Box;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::vec::Vec;
use bincode::de::read::Reader;
use bincode::de::{Decoder, DecoderImpl};
use bincode::enc::EncoderImpl;
use bincode::enc::write::{SizeWriter, Writer};
use bincode::error::{DecodeError, EncodeError};
use bincode::{Decode, Encode};
#[cfg(all(feature = "std", any(feature = "async-cl-io", feature = "parallel-rt")))]
use core::alloc::Layout;
use core::fmt::Result as FmtResult;
use core::fmt::{Debug, Formatter};
use core::marker::PhantomData;

#[cfg(all(feature = "std", feature = "async-cl-io"))]
use rtrb::{Consumer, PopError, Producer, PushError, RingBuffer};
#[cfg(all(feature = "std", feature = "async-cl-io"))]
use std::sync::atomic::{AtomicBool, Ordering};
#[cfg(all(feature = "std", feature = "async-cl-io"))]
use std::thread::{JoinHandle, Thread};

#[cfg(feature = "std")]
#[doc(hidden)]
pub type TasksInstantiator<CT> = for<'c> fn(
    Vec<Option<&'c ComponentConfig>>,
    &mut ResourceManager,
    &[Option<Arc<ThreadPool>>],
) -> CuResult<CT>;
#[cfg(not(feature = "std"))]
#[doc(hidden)]
pub type TasksInstantiator<CT> =
    for<'c> fn(Vec<Option<&'c ComponentConfig>>, &mut ResourceManager) -> CuResult<CT>;
#[doc(hidden)]
pub type BridgesInstantiator<CB> = fn(&CuConfig, &mut ResourceManager) -> CuResult<CB>;
/// Instantiates the rayon thread pools described by `runtime.thread_pools`.
///
/// Returned vector is indexed positionally to `runtime.thread_pools`; reserved
/// pool ids (such as [`crate::config::RT_POOL`]) leave a `None` slot since they
/// are applied directly to runtime-owned worker threads rather than borrowed as
/// a rayon pool.
#[cfg(feature = "std")]
#[doc(hidden)]
pub type ThreadPoolsInstantiator = fn(&CuConfig) -> CuResult<Vec<Option<Arc<ThreadPool>>>>;
#[doc(hidden)]
pub type MonitorInstantiator<M> = fn(&CuConfig, CuMonitoringMetadata, CuMonitoringRuntime) -> M;

#[doc(hidden)]
pub struct CuRuntimeParts<CT, CB, P: CopperListTuple, M: CuMonitor, const NBCL: usize, TI, BI, MI> {
    pub tasks_instanciator: TI,
    pub monitored_components: &'static [MonitorComponentMetadata],
    pub culist_component_mapping: &'static [ComponentId],
    #[cfg(all(feature = "std", feature = "parallel-rt"))]
    pub parallel_rt_metadata: &'static ParallelRtMetadata,
    pub monitor_instanciator: MI,
    pub bridges_instanciator: BI,
    _payload: PhantomData<(CT, CB, P, M, [(); NBCL])>,
}

impl<CT, CB, P: CopperListTuple, M: CuMonitor, const NBCL: usize, TI, BI, MI>
    CuRuntimeParts<CT, CB, P, M, NBCL, TI, BI, MI>
{
    pub const fn new(
        tasks_instanciator: TI,
        monitored_components: &'static [MonitorComponentMetadata],
        culist_component_mapping: &'static [ComponentId],
        #[cfg(all(feature = "std", feature = "parallel-rt"))]
        parallel_rt_metadata: &'static ParallelRtMetadata,
        monitor_instanciator: MI,
        bridges_instanciator: BI,
    ) -> Self {
        Self {
            tasks_instanciator,
            monitored_components,
            culist_component_mapping,
            #[cfg(all(feature = "std", feature = "parallel-rt"))]
            parallel_rt_metadata,
            monitor_instanciator,
            bridges_instanciator,
            _payload: PhantomData,
        }
    }
}

#[doc(hidden)]
pub struct CuRuntimeBuilder<
    'cfg,
    CT,
    CB,
    P: CopperListTuple,
    M: CuMonitor,
    const NBCL: usize,
    TI,
    BI,
    MI,
    CLS,
    KFS,
> {
    clock: RobotClock,
    config: &'cfg CuConfig,
    mission: &'cfg str,
    subsystem: Subsystem,
    instance_id: u32,
    resources: Option<ResourceManager>,
    #[cfg(feature = "std")]
    thread_pools: Option<Vec<Option<Arc<ThreadPool>>>>,
    parts: CuRuntimeParts<CT, CB, P, M, NBCL, TI, BI, MI>,
    copperlist_sink: CLS,
    keyframe_sink: KFS,
    output_requirements: OutputRequirements,
}

impl<'cfg, CT, CB, P: CopperListTuple, M: CuMonitor, const NBCL: usize, TI, BI, MI, CLS, KFS>
    CuRuntimeBuilder<'cfg, CT, CB, P, M, NBCL, TI, BI, MI, CLS, KFS>
{
    pub fn new(
        clock: RobotClock,
        config: &'cfg CuConfig,
        mission: &'cfg str,
        parts: CuRuntimeParts<CT, CB, P, M, NBCL, TI, BI, MI>,
        copperlist_sink: CLS,
        keyframe_sink: KFS,
        output_requirements: OutputRequirements,
    ) -> Self {
        Self {
            clock,
            config,
            mission,
            subsystem: Subsystem::new(None, 0),
            instance_id: 0,
            resources: None,
            #[cfg(feature = "std")]
            thread_pools: None,
            parts,
            copperlist_sink,
            keyframe_sink,
            output_requirements,
        }
    }

    pub fn with_subsystem(mut self, subsystem: Subsystem) -> Self {
        self.subsystem = subsystem;
        self
    }

    pub fn with_instance_id(mut self, instance_id: u32) -> Self {
        self.instance_id = instance_id;
        self
    }

    pub fn with_resources(mut self, resources: ResourceManager) -> Self {
        self.resources = Some(resources);
        self
    }

    pub fn try_with_resources_instantiator(
        mut self,
        resources_instantiator: impl FnOnce(&CuConfig) -> CuResult<ResourceManager>,
    ) -> CuResult<Self> {
        self.resources = Some(resources_instantiator(self.config)?);
        Ok(self)
    }

    /// Provides pre-built thread pools; positions in the slice must match
    /// `runtime.thread_pools` indices. Reserved pool ids (e.g. `"rt"`) belong
    /// to runtime-owned worker threads and stay as `None` slots.
    #[cfg(feature = "std")]
    pub fn with_thread_pools(mut self, pools: Vec<Option<Arc<ThreadPool>>>) -> Self {
        self.thread_pools = Some(pools);
        self
    }

    #[cfg(feature = "std")]
    pub fn try_with_thread_pools_instantiator(
        mut self,
        thread_pools_instantiator: impl FnOnce(&CuConfig) -> CuResult<Vec<Option<Arc<ThreadPool>>>>,
    ) -> CuResult<Self> {
        self.thread_pools = Some(thread_pools_instantiator(self.config)?);
        Ok(self)
    }
}

/// Returns a monotonic instant used for local runtime performance timing.
///
/// When `sysclock-perf` (and `std`) are enabled this uses a process-local
/// `RobotClock::new()` instance for timing. The returned value is a
/// monotonically increasing duration since an unspecified origin (typically
/// process or runtime initialization), not a wall-clock time-of-day. When
/// `sysclock-perf` is disabled it delegates to the provided `RobotClock`.
///
/// This is intentionally separate from `LoopRateLimiter`, which always uses the
/// provided `RobotClock` so `runtime.rate_target_hz` stays tied to robot time.
#[inline]
pub fn perf_now(_clock: &RobotClock) -> CuTime {
    #[cfg(all(feature = "std", feature = "sysclock-perf"))]
    {
        static PERF_CLOCK: std::sync::OnceLock<RobotClock> = std::sync::OnceLock::new();
        return PERF_CLOCK.get_or_init(RobotClock::new).now();
    }

    #[allow(unreachable_code)]
    _clock.now()
}

#[cfg(all(feature = "std", feature = "high-precision-limiter"))]
const HIGH_PRECISION_LIMITER_SPIN_WINDOW_NS: u64 = 200_000;

/// Convert a configured runtime rate target to an integer-nanosecond period.
#[inline]
pub fn rate_target_period(rate_target_hz: u64) -> CuResult<CuDuration> {
    if rate_target_hz == 0 {
        return Err(CuError::from(
            "Runtime rate target cannot be zero. Set runtime.rate_target_hz to at least 1.",
        ));
    }

    if rate_target_hz > MAX_RATE_TARGET_HZ {
        return Err(CuError::from(format!(
            "Runtime rate target ({rate_target_hz} Hz) exceeds the supported maximum of {MAX_RATE_TARGET_HZ} Hz."
        )));
    }

    Ok(CuDuration::from(MAX_RATE_TARGET_HZ / rate_target_hz))
}

/// Runtime loop limiter that preserves phase with absolute deadlines.
///
/// This is intentionally a small runtime helper so generated applications do
/// not have to open-code loop scheduling policy. Deadlines are tracked against
/// the provided `RobotClock`, even when `sysclock-perf` is enabled for
/// process-time measurements.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LoopRateLimiter {
    period: CuDuration,
    next_deadline: CuTime,
}

impl LoopRateLimiter {
    #[inline]
    pub fn from_rate_target_hz(rate_target_hz: u64, clock: &RobotClock) -> CuResult<Self> {
        let period = rate_target_period(rate_target_hz)?;
        Ok(Self {
            period,
            next_deadline: clock.now() + period,
        })
    }

    #[inline]
    pub fn is_ready(&self, clock: &RobotClock) -> bool {
        self.remaining(clock).is_none()
    }

    #[inline]
    pub fn remaining(&self, clock: &RobotClock) -> Option<CuDuration> {
        let now = clock.now();
        if now < self.next_deadline {
            Some(self.next_deadline - now)
        } else {
            None
        }
    }

    #[inline]
    pub fn wait_until_ready(&self, clock: &RobotClock) {
        let deadline = self.next_deadline;
        let Some(remaining) = self.remaining(clock) else {
            return;
        };

        #[cfg(all(feature = "std", feature = "high-precision-limiter"))]
        {
            let spin_window = self.spin_window();
            if remaining > spin_window {
                std::thread::sleep(std::time::Duration::from(remaining - spin_window));
            }
            while clock.now() < deadline {
                core::hint::spin_loop();
            }
        }

        #[cfg(all(feature = "std", not(feature = "high-precision-limiter")))]
        {
            let _ = deadline;
            std::thread::sleep(std::time::Duration::from(remaining));
        }

        #[cfg(not(feature = "std"))]
        {
            let _ = remaining;
            while clock.now() < deadline {
                core::hint::spin_loop();
            }
        }
    }

    #[inline]
    pub fn mark_tick(&mut self, clock: &RobotClock) {
        self.advance_from(clock.now());
    }

    #[inline]
    pub fn limit(&mut self, clock: &RobotClock) {
        self.wait_until_ready(clock);
        self.mark_tick(clock);
    }

    #[inline]
    fn advance_from(&mut self, now: CuTime) {
        let steps = if now < self.next_deadline {
            1
        } else {
            (now - self.next_deadline).as_nanos() / self.period.as_nanos() + 1
        };
        self.next_deadline += steps * self.period;
    }

    #[cfg(all(feature = "std", feature = "high-precision-limiter"))]
    #[inline]
    fn spin_window(&self) -> CuDuration {
        let _ = self.period;
        CuDuration::from(HIGH_PRECISION_LIMITER_SPIN_WINDOW_NS)
    }

    #[cfg(test)]
    #[inline]
    fn next_deadline(&self) -> CuTime {
        self.next_deadline
    }
}

#[cfg(all(feature = "std", feature = "async-cl-io"))]
#[doc(hidden)]
pub trait AsyncCopperListPayload: Send {}

#[cfg(all(feature = "std", feature = "async-cl-io"))]
impl<T: Send> AsyncCopperListPayload for T {}

#[cfg(not(all(feature = "std", feature = "async-cl-io")))]
#[doc(hidden)]
pub trait AsyncCopperListPayload {}

#[cfg(not(all(feature = "std", feature = "async-cl-io")))]
impl<T> AsyncCopperListPayload for T {}

/// Control-flow result returned by one generated process stage.
///
/// `AbortCopperList` preserves the current runtime semantics for monitor
/// decisions that abort the current CopperList without shutting the runtime
/// down. The outer driver remains responsible for ordered cleanup and log
/// handoff.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[doc(hidden)]
pub enum ProcessStepOutcome {
    Continue,
    AbortCopperList,
}

/// Result type used by generated process-step functions.
#[doc(hidden)]
pub type ProcessStepResult = CuResult<ProcessStepOutcome>;

#[cfg(feature = "remote-debug")]
fn encode_completed_copperlist_snapshot<P: CopperListTuple>(
    cl: &CopperList<P>,
) -> CuResult<Vec<u8>> {
    bincode::encode_to_vec(cl, bincode::config::standard())
        .map_err(|e| CuError::new_with_cause("Failed to encode completed CopperList snapshot", e))
}

/// Existing type-erased leaf boundary for a semantic record consumer.
///
/// This deliberately aliases [`WriteStream`] rather than wrapping it. Generated
/// code can statically compose concrete consumers behind this existing boundary
/// without changing its object size or adding another virtual call.
#[doc(hidden)]
pub type SemanticRecordSink<T> = dyn WriteStream<T>;

/// Semantic output boundary for a completed CopperList.
#[doc(hidden)]
pub type CompletedCopperListSink<P> = SemanticRecordSink<CopperList<P>>;

/// Semantic output boundary for a completed keyframe.
#[doc(hidden)]
pub type CompletedKeyFrameSink = SemanticRecordSink<KeyFrame>;

/// Zero-cost placeholder emitted when a semantic record family has no consumer.
#[derive(Clone, Copy, Debug, Default)]
#[doc(hidden)]
pub struct NullWriteStream;

impl<E: Encode> WriteStream<E> for NullWriteStream {
    #[inline]
    fn log(&mut self, _record: &E) -> CuResult<()> {
        Ok(())
    }
}

/// Semantic record families requested by the statically generated downstream graph.
///
/// This is deliberately independent from [`crate::config::LoggingConfig`]. Local
/// unified logging is one possible downstream consumer; generated streaming sinks
/// may request the same records even when local CopperList logging is disabled.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[doc(hidden)]
pub struct OutputRequirements {
    completed_copperlists: bool,
    keyframes: bool,
}

impl OutputRequirements {
    #[inline]
    pub const fn new(completed_copperlists: bool, keyframes: bool) -> Self {
        Self {
            completed_copperlists,
            keyframes,
        }
    }

    #[inline]
    pub const fn completed_copperlists(self) -> bool {
        self.completed_copperlists
    }

    #[inline]
    pub const fn keyframes(self) -> bool {
        self.keyframes
    }

    /// Combines the record needs of independently generated downstream consumers.
    #[inline]
    pub const fn union(self, other: Self) -> Self {
        Self::new(
            self.completed_copperlists || other.completed_copperlists,
            self.keyframes || other.keyframes,
        )
    }
}

/// Manages the lifecycle and completed-list sink on the synchronous path.
#[doc(hidden)]
pub struct SyncCopperListsManager<P: CopperListTuple + Default, const NBCL: usize> {
    inner: CuListsManager<P, NBCL>,
    sink: Option<Box<CompletedCopperListSink<P>>>,
    /// Remote-debug snapshot of the most recently completed CopperList.
    #[cfg(feature = "remote-debug")]
    last_completed_encoded: Option<Vec<u8>>,
    /// Last local-log encoded size reported by the sink.
    pub last_encoded_bytes: u64,
    /// Last handle-backed payload bytes observed while the sink ran.
    pub last_handle_bytes: u64,
}

impl<P: CopperListTuple + Default, const NBCL: usize> SyncCopperListsManager<P, NBCL> {
    pub fn new(sink: Option<Box<CompletedCopperListSink<P>>>) -> CuResult<Self>
    where
        P: CuListZeroedInit,
    {
        Ok(Self {
            inner: CuListsManager::new(),
            sink,
            #[cfg(feature = "remote-debug")]
            last_completed_encoded: None,
            last_encoded_bytes: 0,
            last_handle_bytes: 0,
        })
    }

    pub fn next_cl_id(&self) -> u64 {
        self.inner.next_cl_id()
    }

    pub fn last_cl_id(&self) -> u64 {
        self.inner.last_cl_id()
    }

    pub fn peek(&self) -> Option<&CopperList<P>> {
        self.inner.peek()
    }

    #[cfg(feature = "remote-debug")]
    pub fn last_completed_encoded(&self) -> Option<&[u8]> {
        self.last_completed_encoded.as_deref()
    }

    #[cfg(not(feature = "remote-debug"))]
    pub fn last_completed_encoded(&self) -> Option<&[u8]> {
        None
    }

    #[cfg(feature = "remote-debug")]
    pub fn set_last_completed_encoded(&mut self, snapshot: Option<Vec<u8>>) {
        self.last_completed_encoded = snapshot;
    }

    #[cfg(not(feature = "remote-debug"))]
    pub fn set_last_completed_encoded(&mut self, _snapshot: Option<Vec<u8>>) {}

    pub fn create(&mut self) -> CuResult<&mut CopperList<P>>
    where
        P: CuListZeroedInit,
    {
        self.inner
            .create()
            .ok_or_else(|| CuError::from("Ran out of space for copper lists"))
    }

    pub fn end_of_processing(&mut self, culistid: u64) -> CuResult<()> {
        #[cfg(debug_assertions)]
        self.debug_assert_end_of_processing_target(culistid);

        let mut is_top = true;
        let mut nb_done = 0;
        self.last_encoded_bytes = 0;
        self.last_handle_bytes = 0;
        #[cfg(feature = "remote-debug")]
        let last_completed_encoded = &mut self.last_completed_encoded;
        for cl in self.inner.iter_mut() {
            if cl.id == culistid && cl.get_state() == CopperListState::Processing {
                cl.change_state(CopperListState::DoneProcessing);
                #[cfg(feature = "remote-debug")]
                {
                    *last_completed_encoded = Some(encode_completed_copperlist_snapshot(cl)?);
                }
            }
            if is_top && cl.get_state() == CopperListState::DoneProcessing {
                if let Some(sink) = &mut self.sink {
                    cl.change_state(CopperListState::BeingSerialized);
                    sink.log(cl)?;
                    self.last_encoded_bytes = sink.last_log_bytes().unwrap_or(0) as u64;
                    self.last_handle_bytes = take_last_completed_handle_bytes();
                }
                cl.change_state(CopperListState::Free);
                nb_done += 1;
            } else {
                is_top = false;
            }
        }
        for _ in 0..nb_done {
            let _ = self.inner.pop();
        }
        Ok(())
    }

    pub fn finish_pending(&mut self) -> CuResult<()> {
        Ok(())
    }

    pub fn available_copper_lists(&mut self) -> CuResult<usize> {
        Ok(NBCL - self.inner.len())
    }

    #[inline]
    pub const fn dropped_copperlists_total(&self) -> u64 {
        0
    }

    #[cfg(feature = "std")]
    pub fn end_of_processing_boxed(
        &mut self,
        mut culist: Box<CopperList<P>>,
    ) -> CuResult<OwnedCopperListSubmission<P>> {
        #[cfg(debug_assertions)]
        debug_assert_processing_completion_state(culist.as_ref(), "sync boxed end_of_processing");

        culist.change_state(CopperListState::DoneProcessing);
        self.last_encoded_bytes = 0;
        self.last_handle_bytes = 0;
        if let Some(sink) = &mut self.sink {
            culist.change_state(CopperListState::BeingSerialized);
            sink.log(&culist)?;
            self.last_encoded_bytes = sink.last_log_bytes().unwrap_or(0) as u64;
            self.last_handle_bytes = take_last_completed_handle_bytes();
        }
        culist.change_state(CopperListState::Free);
        Ok(OwnedCopperListSubmission::Recycled(culist))
    }

    #[cfg(feature = "std")]
    pub fn try_reclaim_boxed(&mut self) -> CuResult<Option<Box<CopperList<P>>>> {
        Ok(None)
    }

    #[cfg(feature = "std")]
    pub fn wait_reclaim_boxed(&mut self) -> CuResult<Box<CopperList<P>>> {
        Err(CuError::from(
            "Synchronous CopperList I/O cannot block waiting for boxed completions",
        ))
    }

    #[cfg(feature = "std")]
    pub fn finish_pending_boxed(&mut self) -> CuResult<Vec<Box<CopperList<P>>>> {
        Ok(Vec::new())
    }

    #[cfg(debug_assertions)]
    fn debug_assert_end_of_processing_target(&self, culistid: u64) {
        let mut matches = 0usize;
        let mut state = None;
        for cl in self.inner.iter() {
            if cl.id == culistid {
                matches += 1;
                state = Some(cl.get_state());
            }
        }

        assert_eq!(
            matches, 1,
            "sync end_of_processing expected exactly one active CopperList #{culistid}, found {matches}"
        );
        assert_eq!(
            state,
            Some(CopperListState::Processing),
            "sync end_of_processing expected CopperList #{culistid} to be Processing, found {:?}",
            state
        );
    }
}

/// Result of handing an owned boxed CopperList to the runtime-side CL I/O path.
#[cfg(feature = "std")]
#[doc(hidden)]
pub enum OwnedCopperListSubmission<P: CopperListTuple> {
    /// The CL has been fully handled and can be recycled immediately by the caller.
    Recycled(Box<CopperList<P>>),
    /// The CL was queued asynchronously and will be returned by a later reclaim call.
    Pending,
}

#[cfg(all(feature = "std", feature = "async-cl-io"))]
struct AsyncCopperListCompletion<P: CopperListTuple> {
    culist: Box<CopperList<P>>,
    sink_result: CuResult<(u64, u64)>,
    #[cfg(feature = "remote-debug")]
    completed_snapshot: CuResult<Vec<u8>>,
}

#[cfg(all(feature = "std", feature = "async-cl-io"))]
struct AsyncOutputWorkerRunningGuard(Arc<AtomicBool>);

#[cfg(all(feature = "std", feature = "async-cl-io"))]
impl Drop for AsyncOutputWorkerRunningGuard {
    fn drop(&mut self) {
        self.0.store(false, Ordering::Release);
    }
}

#[cfg(all(feature = "std", any(feature = "async-cl-io", feature = "parallel-rt")))]
fn allocate_zeroed_copperlist<P>() -> Box<CopperList<P>>
where
    P: CopperListTuple + CuListZeroedInit,
{
    // SAFETY: We allocate zeroed memory and immediately initialize required fields.
    let mut culist = unsafe {
        let layout = Layout::new::<CopperList<P>>();
        let ptr = alloc_zeroed(layout) as *mut CopperList<P>;
        if ptr.is_null() {
            handle_alloc_error(layout);
        }
        Box::from_raw(ptr)
    };
    culist.msgs.init_zeroed();
    culist
}

#[cfg(all(feature = "std", feature = "parallel-rt"))]
pub fn allocate_boxed_copperlists<P, const NBCL: usize>() -> Vec<Box<CopperList<P>>>
where
    P: CopperListTuple + CuListZeroedInit,
{
    let mut free_pool = Vec::with_capacity(NBCL);
    for _ in 0..NBCL {
        free_pool.push(allocate_zeroed_copperlist::<P>());
    }
    free_pool
}

/// Manages the lifecycle and completed-list sink on the asynchronous path.
#[cfg(all(feature = "std", feature = "async-cl-io"))]
#[doc(hidden)]
pub struct AsyncCopperListsManager<P: CopperListTuple + Default, const NBCL: usize> {
    free_pool: Vec<Box<CopperList<P>>>,
    current: Option<Box<CopperList<P>>>,
    #[cfg(feature = "remote-debug")]
    last_completed_encoded: Option<Vec<u8>>,
    pending_count: usize,
    next_cl_id: u64,
    pending_producer: Option<Producer<Box<CopperList<P>>>>,
    completion_consumer: Option<Consumer<AsyncCopperListCompletion<P>>>,
    worker_handle: Option<JoinHandle<()>>,
    worker_thread: Option<Thread>,
    worker_shutdown: Option<Arc<AtomicBool>>,
    worker_running: Option<Arc<AtomicBool>>,
    dropped_copperlists_total: u64,
    /// Last local-log encoded size reported by the sink.
    pub last_encoded_bytes: u64,
    /// Last handle-backed payload bytes observed while the sink ran.
    pub last_handle_bytes: u64,
}

#[cfg(all(feature = "std", feature = "async-cl-io"))]
impl<P: CopperListTuple + Default, const NBCL: usize> AsyncCopperListsManager<P, NBCL> {
    pub fn new(sink: Option<Box<CompletedCopperListSink<P>>>) -> CuResult<Self>
    where
        P: CuListZeroedInit + AsyncCopperListPayload + 'static,
    {
        let mut free_pool = Vec::with_capacity(NBCL);
        for _ in 0..NBCL {
            free_pool.push(allocate_zeroed_copperlist::<P>());
        }

        if sink.is_some() && NBCL < 2 {
            return Err(CuError::from(
                "async CopperList output requires at least two CopperList slots",
            ));
        }

        let (
            pending_producer,
            completion_consumer,
            worker_handle,
            worker_thread,
            worker_shutdown,
            worker_running,
        ) = if let Some(mut sink) = sink {
            let handoff_capacity = NBCL - 1;
            let (pending_producer, mut pending_consumer) =
                RingBuffer::<Box<CopperList<P>>>::new(handoff_capacity);
            let (mut completion_producer, completion_consumer) =
                RingBuffer::<AsyncCopperListCompletion<P>>::new(handoff_capacity);
            let worker_shutdown = Arc::new(AtomicBool::new(false));
            let worker_running = Arc::new(AtomicBool::new(true));
            let shutdown = worker_shutdown.clone();
            let running = worker_running.clone();
            let worker_handle = std::thread::Builder::new()
                .name("cu-async-cl-io".to_string())
                .spawn(move || {
                    let _running_guard = AsyncOutputWorkerRunningGuard(running);
                    loop {
                        let mut culist = match pending_consumer.pop() {
                            Ok(culist) => culist,
                            Err(PopError::Empty) => {
                                if shutdown.load(Ordering::Acquire) {
                                    break;
                                }
                                std::thread::park();
                                continue;
                            }
                        };
                        #[cfg(feature = "remote-debug")]
                        let completed_snapshot = {
                            // Preserve the pre-handoff snapshot contract while
                            // keeping its allocation and encoding off the RT path.
                            culist.change_state(CopperListState::DoneProcessing);
                            encode_completed_copperlist_snapshot(&culist)
                        };
                        culist.change_state(CopperListState::BeingSerialized);
                        let sink_result = sink.log(&culist).map(|_| {
                            (
                                sink.last_log_bytes().unwrap_or(0) as u64,
                                take_last_completed_handle_bytes(),
                            )
                        });
                        let should_stop = sink_result.is_err();
                        #[cfg(feature = "remote-debug")]
                        let should_stop = should_stop || completed_snapshot.is_err();
                        let mut completion = AsyncCopperListCompletion {
                            culist,
                            sink_result,
                            #[cfg(feature = "remote-debug")]
                            completed_snapshot,
                        };
                        loop {
                            match completion_producer.push(completion) {
                                Ok(()) => break,
                                Err(PushError::Full(returned)) => {
                                    completion = returned;
                                    std::thread::yield_now();
                                }
                            }
                        }
                        if should_stop {
                            break;
                        }
                    }
                })
                .map_err(|e| {
                    CuError::from("Failed to spawn async CopperList serializer thread")
                        .add_cause(e.to_string().as_str())
                })?;
            let worker_thread = worker_handle.thread().clone();
            (
                Some(pending_producer),
                Some(completion_consumer),
                Some(worker_handle),
                Some(worker_thread),
                Some(worker_shutdown),
                Some(worker_running),
            )
        } else {
            (None, None, None, None, None, None)
        };

        Ok(Self {
            free_pool,
            current: None,
            #[cfg(feature = "remote-debug")]
            last_completed_encoded: None,
            pending_count: 0,
            next_cl_id: 0,
            pending_producer,
            completion_consumer,
            worker_handle,
            worker_thread,
            worker_shutdown,
            worker_running,
            dropped_copperlists_total: 0,
            last_encoded_bytes: 0,
            last_handle_bytes: 0,
        })
    }

    pub fn next_cl_id(&self) -> u64 {
        self.next_cl_id
    }

    pub fn last_cl_id(&self) -> u64 {
        self.next_cl_id.saturating_sub(1)
    }

    pub fn peek(&self) -> Option<&CopperList<P>> {
        self.current.as_deref()
    }

    #[cfg(feature = "remote-debug")]
    pub fn last_completed_encoded(&self) -> Option<&[u8]> {
        self.last_completed_encoded.as_deref()
    }

    #[cfg(not(feature = "remote-debug"))]
    pub fn last_completed_encoded(&self) -> Option<&[u8]> {
        None
    }

    #[cfg(feature = "remote-debug")]
    pub fn set_last_completed_encoded(&mut self, snapshot: Option<Vec<u8>>) {
        self.last_completed_encoded = snapshot;
    }

    #[cfg(not(feature = "remote-debug"))]
    pub fn set_last_completed_encoded(&mut self, _snapshot: Option<Vec<u8>>) {}

    pub fn create(&mut self) -> CuResult<&mut CopperList<P>>
    where
        P: CuListZeroedInit,
    {
        if self.current.is_some() {
            return Err(CuError::from(
                "Attempted to create a CopperList while another one is still active",
            ));
        }

        self.reclaim_completed()?;

        let culist = self.free_pool.pop().ok_or_else(|| {
            CuError::from("CopperList output handoff exhausted the slot reserved for execution")
        })?;
        self.current = Some(culist);

        let current = self
            .current
            .as_mut()
            .expect("current CopperList is missing");
        current.reset_for_runtime_use(self.next_cl_id);
        self.next_cl_id += 1;
        Ok(current.as_mut())
    }

    pub fn end_of_processing(&mut self, culistid: u64) -> CuResult<()> {
        self.reclaim_completed()?;

        let mut culist = self.current.take().ok_or_else(|| {
            CuError::from("Attempted to finish processing without an active CopperList")
        })?;

        if culist.id != culistid {
            return Err(CuError::from(format!(
                "Attempted to finish CopperList #{culistid} while CopperList #{} is active",
                culist.id
            )));
        }
        #[cfg(debug_assertions)]
        debug_assert_processing_completion_state(culist.as_ref(), "async end_of_processing");

        culist.change_state(CopperListState::DoneProcessing);
        self.last_encoded_bytes = 0;
        self.last_handle_bytes = 0;

        match self.try_submit(culist)? {
            OwnedCopperListSubmission::Recycled(culist) => self.free_pool.push(culist),
            OwnedCopperListSubmission::Pending => {}
        }

        Ok(())
    }

    pub fn finish_pending(&mut self) -> CuResult<()> {
        if self.current.is_some() {
            return Err(CuError::from(
                "Cannot flush CopperList I/O while a CopperList is still active",
            ));
        }

        while self.pending_count > 0 {
            self.wait_for_completion()?;
        }
        Ok(())
    }

    pub fn available_copper_lists(&mut self) -> CuResult<usize> {
        self.reclaim_completed()?;
        Ok(self.free_pool.len())
    }

    #[inline]
    pub const fn dropped_copperlists_total(&self) -> u64 {
        self.dropped_copperlists_total
    }

    pub fn end_of_processing_boxed(
        &mut self,
        mut culist: Box<CopperList<P>>,
    ) -> CuResult<OwnedCopperListSubmission<P>> {
        #[cfg(debug_assertions)]
        debug_assert_processing_completion_state(culist.as_ref(), "async boxed end_of_processing");
        culist.change_state(CopperListState::DoneProcessing);
        self.last_encoded_bytes = 0;
        self.last_handle_bytes = 0;

        self.try_submit(culist)
    }

    fn try_submit(
        &mut self,
        mut culist: Box<CopperList<P>>,
    ) -> CuResult<OwnedCopperListSubmission<P>> {
        let Some(pending_producer) = self.pending_producer.as_mut() else {
            culist.change_state(CopperListState::Free);
            return Ok(OwnedCopperListSubmission::Recycled(culist));
        };

        // Keep one of the preallocated CopperLists available to execute the
        // next iteration. Queue capacity alone cannot enforce this because a
        // record may be in the worker or waiting on the completion channel.
        if self.pending_count >= NBCL - 1 {
            return Ok(self.drop_copperlist(culist));
        }

        culist.change_state(CopperListState::QueuedForSerialization);
        match pending_producer.push(culist) {
            Ok(()) => {
                self.pending_count += 1;
                if let Some(worker_thread) = self.worker_thread.as_ref() {
                    worker_thread.unpark();
                }
                Ok(OwnedCopperListSubmission::Pending)
            }
            Err(PushError::Full(culist)) => Ok(self.drop_copperlist(culist)),
        }
    }

    fn drop_copperlist(&mut self, mut culist: Box<CopperList<P>>) -> OwnedCopperListSubmission<P> {
        self.dropped_copperlists_total = self.dropped_copperlists_total.saturating_add(1);
        culist.change_state(CopperListState::Free);
        OwnedCopperListSubmission::Recycled(culist)
    }

    pub fn try_reclaim_boxed(&mut self) -> CuResult<Option<Box<CopperList<P>>>> {
        let pop_result = {
            let Some(completion_consumer) = self.completion_consumer.as_mut() else {
                return Ok(None);
            };
            completion_consumer.pop()
        };
        match pop_result {
            Ok(completion) => self.handle_completion(completion).map(Some),
            Err(PopError::Empty) => {
                if self.pending_count > 0
                    && self
                        .worker_running
                        .as_ref()
                        .is_some_and(|running| !running.load(Ordering::Acquire))
                {
                    Err(CuError::from(
                        "Async CopperList output worker stopped unexpectedly",
                    ))
                } else {
                    Ok(None)
                }
            }
        }
    }

    pub fn wait_reclaim_boxed(&mut self) -> CuResult<Box<CopperList<P>>> {
        if self.completion_consumer.is_none() {
            return Err(CuError::from(
                "No async CopperList output worker is active to return a free slot",
            ));
        }
        loop {
            if let Some(culist) = self.try_reclaim_boxed()? {
                return Ok(culist);
            }
            std::thread::yield_now();
        }
    }

    pub fn finish_pending_boxed(&mut self) -> CuResult<Vec<Box<CopperList<P>>>> {
        let mut reclaimed = Vec::with_capacity(self.pending_count);
        if self.current.is_some() {
            return Err(CuError::from(
                "Cannot flush CopperList I/O while a CopperList is still active",
            ));
        }
        while self.pending_count > 0 {
            reclaimed.push(self.wait_reclaim_boxed()?);
        }
        Ok(reclaimed)
    }

    fn reclaim_completed(&mut self) -> CuResult<()> {
        loop {
            let Some(culist) = self.try_reclaim_boxed()? else {
                break;
            };
            self.free_pool.push(culist);
        }
        Ok(())
    }

    fn wait_for_completion(&mut self) -> CuResult<()> {
        let culist = self.wait_reclaim_boxed()?;
        self.free_pool.push(culist);
        Ok(())
    }

    fn handle_completion(
        &mut self,
        mut completion: AsyncCopperListCompletion<P>,
    ) -> CuResult<Box<CopperList<P>>> {
        self.pending_count = self.pending_count.saturating_sub(1);
        if let Ok((encoded_bytes, handle_bytes)) = completion.sink_result.as_ref() {
            self.last_encoded_bytes = *encoded_bytes;
            self.last_handle_bytes = *handle_bytes;
        }
        completion.culist.change_state(CopperListState::Free);
        completion.sink_result?;
        #[cfg(feature = "remote-debug")]
        {
            self.last_completed_encoded = Some(completion.completed_snapshot?);
        }
        Ok(completion.culist)
    }

    fn shutdown_worker(&mut self) -> CuResult<()> {
        self.finish_pending()?;
        if let Some(shutdown) = self.worker_shutdown.as_ref() {
            shutdown.store(true, Ordering::Release);
        }
        if let Some(worker_thread) = self.worker_thread.as_ref() {
            worker_thread.unpark();
        }
        if let Some(worker_handle) = self.worker_handle.take() {
            worker_handle.join().map_err(|_| {
                CuError::from("Async CopperList output worker panicked while joining")
            })?;
        }
        self.pending_producer.take();
        self.worker_thread.take();
        self.worker_shutdown.take();
        self.worker_running.take();
        Ok(())
    }
}

#[cfg(all(feature = "std", feature = "async-cl-io"))]
impl<P: CopperListTuple + Default, const NBCL: usize> Drop for AsyncCopperListsManager<P, NBCL> {
    fn drop(&mut self) {
        let _ = self.shutdown_worker();
    }
}

#[cfg(all(feature = "std", debug_assertions))]
fn debug_assert_processing_completion_state<P: CopperListTuple>(
    culist: &CopperList<P>,
    context: &str,
) {
    assert_eq!(
        culist.get_state(),
        CopperListState::Processing,
        "{context} expected CopperList #{} to be Processing, found {}",
        culist.id,
        culist.get_state()
    );
}

#[cfg(all(feature = "std", feature = "async-cl-io"))]
#[doc(hidden)]
pub type CopperListsManager<P, const NBCL: usize> = AsyncCopperListsManager<P, NBCL>;

#[cfg(not(all(feature = "std", feature = "async-cl-io")))]
#[doc(hidden)]
pub type CopperListsManager<P, const NBCL: usize> = SyncCopperListsManager<P, NBCL>;

#[cfg(all(feature = "std", feature = "async-cl-io"))]
struct AsyncKeyFrameCompletion {
    keyframe: Box<KeyFrame>,
    sink_result: CuResult<u64>,
}

/// Manages bounded task-state keyframe capture and output.
pub struct KeyFramesManager {
    /// Active capture buffer. It is absent when no downstream consumer needs keyframes.
    inner: Option<KeyFrame>,

    /// Optional override for the timestamp to stamp the next keyframe (used by deterministic replay).
    forced_timestamp: Option<CuTime>,

    /// If set, reuse this keyframe verbatim (e.g., during replay) instead of re-freezing state.
    locked: bool,

    /// Consumer of completed task-state keyframes on the synchronous path.
    #[cfg(not(all(feature = "std", feature = "async-cl-io")))]
    sink: Option<Box<CompletedKeyFrameSink>>,

    /// Spare capture buffers exchanged with completed keyframes at handoff.
    /// Boxing is intentional: ownership moves through the SPSC ring without a
    /// hot-path allocation or copying the potentially large payload buffer.
    #[cfg(all(feature = "std", feature = "async-cl-io"))]
    #[allow(clippy::vec_box)]
    spares: Vec<Box<KeyFrame>>,
    #[cfg(all(feature = "std", feature = "async-cl-io"))]
    pending_count: usize,
    #[cfg(all(feature = "std", feature = "async-cl-io"))]
    pending_producer: Option<Producer<Box<KeyFrame>>>,
    #[cfg(all(feature = "std", feature = "async-cl-io"))]
    completion_consumer: Option<Consumer<AsyncKeyFrameCompletion>>,
    #[cfg(all(feature = "std", feature = "async-cl-io"))]
    worker_handle: Option<JoinHandle<()>>,
    #[cfg(all(feature = "std", feature = "async-cl-io"))]
    worker_thread: Option<Thread>,
    #[cfg(all(feature = "std", feature = "async-cl-io"))]
    worker_shutdown: Option<Arc<AtomicBool>>,
    #[cfg(all(feature = "std", feature = "async-cl-io"))]
    worker_running: Option<Arc<AtomicBool>>,
    #[cfg(all(feature = "std", feature = "async-cl-io"))]
    capture_this_copperlist: Option<u64>,
    #[cfg(all(feature = "std", feature = "async-cl-io"))]
    dropped_keyframes_total: u64,

    /// Capture a keyframe at this CopperList interval.
    keyframe_interval: u32,

    /// Bytes written by the last completed keyframe output.
    pub last_encoded_bytes: u64,

    /// Cold-path sizing accumulator used to reserve the capture buffers before execution.
    capture_size_hint: usize,
}

const MIN_KEYFRAME_CAPTURE_CAPACITY: usize = 4 * 1024;
#[cfg(all(feature = "std", feature = "async-cl-io"))]
const ASYNC_KEYFRAME_HANDOFF_CAPACITY: usize = 2;

/// A `Vec` writer that is forbidden from growing its backing allocation.
struct PreallocatedVecWriter<'a>(&'a mut Vec<u8>);

impl Writer for PreallocatedVecWriter<'_> {
    fn write(&mut self, bytes: &[u8]) -> Result<(), EncodeError> {
        if bytes.len() > self.0.capacity().saturating_sub(self.0.len()) {
            return Err(EncodeError::UnexpectedEnd);
        }
        // The capacity check above makes this append allocation-free.
        self.0.extend_from_slice(bytes);
        Ok(())
    }
}

impl KeyFramesManager {
    #[doc(hidden)]
    pub fn new(sink: Option<Box<CompletedKeyFrameSink>>, keyframe_interval: u32) -> CuResult<Self> {
        if sink.is_some() && keyframe_interval == 0 {
            return Err(CuError::from(
                "Keyframe interval cannot be zero when a downstream consumer requires keyframes",
            ));
        }

        #[cfg(all(feature = "std", feature = "async-cl-io"))]
        {
            let enabled = sink.is_some();
            let (
                pending_producer,
                completion_consumer,
                worker_handle,
                worker_thread,
                worker_shutdown,
                worker_running,
            ) = if let Some(mut sink) = sink {
                let (pending_producer, mut pending_consumer) =
                    RingBuffer::<Box<KeyFrame>>::new(ASYNC_KEYFRAME_HANDOFF_CAPACITY);
                let (mut completion_producer, completion_consumer) =
                    RingBuffer::<AsyncKeyFrameCompletion>::new(ASYNC_KEYFRAME_HANDOFF_CAPACITY);
                let worker_shutdown = Arc::new(AtomicBool::new(false));
                let worker_running = Arc::new(AtomicBool::new(true));
                let shutdown = worker_shutdown.clone();
                let running = worker_running.clone();
                let worker_handle = std::thread::Builder::new()
                    .name("cu-async-kf-io".to_string())
                    .spawn(move || {
                        let _running_guard = AsyncOutputWorkerRunningGuard(running);
                        loop {
                            let keyframe = match pending_consumer.pop() {
                                Ok(keyframe) => keyframe,
                                Err(PopError::Empty) => {
                                    if shutdown.load(Ordering::Acquire) {
                                        break;
                                    }
                                    std::thread::park();
                                    continue;
                                }
                            };
                            let sink_result = sink
                                .log(keyframe.as_ref())
                                .map(|_| sink.last_log_bytes().unwrap_or(0) as u64);
                            let should_stop = sink_result.is_err();
                            let mut completion = AsyncKeyFrameCompletion {
                                keyframe,
                                sink_result,
                            };
                            loop {
                                match completion_producer.push(completion) {
                                    Ok(()) => break,
                                    Err(PushError::Full(returned)) => {
                                        completion = returned;
                                        std::thread::yield_now();
                                    }
                                }
                            }
                            if should_stop {
                                break;
                            }
                        }
                    })
                    .map_err(|error| {
                        CuError::from("Failed to spawn async keyframe output thread")
                            .add_cause(error.to_string().as_str())
                    })?;
                let worker_thread = worker_handle.thread().clone();
                (
                    Some(pending_producer),
                    Some(completion_consumer),
                    Some(worker_handle),
                    Some(worker_thread),
                    Some(worker_shutdown),
                    Some(worker_running),
                )
            } else {
                (None, None, None, None, None, None)
            };

            let spares = if enabled {
                let mut spares = Vec::with_capacity(ASYNC_KEYFRAME_HANDOFF_CAPACITY);
                for _ in 0..ASYNC_KEYFRAME_HANDOFF_CAPACITY {
                    spares.push(Box::new(KeyFrame::new()));
                }
                spares
            } else {
                Vec::new()
            };
            Ok(Self {
                inner: enabled.then(KeyFrame::new),
                forced_timestamp: None,
                locked: false,
                spares,
                pending_count: 0,
                pending_producer,
                completion_consumer,
                worker_handle,
                worker_thread,
                worker_shutdown,
                worker_running,
                capture_this_copperlist: None,
                dropped_keyframes_total: 0,
                keyframe_interval,
                last_encoded_bytes: 0,
                capture_size_hint: KEYFRAME_PAYLOAD_HEADER.len(),
            })
        }

        #[cfg(not(all(feature = "std", feature = "async-cl-io")))]
        {
            let enabled = sink.is_some();
            Ok(Self {
                inner: enabled.then(KeyFrame::new),
                forced_timestamp: None,
                locked: false,
                sink,
                keyframe_interval,
                last_encoded_bytes: 0,
                capture_size_hint: KEYFRAME_PAYLOAD_HEADER.len(),
            })
        }
    }

    fn is_keyframe_due(&self, culistid: u64) -> bool {
        self.inner.is_some() && culistid.is_multiple_of(self.keyframe_interval as u64)
    }

    #[cfg(all(feature = "std", feature = "async-cl-io"))]
    fn is_capturing(&self, culistid: u64) -> bool {
        self.capture_this_copperlist == Some(culistid)
    }

    #[cfg(not(all(feature = "std", feature = "async-cl-io")))]
    fn is_capturing(&self, culistid: u64) -> bool {
        self.is_keyframe_due(culistid)
    }

    #[inline]
    pub fn captures_keyframe(&self, culistid: u64) -> bool {
        self.is_keyframe_due(culistid)
    }

    /// Start a cold-path sizing pass for the next mission's keyframe buffer.
    #[doc(hidden)]
    pub fn begin_capture_preallocation(&mut self) {
        self.capture_size_hint = KEYFRAME_PAYLOAD_HEADER.len();
    }

    /// Include one component's current frozen size in the cold-path capacity estimate.
    #[doc(hidden)]
    pub fn include_capture_capacity(&mut self, item: &impl Freezable) -> CuResult<()> {
        if self.inner.is_none() {
            return Ok(());
        }
        let mut sizer = EncoderImpl::new(SizeWriter::default(), bincode::config::standard());
        BincodeAdapter(item)
            .encode(&mut sizer)
            .map_err(|_| CuError::from("Failed to size component keyframe state"))?;
        let payload_bytes = sizer.into_writer().bytes_written as usize;
        self.capture_size_hint = self
            .capture_size_hint
            .checked_add(KEYFRAME_FRAME_HEADER_LEN)
            .and_then(|size| size.checked_add(payload_bytes))
            .ok_or_else(|| CuError::from("Keyframe capture capacity overflow"))?;
        Ok(())
    }

    /// Reserve the capture buffer before entering the execution loop.
    #[doc(hidden)]
    pub fn finish_capture_preallocation(&mut self) -> CuResult<()> {
        if self.inner.is_none() {
            return Ok(());
        }
        #[cfg(all(feature = "std", feature = "async-cl-io"))]
        self.reclaim_completed()?;
        let requested = self
            .capture_size_hint
            .max(MIN_KEYFRAME_CAPTURE_CAPACITY)
            .checked_next_power_of_two()
            .ok_or_else(|| CuError::from("Keyframe capture capacity overflow"))?;
        reserve_keyframe_capacity(self.inner.as_mut().unwrap(), requested)?;
        #[cfg(all(feature = "std", feature = "async-cl-io"))]
        for spare in &mut self.spares {
            reserve_keyframe_capacity(spare, requested)?;
        }
        #[cfg(all(feature = "std", feature = "async-cl-io"))]
        if self.spares.len() != ASYNC_KEYFRAME_HANDOFF_CAPACITY {
            return Err(CuError::from(
                "Keyframe output worker did not return every capture buffer before preallocation",
            ));
        }
        Ok(())
    }

    /// Fallible reset used by generated runtimes so asynchronous worker failures
    /// are returned before a new keyframe capture starts.
    #[doc(hidden)]
    pub fn try_reset(&mut self, culistid: u64, clock: &RobotClock) -> CuResult<()> {
        if self.is_keyframe_due(culistid) {
            #[cfg(all(feature = "std", feature = "async-cl-io"))]
            {
                self.reclaim_completed()?;
                if self.spares.is_empty() {
                    self.capture_this_copperlist = None;
                    self.dropped_keyframes_total = self.dropped_keyframes_total.saturating_add(1);
                    self.forced_timestamp = None;
                    self.locked = false;
                    return Ok(());
                }
                self.capture_this_copperlist = Some(culistid);
            }
            // If a recorded keyframe was preloaded for this CL, keep it as-is.
            let inner = self.inner.as_mut().unwrap();
            if self.locked && inner.culistid == culistid {
                return Ok(());
            }
            let ts = self.forced_timestamp.take().unwrap_or_else(|| clock.now());
            inner.reset(culistid, ts);
            self.locked = false;
        }
        Ok(())
    }

    /// Reset the capture buffer at a configured keyframe boundary.
    ///
    /// Generated runtimes use the fallible internal variant to surface output
    /// worker failures. This method preserves the existing direct-call API.
    pub fn reset(&mut self, culistid: u64, clock: &RobotClock) {
        let _ = self.try_reset(culistid, clock);
    }

    /// Force the timestamp of the next keyframe to a given value.
    #[cfg(feature = "std")]
    pub fn set_forced_timestamp(&mut self, ts: CuTime) {
        self.forced_timestamp = Some(ts);
    }

    pub fn freeze_task(&mut self, culistid: u64, task: &impl Freezable) -> CuResult<usize> {
        if self.is_capturing(culistid) {
            if self.locked {
                // We are replaying a recorded keyframe verbatim; don't mutate it.
                return Ok(0);
            }
            let inner = self.inner.as_mut().unwrap();
            if inner.culistid != culistid {
                return Err(CuError::from(format!(
                    "Freezing task for culistid {} but current keyframe is {}",
                    culistid, inner.culistid
                )));
            }
            let encoded = inner
                .add_frozen_task(task)
                .map_err(|e| CuError::from(format!("Failed to serialize task: {e}")))?;
            Ok(encoded)
        } else {
            Ok(0)
        }
    }

    /// Generic helper to freeze any `Freezable` state (task or bridge) into the current keyframe.
    pub fn freeze_any(&mut self, culistid: u64, item: &impl Freezable) -> CuResult<usize> {
        self.freeze_task(culistid, item)
    }

    pub fn end_of_processing(&mut self, culistid: u64) -> CuResult<()> {
        if self.is_capturing(culistid) {
            #[cfg(not(all(feature = "std", feature = "async-cl-io")))]
            {
                let sink = self.sink.as_mut().unwrap();
                sink.log(self.inner.as_ref().unwrap())?;
                self.last_encoded_bytes = sink.last_log_bytes().unwrap_or(0) as u64;
            }
            #[cfg(all(feature = "std", feature = "async-cl-io"))]
            {
                self.last_encoded_bytes = 0;
                let mut completed = self.spares.pop().ok_or_else(|| {
                    CuError::from("Missing spare keyframe buffer at output handoff")
                })?;
                core::mem::swap(self.inner.as_mut().unwrap(), completed.as_mut());
                let producer = self.pending_producer.as_mut().ok_or_else(|| {
                    CuError::from("Missing keyframe output producer for active capture")
                })?;
                match producer.push(completed) {
                    Ok(()) => {
                        self.pending_count += 1;
                        if let Some(worker_thread) = self.worker_thread.as_ref() {
                            worker_thread.unpark();
                        }
                    }
                    Err(PushError::Full(spare)) => {
                        self.spares.push(spare);
                        self.dropped_keyframes_total =
                            self.dropped_keyframes_total.saturating_add(1);
                    }
                }
                self.capture_this_copperlist = None;
            }
            // Clear the lock so the next CL can rebuild normally unless re-locked.
            self.locked = false;
            Ok(())
        } else {
            // Not a keyframe for this CL; ensure we don't carry stale sizes forward.
            self.last_encoded_bytes = 0;
            Ok(())
        }
    }

    /// Preload a recorded keyframe so it is logged verbatim on the matching CL.
    #[cfg(feature = "std")]
    pub fn lock_keyframe(&mut self, keyframe: &KeyFrame) {
        if let Some(inner) = self.inner.as_mut() {
            *inner = keyframe.clone();
            self.forced_timestamp = Some(keyframe.timestamp);
            self.locked = true;
        }
    }

    #[inline]
    #[doc(hidden)]
    pub const fn dropped_keyframes_total(&self) -> u64 {
        #[cfg(all(feature = "std", feature = "async-cl-io"))]
        {
            self.dropped_keyframes_total
        }
        #[cfg(not(all(feature = "std", feature = "async-cl-io")))]
        {
            0
        }
    }

    #[doc(hidden)]
    pub fn finish_pending(&mut self) -> CuResult<()> {
        #[cfg(all(feature = "std", feature = "async-cl-io"))]
        while self.pending_count > 0 {
            self.wait_for_completion()?;
        }
        Ok(())
    }

    #[cfg(all(feature = "std", feature = "async-cl-io"))]
    fn reclaim_completed(&mut self) -> CuResult<()> {
        let pop_result = {
            let Some(completion_consumer) = self.completion_consumer.as_mut() else {
                return Ok(());
            };
            completion_consumer.pop()
        };
        match pop_result {
            Ok(completion) => self.handle_completion(completion),
            Err(PopError::Empty) => {
                if self.pending_count > 0
                    && self
                        .worker_running
                        .as_ref()
                        .is_some_and(|running| !running.load(Ordering::Acquire))
                {
                    Err(CuError::from(
                        "Async keyframe output worker stopped unexpectedly",
                    ))
                } else {
                    Ok(())
                }
            }
        }
    }

    #[cfg(all(feature = "std", feature = "async-cl-io"))]
    fn wait_for_completion(&mut self) -> CuResult<()> {
        loop {
            let pending_before = self.pending_count;
            self.reclaim_completed()?;
            if self.pending_count < pending_before {
                return Ok(());
            }
            std::thread::yield_now();
        }
    }

    #[cfg(all(feature = "std", feature = "async-cl-io"))]
    fn handle_completion(&mut self, completion: AsyncKeyFrameCompletion) -> CuResult<()> {
        self.pending_count = self.pending_count.saturating_sub(1);
        if let Ok(encoded_bytes) = completion.sink_result.as_ref() {
            self.last_encoded_bytes = *encoded_bytes;
        }
        self.spares.push(completion.keyframe);
        completion.sink_result.map(|_| ())
    }

    #[cfg(all(feature = "std", feature = "async-cl-io"))]
    fn shutdown_worker(&mut self) -> CuResult<()> {
        self.finish_pending()?;
        if let Some(shutdown) = self.worker_shutdown.as_ref() {
            shutdown.store(true, Ordering::Release);
        }
        if let Some(worker_thread) = self.worker_thread.as_ref() {
            worker_thread.unpark();
        }
        if let Some(worker_handle) = self.worker_handle.take() {
            worker_handle.join().map_err(|_| {
                CuError::from("Async keyframe output worker panicked while joining")
            })?;
        }
        self.pending_producer.take();
        self.worker_thread.take();
        self.worker_shutdown.take();
        self.worker_running.take();
        Ok(())
    }
}

fn reserve_keyframe_capacity(keyframe: &mut KeyFrame, requested: usize) -> CuResult<()> {
    if keyframe.serialized_tasks.capacity() < requested {
        let additional = requested.saturating_sub(keyframe.serialized_tasks.len());
        keyframe
            .serialized_tasks
            .try_reserve_exact(additional)
            .map_err(|error| {
                CuError::from("Failed to preallocate keyframe capture buffer")
                    .add_cause(&error.to_string())
            })?;
    }
    Ok(())
}

#[cfg(all(feature = "std", feature = "async-cl-io"))]
impl Drop for KeyFramesManager {
    fn drop(&mut self) {
        let _ = self.shutdown_worker();
    }
}

/// This is the main structure that will be injected as a member of the Application struct.
/// CT is the tuple of all the tasks in order of execution.
/// CL is the type of the copper list, representing the input/output messages for all the tasks.
pub struct CuRuntime<CT, CB, P: CopperListTuple, M: CuMonitor, const NBCL: usize> {
    /// The base clock the runtime will be using to record time.
    clock: RobotClock,

    /// Compile-time subsystem identity for this Copper process.
    subsystem_code: u16,

    /// Deployment/runtime instance identity for this Copper process.
    #[doc(hidden)]
    pub instance_id: u32,

    /// The tuple of all the tasks in order of execution.
    #[doc(hidden)]
    pub tasks: CT,

    /// Tuple of all instantiated bridges.
    #[doc(hidden)]
    pub bridges: CB,

    /// Resource registry kept alive for tasks borrowing shared handles.
    #[doc(hidden)]
    pub resources: ResourceManager,

    /// Rayon thread pools owned by the runtime, indexed positionally to
    /// `runtime.thread_pools`. Reserved pools (e.g. `"rt"`) leave `None` slots.
    #[cfg(feature = "std")]
    #[doc(hidden)]
    pub thread_pools: Vec<Option<Arc<ThreadPool>>>,

    /// The runtime monitoring.
    #[doc(hidden)]
    pub monitor: M,

    /// Runtime-side execution progress probe for watchdog/diagnostic monitors.
    ///
    /// This probe is written from the generated execution plan before each component
    /// step. Monitors consume it asynchronously (typically from watchdog threads) to
    /// report the last known component/step/culist when the runtime appears stalled.
    #[cfg(feature = "std")]
    #[doc(hidden)]
    pub execution_probe: ExecutionProbeHandle,
    #[cfg(not(feature = "std"))]
    #[doc(hidden)]
    pub execution_probe: RuntimeExecutionProbe,

    /// Lifecycle manager and completed CopperList output boundary.
    #[doc(hidden)]
    pub copperlists_manager: CopperListsManager<P, NBCL>,

    /// Manager for capturing and consuming task-state keyframes.
    #[doc(hidden)]
    pub keyframes_manager: KeyFramesManager,

    /// Feature-gated container for deterministic multi-CopperList execution.
    #[cfg(all(feature = "std", feature = "parallel-rt"))]
    #[doc(hidden)]
    pub parallel_rt: ParallelRt<NBCL>,

    /// The runtime configuration controlling the behavior of the run loop
    #[doc(hidden)]
    pub runtime_config: RuntimeConfig,
}

/// To be able to share the clock we make the runtime a clock provider.
impl<
    CT,
    CB,
    P: CopperListTuple + CuListZeroedInit + Default + AsyncCopperListPayload,
    M: CuMonitor,
    const NBCL: usize,
> ClockProvider for CuRuntime<CT, CB, P, M, NBCL>
{
    fn get_clock(&self) -> RobotClock {
        self.clock.clone()
    }
}

impl<CT, CB, P: CopperListTuple, M: CuMonitor, const NBCL: usize> CuRuntime<CT, CB, P, M, NBCL> {
    /// Returns a clone of the runtime clock handle.
    #[inline]
    pub fn clock(&self) -> RobotClock {
        self.clock.clone()
    }

    /// Returns the runtime clock by reference for generated runtime code.
    #[doc(hidden)]
    #[inline]
    pub fn clock_ref(&self) -> &RobotClock {
        &self.clock
    }

    /// Returns the compile-time subsystem code for this process.
    #[inline]
    pub fn subsystem_code(&self) -> u16 {
        self.subsystem_code
    }

    /// Returns the configured runtime instance id for this process.
    #[inline]
    pub fn instance_id(&self) -> u32 {
        self.instance_id
    }
}

#[cfg(feature = "std")]
impl<
    'cfg,
    CT,
    CB,
    P: CopperListTuple + CuListZeroedInit + Default + AsyncCopperListPayload + 'static,
    M: CuMonitor,
    const NBCL: usize,
    TI,
    BI,
    MI,
    CLS,
    KFS,
> CuRuntimeBuilder<'cfg, CT, CB, P, M, NBCL, TI, BI, MI, CLS, KFS>
where
    TI: for<'c> Fn(
        Vec<Option<&'c ComponentConfig>>,
        &mut ResourceManager,
        &[Option<Arc<ThreadPool>>],
    ) -> CuResult<CT>,
    BI: Fn(&CuConfig, &mut ResourceManager) -> CuResult<CB>,
    MI: Fn(&CuConfig, CuMonitoringMetadata, CuMonitoringRuntime) -> M,
    CLS: WriteStream<CopperList<P>> + 'static,
    KFS: WriteStream<KeyFrame> + 'static,
{
    pub fn build(self) -> CuResult<CuRuntime<CT, CB, P, M, NBCL>> {
        let Self {
            clock,
            config,
            mission,
            subsystem,
            instance_id,
            resources,
            thread_pools,
            parts,
            copperlist_sink,
            keyframe_sink,
            output_requirements,
        } = self;
        let mut resources =
            resources.ok_or_else(|| CuError::from("Resources missing from CuRuntimeBuilder"))?;
        let thread_pools = thread_pools.unwrap_or_default();

        let graph = config.get_graph(Some(mission))?;
        let all_instances_configs: Vec<Option<&ComponentConfig>> = graph
            .get_all_nodes()
            .iter()
            .map(|(_, node)| node.get_instance_config())
            .collect();

        let tasks =
            (parts.tasks_instanciator)(all_instances_configs, &mut resources, &thread_pools)?;

        #[cfg(feature = "std")]
        let execution_probe = std::sync::Arc::new(RuntimeExecutionProbe::default());
        #[cfg(not(feature = "std"))]
        let execution_probe = RuntimeExecutionProbe::default();
        let monitor_metadata = CuMonitoringMetadata::new(
            CompactString::from(mission),
            parts.monitored_components,
            parts.culist_component_mapping,
            CopperListInfo::new(core::mem::size_of::<CopperList<P>>(), NBCL),
            build_monitor_topology(config, mission)?,
            None,
        )?
        .with_subsystem_id(subsystem.id())
        .with_instance_id(instance_id);
        #[cfg(feature = "std")]
        let monitor_runtime =
            CuMonitoringRuntime::new(MonitorExecutionProbe::from_shared(execution_probe.clone()));
        #[cfg(not(feature = "std"))]
        let monitor_runtime = CuMonitoringRuntime::unavailable();
        let monitor = (parts.monitor_instanciator)(config, monitor_metadata, monitor_runtime);
        let bridges = (parts.bridges_instanciator)(config, &mut resources)?;

        let copperlist_sink = output_requirements
            .completed_copperlists()
            .then(|| Box::new(copperlist_sink) as Box<CompletedCopperListSink<P>>);
        let keyframe_sink = output_requirements
            .keyframes()
            .then(|| Box::new(keyframe_sink) as Box<CompletedKeyFrameSink>);
        let keyframe_interval = config
            .logging
            .as_ref()
            .and_then(|logging| logging.keyframe_interval)
            .unwrap_or(DEFAULT_KEYFRAME_INTERVAL);

        let copperlists_manager = CopperListsManager::new(copperlist_sink)?;
        #[cfg(target_os = "none")]
        {
            let cl_size = core::mem::size_of::<CopperList<P>>();
            let total_bytes = cl_size.saturating_mul(NBCL);
            info!(
                "CuRuntimeBuilder: copperlists count={} cl_size={} total_bytes={}",
                NBCL, cl_size, total_bytes
            );
        }

        let keyframes_manager = KeyFramesManager::new(keyframe_sink, keyframe_interval)?;
        #[cfg(all(feature = "std", feature = "parallel-rt"))]
        let parallel_rt = ParallelRt::new(parts.parallel_rt_metadata)?;

        let runtime_config = config.runtime.clone().unwrap_or_default();
        runtime_config.validate()?;

        Ok(CuRuntime {
            subsystem_code: subsystem.code(),
            instance_id,
            tasks,
            bridges,
            resources,
            thread_pools,
            monitor,
            execution_probe,
            clock,
            copperlists_manager,
            keyframes_manager,
            #[cfg(all(feature = "std", feature = "parallel-rt"))]
            parallel_rt,
            runtime_config,
        })
    }
}

#[cfg(not(feature = "std"))]
impl<
    'cfg,
    CT,
    CB,
    P: CopperListTuple + CuListZeroedInit + Default + AsyncCopperListPayload + 'static,
    M: CuMonitor,
    const NBCL: usize,
    TI,
    BI,
    MI,
    CLS,
    KFS,
> CuRuntimeBuilder<'cfg, CT, CB, P, M, NBCL, TI, BI, MI, CLS, KFS>
where
    TI: for<'c> Fn(Vec<Option<&'c ComponentConfig>>, &mut ResourceManager) -> CuResult<CT>,
    BI: Fn(&CuConfig, &mut ResourceManager) -> CuResult<CB>,
    MI: Fn(&CuConfig, CuMonitoringMetadata, CuMonitoringRuntime) -> M,
    CLS: WriteStream<CopperList<P>> + 'static,
    KFS: WriteStream<KeyFrame> + 'static,
{
    pub fn build(self) -> CuResult<CuRuntime<CT, CB, P, M, NBCL>> {
        let Self {
            clock,
            config,
            mission,
            subsystem,
            instance_id,
            resources,
            parts,
            copperlist_sink,
            keyframe_sink,
            output_requirements,
        } = self;
        let mut resources =
            resources.ok_or_else(|| CuError::from("Resources missing from CuRuntimeBuilder"))?;

        let graph = config.get_graph(Some(mission))?;
        let all_instances_configs: Vec<Option<&ComponentConfig>> = graph
            .get_all_nodes()
            .iter()
            .map(|(_, node)| node.get_instance_config())
            .collect();

        let tasks = (parts.tasks_instanciator)(all_instances_configs, &mut resources)?;

        let execution_probe = RuntimeExecutionProbe::default();
        let monitor_metadata = CuMonitoringMetadata::new(
            CompactString::from(mission),
            parts.monitored_components,
            parts.culist_component_mapping,
            CopperListInfo::new(core::mem::size_of::<CopperList<P>>(), NBCL),
            build_monitor_topology(config, mission)?,
            None,
        )?
        .with_subsystem_id(subsystem.id())
        .with_instance_id(instance_id);
        let monitor_runtime = CuMonitoringRuntime::unavailable();
        let monitor = (parts.monitor_instanciator)(config, monitor_metadata, monitor_runtime);
        let bridges = (parts.bridges_instanciator)(config, &mut resources)?;

        let copperlist_sink = output_requirements
            .completed_copperlists()
            .then(|| Box::new(copperlist_sink) as Box<CompletedCopperListSink<P>>);
        let keyframe_sink = output_requirements
            .keyframes()
            .then(|| Box::new(keyframe_sink) as Box<CompletedKeyFrameSink>);
        let keyframe_interval = config
            .logging
            .as_ref()
            .and_then(|logging| logging.keyframe_interval)
            .unwrap_or(DEFAULT_KEYFRAME_INTERVAL);

        let copperlists_manager = CopperListsManager::new(copperlist_sink)?;
        #[cfg(target_os = "none")]
        {
            let cl_size = core::mem::size_of::<CopperList<P>>();
            let total_bytes = cl_size.saturating_mul(NBCL);
            info!(
                "CuRuntimeBuilder: copperlists count={} cl_size={} total_bytes={}",
                NBCL, cl_size, total_bytes
            );
        }

        let keyframes_manager = KeyFramesManager::new(keyframe_sink, keyframe_interval)?;

        let runtime_config = config.runtime.clone().unwrap_or_default();
        runtime_config.validate()?;

        Ok(CuRuntime {
            subsystem_code: subsystem.code(),
            instance_id,
            tasks,
            bridges,
            resources,
            monitor,
            execution_probe,
            clock,
            copperlists_manager,
            keyframes_manager,
            runtime_config,
        })
    }
}

/// A keyframe records a distributed snapshot of component state around a copperlist.
///
/// `serialized_tasks` contains a versioned sequence of length-framed component
/// snapshots in their generated execution-wave freeze order.
#[derive(Clone, Encode, Decode)]
pub struct KeyFrame {
    // This is the id of the copper list that this keyframe is associated with (recorded before the copperlist).
    pub culistid: u64,
    // This is the timestamp when the keyframe was created, using the robot clock.
    pub timestamp: CuTime,
    // Versioned, length-framed bincode snapshots of all generated components.
    pub serialized_tasks: Vec<u8>,
}

impl KeyFrame {
    fn new() -> Self {
        KeyFrame {
            culistid: 0,
            timestamp: CuTime::default(),
            serialized_tasks: KEYFRAME_PAYLOAD_HEADER.to_vec(),
        }
    }

    /// This is to be able to avoid reallocations
    fn reset(&mut self, culistid: u64, timestamp: CuTime) {
        self.culistid = culistid;
        self.timestamp = timestamp;
        self.serialized_tasks.clear();
        self.serialized_tasks
            .extend_from_slice(KEYFRAME_PAYLOAD_HEADER);
    }

    /// Append one length-framed component snapshot in a single `freeze` pass.
    fn add_frozen_task(&mut self, task: &impl Freezable) -> Result<usize, EncodeError> {
        let cfg = bincode::config::standard();
        let start = self.serialized_tasks.len();
        let payload_offset =
            start
                .checked_add(KEYFRAME_FRAME_HEADER_LEN)
                .ok_or(EncodeError::Other(
                    "keyframe component frame offset overflow",
                ))?;
        if payload_offset > self.serialized_tasks.capacity() {
            return Err(EncodeError::UnexpectedEnd);
        }

        self.serialized_tasks.resize(payload_offset, 0);
        let length_offset = start;
        self.serialized_tasks[length_offset..payload_offset].fill(0);

        let mut encoder =
            EncoderImpl::<_, _>::new(PreallocatedVecWriter(&mut self.serialized_tasks), cfg);
        if let Err(error) = BincodeAdapter(task).encode(&mut encoder) {
            self.serialized_tasks.truncate(start);
            return Err(error);
        }
        let payload_len = encoder.into_writer().0.len() - payload_offset;
        let payload_len = u32::try_from(payload_len).map_err(|_| {
            self.serialized_tasks.truncate(start);
            EncodeError::OtherString(
                "keyframe component snapshot exceeds the u32 frame limit".to_string(),
            )
        })?;
        self.serialized_tasks
            .truncate(payload_offset + payload_len as usize);
        self.serialized_tasks[length_offset..payload_offset]
            .copy_from_slice(&payload_len.to_le_bytes());
        Ok(self.serialized_tasks.len() - start)
    }
}

const KEYFRAME_PAYLOAD_MAGIC: &[u8; 4] = b"CUKF";
const KEYFRAME_PAYLOAD_VERSION: u8 = 1;
const KEYFRAME_PAYLOAD_HEADER: &[u8; 5] = b"CUKF\x01";
const KEYFRAME_FRAME_HEADER_LEN: usize = 4;

/// Reader for the versioned component frames inside a [`KeyFrame`].
#[doc(hidden)]
pub struct KeyFramePayloadReader<'a> {
    remaining: &'a [u8],
}

impl<'a> KeyFramePayloadReader<'a> {
    /// Validate a keyframe payload and prepare to consume its component frames.
    pub fn new(keyframe: &'a KeyFrame) -> CuResult<Self> {
        let payload = keyframe.serialized_tasks.as_slice();
        if payload.len() < KEYFRAME_PAYLOAD_HEADER.len()
            || payload[..KEYFRAME_PAYLOAD_MAGIC.len()] != *KEYFRAME_PAYLOAD_MAGIC
        {
            return Err(CuError::from(
                "Unsupported legacy keyframe payload: expected framed format version 1",
            ));
        }
        let version = payload[KEYFRAME_PAYLOAD_MAGIC.len()];
        if version != KEYFRAME_PAYLOAD_VERSION {
            return Err(CuError::from(format!(
                "Unsupported keyframe payload version {version}; expected {KEYFRAME_PAYLOAD_VERSION}"
            )));
        }
        Ok(Self {
            remaining: &payload[KEYFRAME_PAYLOAD_HEADER.len()..],
        })
    }

    /// Consume the next component frame in generated execution order.
    pub fn next_frame(&mut self) -> CuResult<&'a [u8]> {
        if self.remaining.len() < KEYFRAME_FRAME_HEADER_LEN {
            return Err(CuError::from("Keyframe ended before next component frame"));
        }
        let payload_len = u32::from_le_bytes(
            self.remaining[..KEYFRAME_FRAME_HEADER_LEN]
                .try_into()
                .map_err(|_| CuError::from("Invalid keyframe component frame length"))?,
        ) as usize;
        let frame_end = KEYFRAME_FRAME_HEADER_LEN
            .checked_add(payload_len)
            .ok_or_else(|| CuError::from("Keyframe component frame length overflow"))?;
        if frame_end > self.remaining.len() {
            return Err(CuError::from("Keyframe component frame is truncated"));
        }
        let payload = &self.remaining[KEYFRAME_FRAME_HEADER_LEN..frame_end];
        self.remaining = &self.remaining[frame_end..];
        Ok(payload)
    }

    /// Reject trailing component frames that the generated restore did not consume.
    pub fn finish(self) -> CuResult<()> {
        if self.remaining.is_empty() {
            Ok(())
        } else {
            Err(CuError::from("Keyframe contains trailing component data"))
        }
    }
}

struct FrameSliceReader<'a> {
    remaining: &'a [u8],
}

impl Reader for FrameSliceReader<'_> {
    fn read(&mut self, bytes: &mut [u8]) -> Result<(), DecodeError> {
        if bytes.len() > self.remaining.len() {
            return Err(DecodeError::UnexpectedEnd {
                additional: bytes.len() - self.remaining.len(),
            });
        }
        let (read, remaining) = self.remaining.split_at(bytes.len());
        bytes.copy_from_slice(read);
        self.remaining = remaining;
        Ok(())
    }

    fn peek_read(&mut self, length: usize) -> Option<&[u8]> {
        self.remaining.get(..length)
    }

    fn consume(&mut self, length: usize) {
        self.remaining = self.remaining.get(length..).unwrap_or_default();
    }
}

/// Thaw one component from an isolated keyframe frame and require full consumption.
#[doc(hidden)]
pub fn thaw_keyframe_component(item: &mut impl Freezable, frame: &[u8]) -> CuResult<()> {
    let reader = FrameSliceReader { remaining: frame };
    let mut decoder = DecoderImpl::new(reader, bincode::config::standard(), ());
    item.thaw(&mut decoder)
        .map_err(|error| CuError::from(format!("Failed to thaw keyframe component: {error}")))?;
    let trailing = decoder.reader().remaining.len();
    if trailing != 0 {
        return Err(CuError::from(format!(
            "Keyframe component snapshot has {} trailing bytes",
            trailing
        )));
    }
    Ok(())
}

/// Identifies where the effective runtime configuration came from.
#[derive(Clone, Encode, Decode, Debug, PartialEq, Eq)]
pub enum RuntimeLifecycleConfigSource {
    ProgrammaticOverride,
    ExternalFile,
    BundledDefault,
}

/// Stack and process identification metadata persisted in the runtime lifecycle log.
#[derive(Clone, Encode, Decode, Debug, PartialEq, Eq)]
pub struct RuntimeLifecycleStackInfo {
    pub app_name: String,
    pub app_version: String,
    pub git_commit: Option<String>,
    pub git_dirty: Option<bool>,
    pub subsystem_id: Option<String>,
    pub subsystem_code: u16,
    pub instance_id: u32,
}

/// Runtime lifecycle events emitted in the dedicated lifecycle section.
#[derive(Clone, Encode, Decode, Debug, PartialEq, Eq)]
pub enum RuntimeLifecycleEvent {
    Instantiated {
        config_source: RuntimeLifecycleConfigSource,
        effective_config_ron: String,
        stack: RuntimeLifecycleStackInfo,
    },
    MissionStarted {
        mission: String,
    },
    MissionStopped {
        mission: String,
        // TODO(lifecycle): replace free-form reason with a typed stop reason enum once
        // std/no-std behavior and panic integration are split in a follow-up PR.
        reason: String,
    },
    // TODO(lifecycle): wire panic hook / no_std equivalent to emit this event consistently.
    Panic {
        message: String,
        file: Option<String>,
        line: Option<u32>,
        column: Option<u32>,
    },
    ShutdownCompleted,
}

/// One event record persisted in the `UnifiedLogType::RuntimeLifecycle` section.
#[derive(Clone, Encode, Decode, Debug, PartialEq, Eq)]
pub struct RuntimeLifecycleRecord {
    pub timestamp: CuTime,
    pub event: RuntimeLifecycleEvent,
}

/// Semantic output boundary for runtime lifecycle and manifest records.
#[doc(hidden)]
pub type RuntimeLifecycleSink = SemanticRecordSink<RuntimeLifecycleRecord>;

impl<
    CT,
    CB,
    P: CopperListTuple + CuListZeroedInit + Default + AsyncCopperListPayload + 'static,
    M: CuMonitor,
    const NBCL: usize,
> CuRuntime<CT, CB, P, M, NBCL>
{
    /// Records runtime execution progress in the shared probe.
    ///
    /// This is intentionally lightweight and does not call monitor callbacks.
    #[inline]
    pub fn record_execution_marker(&self, marker: ExecutionMarker) {
        self.execution_probe.record(marker);
    }

    /// Returns a shared reference to the concrete runtime execution probe.
    ///
    /// The generated runtime uses this when it needs a uniform
    /// `&RuntimeExecutionProbe` view across `std` and `no_std` builds.
    #[inline]
    pub fn execution_probe_ref(&self) -> &RuntimeExecutionProbe {
        #[cfg(feature = "std")]
        {
            self.execution_probe.as_ref()
        }

        #[cfg(not(feature = "std"))]
        {
            &self.execution_probe
        }
    }
}

/// Copper tasks can be of 3 types:
/// - Source: only producing output messages (usually used for drivers)
/// - Regular: processing input messages and producing output messages, more like compute nodes.
/// - Sink: only consuming input messages (usually used for actuators)
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum CuTaskType {
    Source,
    Regular,
    Sink,
}

impl From<TaskKind> for CuTaskType {
    fn from(value: TaskKind) -> Self {
        match value {
            TaskKind::Source => CuTaskType::Source,
            TaskKind::Regular => CuTaskType::Regular,
            TaskKind::Sink => CuTaskType::Sink,
        }
    }
}

#[derive(Debug, Clone)]
pub struct CuOutputPack {
    pub culist_index: u32,
    pub msg_types: Vec<String>,
}

#[derive(Debug, Clone)]
pub struct CuInputMsg {
    pub culist_index: u32,
    pub msg_type: String,
    pub src_port: usize,
    pub edge_id: usize,
    pub connection_order: usize,
}

/// Which part of its node's job one plan step runs.
///
/// This is deliberately not folded into [`CuTaskType`]: that enum encodes the
/// graph role (Source/Regular/Sink) and drives call-shape decisions everywhere,
/// while the phase is orthogonal — an anytime node stays `Regular` and appears
/// as one base step plus `max_refines` refine steps (see
/// [`expand_anytime_steps`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum CuStepPhase {
    /// The whole `process()` of a non-anytime task.
    #[default]
    Whole,
    /// The dead-on-arrival age check plus `base()`, at the node's topological
    /// position.
    AnytimeBase,
    /// Exactly one `refine()` quantum.
    AnytimeRefine,
}

/// This structure represents a step in the execution plan.
pub struct CuExecutionStep {
    /// NodeId: node id of the task to execute
    pub node_id: NodeId,
    /// Node: node instance
    pub node: Node,
    /// CuTaskType: type of the task
    pub task_type: CuTaskType,
    /// Which part of the node's job this step runs (anytime nodes span several
    /// steps; everything else is a single `Whole` step).
    pub phase: CuStepPhase,

    /// the indices in the copper list of the input messages and their types
    /// (empty for anytime refine steps: refinement reads no input)
    pub input_msg_indices_types: Vec<CuInputMsg>,

    /// the index in the copper list of the output message and its type
    /// (an anytime node's refine steps carry the same pack as its base step)
    pub output_msg_pack: Option<CuOutputPack>,
}

impl Debug for CuExecutionStep {
    fn fmt(&self, f: &mut Formatter<'_>) -> FmtResult {
        f.write_str(format!("   CuExecutionStep: Node Id: {}\n", self.node_id).as_str())?;
        f.write_str(format!("                  task_type: {:?}\n", self.node.get_type()).as_str())?;
        f.write_str(format!("                       task: {:?}\n", self.task_type).as_str())?;
        f.write_str(format!("                      phase: {:?}\n", self.phase).as_str())?;
        f.write_str(
            format!(
                "              input_msg_types: {:?}\n",
                self.input_msg_indices_types
            )
            .as_str(),
        )?;
        f.write_str(format!("       output_msg_pack: {:?}\n", self.output_msg_pack).as_str())?;
        Ok(())
    }
}

/// This structure represents a loop in the execution plan.
/// It is used to represent a sequence of Execution units (loop or steps) that are executed
/// multiple times.
/// if loop_count is None, the loop is infinite.
pub struct CuExecutionLoop {
    pub steps: Vec<CuExecutionUnit>,
    pub loop_count: Option<u32>,
}

impl Debug for CuExecutionLoop {
    fn fmt(&self, f: &mut Formatter<'_>) -> FmtResult {
        f.write_str("CuExecutionLoop:\n")?;
        for step in &self.steps {
            match step {
                CuExecutionUnit::Step(step) => {
                    step.fmt(f)?;
                }
                CuExecutionUnit::Loop(l) => {
                    l.fmt(f)?;
                }
            }
        }

        f.write_str(format!("   count: {:?}", self.loop_count).as_str())?;
        Ok(())
    }
}

/// This structure represents a step in the execution plan.
#[derive(Debug)]
pub enum CuExecutionUnit {
    Step(Box<CuExecutionStep>),
    Loop(CuExecutionLoop),
}

pub fn find_task_type_for_id(graph: &CuGraph, node_id: NodeId) -> CuResult<CuTaskType> {
    let node = graph
        .get_node(node_id)
        .ok_or_else(|| CuError::from(format!("Node id {node_id} not found")))?;

    if node.get_flavor() == crate::config::Flavor::Task {
        return resolve_task_kind_for_id(graph, node_id).map(Into::into);
    }

    let has_inputs = !graph.get_dst_edges(node_id)?.is_empty();
    let has_outputs = !graph.get_src_edges(node_id)?.is_empty();
    Ok(match (has_inputs, has_outputs) {
        (false, true) => CuTaskType::Source,
        (true, false) => CuTaskType::Sink,
        _ => CuTaskType::Regular,
    })
}

/// Compute the default (`Linearity`) execution plan for `graph`.
///
/// The plan is now pluggable: this splits into the shared
/// `order` + `check_order` + `plan_from_order` pipeline in `planner`, kept here
/// so direct callers (tests, tooling) keep a one-call entry point.
pub fn compute_runtime_plan(graph: &CuGraph) -> CuResult<CuExecutionLoop> {
    let order = Linearity.plan(graph)?;
    check_order(graph, &order)?;
    plan_from_order(graph, &order)
}

/// Expands every foreground anytime node of an already-computed plan into its
/// chunked steps.
///
/// The node's single `Whole` step becomes an [`CuStepPhase::AnytimeBase`] step
/// at its topological position, and `max_refines` [`CuStepPhase::AnytimeRefine`]
/// steps (one `refine()` quantum each) are woven between it and the earliest
/// step consuming the node's output: one immediately after the base step, one
/// after each subsequent independent step, and the remainder contiguously
/// before the consumer. If `max_refines` is smaller than the gap, later gap
/// steps get no quantum between them; if the node has no consumer in this
/// plan, every refine step sits right after the base step.
///
/// The refine count must be known here — the emission count *is* the iteration
/// bound — which is why `max_refines` is mandatory for foreground anytime
/// nodes. How many quanta run and where they sit between other steps is
/// entirely this compile-time scheduling decision; the generated code carries
/// no counter.
pub fn expand_anytime_steps(plan: &mut CuExecutionLoop) -> CuResult<()> {
    loop {
        // One node at a time: expanded steps get a non-`Whole` phase, so the
        // scan converges even though insertions shift positions.
        let Some(base_pos) = plan.steps.iter().position(|unit| {
            matches!(
                unit,
                CuExecutionUnit::Step(step) if step.phase == CuStepPhase::Whole
                    && step.node.anytime().is_some()
                    && !step.node.is_background()
            )
        }) else {
            return Ok(());
        };

        let CuExecutionUnit::Step(base_step) = &mut plan.steps[base_pos] else {
            unreachable!("position() only matches steps");
        };
        let anytime = base_step
            .node
            .anytime()
            .expect("position() only matches anytime nodes");
        // Defense in depth for direct API callers: the macro pipeline rejects
        // this at configuration time (config.rs validate_anytime_graph).
        let Some(max_refines) = anytime.max_refines else {
            return Err(CuError::from(format!(
                "Task '{}': a foreground anytime task needs anytime.max_refines to expand into a static plan.",
                base_step.node.get_id()
            )));
        };
        base_step.phase = CuStepPhase::AnytimeBase;
        let output_pack = base_step.output_msg_pack.clone().ok_or_else(|| {
            CuError::from(format!(
                "Task '{}': an anytime task needs an output to refine.",
                base_step.node.get_id()
            ))
        })?;
        let output_index = output_pack.culist_index;
        let node_id = base_step.node_id;
        let node = base_step.node.clone();
        let task_type = base_step.task_type;

        let refine_step = || {
            CuExecutionUnit::Step(Box::new(CuExecutionStep {
                node_id,
                node: node.clone(),
                task_type,
                phase: CuStepPhase::AnytimeRefine,
                input_msg_indices_types: Vec::new(),
                output_msg_pack: Some(output_pack.clone()),
            }))
        };

        // Earliest step consuming the node's output; refine steps never match
        // (their inputs are empty), so already-expanded nodes stay inert here.
        let consumer_pos = plan.steps[base_pos + 1..]
            .iter()
            .position(|unit| {
                matches!(
                    unit,
                    CuExecutionUnit::Step(step) if step
                        .input_msg_indices_types
                        .iter()
                        .any(|input| input.culist_index == output_index)
                )
            })
            .map(|offset| base_pos + 1 + offset)
            .unwrap_or(base_pos + 1);

        let mut tail = plan.steps.split_off(base_pos + 1);
        let suffix = tail.split_off(consumer_pos - base_pos - 1);
        let gap = tail;

        // max_refines >= 1 is enforced by AnytimeConfig::validate.
        let mut remaining = max_refines.max(1);
        remaining -= 1;
        plan.steps.push(refine_step());
        for gap_unit in gap {
            plan.steps.push(gap_unit);
            if remaining > 0 {
                remaining -= 1;
                plan.steps.push(refine_step());
            }
        }
        for _ in 0..remaining {
            plan.steps.push(refine_step());
        }
        plan.steps.extend(suffix);
    }
}

//tests
#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::Node;
    use crate::context::CuContext;
    use crate::cutask::CuSinkTask;
    use crate::cutask::{CuSrcTask, Freezable};
    use crate::monitoring::NoMonitor;
    use crate::reflect::Reflect;
    use bincode::Encode;
    use core::cell::Cell;
    use cu29_traits::{ErasedCuStampedData, ErasedCuStampedDataSet, MatchingTasks};
    use serde_derive::{Deserialize, Serialize};
    #[cfg(all(feature = "std", feature = "async-cl-io"))]
    use std::sync::mpsc::{Receiver, SyncSender, sync_channel};
    #[cfg(feature = "std")]
    use std::sync::{Arc, Mutex};

    struct CountingSnapshot<'a> {
        calls: &'a Cell<usize>,
        value: u32,
        fail: bool,
    }

    impl Freezable for CountingSnapshot<'_> {
        fn freeze<E: bincode::enc::Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
            self.calls.set(self.calls.get() + 1);
            self.value.encode(encoder)?;
            if self.fail {
                Err(EncodeError::OtherString(
                    "intentional freeze failure".to_string(),
                ))
            } else {
                Ok(())
            }
        }
    }

    #[derive(Default)]
    struct SnapshotValue(u32);

    impl Freezable for SnapshotValue {
        fn thaw<D: bincode::de::Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
            self.0 = u32::decode(decoder)?;
            Ok(())
        }
    }

    #[test]
    fn keyframe_frames_freeze_once_and_roll_back_only_failed_frame() {
        let calls = Cell::new(0);
        let mut keyframe = KeyFrame::new();
        keyframe
            .serialized_tasks
            .try_reserve_exact(MIN_KEYFRAME_CAPTURE_CAPACITY)
            .unwrap();
        keyframe.reset(7, CuTime::from_nanos(70));
        keyframe
            .add_frozen_task(&CountingSnapshot {
                calls: &calls,
                value: 11,
                fail: false,
            })
            .unwrap();
        let committed_len = keyframe.serialized_tasks.len();

        let failing = CountingSnapshot {
            calls: &calls,
            value: 99,
            fail: true,
        };
        assert!(keyframe.add_frozen_task(&failing).is_err());
        assert_eq!(keyframe.serialized_tasks.len(), committed_len);

        keyframe
            .add_frozen_task(&CountingSnapshot {
                calls: &calls,
                value: 22,
                fail: false,
            })
            .unwrap();
        assert_eq!(calls.get(), 3, "each append must call freeze exactly once");
        let first_payload_len = bincode::encode_to_vec(11u32, bincode::config::standard())
            .unwrap()
            .len();
        let second_payload_len = bincode::encode_to_vec(22u32, bincode::config::standard())
            .unwrap()
            .len();
        assert_eq!(
            keyframe.serialized_tasks.len(),
            KEYFRAME_PAYLOAD_HEADER.len()
                + 2 * KEYFRAME_FRAME_HEADER_LEN
                + first_payload_len
                + second_payload_len,
            "component frames carry only a length prefix"
        );

        let mut frames = KeyFramePayloadReader::new(&keyframe).unwrap();
        let mut first = SnapshotValue::default();
        thaw_keyframe_component(&mut first, frames.next_frame().unwrap()).unwrap();
        let mut second = SnapshotValue::default();
        thaw_keyframe_component(&mut second, frames.next_frame().unwrap()).unwrap();
        frames.finish().unwrap();
        assert_eq!((first.0, second.0), (11, 22));
    }

    #[cfg(all(feature = "std", feature = "memory_monitoring"))]
    #[test]
    fn preallocated_keyframe_append_does_not_allocate() {
        let calls = Cell::new(0);
        let mut keyframe = KeyFrame::new();
        keyframe
            .serialized_tasks
            .try_reserve_exact(MIN_KEYFRAME_CAPTURE_CAPACITY)
            .unwrap();
        keyframe.reset(3, CuTime::from_nanos(30));

        let allocations = crate::monitoring::ScopedAllocCounter::new();
        keyframe
            .add_frozen_task(&CountingSnapshot {
                calls: &calls,
                value: 42,
                fail: false,
            })
            .unwrap();

        assert_eq!(allocations.allocated(), 0);
        assert_eq!(calls.get(), 1);
    }

    #[test]
    fn keyframe_reader_rejects_legacy_payload_clearly() {
        let keyframe = KeyFrame {
            culistid: 0,
            timestamp: CuTime::default(),
            serialized_tasks: vec![0, 1, 2],
        };
        let error = match KeyFramePayloadReader::new(&keyframe) {
            Ok(_) => panic!("legacy payload unexpectedly accepted"),
            Err(error) => error,
        };
        assert!(error.to_string().contains("legacy keyframe payload"));
    }

    #[derive(Reflect)]
    pub struct TestSource {}

    impl Freezable for TestSource {}

    impl CuSrcTask for TestSource {
        type Resources<'r> = ();
        type Output<'m> = ();
        fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self {})
        }

        fn process(&mut self, _ctx: &CuContext, _empty_msg: &mut Self::Output<'_>) -> CuResult<()> {
            Ok(())
        }
    }

    #[derive(Reflect)]
    pub struct TestSink {}

    impl Freezable for TestSink {}

    impl CuSinkTask for TestSink {
        type Resources<'r> = ();
        type Input<'m> = ();

        fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self {})
        }

        fn process(&mut self, _ctx: &CuContext, _input: &Self::Input<'_>) -> CuResult<()> {
            Ok(())
        }
    }

    // Those should be generated by the derive macro
    type Tasks = (TestSource, TestSink);
    type TestRuntime = CuRuntime<Tasks, (), Msgs, NoMonitor, 2>;
    const TEST_NBCL: usize = 2;

    #[derive(Debug, Encode, Decode, Serialize, Deserialize, Default)]
    struct Msgs(());

    impl ErasedCuStampedDataSet for Msgs {
        fn cumsgs(&self) -> Vec<&dyn ErasedCuStampedData> {
            Vec::new()
        }
    }

    impl MatchingTasks for Msgs {
        fn get_all_task_ids() -> &'static [&'static str] {
            &[]
        }
    }

    impl CuListZeroedInit for Msgs {
        fn init_zeroed(&mut self) {}
    }

    #[derive(Debug, Encode, Decode, Serialize, Deserialize, Default)]
    struct IntMsgs(i32);

    impl ErasedCuStampedDataSet for IntMsgs {
        fn cumsgs(&self) -> Vec<&dyn ErasedCuStampedData> {
            Vec::new()
        }
    }

    impl MatchingTasks for IntMsgs {
        fn get_all_task_ids() -> &'static [&'static str] {
            &[]
        }
    }

    impl CuListZeroedInit for IntMsgs {
        fn init_zeroed(&mut self) {}
    }

    #[cfg(feature = "std")]
    fn tasks_instanciator(
        all_instances_configs: Vec<Option<&ComponentConfig>>,
        _resources: &mut ResourceManager,
        _thread_pools: &[Option<Arc<rayon::ThreadPool>>],
    ) -> CuResult<Tasks> {
        Ok((
            TestSource::new(all_instances_configs[0], ())?,
            TestSink::new(all_instances_configs[1], ())?,
        ))
    }

    #[cfg(not(feature = "std"))]
    fn tasks_instanciator(
        all_instances_configs: Vec<Option<&ComponentConfig>>,
        _resources: &mut ResourceManager,
    ) -> CuResult<Tasks> {
        Ok((
            TestSource::new(all_instances_configs[0], ())?,
            TestSink::new(all_instances_configs[1], ())?,
        ))
    }

    fn monitor_instanciator(
        _config: &CuConfig,
        metadata: CuMonitoringMetadata,
        runtime: CuMonitoringRuntime,
    ) -> NoMonitor {
        NoMonitor::new(metadata, runtime).expect("NoMonitor::new should never fail")
    }

    fn bridges_instanciator(_config: &CuConfig, _resources: &mut ResourceManager) -> CuResult<()> {
        Ok(())
    }

    fn resources_instanciator(_config: &CuConfig) -> CuResult<ResourceManager> {
        Ok(ResourceManager::new(&[]))
    }

    #[derive(Debug)]
    struct FakeWriter {}

    impl<E: Encode> WriteStream<E> for FakeWriter {
        fn log(&mut self, _obj: &E) -> CuResult<()> {
            Ok(())
        }
    }

    #[cfg(not(feature = "async-cl-io"))]
    #[derive(Debug)]
    struct RecordingSyncWriter {
        ids: Arc<Mutex<Vec<u64>>>,
        last_log_bytes: usize,
        fail_on: Option<u64>,
    }

    #[cfg(not(feature = "async-cl-io"))]
    impl WriteStream<CopperList<IntMsgs>> for RecordingSyncWriter {
        fn log(&mut self, culist: &CopperList<IntMsgs>) -> CuResult<()> {
            self.ids.lock().unwrap().push(culist.id);
            if self.fail_on == Some(culist.id) {
                return Err(CuError::from(format!(
                    "logger failed for CopperList #{}",
                    culist.id
                )));
            }
            Ok(())
        }

        fn last_log_bytes(&self) -> Option<usize> {
            Some(self.last_log_bytes)
        }
    }

    #[cfg(feature = "std")]
    #[derive(Debug)]
    struct RecordingSemanticSink {
        ids: Arc<Mutex<Vec<u64>>>,
    }

    #[cfg(feature = "std")]
    impl WriteStream<CopperList<IntMsgs>> for RecordingSemanticSink {
        fn log(&mut self, culist: &CopperList<IntMsgs>) -> CuResult<()> {
            assert_eq!(culist.get_state(), CopperListState::BeingSerialized);
            self.ids.lock().unwrap().push(culist.id);
            Ok(())
        }

        fn last_log_bytes(&self) -> Option<usize> {
            Some(23)
        }
    }

    #[cfg(feature = "std")]
    impl WriteStream<CopperList<Msgs>> for RecordingSemanticSink {
        fn log(&mut self, culist: &CopperList<Msgs>) -> CuResult<()> {
            assert_eq!(culist.get_state(), CopperListState::BeingSerialized);
            self.ids.lock().unwrap().push(culist.id);
            Ok(())
        }

        fn last_log_bytes(&self) -> Option<usize> {
            Some(23)
        }
    }

    #[cfg(feature = "std")]
    #[derive(Debug)]
    struct RecordingKeyFrameSink {
        ids: Arc<Mutex<Vec<u64>>>,
    }

    #[cfg(feature = "std")]
    impl WriteStream<KeyFrame> for RecordingKeyFrameSink {
        fn log(&mut self, keyframe: &KeyFrame) -> CuResult<()> {
            self.ids.lock().unwrap().push(keyframe.culistid);
            Ok(())
        }

        fn last_log_bytes(&self) -> Option<usize> {
            Some(29)
        }
    }

    #[test]
    fn test_runtime_instantiation() {
        let mut config = CuConfig::default();
        let graph = config.get_graph_mut(None).unwrap();
        graph.add_node(Node::new("a", "TestSource")).unwrap();
        graph.add_node(Node::new("b", "TestSink")).unwrap();
        graph.connect(0, 1, "()").unwrap();
        let runtime: CuResult<TestRuntime> =
            CuRuntimeBuilder::<Tasks, (), Msgs, NoMonitor, TEST_NBCL, _, _, _, _, _>::new(
                RobotClock::default(),
                &config,
                crate::config::DEFAULT_MISSION_ID,
                CuRuntimeParts::new(
                    tasks_instanciator,
                    &[],
                    &[],
                    #[cfg(all(feature = "std", feature = "parallel-rt"))]
                    &crate::parallel_rt::DISABLED_PARALLEL_RT_METADATA,
                    monitor_instanciator,
                    bridges_instanciator,
                ),
                FakeWriter {},
                FakeWriter {},
                OutputRequirements::new(true, true),
            )
            .try_with_resources_instantiator(resources_instanciator)
            .and_then(|builder| builder.build());
        assert!(runtime.is_ok());
    }

    #[cfg(feature = "std")]
    #[test]
    fn downstream_requirements_are_independent_from_local_logging() {
        let mut config = CuConfig::default();
        config.logging = Some(crate::config::LoggingConfig {
            enable_task_logging: false,
            enable_keyframe_logging: false,
            keyframe_interval: Some(1),
            ..Default::default()
        });
        let graph = config.get_graph_mut(None).unwrap();
        graph.add_node(Node::new("a", "TestSource")).unwrap();
        graph.add_node(Node::new("b", "TestSink")).unwrap();
        graph.connect(0, 1, "()").unwrap();

        let copperlist_ids = Arc::new(Mutex::new(Vec::new()));
        let keyframe_ids = Arc::new(Mutex::new(Vec::new()));
        let mut runtime: TestRuntime =
            CuRuntimeBuilder::<Tasks, (), Msgs, NoMonitor, TEST_NBCL, _, _, _, _, _>::new(
                RobotClock::default(),
                &config,
                crate::config::DEFAULT_MISSION_ID,
                CuRuntimeParts::new(
                    tasks_instanciator,
                    &[],
                    &[],
                    #[cfg(all(feature = "std", feature = "parallel-rt"))]
                    &crate::parallel_rt::DISABLED_PARALLEL_RT_METADATA,
                    monitor_instanciator,
                    bridges_instanciator,
                ),
                RecordingSemanticSink {
                    ids: copperlist_ids.clone(),
                },
                RecordingKeyFrameSink {
                    ids: keyframe_ids.clone(),
                },
                OutputRequirements::new(true, false).union(OutputRequirements::new(false, true)),
            )
            .try_with_resources_instantiator(resources_instanciator)
            .and_then(|builder| builder.build())
            .unwrap();

        let copperlist = runtime.copperlists_manager.create().unwrap();
        copperlist.change_state(CopperListState::Processing);
        runtime.copperlists_manager.end_of_processing(0).unwrap();
        runtime.copperlists_manager.finish_pending().unwrap();

        runtime
            .keyframes_manager
            .try_reset(0, &runtime.clock)
            .unwrap();
        runtime.keyframes_manager.end_of_processing(0).unwrap();
        runtime.keyframes_manager.finish_pending().unwrap();

        assert_eq!(*copperlist_ids.lock().unwrap(), vec![0]);
        assert_eq!(*keyframe_ids.lock().unwrap(), vec![0]);
    }

    #[test]
    fn test_rate_target_period_rejects_zero() {
        let err = rate_target_period(0).expect_err("zero rate target should fail");
        assert!(
            err.to_string()
                .contains("Runtime rate target cannot be zero"),
            "unexpected error: {err}"
        );
    }

    #[test]
    fn test_loop_rate_limiter_advances_to_next_period_when_on_time() {
        let (clock, mock) = RobotClock::mock();
        let mut limiter = LoopRateLimiter::from_rate_target_hz(100, &clock).unwrap();
        assert_eq!(limiter.next_deadline(), CuTime::from_nanos(10_000_000));

        mock.set_value(10_000_000);
        limiter.mark_tick(&clock);

        assert_eq!(limiter.next_deadline(), CuTime::from_nanos(20_000_000));
    }

    #[test]
    fn test_loop_rate_limiter_skips_missed_periods_without_resetting_phase() {
        let (clock, mock) = RobotClock::mock();
        let mut limiter = LoopRateLimiter::from_rate_target_hz(100, &clock).unwrap();

        mock.set_value(35_000_000);
        limiter.mark_tick(&clock);

        assert_eq!(limiter.next_deadline(), CuTime::from_nanos(40_000_000));
    }

    #[cfg(all(feature = "std", feature = "high-precision-limiter"))]
    #[test]
    fn test_loop_rate_limiter_spin_window_is_fixed_scheduler_window() {
        let (clock, _) = RobotClock::mock();
        let limiter = LoopRateLimiter::from_rate_target_hz(1_000, &clock).unwrap();
        assert_eq!(limiter.spin_window(), CuDuration::from(200_000));

        let fast = LoopRateLimiter::from_rate_target_hz(10_000, &clock).unwrap();
        assert_eq!(fast.spin_window(), CuDuration::from(200_000));
    }

    #[cfg(not(feature = "async-cl-io"))]
    #[test]
    fn test_copperlists_manager_lifecycle() {
        let mut config = CuConfig::default();
        let graph = config.get_graph_mut(None).unwrap();
        graph.add_node(Node::new("a", "TestSource")).unwrap();
        graph.add_node(Node::new("b", "TestSink")).unwrap();
        graph.connect(0, 1, "()").unwrap();

        let mut runtime: TestRuntime =
            CuRuntimeBuilder::<Tasks, (), Msgs, NoMonitor, TEST_NBCL, _, _, _, _, _>::new(
                RobotClock::default(),
                &config,
                crate::config::DEFAULT_MISSION_ID,
                CuRuntimeParts::new(
                    tasks_instanciator,
                    &[],
                    &[],
                    #[cfg(all(feature = "std", feature = "parallel-rt"))]
                    &crate::parallel_rt::DISABLED_PARALLEL_RT_METADATA,
                    monitor_instanciator,
                    bridges_instanciator,
                ),
                FakeWriter {},
                FakeWriter {},
                OutputRequirements::new(true, true),
            )
            .try_with_resources_instantiator(resources_instanciator)
            .and_then(|builder| builder.build())
            .unwrap();

        // Now emulates the generated runtime
        {
            let copperlists = &mut runtime.copperlists_manager;
            let culist0 = copperlists
                .create()
                .expect("Ran out of space for copper lists");
            let id = culist0.id;
            assert_eq!(id, 0);
            culist0.change_state(CopperListState::Processing);
            assert_eq!(copperlists.available_copper_lists().unwrap(), 1);
        }

        {
            let copperlists = &mut runtime.copperlists_manager;
            let culist1 = copperlists
                .create()
                .expect("Ran out of space for copper lists");
            let id = culist1.id;
            assert_eq!(id, 1);
            culist1.change_state(CopperListState::Processing);
            assert_eq!(copperlists.available_copper_lists().unwrap(), 0);
        }

        {
            let copperlists = &mut runtime.copperlists_manager;
            let culist2 = copperlists.create();
            assert!(culist2.is_err());
            assert_eq!(copperlists.available_copper_lists().unwrap(), 0);
            // Free in order, should let the top of the stack be serialized and freed.
            let _ = copperlists.end_of_processing(1);
            assert_eq!(copperlists.available_copper_lists().unwrap(), 1);
        }

        // Readd a CL
        {
            let copperlists = &mut runtime.copperlists_manager;
            let culist2 = copperlists
                .create()
                .expect("Ran out of space for copper lists");
            let id = culist2.id;
            assert_eq!(id, 2);
            culist2.change_state(CopperListState::Processing);
            assert_eq!(copperlists.available_copper_lists().unwrap(), 0);
            // Free out of order, the #0 first
            let _ = copperlists.end_of_processing(0);
            // Should not free up the top of the stack
            assert_eq!(copperlists.available_copper_lists().unwrap(), 0);

            // Free up the top of the stack
            let _ = copperlists.end_of_processing(2);
            // This should free up 2 CLs

            assert_eq!(copperlists.available_copper_lists().unwrap(), 2);
        }
    }

    #[cfg(not(feature = "async-cl-io"))]
    #[test]
    fn test_sync_copperlists_accessors_passthrough_to_inner_manager() {
        let mut copperlists = SyncCopperListsManager::<IntMsgs, 2>::new(None).unwrap();

        assert_eq!(copperlists.next_cl_id(), 0);
        assert_eq!(copperlists.last_cl_id(), 0);
        assert!(copperlists.peek().is_none());

        {
            let culist = copperlists.create().unwrap();
            culist.msgs.0 = 11;
            assert_eq!(culist.id, 0);
            assert_eq!(culist.get_state(), CopperListState::Initialized);
        }

        assert_eq!(copperlists.next_cl_id(), 1);
        assert_eq!(copperlists.last_cl_id(), 0);
        let peeked = copperlists.peek().unwrap();
        assert_eq!(peeked.id, 0);
        assert_eq!(peeked.msgs.0, 11);
        assert_eq!(peeked.get_state(), CopperListState::Initialized);
    }

    #[cfg(not(feature = "async-cl-io"))]
    #[test]
    fn test_sync_reclaimed_slot_reuse_reinitializes_state_but_preserves_payload_storage() {
        let mut copperlists = SyncCopperListsManager::<IntMsgs, 1>::new(None).unwrap();

        {
            let culist = copperlists.create().unwrap();
            culist.msgs.0 = 41;
            culist.change_state(CopperListState::Processing);
            assert_eq!(culist.id, 0);
        }

        copperlists.end_of_processing(0).unwrap();
        assert_eq!(copperlists.available_copper_lists().unwrap(), 1);

        let reused = copperlists.create().unwrap();
        assert_eq!(reused.id, 1);
        assert_eq!(reused.get_state(), CopperListState::Initialized);
        assert_eq!(reused.msgs.0, 41);
    }

    #[cfg(all(not(feature = "async-cl-io"), debug_assertions))]
    #[test]
    #[should_panic(expected = "sync end_of_processing expected exactly one active CopperList #99")]
    fn test_sync_end_of_processing_unknown_id_panics_in_debug() {
        let mut copperlists = SyncCopperListsManager::<IntMsgs, 2>::new(None).unwrap();

        {
            let culist = copperlists.create().unwrap();
            culist.msgs.0 = 10;
            culist.change_state(CopperListState::Processing);
        }
        {
            let culist = copperlists.create().unwrap();
            culist.msgs.0 = 20;
            culist.change_state(CopperListState::Processing);
        }

        let _ = copperlists.end_of_processing(99);
    }

    #[cfg(all(not(feature = "async-cl-io"), debug_assertions))]
    #[test]
    #[should_panic(expected = "sync end_of_processing expected CopperList #0 to be Processing")]
    fn test_sync_end_of_processing_wrong_state_panics_in_debug() {
        let mut copperlists = SyncCopperListsManager::<IntMsgs, 1>::new(None).unwrap();

        {
            let culist = copperlists.create().unwrap();
            culist.msgs.0 = 10;
            assert_eq!(culist.get_state(), CopperListState::Initialized);
        }

        let _ = copperlists.end_of_processing(0);
    }

    #[cfg(not(feature = "async-cl-io"))]
    #[test]
    fn test_sync_end_of_processing_serializes_done_suffix_from_newest_to_oldest() {
        let ids = Arc::new(Mutex::new(Vec::new()));
        let mut copperlists =
            SyncCopperListsManager::<IntMsgs, 2>::new(Some(Box::new(RecordingSyncWriter {
                ids: ids.clone(),
                last_log_bytes: 17,
                fail_on: None,
            })))
            .unwrap();

        {
            let culist = copperlists.create().unwrap();
            culist.msgs.0 = 10;
            culist.change_state(CopperListState::Processing);
        }
        {
            let culist = copperlists.create().unwrap();
            culist.msgs.0 = 20;
            culist.change_state(CopperListState::Processing);
        }

        copperlists.end_of_processing(0).unwrap();
        assert!(ids.lock().unwrap().is_empty());
        assert_eq!(copperlists.available_copper_lists().unwrap(), 0);

        copperlists.end_of_processing(1).unwrap();

        assert_eq!(*ids.lock().unwrap(), vec![1, 0]);
        assert_eq!(copperlists.available_copper_lists().unwrap(), 2);
    }

    #[cfg(not(feature = "async-cl-io"))]
    #[test]
    fn test_sync_end_of_processing_updates_logger_counters_on_success() {
        let ids = Arc::new(Mutex::new(Vec::new()));
        let mut copperlists =
            SyncCopperListsManager::<IntMsgs, 1>::new(Some(Box::new(RecordingSyncWriter {
                ids: ids.clone(),
                last_log_bytes: 17,
                fail_on: None,
            })))
            .unwrap();
        let io_cache = crate::monitoring::CuMsgIoCache::<1>::default();

        {
            let culist = copperlists.create().unwrap();
            culist.msgs.0 = 10;
            culist.change_state(CopperListState::Processing);
        }

        {
            let capture = crate::monitoring::start_copperlist_io_capture(&io_cache);
            capture.select_slot(0);
            crate::monitoring::record_payload_handle_bytes(32);
        }

        copperlists.end_of_processing(0).unwrap();

        assert_eq!(*ids.lock().unwrap(), vec![0]);
        assert_eq!(copperlists.last_encoded_bytes, 17);
        assert_eq!(copperlists.last_handle_bytes, 32);
        assert_eq!(copperlists.available_copper_lists().unwrap(), 1);
    }

    #[cfg(feature = "std")]
    #[test]
    fn test_sync_manager_accepts_nonserializing_semantic_sink() {
        let ids = Arc::new(Mutex::new(Vec::new()));
        let mut copperlists =
            SyncCopperListsManager::<IntMsgs, 1>::new(Some(Box::new(RecordingSemanticSink {
                ids: ids.clone(),
            })))
            .unwrap();

        let culist = copperlists.create().unwrap();
        culist.change_state(CopperListState::Processing);
        copperlists.end_of_processing(0).unwrap();

        assert_eq!(*ids.lock().unwrap(), vec![0]);
        assert_eq!(copperlists.last_encoded_bytes, 23);
        assert_eq!(copperlists.last_handle_bytes, 0);
    }

    #[cfg(feature = "std")]
    #[test]
    fn test_keyframe_manager_accepts_nonserializing_semantic_sink() {
        let ids = Arc::new(Mutex::new(Vec::new()));
        let mut keyframes = KeyFramesManager::new(
            Some(Box::new(RecordingKeyFrameSink { ids: ids.clone() })),
            1,
        )
        .unwrap();

        keyframes.try_reset(7, &RobotClock::default()).unwrap();
        keyframes.end_of_processing(7).unwrap();
        keyframes.finish_pending().unwrap();

        assert_eq!(*ids.lock().unwrap(), vec![7]);
        assert_eq!(keyframes.last_encoded_bytes, 29);
    }

    #[cfg(not(feature = "async-cl-io"))]
    #[test]
    fn test_sync_end_of_processing_preserves_slot_on_logger_error() {
        let ids = Arc::new(Mutex::new(Vec::new()));
        let mut copperlists =
            SyncCopperListsManager::<IntMsgs, 1>::new(Some(Box::new(RecordingSyncWriter {
                ids: ids.clone(),
                last_log_bytes: 17,
                fail_on: Some(0),
            })))
            .unwrap();

        {
            let culist = copperlists.create().unwrap();
            culist.change_state(CopperListState::Processing);
        }

        let err = copperlists.end_of_processing(0).unwrap_err();

        assert!(
            err.to_string().contains("logger failed for CopperList #0"),
            "unexpected error: {err}"
        );
        assert_eq!(*ids.lock().unwrap(), vec![0]);
        assert_eq!(copperlists.available_copper_lists().unwrap(), 0);
        assert_eq!(copperlists.last_encoded_bytes, 0);
        assert_eq!(copperlists.last_handle_bytes, 0);

        let peeked = copperlists.peek().unwrap();
        assert_eq!(peeked.id, 0);
        assert_eq!(peeked.get_state(), CopperListState::BeingSerialized);
    }

    #[cfg(all(not(feature = "async-cl-io"), feature = "std", debug_assertions))]
    #[test]
    #[should_panic(
        expected = "sync boxed end_of_processing expected CopperList #7 to be Processing"
    )]
    fn test_sync_end_of_processing_boxed_wrong_state_panics_in_debug() {
        let mut copperlists = SyncCopperListsManager::<IntMsgs, 1>::new(None).unwrap();
        let culist = Box::new(CopperList::new(7, IntMsgs::default()));

        let _ = copperlists.end_of_processing_boxed(culist);
    }

    #[cfg(all(feature = "std", feature = "async-cl-io"))]
    #[derive(Debug, Default)]
    struct RecordingWriter {
        ids: Arc<Mutex<Vec<u64>>>,
    }

    #[cfg(all(feature = "std", feature = "async-cl-io"))]
    impl WriteStream<CopperList<Msgs>> for RecordingWriter {
        fn log(&mut self, culist: &CopperList<Msgs>) -> CuResult<()> {
            assert_eq!(culist.get_state(), CopperListState::BeingSerialized);
            self.ids.lock().unwrap().push(culist.id);
            std::thread::sleep(std::time::Duration::from_millis(2));
            Ok(())
        }
    }

    #[cfg(all(feature = "std", feature = "async-cl-io"))]
    #[derive(Debug)]
    struct BlockingWriter {
        ids: Arc<Mutex<Vec<u64>>>,
        started: SyncSender<()>,
        release: Arc<Mutex<Receiver<()>>>,
    }

    #[cfg(all(feature = "std", feature = "async-cl-io"))]
    impl WriteStream<CopperList<Msgs>> for BlockingWriter {
        fn log(&mut self, culist: &CopperList<Msgs>) -> CuResult<()> {
            self.ids.lock().unwrap().push(culist.id);
            self.started
                .send(())
                .map_err(|_| CuError::from("failed to signal blocking writer start"))?;
            self.release
                .lock()
                .unwrap()
                .recv()
                .map_err(|_| CuError::from("failed to release blocking writer"))
        }
    }

    #[cfg(all(feature = "std", feature = "async-cl-io"))]
    #[derive(Debug)]
    struct BlockingKeyFrameWriter {
        ids: Arc<Mutex<Vec<u64>>>,
        started: SyncSender<()>,
        release: Arc<Mutex<Receiver<()>>>,
    }

    #[cfg(all(feature = "std", feature = "async-cl-io"))]
    impl WriteStream<KeyFrame> for BlockingKeyFrameWriter {
        fn log(&mut self, keyframe: &KeyFrame) -> CuResult<()> {
            self.ids.lock().unwrap().push(keyframe.culistid);
            self.started
                .send(())
                .map_err(|_| CuError::from("failed to signal blocking keyframe writer start"))?;
            self.release
                .lock()
                .unwrap()
                .recv()
                .map_err(|_| CuError::from("failed to release blocking keyframe writer"))
        }
    }

    #[test]
    fn disabled_keyframes_allocate_no_capture_buffers() {
        let keyframes = KeyFramesManager::new(None, 1).unwrap();

        assert!(keyframes.inner.is_none());
        assert!(!keyframes.captures_keyframe(0));
        #[cfg(all(feature = "std", feature = "async-cl-io"))]
        {
            assert!(keyframes.spares.is_empty());
            assert!(keyframes.pending_producer.is_none());
            assert!(keyframes.worker_handle.is_none());
        }
    }

    #[cfg(all(feature = "std", feature = "memory_monitoring"))]
    #[test]
    fn disabled_keyframe_manager_allocates_nothing() {
        let allocations = crate::monitoring::ScopedAllocCounter::new();
        let keyframes = KeyFramesManager::new(None, 1).unwrap();

        assert_eq!(allocations.allocated(), 0);
        assert!(keyframes.inner.is_none());
    }

    #[cfg(all(feature = "std", feature = "async-cl-io"))]
    #[test]
    fn saturated_keyframe_handoff_skips_capture_before_freezing() {
        let ids = Arc::new(Mutex::new(Vec::new()));
        let (started_tx, started_rx) = sync_channel(1);
        let (release_tx, release_rx) = sync_channel(1);
        let mut keyframes = KeyFramesManager::new(
            Some(Box::new(BlockingKeyFrameWriter {
                ids: ids.clone(),
                started: started_tx,
                release: Arc::new(Mutex::new(release_rx)),
            })),
            1,
        )
        .unwrap();
        let freeze_calls = Cell::new(0);
        let snapshot = CountingSnapshot {
            calls: &freeze_calls,
            value: 42,
            fail: false,
        };
        keyframes.begin_capture_preallocation();
        keyframes.include_capture_capacity(&snapshot).unwrap();
        keyframes.finish_capture_preallocation().unwrap();
        freeze_calls.set(0);

        keyframes.try_reset(0, &RobotClock::default()).unwrap();
        assert_ne!(keyframes.freeze_task(0, &snapshot).unwrap(), 0);
        keyframes.end_of_processing(0).unwrap();
        started_rx
            .recv_timeout(std::time::Duration::from_secs(1))
            .unwrap();

        keyframes.try_reset(1, &RobotClock::default()).unwrap();
        assert_ne!(keyframes.freeze_task(1, &snapshot).unwrap(), 0);
        keyframes.end_of_processing(1).unwrap();

        keyframes.try_reset(2, &RobotClock::default()).unwrap();
        assert_eq!(keyframes.freeze_task(2, &snapshot).unwrap(), 0);
        keyframes.end_of_processing(2).unwrap();

        assert_eq!(freeze_calls.get(), 2);
        assert_eq!(keyframes.dropped_keyframes_total(), 1);
        assert_eq!(*ids.lock().unwrap(), vec![0]);

        release_tx.send(()).unwrap();
        started_rx
            .recv_timeout(std::time::Duration::from_secs(1))
            .unwrap();
        release_tx.send(()).unwrap();
        keyframes.finish_pending().unwrap();
        assert_eq!(*ids.lock().unwrap(), vec![0, 1]);
    }

    #[cfg(all(feature = "std", feature = "async-cl-io"))]
    #[test]
    fn test_async_copperlists_manager_flushes_in_order() {
        let ids = Arc::new(Mutex::new(Vec::new()));
        let mut copperlists = CopperListsManager::<Msgs, 5>::new(Some(Box::new(RecordingWriter {
            ids: ids.clone(),
        })))
        .unwrap();

        for expected_id in 0..4 {
            let culist = copperlists.create().unwrap();
            assert_eq!(culist.id, expected_id);
            culist.change_state(CopperListState::Processing);
            copperlists.end_of_processing(expected_id).unwrap();
        }

        copperlists.finish_pending().unwrap();
        assert_eq!(copperlists.available_copper_lists().unwrap(), 5);
        assert_eq!(*ids.lock().unwrap(), vec![0, 1, 2, 3]);
        assert_eq!(copperlists.dropped_copperlists_total(), 0);
    }

    #[cfg(all(feature = "std", feature = "async-cl-io"))]
    #[test]
    fn test_async_handoff_drops_without_exhausting_execution_slot() {
        let ids = Arc::new(Mutex::new(Vec::new()));
        let (started_tx, started_rx) = sync_channel(1);
        let (release_tx, release_rx) = sync_channel(1);
        let mut copperlists = CopperListsManager::<Msgs, 2>::new(Some(Box::new(BlockingWriter {
            ids: ids.clone(),
            started: started_tx,
            release: Arc::new(Mutex::new(release_rx)),
        })))
        .unwrap();

        let first = copperlists.create().unwrap();
        first.change_state(CopperListState::Processing);
        copperlists.end_of_processing(0).unwrap();
        started_rx
            .recv_timeout(std::time::Duration::from_secs(1))
            .unwrap();

        let second = copperlists.create().unwrap();
        second.change_state(CopperListState::Processing);
        copperlists.end_of_processing(1).unwrap();

        assert_eq!(copperlists.dropped_copperlists_total(), 1);
        assert_eq!(copperlists.available_copper_lists().unwrap(), 1);
        assert_eq!(*ids.lock().unwrap(), vec![0]);

        release_tx.send(()).unwrap();
        copperlists.finish_pending().unwrap();
        assert_eq!(copperlists.available_copper_lists().unwrap(), 2);
    }

    #[cfg(all(feature = "std", feature = "async-cl-io"))]
    #[test]
    fn test_async_boxed_handoff_returns_dropped_copperlist_to_caller() {
        let ids = Arc::new(Mutex::new(Vec::new()));
        let (started_tx, started_rx) = sync_channel(1);
        let (release_tx, release_rx) = sync_channel(1);
        let mut copperlists = CopperListsManager::<Msgs, 2>::new(Some(Box::new(BlockingWriter {
            ids: ids.clone(),
            started: started_tx,
            release: Arc::new(Mutex::new(release_rx)),
        })))
        .unwrap();

        let mut first = Box::new(CopperList::new(0, Msgs::default()));
        first.change_state(CopperListState::Processing);
        assert!(matches!(
            copperlists.end_of_processing_boxed(first).unwrap(),
            OwnedCopperListSubmission::Pending
        ));
        started_rx
            .recv_timeout(std::time::Duration::from_secs(1))
            .unwrap();

        let mut second = Box::new(CopperList::new(1, Msgs::default()));
        second.change_state(CopperListState::Processing);
        let recycled = match copperlists.end_of_processing_boxed(second).unwrap() {
            OwnedCopperListSubmission::Recycled(culist) => culist,
            OwnedCopperListSubmission::Pending => panic!("saturated handoff accepted CopperList"),
        };

        assert_eq!(recycled.id, 1);
        assert_eq!(recycled.get_state(), CopperListState::Free);
        assert_eq!(copperlists.dropped_copperlists_total(), 1);
        assert_eq!(*ids.lock().unwrap(), vec![0]);

        release_tx.send(()).unwrap();
        let completed = copperlists.finish_pending_boxed().unwrap();
        assert_eq!(completed.len(), 1);
        assert_eq!(completed[0].id, 0);
    }

    #[cfg(all(feature = "std", feature = "async-cl-io"))]
    #[test]
    fn test_async_output_requires_a_spare_execution_slot() {
        let error =
            match CopperListsManager::<Msgs, 1>::new(Some(Box::new(RecordingWriter::default()))) {
                Ok(_) => panic!("async output unexpectedly accepted a single CopperList slot"),
                Err(error) => error,
            };

        assert!(error.to_string().contains("at least two CopperList slots"));
    }

    #[cfg(all(feature = "std", feature = "async-cl-io"))]
    #[test]
    fn test_async_create_reinitializes_reclaimed_slot_state_but_preserves_payload_storage() {
        let mut copperlists = CopperListsManager::<IntMsgs, 1>::new(None).unwrap();

        {
            let culist = copperlists.create().unwrap();
            assert_eq!(culist.id, 0);
            assert_eq!(culist.get_state(), CopperListState::Initialized);
            culist.msgs.0 = 41;
            culist.change_state(CopperListState::Processing);
        }

        copperlists.end_of_processing(0).unwrap();
        assert_eq!(copperlists.available_copper_lists().unwrap(), 1);

        let reused = copperlists.create().unwrap();
        assert_eq!(reused.id, 1);
        assert_eq!(reused.get_state(), CopperListState::Initialized);
        assert_eq!(reused.msgs.0, 41);
    }

    #[cfg(all(feature = "std", feature = "async-cl-io", debug_assertions))]
    #[test]
    #[should_panic(expected = "async end_of_processing expected CopperList #0 to be Processing")]
    fn test_async_end_of_processing_wrong_state_panics_in_debug() {
        let mut copperlists = CopperListsManager::<IntMsgs, 1>::new(None).unwrap();

        let culist = copperlists.create().unwrap();
        assert_eq!(culist.id, 0);
        assert_eq!(culist.get_state(), CopperListState::Initialized);

        let _ = copperlists.end_of_processing(0);
    }

    #[test]
    fn test_runtime_task_input_order() {
        let mut config = CuConfig::default();
        let graph = config.get_graph_mut(None).unwrap();
        let src1_id = graph.add_node(Node::new("a", "Source1")).unwrap();
        let src2_id = graph.add_node(Node::new("b", "Source2")).unwrap();
        let sink_id = graph.add_node(Node::new("c", "Sink")).unwrap();

        assert_eq!(src1_id, 0);
        assert_eq!(src2_id, 1);

        // note that the source2 connection is before the source1
        let src1_type = "src1_type";
        let src2_type = "src2_type";
        graph.connect(src2_id, sink_id, src2_type).unwrap();
        graph.connect(src1_id, sink_id, src1_type).unwrap();

        let src1_edge_id = *graph.get_src_edges(src1_id).unwrap().first().unwrap();
        let src2_edge_id = *graph.get_src_edges(src2_id).unwrap().first().unwrap();
        // the edge id depends on the order the connection is created, not
        // on the node id, and that is what determines the input order
        assert_eq!(src1_edge_id, 1);
        assert_eq!(src2_edge_id, 0);

        let runtime = compute_runtime_plan(graph).unwrap();
        let sink_step = runtime
            .steps
            .iter()
            .find_map(|step| match step {
                CuExecutionUnit::Step(step) if step.node_id == sink_id => Some(step),
                _ => None,
            })
            .unwrap();

        // since the src2 connection was added before src1 connection, the src2 type should be
        // first
        assert_eq!(sink_step.input_msg_indices_types[0].msg_type, src2_type);
        assert_eq!(sink_step.input_msg_indices_types[1].msg_type, src1_type);
    }

    #[test]
    fn test_runtime_output_ports_unique_ordered() {
        let mut config = CuConfig::default();
        let graph = config.get_graph_mut(None).unwrap();
        let src_id = graph.add_node(Node::new("src", "Source")).unwrap();
        let dst_a_id = graph.add_node(Node::new("dst_a", "SinkA")).unwrap();
        let dst_b_id = graph.add_node(Node::new("dst_b", "SinkB")).unwrap();
        let dst_a2_id = graph.add_node(Node::new("dst_a2", "SinkA2")).unwrap();
        let dst_c_id = graph.add_node(Node::new("dst_c", "SinkC")).unwrap();

        graph.connect(src_id, dst_a_id, "msg::A").unwrap();
        graph.connect(src_id, dst_b_id, "msg::B").unwrap();
        graph.connect(src_id, dst_a2_id, "msg::A").unwrap();
        graph.connect(src_id, dst_c_id, "msg::C").unwrap();

        let runtime = compute_runtime_plan(graph).unwrap();
        let src_step = runtime
            .steps
            .iter()
            .find_map(|step| match step {
                CuExecutionUnit::Step(step) if step.node_id == src_id => Some(step),
                _ => None,
            })
            .unwrap();

        let output_pack = src_step.output_msg_pack.as_ref().unwrap();
        assert_eq!(output_pack.msg_types, vec!["msg::A", "msg::B", "msg::C"]);

        let dst_a_step = runtime
            .steps
            .iter()
            .find_map(|step| match step {
                CuExecutionUnit::Step(step) if step.node_id == dst_a_id => Some(step),
                _ => None,
            })
            .unwrap();
        let dst_b_step = runtime
            .steps
            .iter()
            .find_map(|step| match step {
                CuExecutionUnit::Step(step) if step.node_id == dst_b_id => Some(step),
                _ => None,
            })
            .unwrap();
        let dst_a2_step = runtime
            .steps
            .iter()
            .find_map(|step| match step {
                CuExecutionUnit::Step(step) if step.node_id == dst_a2_id => Some(step),
                _ => None,
            })
            .unwrap();
        let dst_c_step = runtime
            .steps
            .iter()
            .find_map(|step| match step {
                CuExecutionUnit::Step(step) if step.node_id == dst_c_id => Some(step),
                _ => None,
            })
            .unwrap();

        assert_eq!(dst_a_step.input_msg_indices_types[0].src_port, 0);
        assert_eq!(dst_b_step.input_msg_indices_types[0].src_port, 1);
        assert_eq!(dst_a2_step.input_msg_indices_types[0].src_port, 0);
        assert_eq!(dst_c_step.input_msg_indices_types[0].src_port, 2);
    }

    #[test]
    fn test_runtime_output_ports_fanout_single() {
        let mut config = CuConfig::default();
        let graph = config.get_graph_mut(None).unwrap();
        let src_id = graph.add_node(Node::new("src", "Source")).unwrap();
        let dst_a_id = graph.add_node(Node::new("dst_a", "SinkA")).unwrap();
        let dst_b_id = graph.add_node(Node::new("dst_b", "SinkB")).unwrap();

        graph.connect(src_id, dst_a_id, "i32").unwrap();
        graph.connect(src_id, dst_b_id, "i32").unwrap();

        let runtime = compute_runtime_plan(graph).unwrap();
        let src_step = runtime
            .steps
            .iter()
            .find_map(|step| match step {
                CuExecutionUnit::Step(step) if step.node_id == src_id => Some(step),
                _ => None,
            })
            .unwrap();

        let output_pack = src_step.output_msg_pack.as_ref().unwrap();
        assert_eq!(output_pack.msg_types, vec!["i32"]);
    }

    #[test]
    fn test_runtime_output_ports_include_nc_outputs() {
        let mut config = CuConfig::default();
        let graph = config.get_graph_mut(None).unwrap();
        let src_id = graph.add_node(Node::new("src", "Source")).unwrap();
        let dst_id = graph.add_node(Node::new("dst", "Sink")).unwrap();
        graph.connect(src_id, dst_id, "msg::A").unwrap();
        graph
            .get_node_mut(src_id)
            .expect("missing source node")
            .add_nc_output("msg::B", usize::MAX);

        let runtime = compute_runtime_plan(graph).unwrap();
        let src_step = runtime
            .steps
            .iter()
            .find_map(|step| match step {
                CuExecutionUnit::Step(step) if step.node_id == src_id => Some(step),
                _ => None,
            })
            .unwrap();
        let dst_step = runtime
            .steps
            .iter()
            .find_map(|step| match step {
                CuExecutionUnit::Step(step) if step.node_id == dst_id => Some(step),
                _ => None,
            })
            .unwrap();

        let output_pack = src_step.output_msg_pack.as_ref().unwrap();
        assert_eq!(output_pack.msg_types, vec!["msg::A", "msg::B"]);
        assert_eq!(dst_step.input_msg_indices_types[0].src_port, 0);
    }

    #[test]
    fn test_runtime_plan_infers_regular_task_when_outputs_are_nc_only() {
        let txt = r#"(
            tasks: [
                (id: "src", type: "a"),
                (id: "regular", type: "b"),
            ],
            cnx: [
                (src: "src", dst: "regular", msg: "msg::A"),
                (src: "regular", dst: "__nc__", msg: "msg::B"),
            ]
        )"#;
        let config = CuConfig::deserialize_ron(txt).unwrap();
        let graph = config.get_graph(None).unwrap();
        let regular_id = graph.get_node_id_by_name("regular").unwrap();

        let runtime = compute_runtime_plan(graph).unwrap();
        let regular_step = runtime
            .steps
            .iter()
            .find_map(|step| match step {
                CuExecutionUnit::Step(step) if step.node_id == regular_id => Some(step),
                _ => None,
            })
            .unwrap();

        assert_eq!(regular_step.task_type, CuTaskType::Regular);
        assert_eq!(
            regular_step.output_msg_pack.as_ref().unwrap().msg_types,
            vec!["msg::B"]
        );
    }

    #[test]
    fn test_runtime_output_ports_respect_connection_order_with_nc() {
        let txt = r#"(
            tasks: [(id: "src", type: "a"), (id: "sink", type: "b")],
            cnx: [
                (src: "src", dst: "__nc__", msg: "msg::A"),
                (src: "src", dst: "sink", msg: "msg::B"),
            ]
        )"#;
        let config = CuConfig::deserialize_ron(txt).unwrap();
        let graph = config.get_graph(None).unwrap();
        let src_id = graph.get_node_id_by_name("src").unwrap();
        let dst_id = graph.get_node_id_by_name("sink").unwrap();

        let runtime = compute_runtime_plan(graph).unwrap();
        let src_step = runtime
            .steps
            .iter()
            .find_map(|step| match step {
                CuExecutionUnit::Step(step) if step.node_id == src_id => Some(step),
                _ => None,
            })
            .unwrap();
        let dst_step = runtime
            .steps
            .iter()
            .find_map(|step| match step {
                CuExecutionUnit::Step(step) if step.node_id == dst_id => Some(step),
                _ => None,
            })
            .unwrap();

        let output_pack = src_step.output_msg_pack.as_ref().unwrap();
        assert_eq!(output_pack.msg_types, vec!["msg::A", "msg::B"]);
        assert_eq!(dst_step.input_msg_indices_types[0].src_port, 1);
    }

    #[cfg(feature = "std")]
    #[test]
    fn test_runtime_output_ports_respect_connection_order_with_nc_from_file() {
        let txt = r#"(
            tasks: [(id: "src", type: "a"), (id: "sink", type: "b")],
            cnx: [
                (src: "src", dst: "__nc__", msg: "msg::A"),
                (src: "src", dst: "sink", msg: "msg::B"),
            ]
        )"#;
        let tmp = tempfile::NamedTempFile::new().unwrap();
        std::fs::write(tmp.path(), txt).unwrap();
        let config = crate::config::read_configuration(tmp.path().to_str().unwrap()).unwrap();
        let graph = config.get_graph(None).unwrap();
        let src_id = graph.get_node_id_by_name("src").unwrap();
        let dst_id = graph.get_node_id_by_name("sink").unwrap();

        let runtime = compute_runtime_plan(graph).unwrap();
        let src_step = runtime
            .steps
            .iter()
            .find_map(|step| match step {
                CuExecutionUnit::Step(step) if step.node_id == src_id => Some(step),
                _ => None,
            })
            .unwrap();
        let dst_step = runtime
            .steps
            .iter()
            .find_map(|step| match step {
                CuExecutionUnit::Step(step) if step.node_id == dst_id => Some(step),
                _ => None,
            })
            .unwrap();

        let output_pack = src_step.output_msg_pack.as_ref().unwrap();
        assert_eq!(output_pack.msg_types, vec!["msg::A", "msg::B"]);
        assert_eq!(dst_step.input_msg_indices_types[0].src_port, 1);
    }

    #[test]
    fn test_runtime_output_ports_respect_connection_order_with_nc_primitives() {
        let txt = r#"(
            tasks: [(id: "src", type: "a"), (id: "sink", type: "b")],
            cnx: [
                (src: "src", dst: "__nc__", msg: "i32"),
                (src: "src", dst: "sink", msg: "bool"),
            ]
        )"#;
        let config = CuConfig::deserialize_ron(txt).unwrap();
        let graph = config.get_graph(None).unwrap();
        let src_id = graph.get_node_id_by_name("src").unwrap();
        let dst_id = graph.get_node_id_by_name("sink").unwrap();

        let runtime = compute_runtime_plan(graph).unwrap();
        let src_step = runtime
            .steps
            .iter()
            .find_map(|step| match step {
                CuExecutionUnit::Step(step) if step.node_id == src_id => Some(step),
                _ => None,
            })
            .unwrap();
        let dst_step = runtime
            .steps
            .iter()
            .find_map(|step| match step {
                CuExecutionUnit::Step(step) if step.node_id == dst_id => Some(step),
                _ => None,
            })
            .unwrap();

        let output_pack = src_step.output_msg_pack.as_ref().unwrap();
        assert_eq!(output_pack.msg_types, vec!["i32", "bool"]);
        assert_eq!(dst_step.input_msg_indices_types[0].src_port, 1);
    }

    #[test]
    fn test_runtime_plan_diamond_case1() {
        // more complex topology that tripped the scheduler
        let mut config = CuConfig::default();
        let graph = config.get_graph_mut(None).unwrap();
        let cam0_id = graph
            .add_node(Node::new("cam0", "tasks::IntegerSrcTask"))
            .unwrap();
        let inf0_id = graph
            .add_node(Node::new("inf0", "tasks::Integer2FloatTask"))
            .unwrap();
        let broadcast_id = graph
            .add_node(Node::new("broadcast", "tasks::MergingSinkTask"))
            .unwrap();

        // case 1 order
        graph.connect(cam0_id, broadcast_id, "i32").unwrap();
        graph.connect(cam0_id, inf0_id, "i32").unwrap();
        graph.connect(inf0_id, broadcast_id, "f32").unwrap();

        let edge_cam0_to_broadcast = *graph.get_src_edges(cam0_id).unwrap().first().unwrap();
        let edge_cam0_to_inf0 = graph.get_src_edges(cam0_id).unwrap()[1];

        assert_eq!(edge_cam0_to_inf0, 0);
        assert_eq!(edge_cam0_to_broadcast, 1);

        let runtime = compute_runtime_plan(graph).unwrap();
        let broadcast_step = runtime
            .steps
            .iter()
            .find_map(|step| match step {
                CuExecutionUnit::Step(step) if step.node_id == broadcast_id => Some(step),
                _ => None,
            })
            .unwrap();

        assert_eq!(broadcast_step.input_msg_indices_types[0].msg_type, "i32");
        assert_eq!(broadcast_step.input_msg_indices_types[1].msg_type, "f32");
    }

    #[test]
    fn test_runtime_plan_diamond_case2() {
        // more complex topology that tripped the scheduler variation 2
        let mut config = CuConfig::default();
        let graph = config.get_graph_mut(None).unwrap();
        let cam0_id = graph
            .add_node(Node::new("cam0", "tasks::IntegerSrcTask"))
            .unwrap();
        let inf0_id = graph
            .add_node(Node::new("inf0", "tasks::Integer2FloatTask"))
            .unwrap();
        let broadcast_id = graph
            .add_node(Node::new("broadcast", "tasks::MergingSinkTask"))
            .unwrap();

        // case 2 order
        graph.connect(cam0_id, inf0_id, "i32").unwrap();
        graph.connect(cam0_id, broadcast_id, "i32").unwrap();
        graph.connect(inf0_id, broadcast_id, "f32").unwrap();

        let edge_cam0_to_inf0 = *graph.get_src_edges(cam0_id).unwrap().first().unwrap();
        let edge_cam0_to_broadcast = graph.get_src_edges(cam0_id).unwrap()[1];

        assert_eq!(edge_cam0_to_broadcast, 0);
        assert_eq!(edge_cam0_to_inf0, 1);

        let runtime = compute_runtime_plan(graph).unwrap();
        let broadcast_step = runtime
            .steps
            .iter()
            .find_map(|step| match step {
                CuExecutionUnit::Step(step) if step.node_id == broadcast_id => Some(step),
                _ => None,
            })
            .unwrap();

        assert_eq!(broadcast_step.input_msg_indices_types[0].msg_type, "i32");
        assert_eq!(broadcast_step.input_msg_indices_types[1].msg_type, "f32");
    }

    // --- anytime plan expansion ---

    use crate::config::AnytimeConfig;

    fn anytime_node(id: &str, max_refines: Option<u32>) -> Node {
        let mut node = Node::new(id, "tasks::AnytimeTask");
        node.set_anytime(Some(AnytimeConfig {
            max_refines,
            ..Default::default()
        }));
        node
    }

    /// Renders the plan as `(node_id, phase)` pairs for compact assertions.
    fn plan_shape(plan: &CuExecutionLoop) -> Vec<(NodeId, CuStepPhase)> {
        plan.steps
            .iter()
            .map(|unit| match unit {
                CuExecutionUnit::Step(step) => (step.node_id, step.phase),
                CuExecutionUnit::Loop(_) => panic!("no loops expected"),
            })
            .collect()
    }

    /// A manual step, bypassing the planner heuristic so gap placement is
    /// deterministic: `inputs`/`output` are copperlist indices.
    fn manual_step(node: Node, node_id: NodeId, inputs: &[u32], output: u32) -> CuExecutionUnit {
        CuExecutionUnit::Step(Box::new(CuExecutionStep {
            node_id,
            node,
            task_type: CuTaskType::Regular,
            phase: CuStepPhase::default(),
            input_msg_indices_types: inputs
                .iter()
                .map(|&culist_index| CuInputMsg {
                    culist_index,
                    msg_type: "msg::A".to_string(),
                    src_port: 0,
                    edge_id: 0,
                    connection_order: 0,
                })
                .collect(),
            output_msg_pack: Some(CuOutputPack {
                culist_index: output,
                msg_types: vec!["msg::A".to_string()],
            }),
        }))
    }

    #[test]
    fn test_anytime_expansion_contiguous_without_gap() {
        // src -> any -> sink through the real planner: no independent steps
        // between the node and its consumer, so every refine sits before it.
        let mut config = CuConfig::default();
        let graph = config.get_graph_mut(None).unwrap();
        let src_id = graph.add_node(Node::new("src", "tasks::Src")).unwrap();
        let any_id = graph.add_node(anytime_node("any", Some(3))).unwrap();
        let sink_id = graph.add_node(Node::new("sink", "tasks::Sink")).unwrap();
        graph.connect(src_id, any_id, "msg::A").unwrap();
        graph.connect(any_id, sink_id, "msg::B").unwrap();

        let mut plan = compute_runtime_plan(graph).unwrap();
        expand_anytime_steps(&mut plan).unwrap();

        assert_eq!(
            plan_shape(&plan),
            vec![
                (src_id, CuStepPhase::Whole),
                (any_id, CuStepPhase::AnytimeBase),
                (any_id, CuStepPhase::AnytimeRefine),
                (any_id, CuStepPhase::AnytimeRefine),
                (any_id, CuStepPhase::AnytimeRefine),
                (sink_id, CuStepPhase::Whole),
            ]
        );

        // Refine steps read no input and write the base step's output slot.
        let (base_pack, refine_steps): (Option<CuOutputPack>, Vec<&CuExecutionStep>) = {
            let mut base_pack = None;
            let mut refines = Vec::new();
            for unit in &plan.steps {
                if let CuExecutionUnit::Step(step) = unit {
                    match step.phase {
                        CuStepPhase::AnytimeBase => base_pack = step.output_msg_pack.clone(),
                        CuStepPhase::AnytimeRefine => refines.push(step.as_ref()),
                        CuStepPhase::Whole => {}
                    }
                }
            }
            (base_pack, refines)
        };
        let base_pack = base_pack.unwrap();
        for refine in refine_steps {
            assert!(refine.input_msg_indices_types.is_empty());
            let pack = refine.output_msg_pack.as_ref().unwrap();
            assert_eq!(pack.culist_index, base_pack.culist_index);
        }
    }

    #[test]
    fn test_anytime_expansion_interleaves_with_gap_steps() {
        // Manual plan: [any(base at 1), gapA, gapB, consumer], R = 4.
        // Expected: base, r, gapA, r, gapB, r, r, consumer.
        let mut plan = CuExecutionLoop {
            steps: vec![
                manual_step(anytime_node("any", Some(4)), 0, &[], 0),
                manual_step(Node::new("gap_a", "t"), 1, &[], 1),
                manual_step(Node::new("gap_b", "t"), 2, &[], 2),
                manual_step(Node::new("consumer", "t"), 3, &[0], 3),
            ],
            loop_count: None,
        };
        expand_anytime_steps(&mut plan).unwrap();
        assert_eq!(
            plan_shape(&plan),
            vec![
                (0, CuStepPhase::AnytimeBase),
                (0, CuStepPhase::AnytimeRefine),
                (1, CuStepPhase::Whole),
                (0, CuStepPhase::AnytimeRefine),
                (2, CuStepPhase::Whole),
                (0, CuStepPhase::AnytimeRefine),
                (0, CuStepPhase::AnytimeRefine),
                (3, CuStepPhase::Whole),
            ]
        );
    }

    #[test]
    fn test_anytime_expansion_fewer_refines_than_gaps() {
        // R = 2 with two gap steps: later gap steps get no quantum after them.
        let mut plan = CuExecutionLoop {
            steps: vec![
                manual_step(anytime_node("any", Some(2)), 0, &[], 0),
                manual_step(Node::new("gap_a", "t"), 1, &[], 1),
                manual_step(Node::new("gap_b", "t"), 2, &[], 2),
                manual_step(Node::new("consumer", "t"), 3, &[0], 3),
            ],
            loop_count: None,
        };
        expand_anytime_steps(&mut plan).unwrap();
        assert_eq!(
            plan_shape(&plan),
            vec![
                (0, CuStepPhase::AnytimeBase),
                (0, CuStepPhase::AnytimeRefine),
                (1, CuStepPhase::Whole),
                (0, CuStepPhase::AnytimeRefine),
                (2, CuStepPhase::Whole),
                (3, CuStepPhase::Whole),
            ]
        );
    }

    #[test]
    fn test_anytime_expansion_without_consumer() {
        // No step consumes the node's output: refines sit right after the base.
        let mut plan = CuExecutionLoop {
            steps: vec![
                manual_step(anytime_node("any", Some(2)), 0, &[], 0),
                manual_step(Node::new("other", "t"), 1, &[], 1),
            ],
            loop_count: None,
        };
        expand_anytime_steps(&mut plan).unwrap();
        assert_eq!(
            plan_shape(&plan),
            vec![
                (0, CuStepPhase::AnytimeBase),
                (0, CuStepPhase::AnytimeRefine),
                (0, CuStepPhase::AnytimeRefine),
                (1, CuStepPhase::Whole),
            ]
        );
    }

    #[test]
    fn test_anytime_expansion_two_nodes_interleave() {
        // Two anytime nodes: each expands independently; the second node's base
        // and quanta land in the first node's gap and vice versa.
        let mut plan = CuExecutionLoop {
            steps: vec![
                manual_step(anytime_node("any_a", Some(2)), 0, &[], 0),
                manual_step(anytime_node("any_b", Some(2)), 1, &[], 1),
                manual_step(Node::new("consumer", "t"), 2, &[0, 1], 2),
            ],
            loop_count: None,
        };
        expand_anytime_steps(&mut plan).unwrap();
        // A expands first: base_a, r_a, [b], r_a, consumer. B then expands in
        // place, treating A's second quantum as its gap step.
        assert_eq!(
            plan_shape(&plan),
            vec![
                (0, CuStepPhase::AnytimeBase),
                (0, CuStepPhase::AnytimeRefine),
                (1, CuStepPhase::AnytimeBase),
                (1, CuStepPhase::AnytimeRefine),
                (0, CuStepPhase::AnytimeRefine),
                (1, CuStepPhase::AnytimeRefine),
                (2, CuStepPhase::Whole),
            ]
        );
    }

    #[test]
    fn test_anytime_expansion_requires_max_refines() {
        let mut plan = CuExecutionLoop {
            steps: vec![
                manual_step(anytime_node("any", None), 0, &[], 0),
                manual_step(Node::new("consumer", "t"), 1, &[0], 1),
            ],
            loop_count: None,
        };
        let err = expand_anytime_steps(&mut plan).unwrap_err();
        assert!(err.to_string().contains("needs anytime.max_refines"));
    }
}
