//! CuRuntime is the heart of what copper is running on the robot.
//! It is exposed to the user via the `copper_runtime` macro injecting it as a field in their application struct.
//!

use crate::app::Subsystem;
use crate::config::{ComponentConfig, CuDirection, DEFAULT_KEYFRAME_INTERVAL, Node, TaskKind};
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
use crate::resource::ResourceManager;
use compact_str::CompactString;
use cu29_clock::{ClockProvider, CuDuration, CuTime, RobotClock};
use cu29_traits::CuResult;
use cu29_traits::WriteStream;
use cu29_traits::{CopperListTuple, CuError};

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
use alloc::collections::{BTreeSet, VecDeque};
use alloc::format;
use alloc::string::{String, ToString};
use alloc::vec::Vec;
use bincode::enc::EncoderImpl;
use bincode::enc::write::{SizeWriter, SliceWriter};
use bincode::error::EncodeError;
use bincode::{Decode, Encode};
#[cfg(all(feature = "std", any(feature = "async-cl-io", feature = "parallel-rt")))]
use core::alloc::Layout;
use core::fmt::Result as FmtResult;
use core::fmt::{Debug, Formatter};
use core::marker::PhantomData;

#[cfg(all(feature = "std", feature = "async-cl-io"))]
use std::sync::mpsc::{Receiver, SyncSender, TryRecvError, sync_channel};
#[cfg(all(feature = "std", feature = "async-cl-io"))]
use std::thread::JoinHandle;

pub type TasksInstantiator<CT> =
    for<'c> fn(Vec<Option<&'c ComponentConfig>>, &mut ResourceManager) -> CuResult<CT>;
pub type BridgesInstantiator<CB> = fn(&CuConfig, &mut ResourceManager) -> CuResult<CB>;
pub type MonitorInstantiator<M> = fn(&CuConfig, CuMonitoringMetadata, CuMonitoringRuntime) -> M;

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
    CLW,
    KFW,
> {
    clock: RobotClock,
    config: &'cfg CuConfig,
    mission: &'cfg str,
    subsystem: Subsystem,
    instance_id: u32,
    resources: Option<ResourceManager>,
    parts: CuRuntimeParts<CT, CB, P, M, NBCL, TI, BI, MI>,
    copperlists_logger: CLW,
    keyframes_logger: KFW,
}

impl<'cfg, CT, CB, P: CopperListTuple, M: CuMonitor, const NBCL: usize, TI, BI, MI, CLW, KFW>
    CuRuntimeBuilder<'cfg, CT, CB, P, M, NBCL, TI, BI, MI, CLW, KFW>
{
    pub fn new(
        clock: RobotClock,
        config: &'cfg CuConfig,
        mission: &'cfg str,
        parts: CuRuntimeParts<CT, CB, P, M, NBCL, TI, BI, MI>,
        copperlists_logger: CLW,
        keyframes_logger: KFW,
    ) -> Self {
        Self {
            clock,
            config,
            mission,
            subsystem: Subsystem::new(None, 0),
            instance_id: 0,
            resources: None,
            parts,
            copperlists_logger,
            keyframes_logger,
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
pub enum ProcessStepOutcome {
    Continue,
    AbortCopperList,
}

/// Result type used by generated process-step functions.
pub type ProcessStepResult = CuResult<ProcessStepOutcome>;

#[cfg(feature = "remote-debug")]
fn encode_completed_copperlist_snapshot<P: CopperListTuple>(
    cl: &CopperList<P>,
) -> CuResult<Vec<u8>> {
    bincode::encode_to_vec(cl, bincode::config::standard())
        .map_err(|e| CuError::new_with_cause("Failed to encode completed CopperList snapshot", e))
}

/// Manages the lifecycle of the copper lists and logging on the synchronous path.
pub struct SyncCopperListsManager<P: CopperListTuple + Default, const NBCL: usize> {
    inner: CuListsManager<P, NBCL>,
    /// Logger for the copper lists (messages between tasks)
    logger: Option<Box<dyn WriteStream<CopperList<P>>>>,
    /// Remote-debug snapshot of the most recently completed CopperList.
    #[cfg(feature = "remote-debug")]
    last_completed_encoded: Option<Vec<u8>>,
    /// Last encoded size returned by logger.log
    pub last_encoded_bytes: u64,
    /// Last handle-backed payload bytes observed during logger.log
    pub last_handle_bytes: u64,
}

impl<P: CopperListTuple + Default, const NBCL: usize> SyncCopperListsManager<P, NBCL> {
    pub fn new(logger: Option<Box<dyn WriteStream<CopperList<P>>>>) -> CuResult<Self>
    where
        P: CuListZeroedInit,
    {
        Ok(Self {
            inner: CuListsManager::new(),
            logger,
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

    pub fn create(&mut self) -> CuResult<&mut CopperList<P>> {
        self.inner
            .create()
            .ok_or_else(|| CuError::from("Ran out of space for copper lists"))
    }

    pub fn end_of_processing(&mut self, culistid: u64) -> CuResult<()> {
        let mut is_top = true;
        let mut nb_done = 0;
        self.last_handle_bytes = 0;
        #[cfg(feature = "remote-debug")]
        let last_completed_encoded = &mut self.last_completed_encoded;
        for cl in self.inner.iter_mut() {
            if cl.id == culistid && cl.get_state() == CopperListState::Processing {
                cl.change_state(CopperListState::DoneProcessing);
                match () {
                    #[cfg(feature = "remote-debug")]
                    () => {
                        *last_completed_encoded = Some(encode_completed_copperlist_snapshot(cl)?);
                    }
                    #[cfg(not(feature = "remote-debug"))]
                    () => {}
                }
            }
            if is_top && cl.get_state() == CopperListState::DoneProcessing {
                if let Some(logger) = &mut self.logger {
                    cl.change_state(CopperListState::BeingSerialized);
                    logger.log(cl)?;
                    self.last_encoded_bytes = logger.last_log_bytes().unwrap_or(0) as u64;
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

    #[cfg(feature = "std")]
    pub fn end_of_processing_boxed(
        &mut self,
        mut culist: Box<CopperList<P>>,
    ) -> CuResult<OwnedCopperListSubmission<P>> {
        culist.change_state(CopperListState::DoneProcessing);
        self.last_encoded_bytes = 0;
        self.last_handle_bytes = 0;
        if let Some(logger) = &mut self.logger {
            culist.change_state(CopperListState::BeingSerialized);
            logger.log(&culist)?;
            self.last_encoded_bytes = logger.last_log_bytes().unwrap_or(0) as u64;
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
}

/// Result of handing an owned boxed CopperList to the runtime-side CL I/O path.
#[cfg(feature = "std")]
pub enum OwnedCopperListSubmission<P: CopperListTuple> {
    /// The CL has been fully handled and can be recycled immediately by the caller.
    Recycled(Box<CopperList<P>>),
    /// The CL was queued asynchronously and will be returned by a later reclaim call.
    Pending,
}

#[cfg(all(feature = "std", feature = "async-cl-io"))]
struct AsyncCopperListCompletion<P: CopperListTuple> {
    culist: Box<CopperList<P>>,
    log_result: CuResult<(u64, u64)>,
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

/// Manages the lifecycle of the copper lists and logging on the asynchronous path.
#[cfg(all(feature = "std", feature = "async-cl-io"))]
pub struct AsyncCopperListsManager<P: CopperListTuple + Default, const NBCL: usize> {
    free_pool: Vec<Box<CopperList<P>>>,
    current: Option<Box<CopperList<P>>>,
    #[cfg(feature = "remote-debug")]
    last_completed_encoded: Option<Vec<u8>>,
    pending_count: usize,
    next_cl_id: u64,
    pending_sender: Option<SyncSender<Box<CopperList<P>>>>,
    completion_receiver: Option<Receiver<AsyncCopperListCompletion<P>>>,
    worker_handle: Option<JoinHandle<()>>,
    /// Last encoded size returned by logger.log
    pub last_encoded_bytes: u64,
    /// Last handle-backed payload bytes observed during logger.log
    pub last_handle_bytes: u64,
}

#[cfg(all(feature = "std", feature = "async-cl-io"))]
impl<P: CopperListTuple + Default, const NBCL: usize> AsyncCopperListsManager<P, NBCL> {
    pub fn new(logger: Option<Box<dyn WriteStream<CopperList<P>>>>) -> CuResult<Self>
    where
        P: CuListZeroedInit + AsyncCopperListPayload + 'static,
    {
        let mut free_pool = Vec::with_capacity(NBCL);
        for _ in 0..NBCL {
            free_pool.push(allocate_zeroed_copperlist::<P>());
        }

        let (pending_sender, completion_receiver, worker_handle) = if let Some(mut logger) = logger
        {
            let (pending_sender, pending_receiver) = sync_channel::<Box<CopperList<P>>>(NBCL);
            let (completion_sender, completion_receiver) =
                sync_channel::<AsyncCopperListCompletion<P>>(NBCL);
            let worker_handle = std::thread::Builder::new()
                .name("cu-async-cl-io".to_string())
                .spawn(move || {
                    while let Ok(mut culist) = pending_receiver.recv() {
                        culist.change_state(CopperListState::BeingSerialized);
                        let log_result = logger.log(&culist).map(|_| {
                            (
                                logger.last_log_bytes().unwrap_or(0) as u64,
                                take_last_completed_handle_bytes(),
                            )
                        });
                        let should_stop = log_result.is_err();
                        if completion_sender
                            .send(AsyncCopperListCompletion { culist, log_result })
                            .is_err()
                        {
                            break;
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
            (
                Some(pending_sender),
                Some(completion_receiver),
                Some(worker_handle),
            )
        } else {
            (None, None, None)
        };

        Ok(Self {
            free_pool,
            current: None,
            #[cfg(feature = "remote-debug")]
            last_completed_encoded: None,
            pending_count: 0,
            next_cl_id: 0,
            pending_sender,
            completion_receiver,
            worker_handle,
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

    pub fn create(&mut self) -> CuResult<&mut CopperList<P>> {
        if self.current.is_some() {
            return Err(CuError::from(
                "Attempted to create a CopperList while another one is still active",
            ));
        }

        self.reclaim_completed()?;
        while self.free_pool.is_empty() {
            self.wait_for_completion()?;
        }

        let culist = self
            .free_pool
            .pop()
            .ok_or_else(|| CuError::from("Ran out of space for copper lists"))?;
        self.current = Some(culist);

        let current = self
            .current
            .as_mut()
            .expect("current CopperList is missing");
        current.id = self.next_cl_id;
        current.change_state(CopperListState::Initialized);
        self.next_cl_id += 1;
        Ok(current.as_mut())
    }

    #[cfg(feature = "remote-debug")]
    fn capture_completed_snapshot(&mut self, cl: &CopperList<P>) -> CuResult<()> {
        self.last_completed_encoded = Some(encode_completed_copperlist_snapshot(cl)?);
        Ok(())
    }

    #[cfg(not(feature = "remote-debug"))]
    fn capture_completed_snapshot(&mut self, _cl: &CopperList<P>) -> CuResult<()> {
        Ok(())
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

        culist.change_state(CopperListState::DoneProcessing);
        self.capture_completed_snapshot(&culist)?;
        self.last_encoded_bytes = 0;
        self.last_handle_bytes = 0;

        if let Some(pending_sender) = &self.pending_sender {
            culist.change_state(CopperListState::QueuedForSerialization);
            pending_sender.send(culist).map_err(|e| {
                CuError::from("Failed to enqueue CopperList for async serialization")
                    .add_cause(e.to_string().as_str())
            })?;
            self.pending_count += 1;
            self.reclaim_completed()?;
        } else {
            culist.change_state(CopperListState::Free);
            self.free_pool.push(culist);
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

    pub fn end_of_processing_boxed(
        &mut self,
        mut culist: Box<CopperList<P>>,
    ) -> CuResult<OwnedCopperListSubmission<P>> {
        self.reclaim_completed()?;
        culist.change_state(CopperListState::DoneProcessing);
        self.capture_completed_snapshot(&culist)?;
        self.last_encoded_bytes = 0;
        self.last_handle_bytes = 0;

        if let Some(pending_sender) = &self.pending_sender {
            culist.change_state(CopperListState::QueuedForSerialization);
            pending_sender.send(culist).map_err(|e| {
                CuError::from("Failed to enqueue CopperList for async serialization")
                    .add_cause(e.to_string().as_str())
            })?;
            self.pending_count += 1;
            self.reclaim_completed()?;
            Ok(OwnedCopperListSubmission::Pending)
        } else {
            culist.change_state(CopperListState::Free);
            Ok(OwnedCopperListSubmission::Recycled(culist))
        }
    }

    pub fn try_reclaim_boxed(&mut self) -> CuResult<Option<Box<CopperList<P>>>> {
        let recv_result = {
            let Some(completion_receiver) = self.completion_receiver.as_ref() else {
                return Ok(None);
            };
            completion_receiver.try_recv()
        };
        match recv_result {
            Ok(completion) => self.handle_completion(completion).map(Some),
            Err(TryRecvError::Empty) => Ok(None),
            Err(TryRecvError::Disconnected) => Err(CuError::from(
                "Async CopperList serializer thread disconnected unexpectedly",
            )),
        }
    }

    pub fn wait_reclaim_boxed(&mut self) -> CuResult<Box<CopperList<P>>> {
        let completion = self
            .completion_receiver
            .as_ref()
            .ok_or_else(|| {
                CuError::from("No async CopperList serializer is active to return a free slot")
            })?
            .recv()
            .map_err(|e| {
                CuError::from("Failed to receive completion from async CopperList serializer")
                    .add_cause(e.to_string().as_str())
            })?;
        self.handle_completion(completion)
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
        if let Ok((encoded_bytes, handle_bytes)) = completion.log_result.as_ref() {
            self.last_encoded_bytes = *encoded_bytes;
            self.last_handle_bytes = *handle_bytes;
        }
        completion.culist.change_state(CopperListState::Free);
        completion.log_result?;
        Ok(completion.culist)
    }

    fn shutdown_worker(&mut self) -> CuResult<()> {
        self.finish_pending()?;
        self.pending_sender.take();
        if let Some(worker_handle) = self.worker_handle.take() {
            worker_handle.join().map_err(|_| {
                CuError::from("Async CopperList serializer thread panicked while joining")
            })?;
        }
        Ok(())
    }
}

#[cfg(all(feature = "std", feature = "async-cl-io"))]
impl<P: CopperListTuple + Default, const NBCL: usize> Drop for AsyncCopperListsManager<P, NBCL> {
    fn drop(&mut self) {
        let _ = self.shutdown_worker();
    }
}

#[cfg(all(feature = "std", feature = "async-cl-io"))]
pub type CopperListsManager<P, const NBCL: usize> = AsyncCopperListsManager<P, NBCL>;

#[cfg(not(all(feature = "std", feature = "async-cl-io")))]
pub type CopperListsManager<P, const NBCL: usize> = SyncCopperListsManager<P, NBCL>;

/// Manages the frozen tasks state and logging.
pub struct KeyFramesManager {
    /// Where the serialized tasks are stored following the wave of execution of a CL.
    inner: KeyFrame,

    /// Optional override for the timestamp to stamp the next keyframe (used by deterministic replay).
    forced_timestamp: Option<CuTime>,

    /// If set, reuse this keyframe verbatim (e.g., during replay) instead of re-freezing state.
    locked: bool,

    /// Logger for the state of the tasks (frozen tasks)
    logger: Option<Box<dyn WriteStream<KeyFrame>>>,

    /// Capture a keyframe only each...
    keyframe_interval: u32,

    /// Bytes written by the last keyframe log
    pub last_encoded_bytes: u64,
}

impl KeyFramesManager {
    fn is_keyframe(&self, culistid: u64) -> bool {
        self.logger.is_some() && culistid.is_multiple_of(self.keyframe_interval as u64)
    }

    #[inline]
    pub fn captures_keyframe(&self, culistid: u64) -> bool {
        self.is_keyframe(culistid)
    }

    pub fn reset(&mut self, culistid: u64, clock: &RobotClock) {
        if self.is_keyframe(culistid) {
            // If a recorded keyframe was preloaded for this CL, keep it as-is.
            if self.locked && self.inner.culistid == culistid {
                return;
            }
            let ts = self.forced_timestamp.take().unwrap_or_else(|| clock.now());
            self.inner.reset(culistid, ts);
            self.locked = false;
        }
    }

    /// Force the timestamp of the next keyframe to a given value.
    #[cfg(feature = "std")]
    pub fn set_forced_timestamp(&mut self, ts: CuTime) {
        self.forced_timestamp = Some(ts);
    }

    pub fn freeze_task(&mut self, culistid: u64, task: &impl Freezable) -> CuResult<usize> {
        if self.is_keyframe(culistid) {
            if self.locked {
                // We are replaying a recorded keyframe verbatim; don't mutate it.
                return Ok(0);
            }
            if self.inner.culistid != culistid {
                return Err(CuError::from(format!(
                    "Freezing task for culistid {} but current keyframe is {}",
                    culistid, self.inner.culistid
                )));
            }
            self.inner
                .add_frozen_task(task)
                .map_err(|e| CuError::from(format!("Failed to serialize task: {e}")))
        } else {
            Ok(0)
        }
    }

    /// Generic helper to freeze any `Freezable` state (task or bridge) into the current keyframe.
    pub fn freeze_any(&mut self, culistid: u64, item: &impl Freezable) -> CuResult<usize> {
        self.freeze_task(culistid, item)
    }

    pub fn end_of_processing(&mut self, culistid: u64) -> CuResult<()> {
        if self.is_keyframe(culistid) {
            let logger = self.logger.as_mut().unwrap();
            logger.log(&self.inner)?;
            self.last_encoded_bytes = logger.last_log_bytes().unwrap_or(0) as u64;
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
        self.inner = keyframe.clone();
        self.forced_timestamp = Some(keyframe.timestamp);
        self.locked = true;
    }
}

/// This is the main structure that will be injected as a member of the Application struct.
/// CT is the tuple of all the tasks in order of execution.
/// CL is the type of the copper list, representing the input/output messages for all the tasks.
pub struct CuRuntime<CT, CB, P: CopperListTuple, M: CuMonitor, const NBCL: usize> {
    /// The base clock the runtime will be using to record time.
    pub clock: RobotClock, // TODO: remove public at some point

    /// Compile-time subsystem identity for this Copper process.
    subsystem_code: u16,

    /// Deployment/runtime instance identity for this Copper process.
    pub instance_id: u32,

    /// The tuple of all the tasks in order of execution.
    pub tasks: CT,

    /// Tuple of all instantiated bridges.
    pub bridges: CB,

    /// Resource registry kept alive for tasks borrowing shared handles.
    pub resources: ResourceManager,

    /// The runtime monitoring.
    pub monitor: M,

    /// Runtime-side execution progress probe for watchdog/diagnostic monitors.
    ///
    /// This probe is written from the generated execution plan before each component
    /// step. Monitors consume it asynchronously (typically from watchdog threads) to
    /// report the last known component/step/culist when the runtime appears stalled.
    #[cfg(feature = "std")]
    pub execution_probe: ExecutionProbeHandle,
    #[cfg(not(feature = "std"))]
    pub execution_probe: RuntimeExecutionProbe,

    /// The logger for the copper lists (messages between tasks)
    pub copperlists_manager: CopperListsManager<P, NBCL>,

    /// The logger for the state of the tasks (frozen tasks)
    pub keyframes_manager: KeyFramesManager,

    /// Feature-gated container for deterministic multi-CopperList execution.
    #[cfg(all(feature = "std", feature = "parallel-rt"))]
    pub parallel_rt: ParallelRt<NBCL>,

    /// The runtime configuration controlling the behavior of the run loop
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
    CLW,
    KFW,
> CuRuntimeBuilder<'cfg, CT, CB, P, M, NBCL, TI, BI, MI, CLW, KFW>
where
    TI: for<'c> Fn(Vec<Option<&'c ComponentConfig>>, &mut ResourceManager) -> CuResult<CT>,
    BI: Fn(&CuConfig, &mut ResourceManager) -> CuResult<CB>,
    MI: Fn(&CuConfig, CuMonitoringMetadata, CuMonitoringRuntime) -> M,
    CLW: WriteStream<CopperList<P>> + 'static,
    KFW: WriteStream<KeyFrame> + 'static,
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
            copperlists_logger,
            keyframes_logger,
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

        let (copperlists_logger, keyframes_logger, keyframe_interval) = match &config.logging {
            Some(logging_config) if logging_config.enable_task_logging => (
                Some(Box::new(copperlists_logger) as Box<dyn WriteStream<CopperList<P>>>),
                Some(Box::new(keyframes_logger) as Box<dyn WriteStream<KeyFrame>>),
                logging_config.keyframe_interval.unwrap(),
            ),
            Some(_) => (None, None, 0),
            None => (
                Some(Box::new(copperlists_logger) as Box<dyn WriteStream<CopperList<P>>>),
                Some(Box::new(keyframes_logger) as Box<dyn WriteStream<KeyFrame>>),
                DEFAULT_KEYFRAME_INTERVAL,
            ),
        };

        let copperlists_manager = CopperListsManager::new(copperlists_logger)?;
        #[cfg(target_os = "none")]
        {
            let cl_size = core::mem::size_of::<CopperList<P>>();
            let total_bytes = cl_size.saturating_mul(NBCL);
            info!(
                "CuRuntime::new: copperlists count={} cl_size={} total_bytes={}",
                NBCL, cl_size, total_bytes
            );
        }

        let keyframes_manager = KeyFramesManager {
            inner: KeyFrame::new(),
            logger: keyframes_logger,
            keyframe_interval,
            last_encoded_bytes: 0,
            forced_timestamp: None,
            locked: false,
        };
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

/// A KeyFrame is recording a snapshot of the tasks state before a given copperlist.
/// It is a double encapsulation: this one recording the culistid and another even in
/// bincode in the serialized_tasks.
#[derive(Clone, Encode, Decode)]
pub struct KeyFrame {
    // This is the id of the copper list that this keyframe is associated with (recorded before the copperlist).
    pub culistid: u64,
    // This is the timestamp when the keyframe was created, using the robot clock.
    pub timestamp: CuTime,
    // This is the bincode representation of the tuple of all the tasks.
    pub serialized_tasks: Vec<u8>,
}

impl KeyFrame {
    fn new() -> Self {
        KeyFrame {
            culistid: 0,
            timestamp: CuTime::default(),
            serialized_tasks: Vec::new(),
        }
    }

    /// This is to be able to avoid reallocations
    fn reset(&mut self, culistid: u64, timestamp: CuTime) {
        self.culistid = culistid;
        self.timestamp = timestamp;
        self.serialized_tasks.clear();
    }

    /// We need to be able to accumulate tasks to the serialization as they are executed after the step.
    fn add_frozen_task(&mut self, task: &impl Freezable) -> Result<usize, EncodeError> {
        let cfg = bincode::config::standard();
        let mut sizer = EncoderImpl::<_, _>::new(SizeWriter::default(), cfg);
        BincodeAdapter(task).encode(&mut sizer)?;
        let need = sizer.into_writer().bytes_written as usize;

        let start = self.serialized_tasks.len();
        self.serialized_tasks.resize(start + need, 0);
        let mut enc =
            EncoderImpl::<_, _>::new(SliceWriter::new(&mut self.serialized_tasks[start..]), cfg);
        BincodeAdapter(task).encode(&mut enc)?;
        Ok(need)
    }
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

    // FIXME(gbin): this became REALLY ugly with no-std
    #[allow(clippy::too_many_arguments)]
    #[cfg(feature = "std")]
    #[deprecated(note = "Use CuRuntimeBuilder instead of CuRuntime::new(...).")]
    pub fn new(
        clock: RobotClock,
        subsystem_code: u16,
        config: &CuConfig,
        mission: &str,
        resources_instanciator: impl Fn(&CuConfig) -> CuResult<ResourceManager>,
        tasks_instanciator: impl for<'c> Fn(
            Vec<Option<&'c ComponentConfig>>,
            &mut ResourceManager,
        ) -> CuResult<CT>,
        monitored_components: &'static [MonitorComponentMetadata],
        culist_component_mapping: &'static [ComponentId],
        #[cfg(all(feature = "std", feature = "parallel-rt"))]
        parallel_rt_metadata: &'static ParallelRtMetadata,
        monitor_instanciator: impl Fn(&CuConfig, CuMonitoringMetadata, CuMonitoringRuntime) -> M,
        bridges_instanciator: impl Fn(&CuConfig, &mut ResourceManager) -> CuResult<CB>,
        copperlists_logger: impl WriteStream<CopperList<P>> + 'static,
        keyframes_logger: impl WriteStream<KeyFrame> + 'static,
    ) -> CuResult<Self> {
        let parts = CuRuntimeParts::new(
            tasks_instanciator,
            monitored_components,
            culist_component_mapping,
            #[cfg(all(feature = "std", feature = "parallel-rt"))]
            parallel_rt_metadata,
            monitor_instanciator,
            bridges_instanciator,
        );
        CuRuntimeBuilder::new(
            clock,
            config,
            mission,
            parts,
            copperlists_logger,
            keyframes_logger,
        )
        .with_subsystem(Subsystem::new(None, subsystem_code))
        .try_with_resources_instantiator(resources_instanciator)?
        .build()
    }

    #[allow(clippy::too_many_arguments)]
    #[cfg(feature = "std")]
    #[deprecated(note = "Use CuRuntimeBuilder instead of CuRuntime::new_with_resources(...).")]
    pub fn new_with_resources(
        clock: RobotClock,
        subsystem_code: u16,
        config: &CuConfig,
        mission: &str,
        resources: ResourceManager,
        tasks_instanciator: impl for<'c> Fn(
            Vec<Option<&'c ComponentConfig>>,
            &mut ResourceManager,
        ) -> CuResult<CT>,
        monitored_components: &'static [MonitorComponentMetadata],
        culist_component_mapping: &'static [ComponentId],
        #[cfg(all(feature = "std", feature = "parallel-rt"))]
        parallel_rt_metadata: &'static ParallelRtMetadata,
        monitor_instanciator: impl Fn(&CuConfig, CuMonitoringMetadata, CuMonitoringRuntime) -> M,
        bridges_instanciator: impl Fn(&CuConfig, &mut ResourceManager) -> CuResult<CB>,
        copperlists_logger: impl WriteStream<CopperList<P>> + 'static,
        keyframes_logger: impl WriteStream<KeyFrame> + 'static,
    ) -> CuResult<Self> {
        let parts = CuRuntimeParts::new(
            tasks_instanciator,
            monitored_components,
            culist_component_mapping,
            #[cfg(all(feature = "std", feature = "parallel-rt"))]
            parallel_rt_metadata,
            monitor_instanciator,
            bridges_instanciator,
        );
        CuRuntimeBuilder::new(
            clock,
            config,
            mission,
            parts,
            copperlists_logger,
            keyframes_logger,
        )
        .with_subsystem(Subsystem::new(None, subsystem_code))
        .with_resources(resources)
        .build()
    }

    #[allow(clippy::too_many_arguments)]
    #[cfg(not(feature = "std"))]
    #[deprecated(note = "Use CuRuntimeBuilder instead of CuRuntime::new(...).")]
    pub fn new(
        clock: RobotClock,
        subsystem_code: u16,
        config: &CuConfig,
        mission: &str,
        resources_instanciator: impl Fn(&CuConfig) -> CuResult<ResourceManager>,
        tasks_instanciator: impl for<'c> Fn(
            Vec<Option<&'c ComponentConfig>>,
            &mut ResourceManager,
        ) -> CuResult<CT>,
        monitored_components: &'static [MonitorComponentMetadata],
        culist_component_mapping: &'static [ComponentId],
        monitor_instanciator: impl Fn(&CuConfig, CuMonitoringMetadata, CuMonitoringRuntime) -> M,
        bridges_instanciator: impl Fn(&CuConfig, &mut ResourceManager) -> CuResult<CB>,
        copperlists_logger: impl WriteStream<CopperList<P>> + 'static,
        keyframes_logger: impl WriteStream<KeyFrame> + 'static,
    ) -> CuResult<Self> {
        let parts = CuRuntimeParts::new(
            tasks_instanciator,
            monitored_components,
            culist_component_mapping,
            monitor_instanciator,
            bridges_instanciator,
        );
        CuRuntimeBuilder::new(
            clock,
            config,
            mission,
            parts,
            copperlists_logger,
            keyframes_logger,
        )
        .with_subsystem(Subsystem::new(None, subsystem_code))
        .try_with_resources_instantiator(resources_instanciator)?
        .build()
    }

    #[allow(clippy::too_many_arguments)]
    #[cfg(not(feature = "std"))]
    #[deprecated(note = "Use CuRuntimeBuilder instead of CuRuntime::new_with_resources(...).")]
    pub fn new_with_resources(
        clock: RobotClock,
        subsystem_code: u16,
        config: &CuConfig,
        mission: &str,
        resources: ResourceManager,
        tasks_instanciator: impl for<'c> Fn(
            Vec<Option<&'c ComponentConfig>>,
            &mut ResourceManager,
        ) -> CuResult<CT>,
        monitored_components: &'static [MonitorComponentMetadata],
        culist_component_mapping: &'static [ComponentId],
        monitor_instanciator: impl Fn(&CuConfig, CuMonitoringMetadata, CuMonitoringRuntime) -> M,
        bridges_instanciator: impl Fn(&CuConfig, &mut ResourceManager) -> CuResult<CB>,
        copperlists_logger: impl WriteStream<CopperList<P>> + 'static,
        keyframes_logger: impl WriteStream<KeyFrame> + 'static,
    ) -> CuResult<Self> {
        let parts = CuRuntimeParts::new(
            tasks_instanciator,
            monitored_components,
            culist_component_mapping,
            monitor_instanciator,
            bridges_instanciator,
        );
        CuRuntimeBuilder::new(
            clock,
            config,
            mission,
            parts,
            copperlists_logger,
            keyframes_logger,
        )
        .with_subsystem(Subsystem::new(None, subsystem_code))
        .with_resources(resources)
        .build()
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
}

/// This structure represents a step in the execution plan.
pub struct CuExecutionStep {
    /// NodeId: node id of the task to execute
    pub node_id: NodeId,
    /// Node: node instance
    pub node: Node,
    /// CuTaskType: type of the task
    pub task_type: CuTaskType,

    /// the indices in the copper list of the input messages and their types
    pub input_msg_indices_types: Vec<CuInputMsg>,

    /// the index in the copper list of the output message and its type
    pub output_msg_pack: Option<CuOutputPack>,
}

impl Debug for CuExecutionStep {
    fn fmt(&self, f: &mut Formatter<'_>) -> FmtResult {
        f.write_str(format!("   CuExecutionStep: Node Id: {}\n", self.node_id).as_str())?;
        f.write_str(format!("                  task_type: {:?}\n", self.node.get_type()).as_str())?;
        f.write_str(format!("                       task: {:?}\n", self.task_type).as_str())?;
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

fn find_output_pack_from_nodeid(
    node_id: NodeId,
    steps: &Vec<CuExecutionUnit>,
) -> Option<CuOutputPack> {
    for step in steps {
        match step {
            CuExecutionUnit::Loop(loop_unit) => {
                if let Some(output_pack) = find_output_pack_from_nodeid(node_id, &loop_unit.steps) {
                    return Some(output_pack);
                }
            }
            CuExecutionUnit::Step(step) => {
                if step.node_id == node_id {
                    return step.output_msg_pack.clone();
                }
            }
        }
    }
    None
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

/// The connection id used here is the index of the config graph edge that equates to the wanted
/// connection.
fn sort_inputs_by_cnx_id(input_msg_indices_types: &mut [CuInputMsg]) {
    input_msg_indices_types.sort_by_key(|input| input.edge_id);
}

/// Explores a subbranch and build the partial plan out of it.
fn plan_tasks_tree_branch(
    graph: &CuGraph,
    mut next_culist_output_index: u32,
    starting_point: NodeId,
    plan: &mut Vec<CuExecutionUnit>,
) -> CuResult<(u32, bool)> {
    #[cfg(all(feature = "std", feature = "macro_debug"))]
    eprintln!("-- starting branch from node {starting_point}");

    let mut handled = false;

    for id in graph.bfs_nodes(starting_point) {
        let node_ref = graph.get_node(id).unwrap();
        #[cfg(all(feature = "std", feature = "macro_debug"))]
        eprintln!("  Visiting node: {node_ref:?}");

        let mut input_msg_indices_types: Vec<CuInputMsg> = Vec::new();
        let output_msg_pack: Option<CuOutputPack>;
        let task_type = find_task_type_for_id(graph, id)?;

        match task_type {
            CuTaskType::Source => {
                #[cfg(all(feature = "std", feature = "macro_debug"))]
                eprintln!("    → Source node, assign output index {next_culist_output_index}");
                let msg_types = graph.get_node_output_msg_types_by_id(id)?;
                if msg_types.is_empty() {
                    return Err(CuError::from(format!(
                        "Source node '{}' has no declared outputs",
                        node_ref.get_id()
                    )));
                }
                output_msg_pack = Some(CuOutputPack {
                    culist_index: next_culist_output_index,
                    msg_types,
                });
                next_culist_output_index += 1;
            }
            CuTaskType::Sink => {
                let mut edge_ids = graph.get_dst_edges(id).unwrap_or_default();
                edge_ids.sort();
                #[cfg(all(feature = "std", feature = "macro_debug"))]
                eprintln!("    → Sink with incoming edges: {edge_ids:?}");
                for edge_id in edge_ids {
                    let edge = graph
                        .edge(edge_id)
                        .unwrap_or_else(|| panic!("Missing edge {edge_id} for node {id}"));
                    let pid = graph
                        .get_node_id_by_name(edge.src.as_str())
                        .unwrap_or_else(|| {
                            panic!("Missing source node '{}' for edge {edge_id}", edge.src)
                        });
                    let output_pack = find_output_pack_from_nodeid(pid, plan);
                    if let Some(output_pack) = output_pack {
                        #[cfg(all(feature = "std", feature = "macro_debug"))]
                        eprintln!("      ✓ Input from {pid} ready: {output_pack:?}");
                        let msg_type = edge.msg.as_str();
                        let src_port = output_pack
                            .msg_types
                            .iter()
                            .position(|msg| msg == msg_type)
                            .unwrap_or_else(|| {
                                panic!(
                                    "Missing output port for message type '{msg_type}' on node {pid}"
                                )
                            });
                        input_msg_indices_types.push(CuInputMsg {
                            culist_index: output_pack.culist_index,
                            msg_type: msg_type.to_string(),
                            src_port,
                            edge_id,
                        });
                    } else {
                        #[cfg(all(feature = "std", feature = "macro_debug"))]
                        eprintln!("      ✗ Input from {pid} not ready, returning");
                        return Ok((next_culist_output_index, handled));
                    }
                }
                output_msg_pack = Some(CuOutputPack {
                    culist_index: next_culist_output_index,
                    msg_types: Vec::from(["()".to_string()]),
                });
                next_culist_output_index += 1;
            }
            CuTaskType::Regular => {
                let mut edge_ids = graph.get_dst_edges(id).unwrap_or_default();
                edge_ids.sort();
                #[cfg(all(feature = "std", feature = "macro_debug"))]
                eprintln!("    → Regular task with incoming edges: {edge_ids:?}");
                for edge_id in edge_ids {
                    let edge = graph
                        .edge(edge_id)
                        .unwrap_or_else(|| panic!("Missing edge {edge_id} for node {id}"));
                    let pid = graph
                        .get_node_id_by_name(edge.src.as_str())
                        .unwrap_or_else(|| {
                            panic!("Missing source node '{}' for edge {edge_id}", edge.src)
                        });
                    let output_pack = find_output_pack_from_nodeid(pid, plan);
                    if let Some(output_pack) = output_pack {
                        #[cfg(all(feature = "std", feature = "macro_debug"))]
                        eprintln!("      ✓ Input from {pid} ready: {output_pack:?}");
                        let msg_type = edge.msg.as_str();
                        let src_port = output_pack
                            .msg_types
                            .iter()
                            .position(|msg| msg == msg_type)
                            .unwrap_or_else(|| {
                                panic!(
                                    "Missing output port for message type '{msg_type}' on node {pid}"
                                )
                            });
                        input_msg_indices_types.push(CuInputMsg {
                            culist_index: output_pack.culist_index,
                            msg_type: msg_type.to_string(),
                            src_port,
                            edge_id,
                        });
                    } else {
                        #[cfg(all(feature = "std", feature = "macro_debug"))]
                        eprintln!("      ✗ Input from {pid} not ready, returning");
                        return Ok((next_culist_output_index, handled));
                    }
                }
                let msg_types = graph.get_node_output_msg_types_by_id(id)?;
                if msg_types.is_empty() {
                    return Err(CuError::from(format!(
                        "Regular node '{}' has no declared outputs",
                        node_ref.get_id()
                    )));
                }
                output_msg_pack = Some(CuOutputPack {
                    culist_index: next_culist_output_index,
                    msg_types,
                });
                next_culist_output_index += 1;
            }
        }

        sort_inputs_by_cnx_id(&mut input_msg_indices_types);

        if let Some(pos) = plan
            .iter()
            .position(|step| matches!(step, CuExecutionUnit::Step(s) if s.node_id == id))
        {
            #[cfg(all(feature = "std", feature = "macro_debug"))]
            eprintln!("    → Already in plan, modifying existing step");
            let mut step = plan.remove(pos);
            if let CuExecutionUnit::Step(ref mut s) = step {
                s.input_msg_indices_types = input_msg_indices_types;
            }
            plan.push(step);
        } else {
            #[cfg(all(feature = "std", feature = "macro_debug"))]
            eprintln!("    → New step added to plan");
            let step = CuExecutionStep {
                node_id: id,
                node: node_ref.clone(),
                task_type,
                input_msg_indices_types,
                output_msg_pack,
            };
            plan.push(CuExecutionUnit::Step(Box::new(step)));
        }

        handled = true;
    }

    #[cfg(all(feature = "std", feature = "macro_debug"))]
    eprintln!("-- finished branch from node {starting_point} with handled={handled}");
    Ok((next_culist_output_index, handled))
}

/// This is the main heuristics to compute an execution plan at compilation time.
/// TODO(gbin): Make that heuristic pluggable.
pub fn compute_runtime_plan(graph: &CuGraph) -> CuResult<CuExecutionLoop> {
    #[cfg(all(feature = "std", feature = "macro_debug"))]
    eprintln!("[runtime plan]");
    let mut plan = Vec::new();
    let mut next_culist_output_index = 0u32;

    let mut queue: VecDeque<NodeId> = VecDeque::new();
    for node_id in graph.node_ids() {
        if find_task_type_for_id(graph, node_id)? == CuTaskType::Source {
            queue.push_back(node_id);
        }
    }

    #[cfg(all(feature = "std", feature = "macro_debug"))]
    eprintln!("Initial source nodes: {queue:?}");

    while let Some(start_node) = queue.pop_front() {
        #[cfg(all(feature = "std", feature = "macro_debug"))]
        eprintln!("→ Starting BFS from source {start_node}");
        for node_id in graph.bfs_nodes(start_node) {
            let already_in_plan = plan
                .iter()
                .any(|unit| matches!(unit, CuExecutionUnit::Step(s) if s.node_id == node_id));
            if already_in_plan {
                #[cfg(all(feature = "std", feature = "macro_debug"))]
                eprintln!("    → Node {node_id} already planned, skipping");
                continue;
            }

            #[cfg(all(feature = "std", feature = "macro_debug"))]
            eprintln!("    Planning from node {node_id}");
            let (new_index, handled) =
                plan_tasks_tree_branch(graph, next_culist_output_index, node_id, &mut plan)?;
            next_culist_output_index = new_index;

            if !handled {
                #[cfg(all(feature = "std", feature = "macro_debug"))]
                eprintln!("    ✗ Node {node_id} was not handled, skipping enqueue of neighbors");
                continue;
            }

            #[cfg(all(feature = "std", feature = "macro_debug"))]
            eprintln!("    ✓ Node {node_id} handled successfully, enqueueing neighbors");
            for neighbor in graph.get_neighbor_ids(node_id, CuDirection::Outgoing) {
                #[cfg(all(feature = "std", feature = "macro_debug"))]
                eprintln!("      → Enqueueing neighbor {neighbor}");
                queue.push_back(neighbor);
            }
        }
    }

    let mut planned_nodes = BTreeSet::new();
    for unit in &plan {
        if let CuExecutionUnit::Step(step) = unit {
            planned_nodes.insert(step.node_id);
        }
    }

    let mut missing = Vec::new();
    for node_id in graph.node_ids() {
        if !planned_nodes.contains(&node_id) {
            if let Some(node) = graph.get_node(node_id) {
                missing.push(node.get_id().to_string());
            } else {
                missing.push(format!("node_id_{node_id}"));
            }
        }
    }

    if !missing.is_empty() {
        missing.sort();
        return Err(CuError::from(format!(
            "Execution plan could not include all nodes. Missing: {}. Check for loopback or missing source connections.",
            missing.join(", ")
        )));
    }

    Ok(CuExecutionLoop {
        steps: plan,
        loop_count: None,
    })
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
    use cu29_traits::{ErasedCuStampedData, ErasedCuStampedDataSet, MatchingTasks};
    use serde_derive::{Deserialize, Serialize};

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

    #[cfg(feature = "std")]
    fn tasks_instanciator(
        all_instances_configs: Vec<Option<&ComponentConfig>>,
        _resources: &mut ResourceManager,
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
            )
            .try_with_resources_instantiator(resources_instanciator)
            .and_then(|builder| builder.build());
        assert!(runtime.is_ok());
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

    #[cfg(all(feature = "std", feature = "async-cl-io"))]
    #[derive(Debug, Default)]
    struct RecordingWriter {
        ids: Arc<Mutex<Vec<u64>>>,
    }

    #[cfg(all(feature = "std", feature = "async-cl-io"))]
    impl WriteStream<CopperList<Msgs>> for RecordingWriter {
        fn log(&mut self, culist: &CopperList<Msgs>) -> CuResult<()> {
            self.ids.lock().unwrap().push(culist.id);
            std::thread::sleep(std::time::Duration::from_millis(2));
            Ok(())
        }
    }

    #[cfg(all(feature = "std", feature = "async-cl-io"))]
    #[test]
    fn test_async_copperlists_manager_flushes_in_order() {
        let ids = Arc::new(Mutex::new(Vec::new()));
        let mut copperlists = CopperListsManager::<Msgs, 4>::new(Some(Box::new(RecordingWriter {
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
        assert_eq!(copperlists.available_copper_lists().unwrap(), 4);
        assert_eq!(*ids.lock().unwrap(), vec![0, 1, 2, 3]);
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
}
