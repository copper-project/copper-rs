//! Some basic internal monitoring tooling Copper uses to monitor itself and the tasks it is running.
//!

use crate::config::CuConfig;
use crate::config::{BridgeChannelConfigRepresentation, BridgeConfig, CuGraph, Flavor, NodeId};
use crate::context::CuContext;
use crate::cutask::CuMsgMetadata;
use cu29_clock::CuDuration;
#[allow(unused_imports)]
use cu29_log::CuLogLevel;
#[cfg(all(feature = "std", debug_assertions))]
use cu29_log_runtime::{
    format_message_only, register_live_log_listener, unregister_live_log_listener,
};
use cu29_traits::{CuError, CuResult};
use serde_derive::{Deserialize, Serialize};

#[cfg(not(feature = "std"))]
extern crate alloc;

#[cfg(feature = "std")]
use std::sync::Arc;
#[cfg(feature = "std")]
use std::{collections::HashMap as Map, string::String, string::ToString, vec::Vec};

#[cfg(not(feature = "std"))]
use alloc::{collections::BTreeMap as Map, string::String, string::ToString, vec::Vec};
#[cfg(not(target_has_atomic = "64"))]
use spin::Mutex;

#[cfg(not(feature = "std"))]
mod imp {
    pub use alloc::alloc::{GlobalAlloc, Layout};
    #[cfg(target_has_atomic = "64")]
    pub use core::sync::atomic::AtomicU64;
    pub use core::sync::atomic::{AtomicUsize, Ordering};
    pub use libm::sqrt;
}

#[cfg(feature = "std")]
mod imp {
    #[cfg(feature = "memory_monitoring")]
    use super::CountingAlloc;
    #[cfg(feature = "memory_monitoring")]
    pub use std::alloc::System;
    pub use std::alloc::{GlobalAlloc, Layout};
    #[cfg(target_has_atomic = "64")]
    pub use std::sync::atomic::AtomicU64;
    pub use std::sync::atomic::{AtomicUsize, Ordering};
    #[cfg(feature = "memory_monitoring")]
    #[global_allocator]
    pub static GLOBAL: CountingAlloc<System> = CountingAlloc::new(System);
}

use imp::*;

#[cfg(all(feature = "std", debug_assertions))]
fn format_timestamp(time: CuDuration) -> String {
    // Render CuTime/CuDuration as HH:mm:ss.xxxx (4 fractional digits of a second).
    let nanos = time.as_nanos();
    let total_seconds = nanos / 1_000_000_000;
    let hours = total_seconds / 3600;
    let minutes = (total_seconds / 60) % 60;
    let seconds = total_seconds % 60;
    let fractional_1e4 = (nanos % 1_000_000_000) / 100_000;
    format!("{hours:02}:{minutes:02}:{seconds:02}.{fractional_1e4:04}")
}

/// The state of a task.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum CuTaskState {
    Start,
    Preprocess,
    Process,
    Postprocess,
    Stop,
}

/// Execution progress marker emitted by the runtime before running a component step.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct ExecutionMarker {
    /// Index into TASKS_IDS (tasks + bridge components/channels).
    pub component_id: usize,
    /// Lifecycle phase currently entered.
    pub step: CuTaskState,
    /// CopperList id when available (runtime loop), None during start/stop.
    pub culistid: Option<u64>,
}

/// Lock-free runtime-side progress probe.
///
/// The runtime writes execution markers directly into this probe from the hot path
/// (without calling monitor fan-out callbacks), and monitors can read a coherent
/// snapshot from watchdog threads when diagnosing stalls.
#[derive(Debug)]
pub struct RuntimeExecutionProbe {
    component_id: AtomicUsize,
    step: AtomicUsize,
    #[cfg(target_has_atomic = "64")]
    culistid: AtomicU64,
    #[cfg(target_has_atomic = "64")]
    culistid_present: AtomicUsize,
    #[cfg(not(target_has_atomic = "64"))]
    culistid: Mutex<Option<u64>>,
    sequence: AtomicUsize,
}

impl Default for RuntimeExecutionProbe {
    fn default() -> Self {
        Self {
            component_id: AtomicUsize::new(usize::MAX),
            step: AtomicUsize::new(0),
            #[cfg(target_has_atomic = "64")]
            culistid: AtomicU64::new(0),
            #[cfg(target_has_atomic = "64")]
            culistid_present: AtomicUsize::new(0),
            #[cfg(not(target_has_atomic = "64"))]
            culistid: Mutex::new(None),
            sequence: AtomicUsize::new(0),
        }
    }
}

impl RuntimeExecutionProbe {
    #[inline]
    pub fn record(&self, marker: ExecutionMarker) {
        self.component_id
            .store(marker.component_id, Ordering::Relaxed);
        self.step
            .store(task_state_to_usize(marker.step), Ordering::Relaxed);
        #[cfg(target_has_atomic = "64")]
        match marker.culistid {
            Some(culistid) => {
                self.culistid.store(culistid, Ordering::Relaxed);
                self.culistid_present.store(1, Ordering::Relaxed);
            }
            None => {
                self.culistid_present.store(0, Ordering::Relaxed);
            }
        }
        #[cfg(not(target_has_atomic = "64"))]
        {
            *self.culistid.lock() = marker.culistid;
        }
        self.sequence.fetch_add(1, Ordering::Release);
    }

    #[inline]
    pub fn sequence(&self) -> usize {
        self.sequence.load(Ordering::Acquire)
    }

    #[inline]
    pub fn marker(&self) -> Option<ExecutionMarker> {
        // Read a coherent snapshot. A concurrent writer may change values between reads;
        // in that case we retry to keep the marker and sequence aligned.
        loop {
            let seq_before = self.sequence.load(Ordering::Acquire);
            let component_id = self.component_id.load(Ordering::Relaxed);
            let step = self.step.load(Ordering::Relaxed);
            #[cfg(target_has_atomic = "64")]
            let culistid_present = self.culistid_present.load(Ordering::Relaxed);
            #[cfg(target_has_atomic = "64")]
            let culistid_value = self.culistid.load(Ordering::Relaxed);
            #[cfg(not(target_has_atomic = "64"))]
            let culistid = *self.culistid.lock();
            let seq_after = self.sequence.load(Ordering::Acquire);
            if seq_before == seq_after {
                if component_id == usize::MAX {
                    return None;
                }
                let step = usize_to_task_state(step);
                #[cfg(target_has_atomic = "64")]
                let culistid = if culistid_present == 0 {
                    None
                } else {
                    Some(culistid_value)
                };
                return Some(ExecutionMarker {
                    component_id,
                    step,
                    culistid,
                });
            }
        }
    }
}

#[inline]
const fn task_state_to_usize(step: CuTaskState) -> usize {
    match step {
        CuTaskState::Start => 0,
        CuTaskState::Preprocess => 1,
        CuTaskState::Process => 2,
        CuTaskState::Postprocess => 3,
        CuTaskState::Stop => 4,
    }
}

#[inline]
const fn usize_to_task_state(step: usize) -> CuTaskState {
    match step {
        0 => CuTaskState::Start,
        1 => CuTaskState::Preprocess,
        2 => CuTaskState::Process,
        3 => CuTaskState::Postprocess,
        _ => CuTaskState::Stop,
    }
}

#[cfg(feature = "std")]
pub type ExecutionProbeHandle = Arc<RuntimeExecutionProbe>;

/// Monitor decision to be taken when a task errored out.
#[derive(Debug)]
pub enum Decision {
    Abort,    // for a step (stop, start) or a copperlist, just stop trying to process it.
    Ignore, // Ignore this error and try to continue, ie calling the other tasks steps, setting a None return value and continue a copperlist.
    Shutdown, // This is a fatal error, shutdown the copper as cleanly as possible.
}

fn merge_decision(lhs: Decision, rhs: Decision) -> Decision {
    use Decision::{Abort, Ignore, Shutdown};
    // Pick the strictest monitor decision when multiple monitors disagree.
    // Shutdown dominates Abort, which dominates Ignore.
    match (lhs, rhs) {
        (Shutdown, _) | (_, Shutdown) => Shutdown,
        (Abort, _) | (_, Abort) => Abort,
        _ => Ignore,
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ComponentKind {
    Task,
    Bridge,
}

#[derive(Debug, Clone)]
pub struct MonitorNode {
    pub id: String,
    pub type_name: Option<String>,
    pub kind: ComponentKind,
    /// Ordered list of input port identifiers.
    pub inputs: Vec<String>,
    /// Ordered list of output port identifiers.
    pub outputs: Vec<String>,
}

#[derive(Debug, Clone)]
pub struct MonitorConnection {
    pub src: String,
    pub src_port: Option<String>,
    pub dst: String,
    pub dst_port: Option<String>,
    pub msg: String,
}

#[derive(Debug, Clone, Default)]
pub struct MonitorTopology {
    pub nodes: Vec<MonitorNode>,
    pub connections: Vec<MonitorConnection>,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct CopperListInfo {
    pub size_bytes: usize,
    pub count: usize,
}

impl CopperListInfo {
    pub const fn new(size_bytes: usize, count: usize) -> Self {
        Self { size_bytes, count }
    }
}

/// Reported data about CopperList IO for a single iteration.
#[derive(Debug, Clone, Copy, Default)]
pub struct CopperListIoStats {
    /// CopperList struct size in RAM (excluding dynamic payloads/handles)
    pub raw_culist_bytes: u64,
    /// Bytes held by payloads that will be serialized (currently: pooled handles, vecs, slices)
    pub handle_bytes: u64,
    /// Bytes produced by bincode serialization of the CopperList
    pub encoded_culist_bytes: u64,
    /// Bytes produced by bincode serialization of the KeyFrame (0 if none)
    pub keyframe_bytes: u64,
    /// Cumulative bytes written to the structured log stream so far
    pub structured_log_bytes_total: u64,
    /// CopperList identifier for reference in monitors
    pub culistid: u64,
}

/// Lightweight trait to estimate the amount of data a payload will contribute when serialized.
/// Default implementations return the stack size; specific types override to report dynamic data.
pub trait CuPayloadSize {
    /// Total bytes represented by the payload in memory (stack + heap backing).
    fn raw_bytes(&self) -> usize {
        core::mem::size_of_val(self)
    }

    /// Bytes that correspond to reusable/pooled handles (used for IO budgeting).
    fn handle_bytes(&self) -> usize {
        0
    }
}

impl<T> CuPayloadSize for T
where
    T: crate::cutask::CuMsgPayload,
{
    fn raw_bytes(&self) -> usize {
        core::mem::size_of::<T>()
    }
}

#[derive(Default, Debug, Clone, Copy)]
struct NodeIoUsage {
    has_incoming: bool,
    has_outgoing: bool,
}

fn collect_output_ports(graph: &CuGraph, node_id: NodeId) -> Vec<(String, String)> {
    let mut edge_ids = graph.get_src_edges(node_id).unwrap_or_default();
    edge_ids.sort();

    let mut outputs = Vec::new();
    let mut seen = Vec::new();
    let mut port_idx = 0usize;
    for edge_id in edge_ids {
        let Some(edge) = graph.edge(edge_id) else {
            continue;
        };
        if seen.iter().any(|msg| msg == &edge.msg) {
            continue;
        }
        seen.push(edge.msg.clone());
        let mut port_label = String::from("out");
        port_label.push_str(&port_idx.to_string());
        port_label.push_str(": ");
        port_label.push_str(edge.msg.as_str());
        outputs.push((edge.msg.clone(), port_label));
        port_idx += 1;
    }
    outputs
}

/// Derive a monitor-friendly topology from the runtime configuration.
pub fn build_monitor_topology(
    config: &CuConfig,
    mission: Option<&str>,
) -> CuResult<MonitorTopology> {
    let graph = config.get_graph(mission)?;
    let mut nodes: Map<String, MonitorNode> = Map::new();
    let mut io_usage: Map<String, NodeIoUsage> = Map::new();
    let mut output_port_lookup: Map<String, Map<String, String>> = Map::new();

    let mut bridge_lookup: Map<&str, &BridgeConfig> = Map::new();
    for bridge in &config.bridges {
        bridge_lookup.insert(bridge.id.as_str(), bridge);
    }

    for cnx in graph.edges() {
        io_usage.entry(cnx.src.clone()).or_default().has_outgoing = true;
        io_usage.entry(cnx.dst.clone()).or_default().has_incoming = true;
    }

    for (_, node) in graph.get_all_nodes() {
        let kind = match node.get_flavor() {
            Flavor::Bridge => ComponentKind::Bridge,
            _ => ComponentKind::Task,
        };
        let node_id = node.get_id();

        let mut inputs = Vec::new();
        let mut outputs = Vec::new();
        if kind == ComponentKind::Bridge {
            if let Some(bridge) = bridge_lookup.get(node_id.as_str()) {
                for ch in &bridge.channels {
                    match ch {
                        BridgeChannelConfigRepresentation::Rx { id, .. } => {
                            outputs.push(id.clone())
                        }
                        BridgeChannelConfigRepresentation::Tx { id, .. } => inputs.push(id.clone()),
                    }
                }
            }
        } else {
            let usage = io_usage.get(node_id.as_str()).cloned().unwrap_or_default();
            if usage.has_incoming || !usage.has_outgoing {
                inputs.push("in".to_string());
            }
            if usage.has_outgoing {
                if let Some(node_idx) = graph.get_node_id_by_name(node_id.as_str()) {
                    let ports = collect_output_ports(graph, node_idx);
                    let mut port_map: Map<String, String> = Map::new();
                    for (msg_type, label) in ports {
                        port_map.insert(msg_type, label.clone());
                        outputs.push(label);
                    }
                    output_port_lookup.insert(node_id.clone(), port_map);
                }
            } else if !usage.has_incoming {
                outputs.push("out".to_string());
            }
        }

        nodes.insert(
            node_id.clone(),
            MonitorNode {
                id: node_id,
                type_name: Some(node.get_type().to_string()),
                kind,
                inputs,
                outputs,
            },
        );
    }

    let mut connections = Vec::new();
    for cnx in graph.edges() {
        let src = cnx.src.clone();
        let dst = cnx.dst.clone();

        let src_port = cnx.src_channel.clone().or_else(|| {
            output_port_lookup
                .get(&src)
                .and_then(|ports| ports.get(&cnx.msg).cloned())
                .or_else(|| {
                    nodes
                        .get(&src)
                        .and_then(|node| node.outputs.first().cloned())
                })
        });
        let dst_port = cnx.dst_channel.clone().or_else(|| {
            nodes
                .get(&dst)
                .and_then(|node| node.inputs.first().cloned())
        });

        connections.push(MonitorConnection {
            src,
            src_port,
            dst,
            dst_port,
            msg: cnx.msg.clone(),
        });
    }

    Ok(MonitorTopology {
        nodes: nodes.into_values().collect(),
        connections,
    })
}

/// Trait to implement a monitoring task.
pub trait CuMonitor: Sized {
    fn new(config: &CuConfig, taskids: &'static [&'static str]) -> CuResult<Self>
    where
        Self: Sized;

    fn set_topology(&mut self, _topology: MonitorTopology) {}

    /// Runtime-authoritative mapping from `process_copperlist` slot index to
    /// `TASKS_IDS` component index.
    fn set_culist_task_mapping(&mut self, _mapping: &'static [usize]) {}

    fn set_copperlist_info(&mut self, _info: CopperListInfo) {}

    #[cfg(feature = "std")]
    fn set_execution_probe(&mut self, _probe: ExecutionProbeHandle) {}

    fn start(&mut self, _ctx: &CuContext) -> CuResult<()> {
        Ok(())
    }

    /// Callback that will be trigger at the end of every copperlist (before, on or after the serialization).
    fn process_copperlist(&self, _ctx: &CuContext, msgs: &[&CuMsgMetadata]) -> CuResult<()>;

    /// Called when the runtime finishes serializing a CopperList, giving IO accounting data.
    fn observe_copperlist_io(&self, _stats: CopperListIoStats) {}

    /// Callbacked when a Task errored out. The runtime requires an immediate decision.
    fn process_error(&self, taskid: usize, step: CuTaskState, error: &CuError) -> Decision;

    /// Callback fired when the runtime catches a panic in a std build.
    fn process_panic(&self, _panic_message: &str) {}

    /// Callbacked when copper is stopping.
    fn stop(&mut self, _ctx: &CuContext) -> CuResult<()> {
        Ok(())
    }
}

/// A do nothing monitor if no monitor is provided.
/// This is basically defining the default behavior of Copper in case of error.
pub struct NoMonitor {}
impl CuMonitor for NoMonitor {
    fn new(_config: &CuConfig, _taskids: &'static [&'static str]) -> CuResult<Self> {
        Ok(NoMonitor {})
    }

    fn start(&mut self, _ctx: &CuContext) -> CuResult<()> {
        #[cfg(all(feature = "std", debug_assertions))]
        register_live_log_listener(|entry, format_str, param_names| {
            let params: Vec<String> = entry.params.iter().map(|v| v.to_string()).collect();
            let named: Map<String, String> = param_names
                .iter()
                .zip(params.iter())
                .map(|(k, v)| (k.to_string(), v.clone()))
                .collect();

            if let Ok(msg) = format_message_only(format_str, params.as_slice(), &named) {
                let ts = format_timestamp(entry.time);
                println!("{} [{:?}] {}", ts, entry.level, msg);
            }
        });
        Ok(())
    }

    fn process_copperlist(&self, _ctx: &CuContext, _msgs: &[&CuMsgMetadata]) -> CuResult<()> {
        // By default, do nothing.
        Ok(())
    }

    fn process_error(&self, _taskid: usize, _step: CuTaskState, _error: &CuError) -> Decision {
        // By default, just try to continue.
        Decision::Ignore
    }

    fn stop(&mut self, _ctx: &CuContext) -> CuResult<()> {
        #[cfg(all(feature = "std", debug_assertions))]
        unregister_live_log_listener();
        Ok(())
    }
}

macro_rules! impl_monitor_tuple {
    ($($idx:tt => $name:ident),+) => {
        impl<$($name: CuMonitor),+> CuMonitor for ($($name,)+) {
            fn new(config: &CuConfig, taskids: &'static [&'static str]) -> CuResult<Self>
            where
                Self: Sized,
            {
                Ok(($($name::new(config, taskids)?,)+))
            }

            fn set_topology(&mut self, topology: MonitorTopology) {
                $(self.$idx.set_topology(topology.clone());)+
            }

            fn set_culist_task_mapping(&mut self, mapping: &'static [usize]) {
                $(self.$idx.set_culist_task_mapping(mapping);)+
            }

            fn set_copperlist_info(&mut self, info: CopperListInfo) {
                $(self.$idx.set_copperlist_info(info);)+
            }

            #[cfg(feature = "std")]
            fn set_execution_probe(&mut self, probe: ExecutionProbeHandle) {
                $(self.$idx.set_execution_probe(probe.clone());)+
            }

            fn start(&mut self, ctx: &CuContext) -> CuResult<()> {
                $(self.$idx.start(ctx)?;)+
                Ok(())
            }

            fn process_copperlist(&self, ctx: &CuContext, msgs: &[&CuMsgMetadata]) -> CuResult<()> {
                $(self.$idx.process_copperlist(ctx, msgs)?;)+
                Ok(())
            }

            fn observe_copperlist_io(&self, stats: CopperListIoStats) {
                $(self.$idx.observe_copperlist_io(stats);)+
            }

            fn process_error(&self, taskid: usize, step: CuTaskState, error: &CuError) -> Decision {
                let mut decision = Decision::Ignore;
                $(decision = merge_decision(decision, self.$idx.process_error(taskid, step, error));)+
                decision
            }

            fn process_panic(&self, panic_message: &str) {
                $(self.$idx.process_panic(panic_message);)+
            }

            fn stop(&mut self, ctx: &CuContext) -> CuResult<()> {
                $(self.$idx.stop(ctx)?;)+
                Ok(())
            }
        }
    };
}

impl_monitor_tuple!(0 => M0, 1 => M1);
impl_monitor_tuple!(0 => M0, 1 => M1, 2 => M2);
impl_monitor_tuple!(0 => M0, 1 => M1, 2 => M2, 3 => M3);
impl_monitor_tuple!(0 => M0, 1 => M1, 2 => M2, 3 => M3, 4 => M4);
impl_monitor_tuple!(0 => M0, 1 => M1, 2 => M2, 3 => M3, 4 => M4, 5 => M5);

#[cfg(feature = "std")]
pub fn panic_payload_to_string(payload: &(dyn core::any::Any + Send)) -> String {
    if let Some(msg) = payload.downcast_ref::<&str>() {
        (*msg).to_string()
    } else if let Some(msg) = payload.downcast_ref::<String>() {
        msg.clone()
    } else {
        "panic with non-string payload".to_string()
    }
}

/// A simple allocator that counts the number of bytes allocated and deallocated.
pub struct CountingAlloc<A: GlobalAlloc> {
    inner: A,
    allocated: AtomicUsize,
    deallocated: AtomicUsize,
}

impl<A: GlobalAlloc> CountingAlloc<A> {
    pub const fn new(inner: A) -> Self {
        CountingAlloc {
            inner,
            allocated: AtomicUsize::new(0),
            deallocated: AtomicUsize::new(0),
        }
    }

    pub fn allocated(&self) -> usize {
        self.allocated.load(Ordering::SeqCst)
    }

    pub fn deallocated(&self) -> usize {
        self.deallocated.load(Ordering::SeqCst)
    }

    pub fn reset(&self) {
        self.allocated.store(0, Ordering::SeqCst);
        self.deallocated.store(0, Ordering::SeqCst);
    }
}

// SAFETY: Delegates allocation/deallocation to the inner allocator while tracking sizes.
unsafe impl<A: GlobalAlloc> GlobalAlloc for CountingAlloc<A> {
    // SAFETY: Callers uphold the GlobalAlloc contract; we delegate to the inner allocator.
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        // SAFETY: Forwarding to the inner allocator preserves GlobalAlloc invariants.
        let p = unsafe { self.inner.alloc(layout) };
        if !p.is_null() {
            self.allocated.fetch_add(layout.size(), Ordering::SeqCst);
        }
        p
    }

    // SAFETY: Callers uphold the GlobalAlloc contract; we delegate to the inner allocator.
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        // SAFETY: Forwarding to the inner allocator preserves GlobalAlloc invariants.
        unsafe { self.inner.dealloc(ptr, layout) }
        self.deallocated.fetch_add(layout.size(), Ordering::SeqCst);
    }
}

/// A simple struct that counts the number of bytes allocated and deallocated in a scope.
#[cfg(feature = "memory_monitoring")]
pub struct ScopedAllocCounter {
    bf_allocated: usize,
    bf_deallocated: usize,
}

#[cfg(feature = "memory_monitoring")]
impl Default for ScopedAllocCounter {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(feature = "memory_monitoring")]
impl ScopedAllocCounter {
    pub fn new() -> Self {
        ScopedAllocCounter {
            bf_allocated: GLOBAL.allocated(),
            bf_deallocated: GLOBAL.deallocated(),
        }
    }

    /// Returns the total number of bytes allocated in the current scope
    /// since the creation of this `ScopedAllocCounter`.
    ///
    /// # Example
    /// ```
    /// use cu29_runtime::monitoring::ScopedAllocCounter;
    ///
    /// let counter = ScopedAllocCounter::new();
    /// let _vec = vec![0u8; 1024];
    /// println!("Bytes allocated: {}", counter.get_allocated());
    /// ```
    pub fn allocated(&self) -> usize {
        GLOBAL.allocated() - self.bf_allocated
    }

    /// Returns the total number of bytes deallocated in the current scope
    /// since the creation of this `ScopedAllocCounter`.
    ///
    /// # Example
    /// ```
    /// use cu29_runtime::monitoring::ScopedAllocCounter;
    ///
    /// let counter = ScopedAllocCounter::new();
    /// let _vec = vec![0u8; 1024];
    /// drop(_vec);
    /// println!("Bytes deallocated: {}", counter.get_deallocated());
    /// ```
    pub fn deallocated(&self) -> usize {
        GLOBAL.deallocated() - self.bf_deallocated
    }
}

/// Build a difference between the number of bytes allocated and deallocated in the scope at drop time.
#[cfg(feature = "memory_monitoring")]
impl Drop for ScopedAllocCounter {
    fn drop(&mut self) {
        let _allocated = GLOBAL.allocated() - self.bf_allocated;
        let _deallocated = GLOBAL.deallocated() - self.bf_deallocated;
        // TODO(gbin): Fix this when the logger is ready.
        // debug!(
        //     "Allocations: +{}B -{}B",
        //     allocated = allocated,
        //     deallocated = deallocated,
        // );
    }
}

#[cfg(feature = "std")]
const BUCKET_COUNT: usize = 1024;
#[cfg(not(feature = "std"))]
const BUCKET_COUNT: usize = 256;

/// Accumulative stat object that can give your some real time statistics.
/// Uses a fixed-size bucketed histogram for accurate percentile calculations.
#[derive(Debug, Clone)]
pub struct LiveStatistics {
    buckets: [u64; BUCKET_COUNT],
    min_val: u64,
    max_val: u64,
    sum: u64,
    sum_sq: u64,
    count: u64,
    max_value: u64,
}

impl LiveStatistics {
    /// Creates a new `LiveStatistics` instance with a specified maximum value.
    ///
    /// This function initializes a `LiveStatistics` structure with default values
    /// for tracking statistical data, while setting an upper limit for the data
    /// points that the structure tracks.
    ///
    /// # Parameters
    /// - `max_value` (`u64`): The maximum value that can be recorded or tracked.
    ///
    /// # Returns
    /// A new instance of `LiveStatistics` with:
    /// - `buckets`: An array pre-filled with zeros to categorize data points.
    /// - `min_val`: Initialized to the maximum possible `u64` value to track the minimum correctly.
    /// - `max_val`: Initialized to zero.
    /// - `sum`: The sum of all data points, initialized to zero.
    /// - `sum_sq`: The sum of squares of all data points, initialized to zero.
    /// - `count`: The total number of data points, initialized to zero.
    /// - `max_value`: The maximum allowable value for data points, set to the provided `max_value`.
    ///
    pub fn new_with_max(max_value: u64) -> Self {
        LiveStatistics {
            buckets: [0; BUCKET_COUNT],
            min_val: u64::MAX,
            max_val: 0,
            sum: 0,
            sum_sq: 0,
            count: 0,
            max_value,
        }
    }

    #[inline]
    fn value_to_bucket(&self, value: u64) -> usize {
        if value >= self.max_value {
            BUCKET_COUNT - 1
        } else {
            ((value as u128 * BUCKET_COUNT as u128) / self.max_value as u128) as usize
        }
    }

    #[inline]
    pub fn min(&self) -> u64 {
        if self.count == 0 { 0 } else { self.min_val }
    }

    #[inline]
    pub fn max(&self) -> u64 {
        self.max_val
    }

    #[inline]
    pub fn mean(&self) -> f64 {
        if self.count == 0 {
            0.0
        } else {
            self.sum as f64 / self.count as f64
        }
    }

    #[inline]
    pub fn stdev(&self) -> f64 {
        if self.count == 0 {
            return 0.0;
        }
        let mean = self.mean();
        let variance = (self.sum_sq as f64 / self.count as f64) - (mean * mean);
        if variance < 0.0 {
            return 0.0;
        }
        #[cfg(feature = "std")]
        return variance.sqrt();
        #[cfg(not(feature = "std"))]
        return sqrt(variance);
    }

    #[inline]
    pub fn percentile(&self, percentile: f64) -> u64 {
        if self.count == 0 {
            return 0;
        }

        let target_count = (self.count as f64 * percentile) as u64;
        let mut accumulated = 0u64;

        for (bucket_idx, &bucket_count) in self.buckets.iter().enumerate() {
            accumulated += bucket_count;
            if accumulated >= target_count {
                // Linear interpolation within the bucket
                let bucket_start = (bucket_idx as u64 * self.max_value) / BUCKET_COUNT as u64;
                let bucket_end = ((bucket_idx + 1) as u64 * self.max_value) / BUCKET_COUNT as u64;
                let bucket_fraction = if bucket_count > 0 {
                    (target_count - (accumulated - bucket_count)) as f64 / bucket_count as f64
                } else {
                    0.5
                };
                return bucket_start
                    + ((bucket_end - bucket_start) as f64 * bucket_fraction) as u64;
            }
        }

        self.max_val
    }

    /// Adds a value to the statistics.
    #[inline]
    pub fn record(&mut self, value: u64) {
        if value < self.min_val {
            self.min_val = value;
        }
        if value > self.max_val {
            self.max_val = value;
        }
        self.sum += value;
        self.sum_sq += value * value;
        self.count += 1;

        let bucket = self.value_to_bucket(value);
        self.buckets[bucket] += 1;
    }

    #[inline]
    pub fn len(&self) -> u64 {
        self.count
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    #[inline]
    pub fn reset(&mut self) {
        self.buckets.fill(0);
        self.min_val = u64::MAX;
        self.max_val = 0;
        self.sum = 0;
        self.sum_sq = 0;
        self.count = 0;
    }
}

/// A Specialized statistics object for CuDuration.
/// It will also keep track of the jitter between the values.
#[derive(Debug, Clone)]
pub struct CuDurationStatistics {
    bare: LiveStatistics,
    jitter: LiveStatistics,
    last_value: CuDuration,
}

impl CuDurationStatistics {
    pub fn new(max: CuDuration) -> Self {
        let CuDuration(max) = max;
        CuDurationStatistics {
            bare: LiveStatistics::new_with_max(max),
            jitter: LiveStatistics::new_with_max(max),
            last_value: CuDuration::default(),
        }
    }

    #[inline]
    pub fn min(&self) -> CuDuration {
        CuDuration(self.bare.min())
    }

    #[inline]
    pub fn max(&self) -> CuDuration {
        CuDuration(self.bare.max())
    }

    #[inline]
    pub fn mean(&self) -> CuDuration {
        CuDuration(self.bare.mean() as u64) // CuDuration is in ns, it is ok.
    }

    #[inline]
    pub fn percentile(&self, percentile: f64) -> CuDuration {
        CuDuration(self.bare.percentile(percentile))
    }

    #[inline]
    pub fn stddev(&self) -> CuDuration {
        CuDuration(self.bare.stdev() as u64)
    }

    #[inline]
    pub fn len(&self) -> u64 {
        self.bare.len()
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.bare.len() == 0
    }

    #[inline]
    pub fn jitter_min(&self) -> CuDuration {
        CuDuration(self.jitter.min())
    }

    #[inline]
    pub fn jitter_max(&self) -> CuDuration {
        CuDuration(self.jitter.max())
    }

    #[inline]
    pub fn jitter_mean(&self) -> CuDuration {
        CuDuration(self.jitter.mean() as u64)
    }

    #[inline]
    pub fn jitter_stddev(&self) -> CuDuration {
        CuDuration(self.jitter.stdev() as u64)
    }

    #[inline]
    pub fn jitter_percentile(&self, percentile: f64) -> CuDuration {
        CuDuration(self.jitter.percentile(percentile))
    }

    #[inline]
    pub fn record(&mut self, value: CuDuration) {
        let CuDuration(nanos) = value;
        if self.bare.is_empty() {
            self.bare.record(nanos);
            self.last_value = value;
            return;
        }
        self.bare.record(nanos);
        let CuDuration(last_nanos) = self.last_value;
        self.jitter.record(nanos.abs_diff(last_nanos));
        self.last_value = value;
    }

    #[inline]
    pub fn reset(&mut self) {
        self.bare.reset();
        self.jitter.reset();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::sync::atomic::{AtomicUsize, Ordering};
    #[cfg(feature = "std")]
    use std::sync::Arc;

    #[derive(Clone, Copy)]
    enum TestDecision {
        Ignore,
        Abort,
        Shutdown,
    }

    struct TestMonitor {
        decision: TestDecision,
        copperlist_calls: AtomicUsize,
        panic_calls: AtomicUsize,
        #[cfg(feature = "std")]
        probe_calls: AtomicUsize,
    }

    impl TestMonitor {
        fn new_with(decision: TestDecision) -> Self {
            Self {
                decision,
                copperlist_calls: AtomicUsize::new(0),
                panic_calls: AtomicUsize::new(0),
                #[cfg(feature = "std")]
                probe_calls: AtomicUsize::new(0),
            }
        }
    }

    impl CuMonitor for TestMonitor {
        fn new(_config: &CuConfig, _taskids: &'static [&'static str]) -> CuResult<Self> {
            Ok(Self::new_with(TestDecision::Ignore))
        }

        #[cfg(feature = "std")]
        fn set_execution_probe(&mut self, _probe: ExecutionProbeHandle) {
            self.probe_calls.fetch_add(1, Ordering::SeqCst);
        }

        fn process_copperlist(&self, _ctx: &CuContext, _msgs: &[&CuMsgMetadata]) -> CuResult<()> {
            self.copperlist_calls.fetch_add(1, Ordering::SeqCst);
            Ok(())
        }

        fn process_error(&self, _taskid: usize, _step: CuTaskState, _error: &CuError) -> Decision {
            match self.decision {
                TestDecision::Ignore => Decision::Ignore,
                TestDecision::Abort => Decision::Abort,
                TestDecision::Shutdown => Decision::Shutdown,
            }
        }

        fn process_panic(&self, _panic_message: &str) {
            self.panic_calls.fetch_add(1, Ordering::SeqCst);
        }
    }

    #[test]
    fn test_live_statistics_percentiles() {
        let mut stats = LiveStatistics::new_with_max(1000);

        // Record 100 values from 0 to 99
        for i in 0..100 {
            stats.record(i);
        }

        assert_eq!(stats.len(), 100);
        assert_eq!(stats.min(), 0);
        assert_eq!(stats.max(), 99);
        assert_eq!(stats.mean() as u64, 49); // Average of 0..99

        // Test percentiles - should be approximately correct
        let p50 = stats.percentile(0.5);
        let p90 = stats.percentile(0.90);
        let p95 = stats.percentile(0.95);
        let p99 = stats.percentile(0.99);

        // With 100 samples from 0-99, percentiles should be close to their index
        assert!((p50 as i64 - 49).abs() < 5, "p50={} expected ~49", p50);
        assert!((p90 as i64 - 89).abs() < 5, "p90={} expected ~89", p90);
        assert!((p95 as i64 - 94).abs() < 5, "p95={} expected ~94", p95);
        assert!((p99 as i64 - 98).abs() < 5, "p99={} expected ~98", p99);
    }

    #[test]
    fn test_duration_stats() {
        let mut stats = CuDurationStatistics::new(CuDuration(1000));
        stats.record(CuDuration(100));
        stats.record(CuDuration(200));
        stats.record(CuDuration(500));
        stats.record(CuDuration(400));
        assert_eq!(stats.min(), CuDuration(100));
        assert_eq!(stats.max(), CuDuration(500));
        assert_eq!(stats.mean(), CuDuration(300));
        assert_eq!(stats.len(), 4);
        assert_eq!(stats.jitter.len(), 3);
        assert_eq!(stats.jitter_min(), CuDuration(100));
        assert_eq!(stats.jitter_max(), CuDuration(300));
        assert_eq!(stats.jitter_mean(), CuDuration((100 + 300 + 100) / 3));
        stats.reset();
        assert_eq!(stats.len(), 0);
    }

    #[test]
    fn tuple_monitor_merges_contradictory_decisions_with_strictest_wins() {
        let err = CuError::from("boom");

        let two = (
            TestMonitor::new_with(TestDecision::Ignore),
            TestMonitor::new_with(TestDecision::Shutdown),
        );
        assert!(matches!(
            two.process_error(0, CuTaskState::Process, &err),
            Decision::Shutdown
        ));

        let two = (
            TestMonitor::new_with(TestDecision::Ignore),
            TestMonitor::new_with(TestDecision::Abort),
        );
        assert!(matches!(
            two.process_error(0, CuTaskState::Process, &err),
            Decision::Abort
        ));
    }

    #[test]
    fn tuple_monitor_fans_out_callbacks() {
        let left = TestMonitor::new_with(TestDecision::Ignore);
        let right = TestMonitor::new_with(TestDecision::Ignore);
        let mut monitors = (left, right);
        let (ctx, _clock_control) = CuContext::new_mock_clock();

        #[cfg(feature = "std")]
        monitors.set_execution_probe(Arc::new(RuntimeExecutionProbe::default()));
        monitors
            .process_copperlist(&ctx, &[])
            .expect("process_copperlist should fan out");
        monitors.process_panic("panic marker");

        assert_eq!(monitors.0.copperlist_calls.load(Ordering::SeqCst), 1);
        assert_eq!(monitors.1.copperlist_calls.load(Ordering::SeqCst), 1);
        assert_eq!(monitors.0.panic_calls.load(Ordering::SeqCst), 1);
        assert_eq!(monitors.1.panic_calls.load(Ordering::SeqCst), 1);
        #[cfg(feature = "std")]
        {
            assert_eq!(monitors.0.probe_calls.load(Ordering::SeqCst), 1);
            assert_eq!(monitors.1.probe_calls.load(Ordering::SeqCst), 1);
        }
    }

    #[test]
    fn runtime_execution_probe_roundtrip_marker() {
        let probe = RuntimeExecutionProbe::default();
        assert!(probe.marker().is_none());
        assert_eq!(probe.sequence(), 0);

        probe.record(ExecutionMarker {
            component_id: 7,
            step: CuTaskState::Process,
            culistid: Some(42),
        });

        let marker = probe.marker().expect("marker should be available");
        assert_eq!(marker.component_id, 7);
        assert!(matches!(marker.step, CuTaskState::Process));
        assert_eq!(marker.culistid, Some(42));
        assert_eq!(probe.sequence(), 1);
    }
}
