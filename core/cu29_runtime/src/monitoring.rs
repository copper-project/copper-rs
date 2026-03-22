//! Some basic internal monitoring tooling Copper uses to monitor itself and the components it runs.
//!

use crate::config::CuConfig;
use crate::config::{
    BridgeChannelConfigRepresentation, BridgeConfig, ComponentConfig, CuGraph, Flavor, NodeId,
};
use crate::context::CuContext;
use crate::cutask::CuMsgMetadata;
use bincode::Encode;
use bincode::config::standard;
use bincode::enc::EncoderImpl;
use bincode::enc::write::SizeWriter;
use compact_str::CompactString;
use cu29_clock::CuDuration;
#[allow(unused_imports)]
use cu29_log::CuLogLevel;
#[cfg(all(feature = "std", debug_assertions))]
use cu29_log_runtime::{
    format_message_only, register_live_log_listener, unregister_live_log_listener,
};
use cu29_traits::{
    CuError, CuResult, ObservedWriter, abort_observed_encode, begin_observed_encode,
    finish_observed_encode,
};
use serde_derive::{Deserialize, Serialize};

#[cfg(not(feature = "std"))]
extern crate alloc;

#[cfg(feature = "std")]
use std::sync::Arc;
#[cfg(feature = "std")]
use std::{cell::Cell, thread_local};
#[cfg(feature = "std")]
use std::{collections::HashMap as Map, string::String, string::ToString, vec::Vec};

#[cfg(not(feature = "std"))]
use alloc::{collections::BTreeMap as Map, string::String, string::ToString, vec::Vec};
#[cfg(not(target_has_atomic = "64"))]
use spin::Mutex;
#[cfg(not(feature = "std"))]
use spin::Mutex as SpinMutex;

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

/// Lifecycle state of a monitored component.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum CuComponentState {
    Start,
    Preprocess,
    Process,
    Postprocess,
    Stop,
}

/// Strongly-typed index into [`CuMonitoringMetadata::components`].
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct ComponentId(usize);

impl ComponentId {
    pub const INVALID: Self = Self(usize::MAX);

    #[inline]
    pub const fn new(index: usize) -> Self {
        Self(index)
    }

    #[inline]
    pub const fn index(self) -> usize {
        self.0
    }
}

impl core::fmt::Display for ComponentId {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        self.0.fmt(f)
    }
}

impl From<ComponentId> for usize {
    fn from(value: ComponentId) -> Self {
        value.index()
    }
}

/// Strongly-typed CopperList slot index.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct CuListSlot(usize);

impl CuListSlot {
    #[inline]
    pub const fn new(index: usize) -> Self {
        Self(index)
    }

    #[inline]
    pub const fn index(self) -> usize {
        self.0
    }
}

impl From<CuListSlot> for usize {
    fn from(value: CuListSlot) -> Self {
        value.index()
    }
}

/// Static monitor-side CopperList indexing layout.
///
/// This layout is mission/runtime scoped and remains constant after monitor construction.
#[derive(Debug, Clone, Copy)]
pub struct CopperListLayout {
    components: &'static [MonitorComponentMetadata],
    slot_to_component: &'static [ComponentId],
}

impl CopperListLayout {
    #[inline]
    pub const fn new(
        components: &'static [MonitorComponentMetadata],
        slot_to_component: &'static [ComponentId],
    ) -> Self {
        Self {
            components,
            slot_to_component,
        }
    }

    #[inline]
    pub const fn components(self) -> &'static [MonitorComponentMetadata] {
        self.components
    }

    #[inline]
    pub const fn component_count(self) -> usize {
        self.components.len()
    }

    #[inline]
    pub const fn culist_slot_count(self) -> usize {
        self.slot_to_component.len()
    }

    #[inline]
    pub fn component(self, id: ComponentId) -> &'static MonitorComponentMetadata {
        &self.components[id.index()]
    }

    #[inline]
    pub fn component_for_slot(self, culist_slot: CuListSlot) -> ComponentId {
        self.slot_to_component[culist_slot.index()]
    }

    #[inline]
    pub const fn slot_to_component(self) -> &'static [ComponentId] {
        self.slot_to_component
    }

    #[inline]
    pub fn view<'a>(self, msgs: &'a [&'a CuMsgMetadata]) -> CopperListView<'a> {
        CopperListView::new(self, msgs)
    }
}

/// Per-loop monitor view over CopperList metadata paired with static component mapping.
#[derive(Debug, Clone, Copy)]
pub struct CopperListView<'a> {
    layout: CopperListLayout,
    msgs: &'a [&'a CuMsgMetadata],
}

impl<'a> CopperListView<'a> {
    #[inline]
    pub fn new(layout: CopperListLayout, msgs: &'a [&'a CuMsgMetadata]) -> Self {
        assert_eq!(
            msgs.len(),
            layout.culist_slot_count(),
            "invalid monitor CopperList view: msgs len {} != slot mapping len {}",
            msgs.len(),
            layout.culist_slot_count()
        );
        Self { layout, msgs }
    }

    #[inline]
    pub const fn layout(self) -> CopperListLayout {
        self.layout
    }

    #[inline]
    pub const fn msgs(self) -> &'a [&'a CuMsgMetadata] {
        self.msgs
    }

    #[inline]
    pub const fn len(self) -> usize {
        self.msgs.len()
    }

    #[inline]
    pub const fn is_empty(self) -> bool {
        self.msgs.is_empty()
    }

    #[inline]
    pub fn entry(self, culist_slot: CuListSlot) -> CopperListEntry<'a> {
        let index = culist_slot.index();
        CopperListEntry {
            culist_slot,
            component_id: self.layout.component_for_slot(culist_slot),
            msg: self.msgs[index],
        }
    }

    pub fn entries(self) -> impl Iterator<Item = CopperListEntry<'a>> + 'a {
        self.msgs.iter().enumerate().map(move |(idx, msg)| {
            let culist_slot = CuListSlot::new(idx);
            CopperListEntry {
                culist_slot,
                component_id: self.layout.component_for_slot(culist_slot),
                msg,
            }
        })
    }
}

/// One message entry in CopperList slot order with resolved component identity.
#[derive(Debug, Clone, Copy)]
pub struct CopperListEntry<'a> {
    pub culist_slot: CuListSlot,
    pub component_id: ComponentId,
    pub msg: &'a CuMsgMetadata,
}

impl<'a> CopperListEntry<'a> {
    #[inline]
    pub fn component(self, layout: CopperListLayout) -> &'static MonitorComponentMetadata {
        layout.component(self.component_id)
    }

    #[inline]
    pub fn component_type(self, layout: CopperListLayout) -> ComponentType {
        layout.component(self.component_id).kind()
    }
}

/// Execution progress marker emitted by the runtime before running a component step.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct ExecutionMarker {
    /// Index into `CuMonitoringMetadata::components()`.
    pub component_id: ComponentId,
    /// Lifecycle phase currently entered.
    pub step: CuComponentState,
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
            component_id: AtomicUsize::new(ComponentId::INVALID.index()),
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
            .store(marker.component_id.index(), Ordering::Relaxed);
        self.step
            .store(component_state_to_usize(marker.step), Ordering::Relaxed);
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
                if component_id == ComponentId::INVALID.index() {
                    return None;
                }
                let step = usize_to_component_state(step);
                #[cfg(target_has_atomic = "64")]
                let culistid = if culistid_present == 0 {
                    None
                } else {
                    Some(culistid_value)
                };
                return Some(ExecutionMarker {
                    component_id: ComponentId::new(component_id),
                    step,
                    culistid,
                });
            }
        }
    }
}

#[inline]
const fn component_state_to_usize(step: CuComponentState) -> usize {
    match step {
        CuComponentState::Start => 0,
        CuComponentState::Preprocess => 1,
        CuComponentState::Process => 2,
        CuComponentState::Postprocess => 3,
        CuComponentState::Stop => 4,
    }
}

#[inline]
const fn usize_to_component_state(step: usize) -> CuComponentState {
    match step {
        0 => CuComponentState::Start,
        1 => CuComponentState::Preprocess,
        2 => CuComponentState::Process,
        3 => CuComponentState::Postprocess,
        _ => CuComponentState::Stop,
    }
}

#[cfg(feature = "std")]
pub type ExecutionProbeHandle = Arc<RuntimeExecutionProbe>;

/// Platform-neutral monitor view of runtime execution progress.
///
/// In `std` builds this can wrap a shared runtime probe. In `no_std` builds it is currently
/// unavailable and helper methods return `None`/`false`.
#[derive(Debug, Clone)]
pub struct MonitorExecutionProbe {
    #[cfg(feature = "std")]
    inner: Option<ExecutionProbeHandle>,
}

impl Default for MonitorExecutionProbe {
    fn default() -> Self {
        Self::unavailable()
    }
}

impl MonitorExecutionProbe {
    #[cfg(feature = "std")]
    pub fn from_shared(handle: ExecutionProbeHandle) -> Self {
        Self {
            inner: Some(handle),
        }
    }

    pub const fn unavailable() -> Self {
        Self {
            #[cfg(feature = "std")]
            inner: None,
        }
    }

    pub fn is_available(&self) -> bool {
        #[cfg(feature = "std")]
        {
            self.inner.is_some()
        }
        #[cfg(not(feature = "std"))]
        {
            false
        }
    }

    pub fn marker(&self) -> Option<ExecutionMarker> {
        #[cfg(feature = "std")]
        {
            self.inner.as_ref().and_then(|probe| probe.marker())
        }
        #[cfg(not(feature = "std"))]
        {
            None
        }
    }

    pub fn sequence(&self) -> Option<usize> {
        #[cfg(feature = "std")]
        {
            self.inner.as_ref().map(|probe| probe.sequence())
        }
        #[cfg(not(feature = "std"))]
        {
            None
        }
    }
}

/// Runtime component category used by monitoring metadata and topology.
///
/// A "task" is a regular Copper task (lifecycle callbacks + payload processing). A "bridge"
/// is a monitored bridge-side execution component (bridge nodes and channel endpoints).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum ComponentType {
    Source,
    Task,
    Sink,
    Bridge,
}

impl ComponentType {
    pub const fn is_task(self) -> bool {
        !matches!(self, Self::Bridge)
    }
}

/// Static identity entry for one monitored runtime component.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MonitorComponentMetadata {
    id: &'static str,
    kind: ComponentType,
    type_name: Option<&'static str>,
}

impl MonitorComponentMetadata {
    pub const fn new(
        id: &'static str,
        kind: ComponentType,
        type_name: Option<&'static str>,
    ) -> Self {
        Self {
            id,
            kind,
            type_name,
        }
    }

    /// Stable monitor component id (for logs/debug and joins with runtime markers).
    pub const fn id(&self) -> &'static str {
        self.id
    }

    pub const fn kind(&self) -> ComponentType {
        self.kind
    }

    /// Rust type label when available (typically tasks); `None` for synthetic bridge entries.
    pub const fn type_name(&self) -> Option<&'static str> {
        self.type_name
    }
}

/// Immutable runtime-provided metadata passed once to [`CuMonitor::new`].
///
/// This bundles identifiers, deterministic component layout, and monitor-specific config so monitor
/// construction is explicit and does not need ad-hoc late setters.
#[derive(Debug, Clone)]
pub struct CuMonitoringMetadata {
    mission_id: CompactString,
    layout: CopperListLayout,
    copperlist_info: CopperListInfo,
    topology: MonitorTopology,
    monitor_config: Option<ComponentConfig>,
}

impl CuMonitoringMetadata {
    pub fn new(
        mission_id: CompactString,
        components: &'static [MonitorComponentMetadata],
        culist_component_mapping: &'static [ComponentId],
        copperlist_info: CopperListInfo,
        topology: MonitorTopology,
        monitor_config: Option<ComponentConfig>,
    ) -> CuResult<Self> {
        Self::validate_components(components)?;
        Self::validate_culist_mapping(components.len(), culist_component_mapping)?;
        Ok(Self {
            mission_id,
            layout: CopperListLayout::new(components, culist_component_mapping),
            copperlist_info,
            topology,
            monitor_config,
        })
    }

    fn validate_components(components: &'static [MonitorComponentMetadata]) -> CuResult<()> {
        let mut seen_bridge = false;
        for component in components {
            match component.kind() {
                component_type if component_type.is_task() && seen_bridge => {
                    return Err(CuError::from(
                        "invalid monitor metadata: task-family components must appear before bridges",
                    ));
                }
                ComponentType::Bridge => seen_bridge = true,
                _ => {}
            }
        }
        Ok(())
    }

    fn validate_culist_mapping(
        components_len: usize,
        culist_component_mapping: &'static [ComponentId],
    ) -> CuResult<()> {
        for component_idx in culist_component_mapping {
            if component_idx.index() >= components_len {
                return Err(CuError::from(
                    "invalid monitor metadata: culist mapping points past components table",
                ));
            }
        }
        Ok(())
    }

    /// Active mission identifier for this runtime instance.
    pub fn mission_id(&self) -> &str {
        self.mission_id.as_str()
    }

    /// Canonical table of monitored runtime components.
    ///
    /// Ordering is deterministic and mission-scoped: tasks first, then bridge-side components.
    pub fn components(&self) -> &'static [MonitorComponentMetadata] {
        self.layout.components()
    }

    /// Total number of monitored components.
    pub const fn component_count(&self) -> usize {
        self.layout.component_count()
    }

    /// Static runtime layout used to map CopperList slots to components.
    pub const fn layout(&self) -> CopperListLayout {
        self.layout
    }

    pub fn component(&self, component_id: ComponentId) -> &'static MonitorComponentMetadata {
        self.layout.component(component_id)
    }

    pub fn component_id(&self, component_id: ComponentId) -> &'static str {
        self.component(component_id).id()
    }

    pub fn component_kind(&self, component_id: ComponentId) -> ComponentType {
        self.component(component_id).kind()
    }

    pub fn component_index_by_id(&self, component_id: &str) -> Option<ComponentId> {
        self.layout
            .components()
            .iter()
            .position(|component| component.id() == component_id)
            .map(ComponentId::new)
    }

    /// CopperList slot -> monitored component index mapping.
    ///
    /// This table maps each CopperList slot index to the producing component index.
    pub fn culist_component_mapping(&self) -> &'static [ComponentId] {
        self.layout.slot_to_component()
    }

    pub fn component_for_culist_slot(&self, culist_slot: CuListSlot) -> ComponentId {
        self.layout.component_for_slot(culist_slot)
    }

    pub fn copperlist_view<'a>(&self, msgs: &'a [&'a CuMsgMetadata]) -> CopperListView<'a> {
        self.layout.view(msgs)
    }

    pub const fn copperlist_info(&self) -> CopperListInfo {
        self.copperlist_info
    }

    /// Resolved graph topology for the active mission.
    ///
    /// This is always available. Nodes represent config graph nodes, not every synthetic bridge
    /// channel entry in `components()`.
    pub fn topology(&self) -> &MonitorTopology {
        &self.topology
    }

    pub fn monitor_config(&self) -> Option<&ComponentConfig> {
        self.monitor_config.as_ref()
    }

    pub fn with_monitor_config(mut self, monitor_config: Option<ComponentConfig>) -> Self {
        self.monitor_config = monitor_config;
        self
    }
}

/// Runtime-provided dynamic monitoring handles passed once to [`CuMonitor::new`].
///
/// This context may expose live runtime state (for example execution progress probes).
#[derive(Debug, Clone, Default)]
pub struct CuMonitoringRuntime {
    execution_probe: MonitorExecutionProbe,
}

impl CuMonitoringRuntime {
    pub const fn new(execution_probe: MonitorExecutionProbe) -> Self {
        Self { execution_probe }
    }

    pub const fn unavailable() -> Self {
        Self::new(MonitorExecutionProbe::unavailable())
    }

    pub fn execution_probe(&self) -> &MonitorExecutionProbe {
        &self.execution_probe
    }
}

/// Monitor decision to be taken when a component step errored out.
#[derive(Debug)]
pub enum Decision {
    Abort,    // for a step (stop, start) or a copperlist, just stop trying to process it.
    Ignore, // Ignore this error and try to continue, ie calling the other component steps, setting a None return value and continue a copperlist.
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

#[derive(Debug, Clone)]
pub struct MonitorNode {
    pub id: String,
    pub type_name: Option<String>,
    pub kind: ComponentType,
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
    /// CopperList bytes resident in RAM for this iteration.
    ///
    /// This includes the fixed CopperList struct size plus any pooled or
    /// handle-backed payload bytes observed on the real encode path.
    pub raw_culist_bytes: u64,
    /// Bytes attributed to handle-backed storage while measuring payload IO.
    ///
    /// This is surfaced separately so monitors can show how much of the runtime
    /// footprint lives in pooled payload buffers rather than inside the fixed
    /// CopperList struct.
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

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct PayloadIoStats {
    pub resident_bytes: usize,
    pub encoded_bytes: usize,
    pub handle_bytes: usize,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct CuMsgIoStats {
    pub present: bool,
    pub resident_bytes: u64,
    pub encoded_bytes: u64,
    pub handle_bytes: u64,
}

pub struct CuMsgIoCache<const N: usize> {
    entries: [Cell<CuMsgIoStats>; N],
}

impl<const N: usize> CuMsgIoCache<N> {
    pub fn clear(&self) {
        for entry in &self.entries {
            entry.set(CuMsgIoStats::default());
        }
    }

    pub fn get(&self, idx: usize) -> CuMsgIoStats {
        self.entries[idx].get()
    }

    fn raw_parts(&self) -> (*const Cell<CuMsgIoStats>, usize) {
        (self.entries.as_ptr(), N)
    }
}

impl<const N: usize> Default for CuMsgIoCache<N> {
    fn default() -> Self {
        Self {
            entries: core::array::from_fn(|_| Cell::new(CuMsgIoStats::default())),
        }
    }
}

#[derive(Clone, Copy)]
struct ActiveCuMsgIoCapture {
    cache_ptr: *const Cell<CuMsgIoStats>,
    cache_len: usize,
    current_slot: Option<usize>,
}

#[cfg(feature = "std")]
thread_local! {
    static PAYLOAD_HANDLE_BYTES: Cell<Option<usize>> = const { Cell::new(None) };
    static ACTIVE_COPPERLIST_CAPTURE: Cell<Option<ActiveCuMsgIoCapture>> = const { Cell::new(None) };
    static LAST_COMPLETED_HANDLE_BYTES: Cell<u64> = const { Cell::new(0) };
}

#[cfg(not(feature = "std"))]
static PAYLOAD_HANDLE_BYTES: SpinMutex<Option<usize>> = SpinMutex::new(None);
#[cfg(not(feature = "std"))]
static ACTIVE_COPPERLIST_CAPTURE: SpinMutex<Option<ActiveCuMsgIoCapture>> = SpinMutex::new(None);
#[cfg(not(feature = "std"))]
static LAST_COMPLETED_HANDLE_BYTES: SpinMutex<u64> = SpinMutex::new(0);

fn begin_payload_io_measurement() {
    #[cfg(feature = "std")]
    PAYLOAD_HANDLE_BYTES.with(|bytes| {
        debug_assert!(
            bytes.get().is_none(),
            "payload IO byte measurement must not be nested"
        );
        bytes.set(Some(0));
    });

    #[cfg(not(feature = "std"))]
    {
        let mut bytes = PAYLOAD_HANDLE_BYTES.lock();
        debug_assert!(
            bytes.is_none(),
            "payload IO byte measurement must not be nested"
        );
        *bytes = Some(0);
    }
}

fn finish_payload_io_measurement() -> usize {
    #[cfg(feature = "std")]
    {
        return PAYLOAD_HANDLE_BYTES.with(|bytes| bytes.replace(None).unwrap_or(0));
    }

    #[cfg(not(feature = "std"))]
    {
        PAYLOAD_HANDLE_BYTES.lock().take().unwrap_or(0)
    }
}

fn abort_payload_io_measurement() {
    #[cfg(feature = "std")]
    PAYLOAD_HANDLE_BYTES.with(|bytes| bytes.set(None));

    #[cfg(not(feature = "std"))]
    {
        *PAYLOAD_HANDLE_BYTES.lock() = None;
    }
}

fn current_payload_io_measurement() -> usize {
    #[cfg(feature = "std")]
    {
        return PAYLOAD_HANDLE_BYTES.with(|bytes| bytes.get().unwrap_or(0));
    }

    #[cfg(not(feature = "std"))]
    {
        PAYLOAD_HANDLE_BYTES.lock().as_ref().copied().unwrap_or(0)
    }
}

pub(crate) fn record_payload_handle_bytes(bytes: usize) {
    #[cfg(feature = "std")]
    PAYLOAD_HANDLE_BYTES.with(|total| {
        if let Some(current) = total.get() {
            total.set(Some(current.saturating_add(bytes)));
        }
    });

    #[cfg(not(feature = "std"))]
    {
        let mut total = PAYLOAD_HANDLE_BYTES.lock();
        if let Some(current) = *total {
            *total = Some(current.saturating_add(bytes));
        }
    }
}

fn set_last_completed_handle_bytes(bytes: u64) {
    #[cfg(feature = "std")]
    LAST_COMPLETED_HANDLE_BYTES.with(|total| total.set(bytes));

    #[cfg(not(feature = "std"))]
    {
        *LAST_COMPLETED_HANDLE_BYTES.lock() = bytes;
    }
}

pub fn take_last_completed_handle_bytes() -> u64 {
    #[cfg(feature = "std")]
    {
        return LAST_COMPLETED_HANDLE_BYTES.with(|total| total.replace(0));
    }

    #[cfg(not(feature = "std"))]
    {
        let mut total = LAST_COMPLETED_HANDLE_BYTES.lock();
        let value = *total;
        *total = 0;
        value
    }
}

fn with_active_capture_mut<R>(f: impl FnOnce(&mut ActiveCuMsgIoCapture) -> R) -> Option<R> {
    #[cfg(feature = "std")]
    {
        return ACTIVE_COPPERLIST_CAPTURE.with(|capture| {
            let mut state = capture.get()?;
            let result = f(&mut state);
            capture.set(Some(state));
            Some(result)
        });
    }

    #[cfg(not(feature = "std"))]
    {
        let mut capture = ACTIVE_COPPERLIST_CAPTURE.lock();
        let state = capture.as_mut()?;
        Some(f(state))
    }
}

pub struct CuMsgIoCaptureGuard;

impl CuMsgIoCaptureGuard {
    pub fn select_slot(&self, slot: usize) {
        let _ = with_active_capture_mut(|capture| {
            debug_assert!(slot < capture.cache_len, "payload IO slot out of range");
            capture.current_slot = Some(slot);
        });
    }
}

impl Drop for CuMsgIoCaptureGuard {
    fn drop(&mut self) {
        set_last_completed_handle_bytes(finish_payload_io_measurement() as u64);

        #[cfg(feature = "std")]
        ACTIVE_COPPERLIST_CAPTURE.with(|capture| capture.set(None));

        #[cfg(not(feature = "std"))]
        {
            *ACTIVE_COPPERLIST_CAPTURE.lock() = None;
        }
    }
}

pub fn start_copperlist_io_capture<const N: usize>(cache: &CuMsgIoCache<N>) -> CuMsgIoCaptureGuard {
    cache.clear();
    set_last_completed_handle_bytes(0);
    begin_payload_io_measurement();
    let (cache_ptr, cache_len) = cache.raw_parts();
    let capture = ActiveCuMsgIoCapture {
        cache_ptr,
        cache_len,
        current_slot: None,
    };

    #[cfg(feature = "std")]
    ACTIVE_COPPERLIST_CAPTURE.with(|state| {
        debug_assert!(
            state.get().is_none(),
            "CopperList payload IO capture must not be nested"
        );
        state.set(Some(capture));
    });

    #[cfg(not(feature = "std"))]
    {
        let mut state = ACTIVE_COPPERLIST_CAPTURE.lock();
        debug_assert!(
            state.is_none(),
            "CopperList payload IO capture must not be nested"
        );
        *state = Some(capture);
    }

    CuMsgIoCaptureGuard
}

pub(crate) fn current_payload_handle_bytes() -> usize {
    current_payload_io_measurement()
}

pub(crate) fn record_current_slot_payload_io_stats(
    fixed_bytes: usize,
    encoded_bytes: usize,
    handle_bytes: usize,
) {
    let _ = with_active_capture_mut(|capture| {
        let Some(slot) = capture.current_slot else {
            return;
        };
        if slot >= capture.cache_len {
            return;
        }
        // SAFETY: the capture guard holds the cache alive for the duration of the encode pass.
        let entry = unsafe { &*capture.cache_ptr.add(slot) };
        entry.set(CuMsgIoStats {
            present: true,
            resident_bytes: (fixed_bytes.saturating_add(handle_bytes)) as u64,
            encoded_bytes: encoded_bytes as u64,
            handle_bytes: handle_bytes as u64,
        });
    });
}

/// Measures payload bytes using the same encode path Copper uses for
/// logging/export.
///
/// `resident_bytes` is the payload's in-memory fixed footprint plus any
/// handle-backed dynamic storage reported during encoding. `encoded_bytes` is
/// the exact bincode payload size.
pub fn payload_io_stats<T>(payload: &T) -> CuResult<PayloadIoStats>
where
    T: Encode,
{
    begin_payload_io_measurement();
    begin_observed_encode();

    let result = (|| {
        let mut encoder =
            EncoderImpl::<_, _>::new(ObservedWriter::new(SizeWriter::default()), standard());
        payload.encode(&mut encoder).map_err(|e| {
            CuError::from("Failed to measure payload IO bytes").add_cause(&e.to_string())
        })?;
        let encoded_bytes = encoder.into_writer().into_inner().bytes_written;
        debug_assert_eq!(encoded_bytes, finish_observed_encode());
        let handle_bytes = finish_payload_io_measurement();
        Ok(PayloadIoStats {
            resident_bytes: core::mem::size_of::<T>().saturating_add(handle_bytes),
            encoded_bytes,
            handle_bytes,
        })
    })();

    if result.is_err() {
        abort_payload_io_measurement();
        abort_observed_encode();
    }

    result
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
pub fn build_monitor_topology(config: &CuConfig, mission: &str) -> CuResult<MonitorTopology> {
    let graph = config.get_graph(Some(mission))?;
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
        let node_id = node.get_id();
        let usage = io_usage.get(node_id.as_str()).cloned().unwrap_or_default();
        let kind = match node.get_flavor() {
            Flavor::Bridge => ComponentType::Bridge,
            _ if !usage.has_incoming && usage.has_outgoing => ComponentType::Source,
            _ if usage.has_incoming && !usage.has_outgoing => ComponentType::Sink,
            _ => ComponentType::Task,
        };

        let mut inputs = Vec::new();
        let mut outputs = Vec::new();
        if kind == ComponentType::Bridge {
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

/// Runtime monitoring contract implemented by monitor components.
///
/// Lifecycle:
/// 1. [`CuMonitor::new`] is called once at runtime construction time.
/// 2. [`CuMonitor::start`] is called once before the first runtime iteration.
/// 3. For each iteration, [`CuMonitor::process_copperlist`] is called after component execution,
///    then [`CuMonitor::observe_copperlist_io`] after serialization accounting.
/// 4. [`CuMonitor::process_error`] is called synchronously when a monitored component step fails.
/// 5. [`CuMonitor::process_panic`] is called when the runtime catches a panic (`std` builds).
/// 6. [`CuMonitor::stop`] is called once during runtime shutdown.
///
/// Indexing model:
/// - `process_error(component_id, ..)` uses component indices into `metadata.components()`.
/// - `process_copperlist(..., view)` iterates CopperList slots with resolved component identity.
///
/// Error policy:
/// - [`Decision::Ignore`] continues execution.
/// - [`Decision::Abort`] aborts the current operation (step/copperlist scope).
/// - [`Decision::Shutdown`] triggers runtime shutdown.
pub trait CuMonitor: Sized {
    /// Construct the monitor once, before component execution starts.
    ///
    /// `metadata` contains mission/config/topology/static mapping information.
    /// `runtime` exposes dynamic runtime handles (for example execution probes).
    /// Use `metadata.monitor_config()` to decode monitor-specific parameters.
    fn new(metadata: CuMonitoringMetadata, runtime: CuMonitoringRuntime) -> CuResult<Self>
    where
        Self: Sized;

    /// Called once before processing the first CopperList.
    fn start(&mut self, _ctx: &CuContext) -> CuResult<()> {
        Ok(())
    }

    /// Called once per processed CopperList after component execution.
    fn process_copperlist(&self, _ctx: &CuContext, view: CopperListView<'_>) -> CuResult<()>;

    /// Called when runtime finishes CopperList serialization/IO accounting.
    fn observe_copperlist_io(&self, _stats: CopperListIoStats) {}

    /// Called when a monitored component step fails; must return an immediate runtime decision.
    ///
    /// `component_id` is an index into [`CuMonitoringMetadata::components`].
    fn process_error(
        &self,
        component_id: ComponentId,
        step: CuComponentState,
        error: &CuError,
    ) -> Decision;

    /// Called when the runtime catches a panic (`std` builds).
    fn process_panic(&self, _panic_message: &str) {}

    /// Called once during runtime shutdown.
    fn stop(&mut self, _ctx: &CuContext) -> CuResult<()> {
        Ok(())
    }
}

/// A do nothing monitor if no monitor is provided.
/// This is basically defining the default behavior of Copper in case of error.
pub struct NoMonitor {}
impl CuMonitor for NoMonitor {
    fn new(_metadata: CuMonitoringMetadata, _runtime: CuMonitoringRuntime) -> CuResult<Self> {
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

    fn process_copperlist(&self, _ctx: &CuContext, _view: CopperListView<'_>) -> CuResult<()> {
        // By default, do nothing.
        Ok(())
    }

    fn process_error(
        &self,
        _component_id: ComponentId,
        _step: CuComponentState,
        _error: &CuError,
    ) -> Decision {
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
            fn new(metadata: CuMonitoringMetadata, runtime: CuMonitoringRuntime) -> CuResult<Self>
            where
                Self: Sized,
            {
                Ok(($($name::new(metadata.clone(), runtime.clone())?,)+))
            }

            fn start(&mut self, ctx: &CuContext) -> CuResult<()> {
                $(self.$idx.start(ctx)?;)+
                Ok(())
            }

            fn process_copperlist(&self, ctx: &CuContext, view: CopperListView<'_>) -> CuResult<()> {
                $(self.$idx.process_copperlist(ctx, view)?;)+
                Ok(())
            }

            fn observe_copperlist_io(&self, stats: CopperListIoStats) {
                $(self.$idx.observe_copperlist_io(stats);)+
            }

            fn process_error(
                &self,
                component_id: ComponentId,
                step: CuComponentState,
                error: &CuError,
            ) -> Decision {
                let mut decision = Decision::Ignore;
                $(decision = merge_decision(decision, self.$idx.process_error(component_id, step, error));)+
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
    }

    impl TestMonitor {
        fn new_with(decision: TestDecision) -> Self {
            Self {
                decision,
                copperlist_calls: AtomicUsize::new(0),
                panic_calls: AtomicUsize::new(0),
            }
        }
    }

    fn test_metadata() -> CuMonitoringMetadata {
        const COMPONENTS: &[MonitorComponentMetadata] = &[
            MonitorComponentMetadata::new("a", ComponentType::Task, None),
            MonitorComponentMetadata::new("b", ComponentType::Task, None),
        ];
        CuMonitoringMetadata::new(
            CompactString::from(crate::config::DEFAULT_MISSION_ID),
            COMPONENTS,
            &[],
            CopperListInfo::new(0, 0),
            MonitorTopology::default(),
            None,
        )
        .expect("test metadata should be valid")
    }

    impl CuMonitor for TestMonitor {
        fn new(_metadata: CuMonitoringMetadata, runtime: CuMonitoringRuntime) -> CuResult<Self> {
            let monitor = Self::new_with(TestDecision::Ignore);
            #[cfg(feature = "std")]
            let _ = runtime.execution_probe();
            Ok(monitor)
        }

        fn process_copperlist(&self, _ctx: &CuContext, _view: CopperListView<'_>) -> CuResult<()> {
            self.copperlist_calls.fetch_add(1, Ordering::SeqCst);
            Ok(())
        }

        fn process_error(
            &self,
            _component_id: ComponentId,
            _step: CuComponentState,
            _error: &CuError,
        ) -> Decision {
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
            two.process_error(ComponentId::new(0), CuComponentState::Process, &err),
            Decision::Shutdown
        ));

        let two = (
            TestMonitor::new_with(TestDecision::Ignore),
            TestMonitor::new_with(TestDecision::Abort),
        );
        assert!(matches!(
            two.process_error(ComponentId::new(0), CuComponentState::Process, &err),
            Decision::Abort
        ));
    }

    #[test]
    fn tuple_monitor_fans_out_callbacks() {
        let monitors = <(TestMonitor, TestMonitor) as CuMonitor>::new(
            test_metadata(),
            CuMonitoringRuntime::unavailable(),
        )
        .expect("tuple new");
        let (ctx, _clock_control) = CuContext::new_mock_clock();
        let empty_view = test_metadata().layout().view(&[]);
        monitors
            .process_copperlist(&ctx, empty_view)
            .expect("process_copperlist should fan out");
        monitors.process_panic("panic marker");

        assert_eq!(monitors.0.copperlist_calls.load(Ordering::SeqCst), 1);
        assert_eq!(monitors.1.copperlist_calls.load(Ordering::SeqCst), 1);
        assert_eq!(monitors.0.panic_calls.load(Ordering::SeqCst), 1);
        assert_eq!(monitors.1.panic_calls.load(Ordering::SeqCst), 1);
    }

    fn encoded_size<E: Encode>(value: &E) -> usize {
        let mut encoder = EncoderImpl::<_, _>::new(SizeWriter::default(), standard());
        value
            .encode(&mut encoder)
            .expect("size measurement encoder should not fail");
        encoder.into_writer().bytes_written
    }

    #[test]
    fn payload_io_stats_tracks_encode_path_size_for_plain_payloads() {
        let payload = vec![1u8, 2, 3, 4];
        let io = payload_io_stats(&payload).expect("payload IO measurement should succeed");

        assert_eq!(io.encoded_bytes, encoded_size(&payload));
        assert_eq!(io.resident_bytes, core::mem::size_of::<Vec<u8>>());
        assert_eq!(io.handle_bytes, 0);
    }

    #[test]
    fn payload_io_stats_tracks_handle_backed_storage() {
        let payload = crate::pool::CuHandle::new_detached(vec![0u8; 32]);
        let io = payload_io_stats(&payload).expect("payload IO measurement should succeed");

        assert_eq!(io.encoded_bytes, encoded_size(&payload));
        assert_eq!(
            io.resident_bytes,
            core::mem::size_of::<crate::pool::CuHandle<Vec<u8>>>() + 32
        );
        assert_eq!(io.handle_bytes, 32);
    }

    #[test]
    fn runtime_execution_probe_roundtrip_marker() {
        let probe = RuntimeExecutionProbe::default();
        assert!(probe.marker().is_none());
        assert_eq!(probe.sequence(), 0);

        probe.record(ExecutionMarker {
            component_id: ComponentId::new(7),
            step: CuComponentState::Process,
            culistid: Some(42),
        });

        let marker = probe.marker().expect("marker should be available");
        assert_eq!(marker.component_id, ComponentId::new(7));
        assert!(matches!(marker.step, CuComponentState::Process));
        assert_eq!(marker.culistid, Some(42));
        assert_eq!(probe.sequence(), 1);
    }
}
