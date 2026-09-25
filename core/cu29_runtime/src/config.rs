//! This module defines the configuration of the copper runtime.
//! The configuration is a directed graph where nodes are tasks and edges are connections between tasks.
//! The configuration is serialized in the RON format.
//! The configuration is used to generate the runtime code at compile time.
#[cfg(not(feature = "std"))]
extern crate alloc;

use ConfigGraphs::{Missions, Simple};
#[cfg(feature = "std")]
use core::fmt;
#[cfg(feature = "std")]
use core::fmt::Display;
use cu29_traits::{CuError, CuResult};
use hashbrown::HashMap;
pub use petgraph::Direction::Incoming;
pub use petgraph::Direction::Outgoing;
use petgraph::stable_graph::{EdgeIndex, NodeIndex, StableDiGraph};
#[cfg(feature = "std")]
use petgraph::visit::IntoEdgeReferences;
use petgraph::visit::{Bfs, EdgeRef};
use ron::Options;
use ron::extensions::Extensions;
use serde::{Deserialize, Deserializer, Serialize, Serializer};

#[cfg(not(feature = "std"))]
use alloc::boxed::Box;
#[cfg(not(feature = "std"))]
use alloc::collections::BTreeMap;
#[cfg(not(feature = "std"))]
use alloc::vec;
#[cfg(feature = "std")]
use std::collections::BTreeMap;

#[cfg(not(feature = "std"))]
mod imp {
    pub use alloc::borrow::ToOwned;
    pub use alloc::format;
    pub use alloc::string::String;
    pub use alloc::string::ToString;
    pub use alloc::vec::Vec;
}

#[cfg(feature = "std")]
mod imp {
    pub use html_escape::encode_text;
    pub use std::fs::read_to_string;
}

use imp::*;

mod value;

pub use value::{
    ComponentConfig, ConfigError, ConstantConfig, ConstantNumber, ConstantStorage, Value,
};

/// NodeId is the unique identifier of a node in the configuration graph for petgraph
/// and the code generation.
pub type NodeId = u32;
pub const DEFAULT_MISSION_ID: &str = "default";
/// Default number of preallocated CopperLists compiled into a runtime.
#[doc(hidden)]
pub const DEFAULT_COPPERLIST_COUNT: usize = 2;

/// Logging policy for a `CuHandle`'s payload content.
///
/// Set by the source that produces the handle (typically via this enum's slot under
/// `NodeLogging`) and propagated through clones. The unified-log encoder reads this to
/// decide whether to write the payload bytes or just a metadata-only record for the
/// frame. See `cu29_runtime::pool::CuHandle` for the runtime side.
///
/// Defined here because this policy is part of the serialized runtime configuration.
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum HandleContent {
    /// Always log the full payload (current default).
    #[serde(rename = "all", alias = "All")]
    #[default]
    All = 0,
    /// Log the payload only if a downstream consumer called `CuHandle::mark_touched`.
    #[serde(rename = "touched_only", alias = "TouchedOnly")]
    TouchedOnly = 1,
    /// Never log the payload; keep only the surrounding metadata (timestamps, status).
    #[serde(rename = "none", alias = "None")]
    None = 2,
}

impl HandleContent {
    /// Reconstruct a [`HandleContent`] from its `AtomicU8` representation. Unknown
    /// values fall back to `All` so corrupt state never silently drops payload bytes.
    #[allow(dead_code)] // Only the lib's pool module calls this; the graph viewer doesn't.
    pub fn from_u8(v: u8) -> Self {
        match v {
            1 => HandleContent::TouchedOnly,
            2 => HandleContent::None,
            _ => HandleContent::All,
        }
    }
}

/// Configuration for logging in the node.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct NodeLogging {
    #[serde(default = "default_as_true")]
    enabled: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    codec: Option<String>,
    #[serde(default, skip_serializing_if = "HashMap::is_empty")]
    codecs: HashMap<String, String>,
    /// Logging policy applied to the source's pool-acquired `CuHandle`s. Surfaced
    /// in user RON config as e.g. `logging: ( handle_content: "touched_only" )`.
    #[serde(default, skip_serializing_if = "is_default_handle_content")]
    handle_content: HandleContent,
}

fn is_default_handle_content(c: &HandleContent) -> bool {
    *c == HandleContent::default()
}

impl NodeLogging {
    #[allow(dead_code)]
    pub fn enabled(&self) -> bool {
        self.enabled
    }

    #[allow(dead_code)]
    pub fn codec(&self) -> Option<&str> {
        self.codec.as_deref()
    }

    #[allow(dead_code)]
    pub fn codecs(&self) -> &HashMap<String, String> {
        &self.codecs
    }

    #[allow(dead_code)]
    pub fn codec_for_msg_type(&self, msg_type: &str) -> Option<&str> {
        self.codecs
            .get(msg_type)
            .map(String::as_str)
            .or(self.codec.as_deref())
    }

    /// Logging policy applied to handles minted by this node's pool. Defaults to
    /// `HandleContent::All` — i.e. existing behavior.
    pub fn handle_content(&self) -> HandleContent {
        self.handle_content
    }
}

impl Default for NodeLogging {
    fn default() -> Self {
        Self {
            enabled: true,
            codec: None,
            codecs: HashMap::new(),
            handle_content: HandleContent::default(),
        }
    }
}

/// Distinguishes regular tasks from bridge nodes so downstream stages can apply
/// bridge-specific instantiation rules.
#[derive(Default, Debug, Copy, Clone, PartialEq, Eq)]
pub enum Flavor {
    #[default]
    Task,
    Bridge,
}

/// Declares which Copper task trait a task node implements.
///
/// This lets config express the runtime role explicitly instead of forcing the
/// proc-macro to guess from graph shape alone.
#[derive(Serialize, Deserialize, Debug, Copy, Clone, PartialEq, Eq)]
pub enum TaskKind {
    #[serde(rename = "source", alias = "src")]
    Source,
    #[serde(rename = "task", alias = "regular", alias = "cutask")]
    Regular,
    /// A transform implementing `CuStatelessTask`. It has the regular graph
    /// shape but immutable per-CopperList callbacks.
    #[serde(rename = "stateless_task", alias = "stateless")]
    Stateless,
    #[serde(rename = "sink", alias = "snk")]
    Sink,
}

impl TaskKind {
    #[allow(dead_code)]
    pub fn as_str(&self) -> &'static str {
        match self {
            TaskKind::Source => "source",
            TaskKind::Regular => "task",
            TaskKind::Stateless => "stateless_task",
            TaskKind::Sink => "sink",
        }
    }
}

/// Default thread pool name used by `background: true` tasks.
pub const DEFAULT_BACKGROUND_POOL: &str = "background";

/// Reserved thread pool name driving the Pipeline execution engine. Applied to
/// each lane worker at startup; never task-bound nor built as a rayon pool.
#[allow(dead_code)] // consumed by cu29_derive; unused in some binary targets
pub const RT_POOL: &str = "rt";

/// How a task is backgrounded.
///
/// Either a simple on/off flag (`background: true`), which runs the task on the
/// default [`DEFAULT_BACKGROUND_POOL`] pool, or an explicit pool selection
/// (`background: (pool: "vision")`).
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
#[serde(untagged)]
pub enum BackgroundConfig {
    /// `background: true` / `background: false`.
    Flag(bool),
    /// `background: (pool: "vision")`.
    Pool { pool: String },
}

/// Refinement policy for an anytime node (`anytime:` on a task).
///
/// Every field is optional, but validation requires at least one - `time_budget_ms`,
/// `max_age_ms` and `max_refines`; see [`CuConfig::validate_anytime_configs`].
///
/// Two orthogonal axes organize the fields:
///
/// - **Budget** — how much to *spend* per result: `time_budget_ms` and
///   `max_refines` are hard bounds, `quality_target` and `max_stall` stop
///   spending early when more is provably not worth it.
/// - **Utility** — whether the result is *worth having* at all: `max_age_ms`
///   (worthless because too old) and `quality_floor` (worthless because too
///   crude).
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Default)]
pub struct AnytimeConfig {
    /// Wall-clock window for one job in milliseconds, measured from the start of
    /// the base computation and checked *between* refinement quanta.
    /// In background placement it is measured on the worker thread and may exceed the
    /// copperlist period.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub time_budget_ms: Option<f64>,

    /// Validity deadline in milliseconds, measured from the input's earliest time
    /// of validity (Tov): past this data age a result is no longer worth starting
    /// or waiting for.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_age_ms: Option<f64>,

    /// Stop refining early once the reported quality reaches this target, in
    /// `(0.0, 1.0]` on the normalized quality scale. Only valid for tasks that
    /// report a comparable quality.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub quality_target: Option<f32>,

    /// Publish only if the final reported quality is at least this floor, in
    /// `(0.0, 1.0)` on the normalized quality scale; below it the payload is
    /// cleared. Only valid for tasks that report a comparable quality.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub quality_floor: Option<f32>,

    /// Hard bound on refinement quanta per job.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_refines: Option<u32>,

    /// Stop after this many quanta without the published quality improving.
    /// Only valid for tasks that report a comparable quality.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_stall: Option<u32>,
}

impl AnytimeConfig {
    /// Validates the node-local invariants of this policy.
    ///
    /// Ranges are written as positive containment checks so a NaN coming from
    /// the RON fails the check and is rejected, and at least one hard bound is
    /// mandatory: `quality_target`, `max_stall` and `quality_floor` alone leave
    /// refinement unbounded.
    fn validate(&self, task_id: &str) -> CuResult<()> {
        if let Some(budget) = self.time_budget_ms {
            let valid = budget.is_finite() && budget > 0.0;
            if !valid {
                return Err(CuError::from(format!(
                    "Task '{task_id}': anytime.time_budget_ms must be a positive number of milliseconds (got {budget})."
                )));
            }
        }
        if let Some(age) = self.max_age_ms {
            let valid = age.is_finite() && age > 0.0;
            if !valid {
                return Err(CuError::from(format!(
                    "Task '{task_id}': anytime.max_age_ms must be a positive number of milliseconds (got {age})."
                )));
            }
        }
        if let Some(target) = self.quality_target {
            let valid = target > 0.0 && target <= 1.0;
            if !valid {
                return Err(CuError::from(format!(
                    "Task '{task_id}': anytime.quality_target must be within (0.0, 1.0] (got {target})."
                )));
            }
        }
        if let Some(floor) = self.quality_floor {
            let valid = floor > 0.0 && floor < 1.0;
            if !valid {
                return Err(CuError::from(format!(
                    "Task '{task_id}': anytime.quality_floor must be within (0.0, 1.0) (got {floor})."
                )));
            }
        }
        if let Some(refines) = self.max_refines
            && refines == 0
        {
            return Err(CuError::from(format!(
                "Task '{task_id}': anytime.max_refines must be at least 1."
            )));
        }
        if let Some(stall) = self.max_stall
            && stall == 0
        {
            return Err(CuError::from(format!(
                "Task '{task_id}': anytime.max_stall must be at least 1."
            )));
        }
        if let (Some(floor), Some(target)) = (self.quality_floor, self.quality_target)
            && floor > target
        {
            return Err(CuError::from(format!(
                "Task '{task_id}': anytime.quality_floor ({floor}) must not exceed anytime.quality_target ({target}): refinement could stop at the target and then always discard the result."
            )));
        }
        if self.time_budget_ms.is_none() && self.max_age_ms.is_none() && self.max_refines.is_none()
        {
            return Err(CuError::from(format!(
                "Task '{task_id}': anytime needs at least one hard bound: set time_budget_ms, max_age_ms or max_refines. quality_target, max_stall and quality_floor alone leave refinement unbounded."
            )));
        }
        Ok(())
    }
}

/// Whether a task output is transmitted or deterministically recomputed on the ground.
#[derive(Serialize, Deserialize, Debug, Clone, Copy, Default, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum StreamReplay {
    #[default]
    Capture,
    Reconstruct,
}

/// Static streaming contract for an ordinary deterministic task.
#[derive(Serialize, Deserialize, Debug, Clone, Copy, Default)]
#[serde(deny_unknown_fields)]
pub struct NodeStreaming {
    #[serde(default)]
    pub replay: StreamReplay,
}

/// A node in the configuration graph.
/// A node represents a Task in the system Graph.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct Node {
    /// Unique node identifier.
    id: String,

    /// Task rust struct underlying type, e.g. "mymodule::Sensor", etc.
    #[serde(rename = "type", skip_serializing_if = "Option::is_none")]
    type_: Option<String>,

    /// Declared Copper task role. When omitted, legacy configs still infer it
    /// from graph shape when that is unambiguous.
    #[serde(skip_serializing_if = "Option::is_none")]
    kind: Option<TaskKind>,

    /// Config passed to the task.
    #[serde(skip_serializing_if = "Option::is_none")]
    config: Option<ComponentConfig>,

    /// Resources requested by the task.
    #[serde(skip_serializing_if = "Option::is_none")]
    resources: Option<HashMap<String, String>>,

    /// Missions for which this task is run.
    missions: Option<Vec<String>>,

    /// Run this task in the background:
    /// ie. Will be set to run on a background thread and until it is finished `CuTask::process` will return None.
    ///
    /// Accepts either a simple flag (`background: true`, which uses the default
    /// [`DEFAULT_BACKGROUND_POOL`] pool) or an explicit pool selection
    /// (`background: (pool: "vision")`).
    #[serde(skip_serializing_if = "Option::is_none")]
    background: Option<BackgroundConfig>,

    /// Anytime refinement policy for this task (base + bounded refinements).
    ///
    /// Only supported on regular tasks. Orthogonal to `background:`, which adds
    /// the async placement layer on top of the refinement loop.
    #[serde(skip_serializing_if = "Option::is_none")]
    anytime: Option<AnytimeConfig>,

    /// Option to include/exclude stubbing for simulation.
    /// By default, sources and sinks are replaces (stubbed) by the runtime to avoid trying to compile hardware specific code for sensing or actuation.
    /// In some cases, for example a sink or source used as a middleware bridge, you might want to run the real code even in simulation.
    /// This option allows to control this behavior.
    /// Note: Normal tasks will be run in sim and this parameter ignored.
    #[serde(skip_serializing_if = "Option::is_none")]
    run_in_sim: Option<bool>,

    /// Config passed to the task.
    #[serde(skip_serializing_if = "Option::is_none")]
    logging: Option<NodeLogging>,

    #[serde(skip_serializing_if = "Option::is_none")]
    streaming: Option<NodeStreaming>,

    /// Node role in the runtime graph (normal task or bridge endpoint).
    #[serde(skip, default)]
    flavor: Flavor,
    /// Message types that are intentionally not connected (NC) in configuration.
    #[serde(skip, default)]
    nc_outputs: Vec<String>,
    /// Original config connection order for each NC output message type.
    #[serde(skip, default)]
    nc_output_orders: Vec<usize>,
}

impl Node {
    #[allow(dead_code)]
    pub fn new(id: &str, ptype: &str) -> Self {
        Node {
            id: id.to_string(),
            type_: Some(ptype.to_string()),
            kind: None,
            config: None,
            resources: None,
            missions: None,
            background: None,
            anytime: None,
            run_in_sim: None,
            logging: None,
            streaming: None,
            flavor: Flavor::Task,
            nc_outputs: Vec::new(),
            nc_output_orders: Vec::new(),
        }
    }

    #[allow(dead_code)]
    pub fn new_with_flavor(id: &str, ptype: &str, flavor: Flavor) -> Self {
        let mut node = Self::new(id, ptype);
        node.flavor = flavor;
        node
    }

    #[allow(dead_code)]
    pub fn get_id(&self) -> String {
        self.id.clone()
    }

    #[allow(dead_code)]
    pub fn get_type(&self) -> &str {
        self.type_.as_ref().unwrap()
    }

    #[allow(dead_code)]
    pub fn set_type(mut self, name: Option<String>) -> Self {
        self.type_ = name;
        self
    }

    #[allow(dead_code)]
    pub fn get_declared_task_kind(&self) -> Option<TaskKind> {
        self.kind
    }

    #[allow(dead_code)]
    pub fn set_task_kind(&mut self, kind: Option<TaskKind>) {
        self.kind = kind;
    }

    #[allow(dead_code)]
    pub fn set_resources<I>(&mut self, resources: Option<I>)
    where
        I: IntoIterator<Item = (String, String)>,
    {
        self.resources = resources.map(|iter| iter.into_iter().collect());
    }

    #[allow(dead_code)]
    pub fn is_background(&self) -> bool {
        match &self.background {
            Some(BackgroundConfig::Flag(flag)) => *flag,
            Some(BackgroundConfig::Pool { .. }) => true,
            None => false,
        }
    }

    /// Name of the thread pool this task should run on when backgrounded.
    /// Defaults to [`DEFAULT_BACKGROUND_POOL`] when no explicit pool is set.
    #[allow(dead_code)]
    pub fn background_pool(&self) -> &str {
        match &self.background {
            Some(BackgroundConfig::Pool { pool }) => pool.as_str(),
            _ => DEFAULT_BACKGROUND_POOL,
        }
    }

    #[allow(dead_code)]
    pub fn is_anytime(&self) -> bool {
        self.anytime.is_some()
    }

    /// Anytime refinement policy configured on this node, if any.
    #[allow(dead_code)]
    pub fn anytime(&self) -> Option<&AnytimeConfig> {
        self.anytime.as_ref()
    }

    /// Sets the anytime refinement policy for this node.
    #[allow(dead_code)]
    pub fn set_anytime(&mut self, anytime: Option<AnytimeConfig>) {
        self.anytime = anytime;
    }

    #[allow(dead_code)]
    pub fn get_instance_config(&self) -> Option<&ComponentConfig> {
        self.config.as_ref()
    }

    #[allow(dead_code)]
    pub fn get_resources(&self) -> Option<&HashMap<String, String>> {
        self.resources.as_ref()
    }

    /// By default, assume a source or a sink is not run in sim.
    /// Normal tasks will be run in sim and this parameter ignored.
    #[allow(dead_code)]
    pub fn is_run_in_sim(&self) -> bool {
        self.run_in_sim.unwrap_or(false)
    }

    #[allow(dead_code)]
    pub fn is_logging_enabled(&self) -> bool {
        if let Some(logging) = &self.logging {
            logging.enabled()
        } else {
            true
        }
    }

    /// Convenience wrapper around [`NodeLogging::handle_content`]: returns the per-handle
    /// logging policy for this node, defaulting to [`HandleContent::All`] when no
    /// `logging` block is configured.
    #[allow(dead_code)]
    pub fn handle_content_policy(&self) -> HandleContent {
        self.logging
            .as_ref()
            .map(NodeLogging::handle_content)
            .unwrap_or_default()
    }

    #[allow(dead_code)]
    pub fn streaming(&self) -> NodeStreaming {
        self.streaming.unwrap_or_default()
    }

    #[allow(dead_code)]
    pub fn get_logging(&self) -> Option<&NodeLogging> {
        self.logging.as_ref()
    }

    #[allow(dead_code)]
    pub fn get_param<T>(&self, key: &str) -> Result<Option<T>, ConfigError>
    where
        T: for<'a> TryFrom<&'a Value, Error = ConfigError>,
    {
        let pc = match self.config.as_ref() {
            Some(pc) => pc,
            None => return Ok(None),
        };
        let ComponentConfig(pc) = pc;
        match pc.get(key) {
            Some(v) => T::try_from(v).map(Some),
            None => Ok(None),
        }
    }

    #[allow(dead_code)]
    pub fn set_param<T: Into<Value>>(&mut self, key: &str, value: T) {
        if self.config.is_none() {
            self.config = Some(ComponentConfig(HashMap::new()));
        }
        let ComponentConfig(config) = self.config.as_mut().unwrap();
        config.insert(key.to_string(), value.into());
    }

    /// Returns whether this node is treated as a normal task or as a bridge.
    #[allow(dead_code)]
    pub fn get_flavor(&self) -> Flavor {
        self.flavor
    }

    /// Overrides the node flavor; primarily used when injecting bridge nodes.
    #[allow(dead_code)]
    pub fn set_flavor(&mut self, flavor: Flavor) {
        self.flavor = flavor;
    }

    /// Registers an intentionally unconnected output message type for this node.
    #[allow(dead_code)]
    pub fn add_nc_output(&mut self, msg_type: &str, order: usize) {
        if let Some(pos) = self
            .nc_outputs
            .iter()
            .position(|existing| existing == msg_type)
        {
            if order < self.nc_output_orders[pos] {
                self.nc_output_orders[pos] = order;
            }
            return;
        }
        self.nc_outputs.push(msg_type.to_string());
        self.nc_output_orders.push(order);
    }

    /// Returns message types intentionally marked as not connected.
    #[allow(dead_code)]
    pub fn nc_outputs(&self) -> &[String] {
        &self.nc_outputs
    }

    /// Returns NC outputs paired with original config order.
    #[allow(dead_code)]
    pub fn nc_outputs_with_order(&self) -> impl Iterator<Item = (&String, usize)> {
        self.nc_outputs
            .iter()
            .zip(self.nc_output_orders.iter().copied())
    }
}

/// Directional mapping for bridge channels.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub enum BridgeChannelConfigRepresentation {
    /// Channel that receives data from the bridge into the graph.
    Rx {
        id: String,
        /// Optional transport/topic identifier specific to the bridge backend.
        #[serde(skip_serializing_if = "Option::is_none")]
        route: Option<String>,
        /// Optional per-channel configuration forwarded to the bridge implementation.
        #[serde(skip_serializing_if = "Option::is_none")]
        config: Option<ComponentConfig>,
    },
    /// Channel that transmits data from the graph into the bridge.
    Tx {
        id: String,
        /// Optional transport/topic identifier specific to the bridge backend.
        #[serde(skip_serializing_if = "Option::is_none")]
        route: Option<String>,
        /// Optional per-channel configuration forwarded to the bridge implementation.
        #[serde(skip_serializing_if = "Option::is_none")]
        config: Option<ComponentConfig>,
    },
}

impl BridgeChannelConfigRepresentation {
    /// Stable logical identifier to reference this channel in connections.
    #[allow(dead_code)]
    pub fn id(&self) -> &str {
        match self {
            BridgeChannelConfigRepresentation::Rx { id, .. }
            | BridgeChannelConfigRepresentation::Tx { id, .. } => id,
        }
    }

    /// Bridge-specific transport path (topic, route, path...) describing this channel.
    #[allow(dead_code)]
    pub fn route(&self) -> Option<&str> {
        match self {
            BridgeChannelConfigRepresentation::Rx { route, .. }
            | BridgeChannelConfigRepresentation::Tx { route, .. } => route.as_deref(),
        }
    }
}

enum EndpointRole {
    Source,
    Destination,
}

fn validate_bridge_channel(
    bridge: &BridgeConfig,
    channel_id: &str,
    role: EndpointRole,
) -> Result<(), String> {
    let channel = bridge
        .channels
        .iter()
        .find(|ch| ch.id() == channel_id)
        .ok_or_else(|| {
            format!(
                "Bridge '{}' does not declare a channel named '{}'",
                bridge.id, channel_id
            )
        })?;

    match (role, channel) {
        (EndpointRole::Source, BridgeChannelConfigRepresentation::Rx { .. }) => Ok(()),
        (EndpointRole::Destination, BridgeChannelConfigRepresentation::Tx { .. }) => Ok(()),
        (EndpointRole::Source, BridgeChannelConfigRepresentation::Tx { .. }) => Err(format!(
            "Bridge '{}' channel '{}' is Tx and cannot act as a source",
            bridge.id, channel_id
        )),
        (EndpointRole::Destination, BridgeChannelConfigRepresentation::Rx { .. }) => Err(format!(
            "Bridge '{}' channel '{}' is Rx and cannot act as a destination",
            bridge.id, channel_id
        )),
    }
}

/// Declarative definition of a resource bundle.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ResourceBundleConfig {
    /// Resource inputs consumed by this provider at startup.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub resources: Option<HashMap<String, String>>,
    pub id: String,
    #[serde(rename = "provider")]
    pub provider: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub config: Option<ComponentConfig>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub missions: Option<Vec<String>>,
}

/// Static log-streaming policy compiled into a Copper application.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct LogStreamingConfig {
    pub destinations: Vec<LogStreamDestinationConfig>,
}

/// One statically generated log-stream destination.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct LogStreamDestinationConfig {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub feedback: Option<LogStreamFeedbackConfig>,
    pub id: String,
    pub transport: LogStreamTransportConfig,
    pub link: LogStreamLinkConfig,
    pub fec: LogStreamFecConfig,
    /// Recovery interval in CopperLists; a nonzero multiple of logging.keyframe_interval.
    pub recovery_interval: u32,
    pub max_record_bytes: u64,
}

/// Explicit reverse resource and advisory feedback policy for one destination.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct LogStreamFeedbackConfig {
    pub transport: LogStreamTransportConfig,
    pub report_interval_ms: u32,
    pub timeout_ms: u32,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub adaptation: Option<LogStreamAdaptationConfig>,
}

/// Bounds on the number of future source symbols between continuous repairs.
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct LogStreamAdaptationConfig {
    pub min_repair_every_source_symbols: u16,
    pub max_repair_every_source_symbols: u16,
}

/// Concrete Copper resource used as the destination's packet transmitter.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct LogStreamTransportConfig {
    #[serde(rename = "type")]
    pub type_: String,
    pub resource: String,
}

/// Physical-link assumptions and sender bounds.
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct LogStreamLinkConfig {
    pub mtu_bytes: u16,
    pub bitrate_bps: u64,
    pub memory_budget_kib: u32,
    pub max_latency_ms: u32,
    pub burst_packets: u32,
}

/// Explicit FEC policy. Lane identity fixes the algorithms: continuous is RLC
/// and objects is RaptorQ, so there is deliberately no configurable scheme.
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct LogStreamFecConfig {
    pub continuous: LogStreamContinuousFecConfig,
    pub objects: LogStreamObjectFecConfig,
}

#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct LogStreamContinuousFecConfig {
    pub field: LogStreamRlcField,
    pub window_symbols: u16,
    pub repair_every_source_symbols: u16,
    pub repair_density: LogStreamRepairDensity,
}

#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub enum LogStreamRlcField {
    Gf2,
    Gf256,
}

#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub enum LogStreamRepairDensity {
    Full,
    Threshold(u8),
}

impl LogStreamRepairDensity {
    pub const fn threshold(self) -> u8 {
        match self {
            Self::Full => 15,
            Self::Threshold(value) => value,
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct LogStreamObjectFecConfig {
    pub max_object_bytes: u64,
    pub repair_symbols_per_block: u32,
}

/// Declarative definition of a bridge component with a list of channels.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct BridgeConfig {
    pub id: String,
    #[serde(rename = "type")]
    pub type_: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub config: Option<ComponentConfig>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub resources: Option<HashMap<String, String>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub missions: Option<Vec<String>>,
    /// Whether this bridge should run as the real implementation in simulation mode.
    ///
    /// Default is `true` to preserve historical behavior where bridges were always
    /// instantiated in sim mode.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub run_in_sim: Option<bool>,
    /// List of logical endpoints exposed by this bridge.
    pub channels: Vec<BridgeChannelConfigRepresentation>,
}

impl BridgeConfig {
    /// By default, bridges run as real implementations in sim mode for backward compatibility.
    #[allow(dead_code)]
    pub fn is_run_in_sim(&self) -> bool {
        self.run_in_sim.unwrap_or(true)
    }

    fn to_node(&self) -> Node {
        let mut node = Node::new_with_flavor(&self.id, &self.type_, Flavor::Bridge);
        node.config = self.config.clone();
        node.resources = self.resources.clone();
        node.missions = self.missions.clone();
        node
    }
}

fn insert_bridge_node(graph: &mut CuGraph, bridge: &BridgeConfig) -> Result<(), String> {
    if graph.get_node_id_by_name(bridge.id.as_str()).is_some() {
        return Err(format!(
            "Bridge '{}' reuses an existing node id. Bridge ids must be unique.",
            bridge.id
        ));
    }
    graph
        .add_node(bridge.to_node())
        .map(|_| ())
        .map_err(|e| e.to_string())
}

/// Serialized representation of a connection used for the RON config.
#[derive(Serialize, Deserialize, Debug, Clone)]
struct SerializedCnx {
    src: String,
    dst: String,
    msg: String,
    missions: Option<Vec<String>>,
}

/// Special destination endpoint used to mark an output as intentionally not connected.
pub const NC_ENDPOINT: &str = "__nc__";

/// This represents a connection between 2 tasks (nodes) in the configuration graph.
#[derive(Debug, Clone)]
pub struct Cnx {
    /// Source node id.
    pub src: String,
    /// Destination node id.
    pub dst: String,
    /// Message type exchanged between src and dst.
    pub msg: String,
    /// Restrict this connection for this list of missions.
    pub missions: Option<Vec<String>>,
    /// Optional channel id when the source endpoint is a bridge.
    pub src_channel: Option<String>,
    /// Optional channel id when the destination endpoint is a bridge.
    pub dst_channel: Option<String>,
    /// Original serialized connection index used to preserve output ordering.
    pub order: usize,
}

impl From<&Cnx> for SerializedCnx {
    fn from(cnx: &Cnx) -> Self {
        SerializedCnx {
            src: format_endpoint(&cnx.src, cnx.src_channel.as_deref()),
            dst: format_endpoint(&cnx.dst, cnx.dst_channel.as_deref()),
            msg: cnx.msg.clone(),
            missions: cnx.missions.clone(),
        }
    }
}

fn format_endpoint(node: &str, channel: Option<&str>) -> String {
    match channel {
        Some(ch) => format!("{node}/{ch}"),
        None => node.to_string(),
    }
}

fn parse_endpoint(
    endpoint: &str,
    role: EndpointRole,
    bridges: &HashMap<&str, &BridgeConfig>,
) -> Result<(String, Option<String>), String> {
    if let Some((node, channel)) = endpoint.split_once('/') {
        if let Some(bridge) = bridges.get(node) {
            validate_bridge_channel(bridge, channel, role)?;
            return Ok((node.to_string(), Some(channel.to_string())));
        } else {
            return Err(format!(
                "Endpoint '{endpoint}' references an unknown bridge '{node}'"
            ));
        }
    }

    if let Some(bridge) = bridges.get(endpoint) {
        return Err(format!(
            "Bridge '{}' connections must reference a channel using '{}/<channel>'",
            bridge.id, bridge.id
        ));
    }

    Ok((endpoint.to_string(), None))
}

fn build_bridge_lookup(bridges: Option<&Vec<BridgeConfig>>) -> HashMap<&str, &BridgeConfig> {
    let mut map = HashMap::new();
    if let Some(bridges) = bridges {
        for bridge in bridges {
            map.insert(bridge.id.as_str(), bridge);
        }
    }
    map
}

fn mission_applies(missions: &Option<Vec<String>>, mission_id: &str) -> bool {
    missions
        .as_ref()
        .map(|mission_list| mission_list.iter().any(|m| m == mission_id))
        .unwrap_or(true)
}

fn merge_connection_missions(existing: &mut Option<Vec<String>>, incoming: &Option<Vec<String>>) {
    if incoming.is_none() {
        *existing = None;
        return;
    }
    if existing.is_none() {
        return;
    }

    if let (Some(existing_missions), Some(incoming_missions)) =
        (existing.as_mut(), incoming.as_ref())
    {
        for mission in incoming_missions {
            if !existing_missions
                .iter()
                .any(|existing_mission| existing_mission == mission)
            {
                existing_missions.push(mission.clone());
            }
        }
        existing_missions.sort();
        existing_missions.dedup();
    }
}

fn register_nc_output<E>(
    graph: &mut CuGraph,
    src_endpoint: &str,
    msg_type: &str,
    order: usize,
    bridge_lookup: &HashMap<&str, &BridgeConfig>,
) -> Result<(), E>
where
    E: From<String>,
{
    let (src_name, src_channel) =
        parse_endpoint(src_endpoint, EndpointRole::Source, bridge_lookup).map_err(E::from)?;
    if src_channel.is_some() {
        return Err(E::from(format!(
            "NC destination '{}' does not support bridge channels in source endpoint '{}'",
            NC_ENDPOINT, src_endpoint
        )));
    }

    let src = graph
        .get_node_id_by_name(src_name.as_str())
        .ok_or_else(|| E::from(format!("Source node not found: {src_endpoint}")))?;
    let src_node = graph
        .get_node_mut(src)
        .ok_or_else(|| E::from(format!("Source node id {src} not found for NC output")))?;
    if src_node.get_flavor() != Flavor::Task {
        return Err(E::from(format!(
            "NC destination '{}' is only supported for task outputs (source '{}')",
            NC_ENDPOINT, src_endpoint
        )));
    }
    src_node.add_nc_output(msg_type, order);
    Ok(())
}

/// A simple wrapper enum for `petgraph::Direction`,
/// designed to be converted *into* it via the `From` trait.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CuDirection {
    Outgoing,
    Incoming,
}

impl From<CuDirection> for petgraph::Direction {
    fn from(dir: CuDirection) -> Self {
        match dir {
            CuDirection::Outgoing => petgraph::Direction::Outgoing,
            CuDirection::Incoming => petgraph::Direction::Incoming,
        }
    }
}

#[derive(Default, Debug, Clone)]
pub struct CuGraph(pub StableDiGraph<Node, Cnx, NodeId>);

impl CuGraph {
    #[allow(dead_code)]
    pub fn get_all_nodes(&self) -> Vec<(NodeId, &Node)> {
        self.0
            .node_indices()
            .map(|index| (index.index() as u32, &self.0[index]))
            .collect()
    }

    #[allow(dead_code)]
    pub fn get_neighbor_ids(&self, node_id: NodeId, dir: CuDirection) -> Vec<NodeId> {
        self.0
            .neighbors_directed(node_id.into(), dir.into())
            .map(|petgraph_index| petgraph_index.index() as NodeId)
            .collect()
    }

    #[allow(dead_code)]
    pub fn node_ids(&self) -> Vec<NodeId> {
        self.0
            .node_indices()
            .map(|index| index.index() as NodeId)
            .collect()
    }

    #[allow(dead_code)]
    pub fn edge_id_between(&self, source: NodeId, target: NodeId) -> Option<usize> {
        self.0
            .find_edge(source.into(), target.into())
            .map(|edge| edge.index())
    }

    #[allow(dead_code)]
    pub fn edge(&self, edge_id: usize) -> Option<&Cnx> {
        self.0.edge_weight(EdgeIndex::new(edge_id))
    }

    #[allow(dead_code)]
    pub fn edges(&self) -> impl Iterator<Item = &Cnx> {
        self.0
            .edge_indices()
            .filter_map(|edge| self.0.edge_weight(edge))
    }

    #[allow(dead_code)]
    pub fn bfs_nodes(&self, start: NodeId) -> Vec<NodeId> {
        let mut visitor = Bfs::new(&self.0, start.into());
        let mut nodes = Vec::new();
        while let Some(node) = visitor.next(&self.0) {
            nodes.push(node.index() as NodeId);
        }
        nodes
    }

    #[allow(dead_code)]
    pub fn incoming_neighbor_count(&self, node_id: NodeId) -> usize {
        self.0.neighbors_directed(node_id.into(), Incoming).count()
    }

    #[allow(dead_code)]
    pub fn outgoing_neighbor_count(&self, node_id: NodeId) -> usize {
        self.0.neighbors_directed(node_id.into(), Outgoing).count()
    }

    pub fn node_indices(&self) -> Vec<petgraph::stable_graph::NodeIndex> {
        self.0.node_indices().collect()
    }

    pub fn add_node(&mut self, node: Node) -> CuResult<NodeId> {
        Ok(self.0.add_node(node).index() as NodeId)
    }

    #[allow(dead_code)]
    pub fn connection_exists(&self, source: NodeId, target: NodeId) -> bool {
        self.0.find_edge(source.into(), target.into()).is_some()
    }

    pub fn connect_ext(
        &mut self,
        source: NodeId,
        target: NodeId,
        msg_type: &str,
        missions: Option<Vec<String>>,
        src_channel: Option<String>,
        dst_channel: Option<String>,
    ) -> CuResult<()> {
        self.connect_ext_with_order(
            source,
            target,
            msg_type,
            missions,
            src_channel,
            dst_channel,
            usize::MAX,
        )
    }

    #[allow(clippy::too_many_arguments)]
    pub fn connect_ext_with_order(
        &mut self,
        source: NodeId,
        target: NodeId,
        msg_type: &str,
        missions: Option<Vec<String>>,
        src_channel: Option<String>,
        dst_channel: Option<String>,
        order: usize,
    ) -> CuResult<()> {
        let (src_id, dst_id) = (
            self.0
                .node_weight(source.into())
                .ok_or("Source node not found")?
                .id
                .clone(),
            self.0
                .node_weight(target.into())
                .ok_or("Target node not found")?
                .id
                .clone(),
        );

        let _ = self.0.add_edge(
            petgraph::stable_graph::NodeIndex::from(source),
            petgraph::stable_graph::NodeIndex::from(target),
            Cnx {
                src: src_id,
                dst: dst_id,
                msg: msg_type.to_string(),
                missions,
                src_channel,
                dst_channel,
                order,
            },
        );
        Ok(())
    }
    /// Get the node with the given id.
    /// If mission_id is provided, get the node from that mission's graph.
    /// Otherwise get the node from the simple graph.
    #[allow(dead_code)]
    pub fn get_node(&self, node_id: NodeId) -> Option<&Node> {
        self.0.node_weight(node_id.into())
    }

    #[allow(dead_code)]
    pub fn get_node_weight(&self, index: NodeId) -> Option<&Node> {
        self.0.node_weight(index.into())
    }

    #[allow(dead_code)]
    pub fn get_node_mut(&mut self, node_id: NodeId) -> Option<&mut Node> {
        self.0.node_weight_mut(node_id.into())
    }

    pub fn get_node_id_by_name(&self, name: &str) -> Option<NodeId> {
        self.0
            .node_indices()
            .into_iter()
            .find(|idx| self.0[*idx].get_id() == name)
            .map(|i| i.index() as NodeId)
    }

    #[allow(dead_code)]
    pub fn get_edge_weight(&self, index: usize) -> Option<Cnx> {
        self.0.edge_weight(EdgeIndex::new(index)).cloned()
    }

    #[allow(dead_code)]
    pub fn get_node_output_msg_type(&self, node_id: &str) -> Option<String> {
        self.get_node_output_msg_types(node_id)
            .and_then(|mut msgs| msgs.drain(..1).next())
    }

    #[allow(dead_code)]
    pub fn get_node_output_msg_types(&self, node_id: &str) -> Option<Vec<String>> {
        let node_id = self.get_node_id_by_name(node_id)?;
        let msgs = self.get_node_output_msg_types_by_id(node_id).ok()?;
        (!msgs.is_empty()).then_some(msgs)
    }

    #[allow(dead_code)]
    pub fn get_node_output_msg_types_by_id(&self, node_id: NodeId) -> CuResult<Vec<String>> {
        let mut edge_ids = self.get_src_edges(node_id)?;
        edge_ids.sort();

        let node = self
            .get_node(node_id)
            .ok_or_else(|| CuError::from(format!("Node id {node_id} not found")))?;

        let mut msg_order: Vec<(usize, String)> = Vec::new();
        let mut record_msg = |msg: String, order: usize| {
            if let Some((existing_order, _)) = msg_order
                .iter_mut()
                .find(|(_, existing_msg)| *existing_msg == msg)
            {
                if order < *existing_order {
                    *existing_order = order;
                }
                return;
            }
            msg_order.push((order, msg));
        };

        for edge_id in edge_ids {
            let Some(edge) = self.edge(edge_id) else {
                continue;
            };
            let order = if edge.order == usize::MAX {
                edge_id
            } else {
                edge.order
            };
            record_msg(edge.msg.clone(), order);
        }

        for (msg, order) in node.nc_outputs_with_order() {
            record_msg(msg.clone(), order);
        }

        msg_order.sort_by(|(order_a, msg_a), (order_b, msg_b)| {
            order_a.cmp(order_b).then_with(|| msg_a.cmp(msg_b))
        });
        Ok(msg_order.into_iter().map(|(_, msg)| msg).collect())
    }

    /// Channel-aware variant of [`CuGraph::get_node_output_msg_types_by_id`].
    ///
    /// Returns each distinct output port as `(msg_type, src_channel)`, where
    /// `src_channel` is `None` for ordinary task ports and `Some(id)` for
    /// bridge-channel ports. Unlike the msg-only variant, two ports sharing the
    /// same message type but living on different bridge channels are kept as
    /// separate entries, preserving their relative ordering (see #791).
    #[allow(dead_code)]
    pub fn get_node_output_ports_by_id(
        &self,
        node_id: NodeId,
    ) -> CuResult<Vec<(String, Option<String>)>> {
        let mut edge_ids = self.get_src_edges(node_id)?;
        edge_ids.sort();

        let node = self
            .get_node(node_id)
            .ok_or_else(|| CuError::from(format!("Node id {node_id} not found")))?;

        let mut port_order: Vec<(usize, String, Option<String>)> = Vec::new();
        let mut record_port = |msg: String, channel: Option<String>, order: usize| {
            if let Some((existing_order, _, _)) = port_order
                .iter_mut()
                .find(|(_, m, c)| *m == msg && *c == channel)
            {
                if order < *existing_order {
                    *existing_order = order;
                }
                return;
            }
            port_order.push((order, msg, channel));
        };

        for edge_id in edge_ids {
            let Some(edge) = self.edge(edge_id) else {
                continue;
            };
            let order = if edge.order == usize::MAX {
                edge_id
            } else {
                edge.order
            };
            record_port(edge.msg.clone(), edge.src_channel.clone(), order);
        }

        for (msg, order) in node.nc_outputs_with_order() {
            record_port(msg.clone(), None, order);
        }

        port_order.sort_by(|(order_a, msg_a, ch_a), (order_b, msg_b, ch_b)| {
            order_a
                .cmp(order_b)
                .then_with(|| msg_a.cmp(msg_b))
                .then_with(|| ch_a.cmp(ch_b))
        });
        Ok(port_order
            .into_iter()
            .map(|(_, msg, ch)| (msg, ch))
            .collect())
    }

    #[allow(dead_code)]
    pub fn get_node_input_msg_type(&self, node_id: &str) -> Option<String> {
        self.get_node_input_msg_types(node_id)
            .and_then(|mut v| v.pop())
    }

    pub fn get_node_input_msg_types(&self, node_id: &str) -> Option<Vec<String>> {
        self.0.node_indices().find_map(|node_index| {
            if let Some(node) = self.0.node_weight(node_index) {
                if node.id != node_id {
                    return None;
                }
                let edges: Vec<_> = self
                    .0
                    .edges_directed(node_index, Incoming)
                    .map(|edge| edge.id().index())
                    .collect();
                if edges.is_empty() {
                    return None;
                }
                let mut edges = edges;
                edges.sort();
                let msgs = edges
                    .into_iter()
                    .map(|edge_id| {
                        let cnx = self
                            .0
                            .edge_weight(EdgeIndex::new(edge_id))
                            .expect("Found an cnx id but could not retrieve it back");
                        cnx.msg.clone()
                    })
                    .collect();
                return Some(msgs);
            }
            None
        })
    }

    #[allow(dead_code)]
    pub fn get_connection_msg_type(&self, source: NodeId, target: NodeId) -> Option<&str> {
        self.0
            .find_edge(source.into(), target.into())
            .map(|edge_index| self.0[edge_index].msg.as_str())
    }

    /// Get the list of edges that are connected to the given node as a source.
    fn get_edges_by_direction(
        &self,
        node_id: NodeId,
        direction: petgraph::Direction,
    ) -> CuResult<Vec<usize>> {
        Ok(self
            .0
            .edges_directed(node_id.into(), direction)
            .map(|edge| edge.id().index())
            .collect())
    }

    pub fn get_src_edges(&self, node_id: NodeId) -> CuResult<Vec<usize>> {
        self.get_edges_by_direction(node_id, Outgoing)
    }

    /// Get the list of edges that are connected to the given node as a destination.
    pub fn get_dst_edges(&self, node_id: NodeId) -> CuResult<Vec<usize>> {
        self.get_edges_by_direction(node_id, Incoming)
    }

    #[allow(dead_code)]
    pub fn node_count(&self) -> usize {
        self.0.node_count()
    }

    #[allow(dead_code)]
    pub fn edge_count(&self) -> usize {
        self.0.edge_count()
    }

    /// Adds an edge between two nodes/tasks in the configuration graph.
    /// msg_type is the type of message exchanged between the two nodes/tasks.
    #[allow(dead_code)]
    pub fn connect(&mut self, source: NodeId, target: NodeId, msg_type: &str) -> CuResult<()> {
        self.connect_ext(source, target, msg_type, None, None, None)
    }
}

fn validate_task_kind(
    node_id: &str,
    kind: TaskKind,
    has_inputs: bool,
    has_outputs: bool,
) -> CuResult<()> {
    match kind {
        TaskKind::Source if has_inputs => Err(CuError::from(format!(
            "Task '{node_id}' is declared as kind 'source' but has incoming connections. Sources map to CuSrcTask and cannot consume inputs. Use kind: task instead."
        ))),
        TaskKind::Regular | TaskKind::Stateless if !has_inputs => Err(CuError::from(format!(
            "Task '{node_id}' is declared as kind '{}' but has no incoming connections. Transform tasks need at least one input connection. Use kind: source if it is input-free.",
            kind.as_str()
        ))),
        TaskKind::Sink if has_outputs => Err(CuError::from(format!(
            "Task '{node_id}' is declared as kind 'sink' but has outgoing or NC outputs. Sinks map to CuSinkTask and cannot produce outputs. Use kind: task instead."
        ))),
        TaskKind::Sink if !has_inputs => Err(CuError::from(format!(
            "Task '{node_id}' is declared as kind 'sink' but has no incoming connections. Sinks need at least one input connection so Copper can determine their input message type."
        ))),
        _ => Ok(()),
    }
}

#[allow(dead_code)]
pub fn infer_task_kind_for_id(graph: &CuGraph, node_id: NodeId) -> Option<TaskKind> {
    let node = graph.get_node(node_id)?;
    if node.get_flavor() != Flavor::Task {
        return None;
    }

    let has_inputs = !graph.get_dst_edges(node_id).ok()?.is_empty();
    let has_outputs = !graph
        .get_node_output_msg_types_by_id(node_id)
        .ok()?
        .is_empty();

    match (has_inputs, has_outputs) {
        (false, true) => Some(TaskKind::Source),
        (true, true) => Some(TaskKind::Regular),
        (true, false) => Some(TaskKind::Sink),
        (false, false) => None,
    }
}

#[allow(dead_code)]
pub fn resolve_task_kind_for_id(graph: &CuGraph, node_id: NodeId) -> CuResult<TaskKind> {
    let node = graph
        .get_node(node_id)
        .ok_or_else(|| CuError::from(format!("Task node id {node_id} not found")))?;
    if node.get_flavor() != Flavor::Task {
        return Err(CuError::from(format!(
            "Node '{}' is not a task and does not have a task kind.",
            node.id
        )));
    }

    let has_inputs = !graph.get_dst_edges(node_id)?.is_empty();
    let has_outputs = !graph.get_node_output_msg_types_by_id(node_id)?.is_empty();

    if let Some(kind) = node.get_declared_task_kind() {
        validate_task_kind(node.id.as_str(), kind, has_inputs, has_outputs)?;
        return Ok(kind);
    }

    let inferred = match (has_inputs, has_outputs) {
        (false, true) => TaskKind::Source,
        (true, true) => TaskKind::Regular,
        (true, false) => TaskKind::Sink,
        (false, false) => {
            return Err(CuError::from(format!(
                "Task '{}' has no declared inputs or outputs, so Copper cannot infer whether it is a source, task, or sink. Add `kind: source|task|sink`; source/task nodes also need an output declaration via a connection or `dst: \"{NC_ENDPOINT}\"`.",
                node.id
            )));
        }
    };

    validate_task_kind(node.id.as_str(), inferred, has_inputs, has_outputs)?;
    Ok(inferred)
}

impl core::ops::Index<NodeIndex> for CuGraph {
    type Output = Node;

    fn index(&self, index: NodeIndex) -> &Self::Output {
        &self.0[index]
    }
}

#[derive(Debug, Clone)]
pub enum ConfigGraphs {
    Simple(CuGraph),
    Missions(HashMap<String, CuGraph>),
}

impl ConfigGraphs {
    /// Returns a consistent hashmap of mission names to Graphs whatever the shape of the config is.
    /// Note: if there is only one anonymous mission it will be called "default"
    #[allow(dead_code)]
    pub fn get_all_missions_graphs(&self) -> HashMap<String, CuGraph> {
        match self {
            Simple(graph) => HashMap::from([(DEFAULT_MISSION_ID.to_string(), graph.clone())]),
            Missions(graphs) => graphs.clone(),
        }
    }

    #[allow(dead_code)]
    pub fn get_default_mission_graph(&self) -> CuResult<&CuGraph> {
        match self {
            Simple(graph) => Ok(graph),
            Missions(graphs) => {
                if graphs.len() == 1 {
                    Ok(graphs.values().next().unwrap())
                } else {
                    Err("Cannot get default mission graph from mission config".into())
                }
            }
        }
    }

    #[allow(dead_code)]
    pub fn get_graph(&self, mission_id: Option<&str>) -> CuResult<&CuGraph> {
        match self {
            Simple(graph) => match mission_id {
                None | Some(DEFAULT_MISSION_ID) => Ok(graph),
                Some(_) => Err("Cannot get mission graph from simple config".into()),
            },
            Missions(graphs) => {
                let id = mission_id
                    .ok_or_else(|| "Mission ID required for mission configs".to_string())?;
                graphs
                    .get(id)
                    .ok_or_else(|| format!("Mission {id} not found").into())
            }
        }
    }

    #[allow(dead_code)]
    pub fn get_graph_mut(&mut self, mission_id: Option<&str>) -> CuResult<&mut CuGraph> {
        match self {
            Simple(graph) => match mission_id {
                None => Ok(graph),
                Some(_) => Err("Cannot get mission graph from simple config".into()),
            },
            Missions(graphs) => {
                let id = mission_id
                    .ok_or_else(|| "Mission ID required for mission configs".to_string())?;
                graphs
                    .get_mut(id)
                    .ok_or_else(|| format!("Mission {id} not found").into())
            }
        }
    }

    pub fn add_mission(&mut self, mission_id: &str) -> CuResult<&mut CuGraph> {
        match self {
            Simple(_) => Err("Cannot add mission to simple config".into()),
            Missions(graphs) => match graphs.entry(mission_id.to_string()) {
                hashbrown::hash_map::Entry::Occupied(_) => {
                    Err(format!("Mission {mission_id} already exists").into())
                }
                hashbrown::hash_map::Entry::Vacant(entry) => Ok(entry.insert(CuGraph::default())),
            },
        }
    }
}

/// CuConfig is the programmatic representation of the configuration graph.
/// It is a directed graph where nodes are tasks and edges are connections between tasks.
///
/// The core of CuConfig is its `graphs` field which can be either a simple graph
/// or a collection of mission-specific graphs. The graph structure is based on petgraph.
#[derive(Debug, Clone)]
pub struct CuConfig {
    /// Values baked into the application by `#[copper_runtime]`.
    #[doc(hidden)]
    pub constants: Vec<ConstantConfig>,
    /// Monitoring configuration list.
    pub monitors: Vec<MonitorConfig>,
    /// Optional logging configuration
    pub logging: Option<LoggingConfig>,
    /// Optional runtime configuration
    pub runtime: Option<RuntimeConfig>,
    /// Declarative resource bundle definitions
    pub resources: Vec<ResourceBundleConfig>,
    /// Optional statically generated semantic log-stream destinations.
    pub log_streaming: Option<LogStreamingConfig>,
    /// Declarative bridge definitions that are yet to be expanded into the graph
    pub bridges: Vec<BridgeConfig>,
    /// Graph structure - either a single graph or multiple mission-specific graphs
    pub graphs: ConfigGraphs,
}

/// Every reconstructed node needs recorded or reconstructed inputs. Checking
/// direct edges at every reconstructed node also covers chains and fan-in.
fn validate_reconstruction_inputs(graph: &CuGraph) -> CuResult<()> {
    for index in graph.0.node_indices() {
        let node = &graph.0[index];
        if node.streaming().replay != StreamReplay::Reconstruct {
            continue;
        }
        for edge in graph.0.edges_directed(index, Incoming) {
            let source = &graph.0[edge.source()];
            if !source.is_logging_enabled() {
                return Err(CuError::from(format!(
                    "Task '{}' uses streaming.replay: reconstruct but input '{}' from '{}' has logging.enabled: false. Enable logging on '{}' or use streaming.replay: capture on '{}'.",
                    node.id,
                    edge.weight().msg,
                    source.id,
                    source.id,
                    node.id
                )));
            }
        }
    }
    Ok(())
}

impl CuConfig {
    /// Validates reconstruction inputs, static log-stream topology and bounds before code generation.
    pub fn validate_log_streaming_config(&self) -> CuResult<()> {
        match &self.graphs {
            Simple(graph) => validate_reconstruction_inputs(graph)?,
            Missions(graphs) => {
                for (mission, graph) in graphs {
                    validate_reconstruction_inputs(graph)
                        .map_err(|error| CuError::from(format!("Mission '{mission}': {error}")))?;
                }
            }
        }

        let Some(streaming) = &self.log_streaming else {
            return Ok(());
        };
        if streaming.destinations.is_empty() {
            return Err(CuError::from(
                "log_streaming.destinations must contain at least one destination",
            ));
        }

        let keyframe_interval = self
            .logging
            .as_ref()
            .and_then(|logging| logging.keyframe_interval)
            .unwrap_or(DEFAULT_KEYFRAME_INTERVAL);
        for (index, destination) in streaming.destinations.iter().enumerate() {
            if destination.id.trim().is_empty() {
                return Err(CuError::from(format!(
                    "log_streaming destination at index {index} has an empty id"
                )));
            }
            if streaming.destinations[..index]
                .iter()
                .any(|other| other.id == destination.id)
            {
                return Err(CuError::from(format!(
                    "Duplicate log_streaming destination id '{}'",
                    destination.id
                )));
            }
            if destination.transport.type_.trim().is_empty() {
                return Err(CuError::from(format!(
                    "log_streaming destination '{}' has an empty transport type",
                    destination.id
                )));
            }
            let Some((bundle_id, resource_name)) = destination.transport.resource.split_once('.')
            else {
                return Err(CuError::from(format!(
                    "log_streaming destination '{}' resource '{}' must use 'bundle.resource' syntax",
                    destination.id, destination.transport.resource
                )));
            };
            if bundle_id.is_empty()
                || resource_name.is_empty()
                || resource_name.contains('.')
                || !self.resources.iter().any(|bundle| bundle.id == bundle_id)
            {
                return Err(CuError::from(format!(
                    "log_streaming destination '{}' references invalid resource '{}'",
                    destination.id, destination.transport.resource
                )));
            }
            if streaming.destinations[..index]
                .iter()
                .any(|other| other.transport.resource == destination.transport.resource)
            {
                return Err(CuError::from(format!(
                    "log_streaming resource '{}' is bound by more than one destination",
                    destination.transport.resource
                )));
            }
            if let Some(feedback) = &destination.feedback {
                let baseline = destination.fec.continuous.repair_every_source_symbols;
                if feedback.report_interval_ms == 0
                    || feedback.timeout_ms <= feedback.report_interval_ms
                    || feedback.adaptation.is_some_and(|bounds| {
                        bounds.min_repair_every_source_symbols == 0
                            || bounds.min_repair_every_source_symbols > baseline
                            || baseline > bounds.max_repair_every_source_symbols
                    })
                {
                    return Err(CuError::from(
                        "Invalid log_streaming feedback cadence or FEC bounds",
                    ));
                }
                let resource = &feedback.transport.resource;
                let valid_resource = resource.split_once('.').is_some_and(|(bundle, slot)| {
                    !slot.is_empty()
                        && !slot.contains('.')
                        && self.resources.iter().any(|r| r.id == bundle)
                });
                if feedback.transport.type_.trim().is_empty() || !valid_resource {
                    return Err(CuError::from("Invalid log_streaming feedback resource"));
                }
            }
            // All logical receivers have one owner, including shared-carrier handles.
            let resources = core::iter::once(&destination.transport.resource)
                .chain(destination.feedback.iter().map(|f| &f.transport.resource));
            for resource in resources {
                let uses = streaming
                    .destinations
                    .iter()
                    .map(|d| {
                        usize::from(&d.transport.resource == resource)
                            + usize::from(
                                d.feedback
                                    .as_ref()
                                    .is_some_and(|f| &f.transport.resource == resource),
                            )
                    })
                    .sum::<usize>();
                if uses > 1 {
                    return Err(CuError::from(format!(
                        "Log-stream resource '{resource}' has multiple owners"
                    )));
                }
            }
            if destination.link.mtu_bytes <= 72 {
                return Err(CuError::from(format!(
                    "log_streaming destination '{}' mtu_bytes must exceed the 72-byte packet header",
                    destination.id
                )));
            }
            if destination.link.bitrate_bps == 0
                || destination.link.memory_budget_kib == 0
                || destination.link.max_latency_ms == 0
                || destination.link.burst_packets == 0
            {
                return Err(CuError::from(format!(
                    "log_streaming destination '{}' link values must be nonzero",
                    destination.id
                )));
            }
            if destination.fec.continuous.window_symbols == 0
                || destination.fec.continuous.repair_every_source_symbols == 0
            {
                return Err(CuError::from(format!(
                    "log_streaming destination '{}' continuous FEC values must be nonzero",
                    destination.id
                )));
            }
            if destination.fec.continuous.repair_density.threshold() > 15 {
                return Err(CuError::from(format!(
                    "log_streaming destination '{}' repair density threshold must be in 0..=15",
                    destination.id
                )));
            }
            if destination.fec.objects.max_object_bytes == 0
                || destination.fec.objects.repair_symbols_per_block == 0
                || destination.max_record_bytes == 0
            {
                return Err(CuError::from(format!(
                    "log_streaming destination '{}' object and record bounds must be nonzero",
                    destination.id
                )));
            }
            if destination.recovery_interval == 0
                || !destination
                    .recovery_interval
                    .is_multiple_of(keyframe_interval)
            {
                return Err(CuError::from(format!(
                    "log_streaming destination '{}' recovery_interval must be a nonzero multiple of logging.keyframe_interval ({keyframe_interval})",
                    destination.id
                )));
            }
        }
        Ok(())
    }

    /// Guarantees that a default `"background"` thread pool entry exists in
    /// `runtime.thread_pools` whenever the graph has any `background: true`
    /// task that didn't explicitly select a pool. Thread pools are otherwise
    /// constructed straight from `runtime.thread_pools` by the runtime — they
    /// are not stored in `ResourceManager`.
    #[cfg(feature = "std")]
    fn ensure_default_background_pool(&mut self) {
        if !self.has_background_tasks() {
            return;
        }

        const DEFAULT_BACKGROUND_THREADS: usize = 2;

        let runtime = self.runtime.get_or_insert_with(RuntimeConfig::default);
        if !runtime
            .thread_pools
            .iter()
            .any(|pool| pool.id == DEFAULT_BACKGROUND_POOL)
        {
            runtime.thread_pools.push(ThreadPoolConfig {
                id: DEFAULT_BACKGROUND_POOL.to_string(),
                threads: DEFAULT_BACKGROUND_THREADS,
                affinity: None,
                policy: SchedulingPolicy::Fair,
                on_error: OnError::Warn,
            });
        }
    }

    /// The configured planner selection, if any (absent means serial execution).
    pub fn planner_config(&self) -> Option<&PlannerConfig> {
        self.runtime.as_ref()?.planner.as_ref()
    }

    /// Execution strategy selected by the configuration.
    pub fn planner_kind(&self) -> PlannerKind {
        self.planner_config()
            .map_or(PlannerKind::Serial, PlannerConfig::kind)
    }

    #[cfg(feature = "std")]
    fn has_background_tasks(&self) -> bool {
        match &self.graphs {
            ConfigGraphs::Simple(graph) => graph
                .get_all_nodes()
                .iter()
                .any(|(_, node)| node.is_background()),
            ConfigGraphs::Missions(graphs) => graphs.values().any(|graph| {
                graph
                    .get_all_nodes()
                    .iter()
                    .any(|(_, node)| node.is_background())
            }),
        }
    }
}

#[derive(Serialize, Deserialize, Default, Debug, Clone)]
pub struct MonitorConfig {
    #[serde(rename = "type")]
    type_: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    config: Option<ComponentConfig>,
}

impl MonitorConfig {
    #[allow(dead_code)]
    pub fn get_type(&self) -> &str {
        &self.type_
    }

    #[allow(dead_code)]
    pub fn get_config(&self) -> Option<&ComponentConfig> {
        self.config.as_ref()
    }
}

fn default_as_true() -> bool {
    true
}

pub const DEFAULT_KEYFRAME_INTERVAL: u32 = 100;

fn default_keyframe_interval() -> Option<u32> {
    Some(DEFAULT_KEYFRAME_INTERVAL)
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct LoggingConfig {
    /// Enable task logging to the log file.
    #[serde(default = "default_as_true", skip_serializing_if = "Clone::clone")]
    pub enable_task_logging: bool,

    /// Generate and record task-state keyframes.
    ///
    /// Without another generated keyframe consumer, this is a compile-time application
    /// property and `#[copper_runtime]` emits no capture calls when it is `false`.
    /// A log-stream destination independently requests keyframe capture for recovery points.
    #[serde(default = "default_as_true", skip_serializing_if = "Clone::clone")]
    pub enable_keyframe_logging: bool,

    /// Number of preallocated CopperLists available to the runtime.
    ///
    /// This is consumed by proc-macro codegen and must match the value compiled into the
    /// application binary.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub copperlist_count: Option<usize>,

    /// Size of each slab in the log file. (it is the size of the memory mapped file at a time)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub slab_size_mib: Option<u64>,

    /// Pre-allocated size for each section in the log file.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub section_size_mib: Option<u64>,

    /// Interval in copperlists between two "keyframes" in the log file i.e. freezing tasks.
    #[serde(
        default = "default_keyframe_interval",
        skip_serializing_if = "Option::is_none"
    )]
    pub keyframe_interval: Option<u32>,

    /// Named log codec specs reusable across task output bindings.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub codecs: Vec<LoggingCodecSpec>,
}

impl Default for LoggingConfig {
    fn default() -> Self {
        Self {
            enable_task_logging: true,
            enable_keyframe_logging: true,
            copperlist_count: None,
            slab_size_mib: None,
            section_size_mib: None,
            keyframe_interval: default_keyframe_interval(),
            codecs: Vec::new(),
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct LoggingCodecSpec {
    pub id: String,
    #[serde(rename = "type")]
    pub type_: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub config: Option<ComponentConfig>,
}

#[derive(Serialize, Deserialize, Default, Debug, Clone)]
pub struct RuntimeConfig {
    /// Set a CopperList execution rate target in Hz
    /// It will act as a rate limiter: if the execution is slower than this rate,
    /// it will continue to execute at "best effort".
    ///
    /// The main usecase is to not waste cycles when the system doesn't need an unbounded execution rate.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub rate_target_hz: Option<u64>,

    /// Declarative thread pool definitions used by the background-task pools and
    /// the Pipeline execution engine. Each pool carries an optional CPU
    /// affinity and a scheduling policy/priority.
    ///
    /// This is a `std`-only concept; on `no_std`/embedded targets there are no
    /// threads and this section is ignored.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub thread_pools: Vec<ThreadPoolConfig>,

    /// Execution planner selection, one for the whole config.
    ///
    /// This is a codegen input: `#[copper_runtime]` bakes the resulting plan
    /// into the binary. Editing it in a deployed app's RON at startup does not
    /// change the compiled plan (same class as `logging.copperlist_count`); the
    /// RON must match the binary that wrote the log.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub planner: Option<PlannerConfig>,
}

/// Selects the compile-time execution strategy for every mission graph.
///
/// Pipeline execution is selected explicitly with:
///
/// ```text
/// runtime: (
///     planner: (
///         kind: Pipeline,
///         config: { "max_in_flight": 2 },
///     ),
/// )
/// ```
///
/// `Pipeline` requires the `parallel-rt` Cargo feature. Omitting `planner`
/// selects [`PlannerKind::Serial`], even when that feature is enabled.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct PlannerConfig {
    pub(crate) kind: PlannerKind,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub(crate) config: Option<ComponentConfig>,
}

impl PlannerConfig {
    #[allow(dead_code)]
    pub fn kind(&self) -> PlannerKind {
        self.kind
    }

    #[allow(dead_code)]
    pub fn get_config(&self) -> Option<&ComponentConfig> {
        self.config.as_ref()
    }

    /// Pipeline capacity, defaulting to the number of preallocated CopperLists.
    pub fn max_in_flight(&self, copperlist_count: usize) -> CuResult<usize> {
        if self.kind != PlannerKind::Pipeline {
            return Ok(1);
        }
        let configured = match self.config.as_ref() {
            Some(config) => config
                .get_value::<usize>("max_in_flight")?
                .unwrap_or(copperlist_count),
            None => copperlist_count,
        };
        if configured == 0 {
            return Err(CuError::from(
                "Pipeline planner max_in_flight cannot be zero.",
            ));
        }
        if configured > copperlist_count {
            return Err(CuError::from(format!(
                "Pipeline planner max_in_flight ({configured}) exceeds logging.copperlist_count ({copperlist_count})."
            )));
        }
        Ok(configured)
    }
}

/// Built-in compile-time execution strategies.
#[derive(Serialize, Deserialize, Default, Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlannerKind {
    /// Deterministic single-threaded execution in graph order.
    #[default]
    Serial,
    /// Deterministic single-threaded execution in the configured task order.
    TaskOrder,
    /// An exact portable schedule validated and compiled with the application.
    ExplicitSchedule,
    /// One ordered worker lane per process stage.
    Pipeline,
}

/// Smallest valid real-time priority for [`SchedulingPolicy::Fifo`]/[`SchedulingPolicy::RoundRobin`].
pub const MIN_RT_PRIORITY: u8 = 1;
/// Largest valid real-time priority for [`SchedulingPolicy::Fifo`]/[`SchedulingPolicy::RoundRobin`].
pub const MAX_RT_PRIORITY: u8 = 99;
/// Lowest valid niceness for [`SchedulingPolicy::Nice`] (most favorable).
pub const MIN_NICE: i8 = -20;
/// Highest valid niceness for [`SchedulingPolicy::Nice`] (least favorable).
pub const MAX_NICE: i8 = 19;

/// Scheduling policy applied to every worker thread of a [`ThreadPoolConfig`].
///
/// On Linux these map directly onto the POSIX scheduling policies. On other
/// platforms they are applied best-effort (see the per-pool
/// [`ThreadPoolConfig::on_error`] behavior).
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum SchedulingPolicy {
    /// Normal fair time-sharing scheduler (`SCHED_OTHER`/CFS on Linux) with default
    /// niceness. The OS shares the CPU fairly across threads and no thread starves.
    ///
    /// Use for everything that isn't latency-critical. This is the default.
    #[default]
    Fair,
    /// Fair scheduler with an explicit niceness (`-20..=19`, lower is more favorable).
    ///
    /// A soft priority hint, not a guarantee: a higher (nicer) value yields the CPU
    /// more readily. Use to bias a pool below or above normal work without leaving
    /// the fair scheduler — e.g. `Nice(10)` for heavy background work that should
    /// step aside for the control loop.
    Nice(i8),
    /// `SCHED_FIFO` real-time policy, priority `1..=99` (higher wins).
    ///
    /// Hard real-time: a FIFO thread runs ahead of every fair thread and is not
    /// time-sliced — it runs until it blocks or a higher-priority RT thread preempts
    /// it. Use for the latency-critical pipeline, and pin it with `affinity` so a
    /// busy worker cannot starve other work on the same core. Linux-only; typically
    /// needs `CAP_SYS_NICE`.
    Fifo { priority: u8 },
    /// `SCHED_RR` real-time policy, priority `1..=99` (higher wins).
    ///
    /// Same real-time semantics as [`Fifo`](Self::Fifo), except threads at the same
    /// priority are round-robin time-sliced rather than run-to-block. Use when
    /// several RT workers share a priority and should interleave fairly. Linux-only;
    /// typically needs `CAP_SYS_NICE`.
    RoundRobin { priority: u8 },
}

/// What to do when a pool's affinity or scheduling request cannot be applied
/// (for example, setting a real-time priority without `CAP_SYS_NICE`).
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum OnError {
    /// Log a warning and fall back to default scheduling. This keeps unprivileged
    /// dev/laptop runs working out of the box.
    #[default]
    Warn,
    /// Hard-fail at startup if the requested affinity/scheduler cannot be applied.
    /// Use this for deployed real-time robots that must fail loudly.
    Strict,
}

/// Declarative definition of a single thread pool.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct ThreadPoolConfig {
    /// Unique pool id. Reserved ids: [`RT_POOL`] (the Pipeline execution
    /// engine) and [`DEFAULT_BACKGROUND_POOL`] (the default background pool).
    pub id: String,
    /// Number of worker threads in the pool.
    pub threads: usize,
    /// Optional set of logical CPU cores the pool may use. When set, worker `i`
    /// is pinned to `affinity[i % affinity.len()]` (Spread): `threads ==
    /// affinity.len()` yields one worker pinned per dedicated core.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub affinity: Option<Vec<usize>>,
    /// Scheduling policy/priority applied to each worker thread.
    #[serde(default)]
    pub policy: SchedulingPolicy,
    /// What to do if affinity/scheduling cannot be applied.
    #[serde(default)]
    pub on_error: OnError,
}

/// Validates the declarative thread pool definitions of a runtime config.
///
/// Checks ids are non-empty and unique, thread counts are non-zero, real-time
/// priorities and niceness values are in range, and affinity lists are non-empty
/// when present. This is purely a config-level check; pools are built later.
fn validate_thread_pools<E>(runtime: &Option<RuntimeConfig>) -> Result<(), E>
where
    E: From<String>,
{
    let Some(runtime) = runtime else {
        return Ok(());
    };

    let mut seen: Vec<&str> = Vec::new();
    for pool in &runtime.thread_pools {
        if pool.id.is_empty() {
            return Err(E::from("Thread pool id cannot be empty".to_string()));
        }
        if seen.contains(&pool.id.as_str()) {
            return Err(E::from(format!("Duplicate thread pool id '{}'", pool.id)));
        }
        seen.push(pool.id.as_str());

        if pool.threads == 0 {
            return Err(E::from(format!(
                "Thread pool '{}' must have at least 1 thread",
                pool.id
            )));
        }

        match pool.policy {
            SchedulingPolicy::Fifo { priority } | SchedulingPolicy::RoundRobin { priority } => {
                if !(MIN_RT_PRIORITY..=MAX_RT_PRIORITY).contains(&priority) {
                    return Err(E::from(format!(
                        "Thread pool '{}' real-time priority {priority} is out of range ({MIN_RT_PRIORITY}..={MAX_RT_PRIORITY})",
                        pool.id
                    )));
                }
            }
            SchedulingPolicy::Nice(nice) => {
                if !(MIN_NICE..=MAX_NICE).contains(&nice) {
                    return Err(E::from(format!(
                        "Thread pool '{}' niceness {nice} is out of range ({MIN_NICE}..={MAX_NICE})",
                        pool.id
                    )));
                }
            }
            SchedulingPolicy::Fair => {}
        }

        if let Some(affinity) = &pool.affinity
            && affinity.is_empty()
        {
            return Err(E::from(format!(
                "Thread pool '{}' has an empty affinity list; omit `affinity` for no pinning",
                pool.id
            )));
        }
    }

    Ok(())
}

/// Maximum representable Copper runtime rate target in whole Hertz.
///
/// Copper stores runtime periods in integer nanoseconds, so anything above 1 GHz
/// would round down to a zero-duration period.
pub const MAX_RATE_TARGET_HZ: u64 = 1_000_000_000;

/// Missions are used to generate alternative DAGs within the same configuration.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct MissionsConfig {
    pub id: String,
}

/// A compile-time predicate controlling whether a configuration fragment is included.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum ConfigPredicate {
    Feature(String),
    Not(Box<ConfigPredicate>),
    All(Vec<ConfigPredicate>),
    Any(Vec<ConfigPredicate>),
}

#[cfg(feature = "std")]
impl ConfigPredicate {
    fn evaluate(&self, active_features: &[&str]) -> bool {
        match self {
            Self::Feature(feature) => active_features.contains(&feature.as_str()),
            Self::Not(predicate) => !predicate.evaluate(active_features),
            Self::All(predicates) => predicates
                .iter()
                .all(|predicate| predicate.evaluate(active_features)),
            Self::Any(predicates) => predicates
                .iter()
                .any(|predicate| predicate.evaluate(active_features)),
        }
    }
}

/// Includes are used to include other configuration files.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct IncludesConfig {
    pub path: String,
    #[serde(default)]
    pub params: HashMap<String, Value>,
    #[serde(default)]
    pub missions: Option<Vec<String>>,
    #[serde(default)]
    pub when: Option<ConfigPredicate>,
}

/// One subsystem participating in a multi-Copper deployment.
#[cfg(feature = "std")]
#[allow(dead_code)]
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct MultiCopperSubsystemConfig {
    pub id: String,
    pub config: String,
}

/// One explicit interconnect between two subsystem bridge channels.
#[cfg(feature = "std")]
#[allow(dead_code)]
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct MultiCopperInterconnectConfig {
    pub from: String,
    pub to: String,
    pub msg: String,
    #[serde(default)]
    pub when: Option<ConfigPredicate>,
}

/// One path-based config overlay applied to a parsed local Copper config.
#[cfg(feature = "std")]
#[allow(dead_code)]
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct InstanceConfigSetOperation {
    pub path: String,
    pub value: ComponentConfig,
}

/// Typed endpoint reference used by validated multi-Copper interconnects.
#[cfg(feature = "std")]
#[allow(dead_code)]
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct MultiCopperEndpoint {
    pub subsystem_id: String,
    pub bridge_id: String,
    pub channel_id: String,
}

#[cfg(feature = "std")]
impl Display for MultiCopperEndpoint {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{}/{}/{}",
            self.subsystem_id, self.bridge_id, self.channel_id
        )
    }
}

/// Validated subsystem entry with its compiler-assigned numeric subsystem code and parsed local Copper config.
#[cfg(feature = "std")]
#[allow(dead_code)]
#[derive(Debug, Clone)]
pub struct MultiCopperSubsystem {
    pub id: String,
    pub subsystem_code: u16,
    pub config_path: String,
    pub config: CuConfig,
}

/// Validated explicit interconnect between two subsystem endpoints.
#[cfg(feature = "std")]
#[allow(dead_code)]
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MultiCopperInterconnect {
    pub from: MultiCopperEndpoint,
    pub to: MultiCopperEndpoint,
    pub msg: String,
    pub bridge_type: String,
}

/// Strict umbrella configuration describing multiple Copper subsystems and their explicit links.
#[cfg(feature = "std")]
#[allow(dead_code)]
#[derive(Debug, Clone)]
pub struct MultiCopperConfig {
    pub subsystems: Vec<MultiCopperSubsystem>,
    pub interconnects: Vec<MultiCopperInterconnect>,
    pub instance_overrides_root: Option<String>,
}

#[cfg(feature = "std")]
impl MultiCopperConfig {
    #[allow(dead_code)]
    pub fn subsystem(&self, id: &str) -> Option<&MultiCopperSubsystem> {
        self.subsystems.iter().find(|subsystem| subsystem.id == id)
    }

    #[allow(dead_code)]
    pub fn resolve_subsystem_config_for_instance(
        &self,
        subsystem_id: &str,
        instance_id: u32,
    ) -> CuResult<CuConfig> {
        let subsystem = self.subsystem(subsystem_id).ok_or_else(|| {
            CuError::from(format!(
                "Multi-Copper config does not define subsystem '{}'.",
                subsystem_id
            ))
        })?;
        let mut config = subsystem.config.clone();

        let Some(root) = &self.instance_overrides_root else {
            return Ok(config);
        };

        let override_path = std::path::Path::new(root)
            .join(instance_id.to_string())
            .join(format!("{subsystem_id}.ron"));
        if !override_path.exists() {
            return Ok(config);
        }

        apply_instance_overrides_from_file(&mut config, &override_path)?;
        Ok(config)
    }
}

#[cfg(feature = "std")]
#[allow(dead_code)]
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
struct MultiCopperConfigRepresentation {
    subsystems: Vec<MultiCopperSubsystemConfig>,
    interconnects: Vec<MultiCopperInterconnectConfig>,
    instance_overrides_root: Option<String>,
}

#[cfg(feature = "std")]
#[derive(Serialize, Deserialize, Debug, Clone, Default)]
struct InstanceConfigOverridesRepresentation {
    #[serde(default)]
    set: Vec<InstanceConfigSetOperation>,
}

#[cfg(feature = "std")]
#[allow(dead_code)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum MultiCopperChannelDirection {
    Rx,
    Tx,
}

#[cfg(feature = "std")]
#[allow(dead_code)]
#[derive(Debug, Clone)]
struct MultiCopperChannelContract {
    bridge_type: String,
    direction: MultiCopperChannelDirection,
    msg: Option<String>,
}

#[cfg(feature = "std")]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum InstanceConfigTargetKind {
    Task,
    Resource,
    Bridge,
}

/// This is the main Copper configuration representation.
#[derive(Serialize, Deserialize, Default)]
struct CuConfigRepresentation {
    constants: Option<Vec<ConstantConfig>>,
    tasks: Option<Vec<Node>>,
    resources: Option<Vec<ResourceBundleConfig>>,
    log_streaming: Option<LogStreamingConfig>,
    bridges: Option<Vec<BridgeConfig>>,
    cnx: Option<Vec<SerializedCnx>>,
    #[serde(
        default,
        alias = "monitor",
        deserialize_with = "deserialize_monitor_configs"
    )]
    monitors: Option<Vec<MonitorConfig>>,
    logging: Option<LoggingConfig>,
    runtime: Option<RuntimeConfig>,
    missions: Option<Vec<MissionsConfig>>,
    includes: Option<Vec<IncludesConfig>>,
}

#[derive(Deserialize)]
#[serde(untagged)]
enum OneOrManyMonitorConfig {
    One(MonitorConfig),
    Many(Vec<MonitorConfig>),
}

fn deserialize_monitor_configs<'de, D>(
    deserializer: D,
) -> Result<Option<Vec<MonitorConfig>>, D::Error>
where
    D: Deserializer<'de>,
{
    let parsed = Option::<OneOrManyMonitorConfig>::deserialize(deserializer)?;
    Ok(parsed.map(|value| match value {
        OneOrManyMonitorConfig::One(single) => vec![single],
        OneOrManyMonitorConfig::Many(many) => many,
    }))
}

/// Shared implementation for deserializing a CuConfigRepresentation into a CuConfig
fn deserialize_config_representation<E>(
    representation: &CuConfigRepresentation,
) -> Result<CuConfig, E>
where
    E: From<String>,
{
    let mut cuconfig = CuConfig::default();
    let bridge_lookup = build_bridge_lookup(representation.bridges.as_ref());

    if let Some(mission_configs) = &representation.missions {
        // This is the multi-mission case
        let mut missions = Missions(HashMap::new());

        for mission_config in mission_configs {
            let mission_id = mission_config.id.as_str();
            let graph = missions
                .add_mission(mission_id)
                .map_err(|e| E::from(e.to_string()))?;

            if let Some(tasks) = &representation.tasks {
                for task in tasks {
                    if let Some(task_missions) = &task.missions {
                        // if there is a filter by mission on the task, only add the task to the mission if it matches the filter.
                        if task_missions.contains(&mission_id.to_owned()) {
                            graph
                                .add_node(task.clone())
                                .map_err(|e| E::from(e.to_string()))?;
                        }
                    } else {
                        // if there is no filter by mission on the task, add the task to the mission.
                        graph
                            .add_node(task.clone())
                            .map_err(|e| E::from(e.to_string()))?;
                    }
                }
            }

            if let Some(bridges) = &representation.bridges {
                for bridge in bridges {
                    if mission_applies(&bridge.missions, mission_id) {
                        insert_bridge_node(graph, bridge).map_err(E::from)?;
                    }
                }
            }

            if let Some(cnx) = &representation.cnx {
                for (connection_order, c) in cnx.iter().enumerate() {
                    if let Some(cnx_missions) = &c.missions {
                        // if there is a filter by mission on the connection, only add the connection to the mission if it matches the filter.
                        if cnx_missions.contains(&mission_id.to_owned()) {
                            if c.dst == NC_ENDPOINT {
                                register_nc_output::<E>(
                                    graph,
                                    &c.src,
                                    &c.msg,
                                    connection_order,
                                    &bridge_lookup,
                                )?;
                                continue;
                            }
                            let (src_name, src_channel) =
                                parse_endpoint(&c.src, EndpointRole::Source, &bridge_lookup)
                                    .map_err(E::from)?;
                            let (dst_name, dst_channel) =
                                parse_endpoint(&c.dst, EndpointRole::Destination, &bridge_lookup)
                                    .map_err(E::from)?;
                            let src =
                                graph
                                    .get_node_id_by_name(src_name.as_str())
                                    .ok_or_else(|| {
                                        E::from(format!("Source node not found: {}", c.src))
                                    })?;
                            let dst =
                                graph
                                    .get_node_id_by_name(dst_name.as_str())
                                    .ok_or_else(|| {
                                        E::from(format!("Destination node not found: {}", c.dst))
                                    })?;
                            graph
                                .connect_ext_with_order(
                                    src,
                                    dst,
                                    &c.msg,
                                    Some(cnx_missions.clone()),
                                    src_channel,
                                    dst_channel,
                                    connection_order,
                                )
                                .map_err(|e| E::from(e.to_string()))?;
                        }
                    } else {
                        // if there is no filter by mission on the connection, add the connection to the mission.
                        if c.dst == NC_ENDPOINT {
                            register_nc_output::<E>(
                                graph,
                                &c.src,
                                &c.msg,
                                connection_order,
                                &bridge_lookup,
                            )?;
                            continue;
                        }
                        let (src_name, src_channel) =
                            parse_endpoint(&c.src, EndpointRole::Source, &bridge_lookup)
                                .map_err(E::from)?;
                        let (dst_name, dst_channel) =
                            parse_endpoint(&c.dst, EndpointRole::Destination, &bridge_lookup)
                                .map_err(E::from)?;
                        let src = graph
                            .get_node_id_by_name(src_name.as_str())
                            .ok_or_else(|| E::from(format!("Source node not found: {}", c.src)))?;
                        let dst =
                            graph
                                .get_node_id_by_name(dst_name.as_str())
                                .ok_or_else(|| {
                                    E::from(format!("Destination node not found: {}", c.dst))
                                })?;
                        graph
                            .connect_ext_with_order(
                                src,
                                dst,
                                &c.msg,
                                None,
                                src_channel,
                                dst_channel,
                                connection_order,
                            )
                            .map_err(|e| E::from(e.to_string()))?;
                    }
                }
            }
        }
        cuconfig.graphs = missions;
    } else {
        // this is the simple case
        let mut graph = CuGraph::default();

        if let Some(tasks) = &representation.tasks {
            for task in tasks {
                graph
                    .add_node(task.clone())
                    .map_err(|e| E::from(e.to_string()))?;
            }
        }

        if let Some(bridges) = &representation.bridges {
            for bridge in bridges {
                insert_bridge_node(&mut graph, bridge).map_err(E::from)?;
            }
        }

        if let Some(cnx) = &representation.cnx {
            for (connection_order, c) in cnx.iter().enumerate() {
                if c.dst == NC_ENDPOINT {
                    register_nc_output::<E>(
                        &mut graph,
                        &c.src,
                        &c.msg,
                        connection_order,
                        &bridge_lookup,
                    )?;
                    continue;
                }
                let (src_name, src_channel) =
                    parse_endpoint(&c.src, EndpointRole::Source, &bridge_lookup)
                        .map_err(E::from)?;
                let (dst_name, dst_channel) =
                    parse_endpoint(&c.dst, EndpointRole::Destination, &bridge_lookup)
                        .map_err(E::from)?;
                let src = graph
                    .get_node_id_by_name(src_name.as_str())
                    .ok_or_else(|| E::from(format!("Source node not found: {}", c.src)))?;
                let dst = graph
                    .get_node_id_by_name(dst_name.as_str())
                    .ok_or_else(|| E::from(format!("Destination node not found: {}", c.dst)))?;
                graph
                    .connect_ext_with_order(
                        src,
                        dst,
                        &c.msg,
                        None,
                        src_channel,
                        dst_channel,
                        connection_order,
                    )
                    .map_err(|e| E::from(e.to_string()))?;
            }
        }
        cuconfig.graphs = Simple(graph);
    }

    cuconfig.monitors = representation.monitors.clone().unwrap_or_default();
    cuconfig.constants = representation.constants.clone().unwrap_or_default();
    cuconfig.logging = representation.logging.clone();
    cuconfig.runtime = representation.runtime.clone();
    cuconfig.resources = representation.resources.clone().unwrap_or_default();
    cuconfig.log_streaming = representation.log_streaming.clone();
    cuconfig.bridges = representation.bridges.clone().unwrap_or_default();

    validate_thread_pools::<E>(&cuconfig.runtime)?;

    Ok(cuconfig)
}

impl<'de> Deserialize<'de> for CuConfig {
    /// This is a custom serialization to make this implementation independent of petgraph.
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let representation =
            CuConfigRepresentation::deserialize(deserializer).map_err(serde::de::Error::custom)?;

        // Convert String errors to D::Error using serde::de::Error::custom
        match deserialize_config_representation::<String>(&representation) {
            Ok(config) => Ok(config),
            Err(e) => Err(serde::de::Error::custom(e)),
        }
    }
}

impl Serialize for CuConfig {
    /// This is a custom serialization to make this implementation independent of petgraph.
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let bridges = if self.bridges.is_empty() {
            None
        } else {
            Some(self.bridges.clone())
        };
        let resources = if self.resources.is_empty() {
            None
        } else {
            Some(self.resources.clone())
        };
        let monitors = (!self.monitors.is_empty()).then_some(self.monitors.clone());
        match &self.graphs {
            Simple(graph) => {
                let tasks: Vec<Node> = graph
                    .0
                    .node_indices()
                    .map(|idx| graph.0[idx].clone())
                    .filter(|node| node.get_flavor() == Flavor::Task)
                    .collect();

                let mut ordered_cnx: Vec<(usize, SerializedCnx)> = graph
                    .0
                    .edge_indices()
                    .map(|edge_idx| {
                        let edge = &graph.0[edge_idx];
                        let order = if edge.order == usize::MAX {
                            edge_idx.index()
                        } else {
                            edge.order
                        };
                        (order, SerializedCnx::from(edge))
                    })
                    .collect();
                for node_idx in graph.0.node_indices() {
                    let node = &graph.0[node_idx];
                    if node.get_flavor() != Flavor::Task {
                        continue;
                    }
                    for (msg, order) in node.nc_outputs_with_order() {
                        ordered_cnx.push((
                            order,
                            SerializedCnx {
                                src: node.get_id(),
                                dst: NC_ENDPOINT.to_string(),
                                msg: msg.clone(),
                                missions: None,
                            },
                        ));
                    }
                }
                ordered_cnx.sort_by(|(order_a, cnx_a), (order_b, cnx_b)| {
                    order_a
                        .cmp(order_b)
                        .then_with(|| cnx_a.src.cmp(&cnx_b.src))
                        .then_with(|| cnx_a.dst.cmp(&cnx_b.dst))
                        .then_with(|| cnx_a.msg.cmp(&cnx_b.msg))
                });
                let cnx: Vec<SerializedCnx> = ordered_cnx
                    .into_iter()
                    .map(|(_, serialized)| serialized)
                    .collect();

                CuConfigRepresentation {
                    constants: (!self.constants.is_empty()).then_some(self.constants.clone()),
                    tasks: Some(tasks),
                    bridges: bridges.clone(),
                    cnx: Some(cnx),
                    monitors: monitors.clone(),
                    logging: self.logging.clone(),
                    runtime: self.runtime.clone(),
                    resources: resources.clone(),
                    log_streaming: self.log_streaming.clone(),
                    missions: None,
                    includes: None,
                }
                .serialize(serializer)
            }
            Missions(graphs) => {
                let missions = graphs
                    .keys()
                    .map(|id| MissionsConfig { id: id.clone() })
                    .collect();

                // Collect all unique tasks across missions
                let mut tasks = Vec::new();
                let mut ordered_cnx: Vec<(usize, SerializedCnx)> = Vec::new();

                for (mission_id, graph) in graphs {
                    // Add all nodes from this mission
                    for node_idx in graph.node_indices() {
                        let node = &graph[node_idx];
                        if node.get_flavor() == Flavor::Task
                            && !tasks.iter().any(|n: &Node| n.id == node.id)
                        {
                            tasks.push(node.clone());
                        }
                    }

                    // Add all edges from this mission
                    for edge_idx in graph.0.edge_indices() {
                        let edge = &graph.0[edge_idx];
                        let order = if edge.order == usize::MAX {
                            edge_idx.index()
                        } else {
                            edge.order
                        };
                        let serialized = SerializedCnx::from(edge);
                        if let Some((existing_order, existing_serialized)) =
                            ordered_cnx.iter_mut().find(|(_, c)| {
                                c.src == serialized.src
                                    && c.dst == serialized.dst
                                    && c.msg == serialized.msg
                            })
                        {
                            if order < *existing_order {
                                *existing_order = order;
                            }
                            merge_connection_missions(
                                &mut existing_serialized.missions,
                                &serialized.missions,
                            );
                        } else {
                            ordered_cnx.push((order, serialized));
                        }
                    }
                    for node_idx in graph.0.node_indices() {
                        let node = &graph.0[node_idx];
                        if node.get_flavor() != Flavor::Task {
                            continue;
                        }
                        for (msg, order) in node.nc_outputs_with_order() {
                            let serialized = SerializedCnx {
                                src: node.get_id(),
                                dst: NC_ENDPOINT.to_string(),
                                msg: msg.clone(),
                                missions: Some(vec![mission_id.clone()]),
                            };
                            if let Some((existing_order, existing_serialized)) =
                                ordered_cnx.iter_mut().find(|(_, c)| {
                                    c.src == serialized.src
                                        && c.dst == serialized.dst
                                        && c.msg == serialized.msg
                                })
                            {
                                if order < *existing_order {
                                    *existing_order = order;
                                }
                                merge_connection_missions(
                                    &mut existing_serialized.missions,
                                    &serialized.missions,
                                );
                            } else {
                                ordered_cnx.push((order, serialized));
                            }
                        }
                    }
                }
                ordered_cnx.sort_by(|(order_a, cnx_a), (order_b, cnx_b)| {
                    order_a
                        .cmp(order_b)
                        .then_with(|| cnx_a.src.cmp(&cnx_b.src))
                        .then_with(|| cnx_a.dst.cmp(&cnx_b.dst))
                        .then_with(|| cnx_a.msg.cmp(&cnx_b.msg))
                });
                let cnx: Vec<SerializedCnx> = ordered_cnx
                    .into_iter()
                    .map(|(_, serialized)| serialized)
                    .collect();

                CuConfigRepresentation {
                    constants: (!self.constants.is_empty()).then_some(self.constants.clone()),
                    tasks: Some(tasks),
                    resources: resources.clone(),
                    log_streaming: self.log_streaming.clone(),
                    bridges,
                    cnx: Some(cnx),
                    monitors,
                    logging: self.logging.clone(),
                    runtime: self.runtime.clone(),
                    missions: Some(missions),
                    includes: None,
                }
                .serialize(serializer)
            }
        }
    }
}

impl Default for CuConfig {
    fn default() -> Self {
        CuConfig {
            constants: Vec::new(),
            graphs: Simple(CuGraph(StableDiGraph::new())),
            monitors: Vec::new(),
            logging: None,
            runtime: None,
            resources: Vec::new(),
            log_streaming: None,
            bridges: Vec::new(),
        }
    }
}

/// The implementation has a lot of convenience methods to manipulate
/// the configuration to give some flexibility into programmatically creating the configuration.
impl CuConfig {
    #[allow(dead_code)]
    pub fn new_simple_type() -> Self {
        Self::default()
    }

    #[allow(dead_code)]
    pub fn new_mission_type() -> Self {
        CuConfig {
            constants: Vec::new(),
            graphs: Missions(HashMap::new()),
            monitors: Vec::new(),
            logging: None,
            runtime: None,
            resources: Vec::new(),
            log_streaming: None,
            bridges: Vec::new(),
        }
    }

    pub(crate) fn get_options() -> Options {
        Options::default()
            .with_default_extension(Extensions::IMPLICIT_SOME)
            .with_default_extension(Extensions::UNWRAP_NEWTYPES)
            .with_default_extension(Extensions::UNWRAP_VARIANT_NEWTYPES)
    }

    #[allow(dead_code)]
    pub fn serialize_ron(&self) -> CuResult<String> {
        let ron = Self::get_options();
        let pretty = ron::ser::PrettyConfig::default();
        ron.to_string_pretty(&self, pretty)
            .map_err(|e| CuError::from(format!("Error serializing configuration: {e}")))
    }

    #[allow(dead_code)]
    pub fn deserialize_ron(ron: &str) -> CuResult<Self> {
        let representation = Self::get_options().from_str(ron).map_err(|e| {
            CuError::from(format!(
                "Syntax Error in config: {} at position {}",
                e.code, e.span
            ))
        })?;
        Self::deserialize_impl(representation)
            .map_err(|e| CuError::from(format!("Error deserializing configuration: {e}")))
    }

    fn deserialize_impl(representation: CuConfigRepresentation) -> Result<Self, String> {
        deserialize_config_representation(&representation)
    }

    /// Render the configuration graph in the dot format.
    #[cfg(feature = "std")]
    #[allow(dead_code)]
    pub fn render(
        &self,
        output: &mut dyn std::io::Write,
        mission_id: Option<&str>,
    ) -> CuResult<()> {
        writeln!(output, "digraph G {{")
            .map_err(|e| CuError::new_with_cause("Failed to write render output", e))?;
        writeln!(output, "    graph [rankdir=LR, nodesep=0.8, ranksep=1.2];")
            .map_err(|e| CuError::new_with_cause("Failed to write render output", e))?;
        writeln!(output, "    node [shape=plain, fontname=\"Noto Sans\"];")
            .map_err(|e| CuError::new_with_cause("Failed to write render output", e))?;
        writeln!(output, "    edge [fontname=\"Noto Sans\"];")
            .map_err(|e| CuError::new_with_cause("Failed to write render output", e))?;

        let sections = match (&self.graphs, mission_id) {
            (Simple(graph), _) => vec![RenderSection { label: None, graph }],
            (Missions(graphs), Some(id)) => {
                let graph = graphs
                    .get(id)
                    .ok_or_else(|| CuError::from(format!("Mission {id} not found")))?;
                vec![RenderSection {
                    label: Some(id.to_string()),
                    graph,
                }]
            }
            (Missions(graphs), None) => {
                let mut missions: Vec<_> = graphs.iter().collect();
                missions.sort_by(|a, b| a.0.cmp(b.0));
                missions
                    .into_iter()
                    .map(|(label, graph)| RenderSection {
                        label: Some(label.clone()),
                        graph,
                    })
                    .collect()
            }
        };

        for section in sections {
            self.render_section(output, section.graph, section.label.as_deref())?;
        }

        writeln!(output, "}}")
            .map_err(|e| CuError::new_with_cause("Failed to write render output", e))?;
        Ok(())
    }

    #[allow(dead_code)]
    pub fn get_all_instances_configs(
        &self,
        mission_id: Option<&str>,
    ) -> Vec<Option<&ComponentConfig>> {
        let graph = self.graphs.get_graph(mission_id).unwrap();
        graph
            .get_all_nodes()
            .iter()
            .map(|(_, node)| node.get_instance_config())
            .collect()
    }

    #[allow(dead_code)]
    pub fn get_graph(&self, mission_id: Option<&str>) -> CuResult<&CuGraph> {
        self.graphs.get_graph(mission_id)
    }

    #[allow(dead_code)]
    pub fn get_graph_mut(&mut self, mission_id: Option<&str>) -> CuResult<&mut CuGraph> {
        self.graphs.get_graph_mut(mission_id)
    }

    #[allow(dead_code)]
    pub fn get_monitor_config(&self) -> Option<&MonitorConfig> {
        self.monitors.first()
    }

    #[allow(dead_code)]
    pub fn get_monitor_configs(&self) -> &[MonitorConfig] {
        &self.monitors
    }

    #[allow(dead_code)]
    pub fn get_runtime_config(&self) -> Option<&RuntimeConfig> {
        self.runtime.as_ref()
    }

    #[allow(dead_code)]
    pub fn find_task_node(&self, mission_id: Option<&str>, task_id: &str) -> Option<&Node> {
        self.get_graph(mission_id)
            .ok()?
            .get_all_nodes()
            .into_iter()
            .find_map(|(_, node)| {
                (node.get_flavor() == Flavor::Task && node.id == task_id).then_some(node)
            })
    }

    #[allow(dead_code)]
    pub fn find_logging_codec_spec(&self, codec_id: &str) -> Option<&LoggingCodecSpec> {
        self.logging
            .as_ref()?
            .codecs
            .iter()
            .find(|spec| spec.id == codec_id)
    }

    /// Validate compile-time constant names, shapes, scalar ranges, and unit compatibility.
    pub fn validate_constants(&self) -> CuResult<()> {
        fn validate_integer(
            id: &str,
            storage: ConstantStorage,
            number: ConstantNumber,
        ) -> CuResult<()> {
            let valid = match (storage, number) {
                (ConstantStorage::I8, ConstantNumber::Signed(value)) => i8::try_from(value).is_ok(),
                (ConstantStorage::I16, ConstantNumber::Signed(value)) => {
                    i16::try_from(value).is_ok()
                }
                (ConstantStorage::I32, ConstantNumber::Signed(value)) => {
                    i32::try_from(value).is_ok()
                }
                (ConstantStorage::I64, ConstantNumber::Signed(_)) => true,
                (ConstantStorage::Isize, ConstantNumber::Signed(value)) => {
                    isize::try_from(value).is_ok()
                }
                (ConstantStorage::U8, ConstantNumber::Unsigned(value)) => {
                    u8::try_from(value).is_ok()
                }
                (ConstantStorage::U16, ConstantNumber::Unsigned(value)) => {
                    u16::try_from(value).is_ok()
                }
                (ConstantStorage::U32, ConstantNumber::Unsigned(value)) => {
                    u32::try_from(value).is_ok()
                }
                (ConstantStorage::U64, ConstantNumber::Unsigned(_)) => true,
                (ConstantStorage::Usize, ConstantNumber::Unsigned(value)) => {
                    usize::try_from(value).is_ok()
                }
                _ => false,
            };
            if valid {
                Ok(())
            } else {
                Err(CuError::from(format!(
                    "Constant '{id}' value {number:?} cannot be represented as {}",
                    storage.rust_type()
                )))
            }
        }

        let mut ids = HashMap::new();
        for constant in &self.constants {
            if constant.id().is_empty() {
                return Err(CuError::from("Constant ids cannot be empty"));
            }
            if ids
                .insert((constant.module_path(), constant.id()), ())
                .is_some()
            {
                return Err(CuError::from(format!(
                    "Duplicate constant '{}'. Constant ids must be unique within a module.",
                    constant.qualified_id()
                )));
            }

            match (
                constant.value.is_some(),
                constant.rust_type.as_deref(),
                constant.expression.as_deref(),
            ) {
                (true, None, None) => {}
                (true, _, _) => {
                    return Err(CuError::from(format!(
                        "Constant '{}' cannot combine numeric 'value' with 'type' or 'expression'",
                        constant.id()
                    )));
                }
                (false, Some(rust_type), Some(expression)) => {
                    if constant.storage.is_some()
                        || constant.quantity.is_some()
                        || constant.unit.is_some()
                    {
                        return Err(CuError::from(format!(
                            "Constant '{}' cannot combine 'type' and 'expression' with numeric 'storage', 'quantity', or 'unit'",
                            constant.id()
                        )));
                    }
                    if rust_type.trim().is_empty() {
                        return Err(CuError::from(format!(
                            "Constant '{}' type cannot be empty",
                            constant.id()
                        )));
                    }
                    if expression.trim().is_empty() {
                        return Err(CuError::from(format!(
                            "Constant '{}' expression cannot be empty",
                            constant.id()
                        )));
                    }
                    continue;
                }
                (false, Some(_), None) => {
                    return Err(CuError::from(format!(
                        "Constant '{}' declares 'type' without 'expression'",
                        constant.id()
                    )));
                }
                (false, None, Some(_)) => {
                    return Err(CuError::from(format!(
                        "Constant '{}' declares 'expression' without 'type'",
                        constant.id()
                    )));
                }
                (false, None, None) => {
                    return Err(CuError::from(format!(
                        "Constant '{}' must declare either numeric 'value' or both 'type' and 'expression'",
                        constant.id()
                    )));
                }
            }

            if constant.quantity().is_none() && constant.explicit_unit().is_some() {
                return Err(CuError::from(format!(
                    "Constant '{}' declares a unit without a quantity",
                    constant.id()
                )));
            }

            if constant.quantity().is_some() {
                if !constant.storage().supports_quantity() {
                    return Err(CuError::from(format!(
                        "Constant '{}' quantity '{}' requires storage f32 or f64, not {}",
                        constant.id(),
                        constant.quantity().map_or("", |quantity| quantity.name()),
                        constant.storage().rust_type()
                    )));
                }
                let normalized = match constant.storage() {
                    ConstantStorage::F32 => constant.normalized_f32().map(|_| ()),
                    ConstantStorage::F64 => constant.normalized_f64().map(|_| ()),
                    _ => unreachable!("quantity storage was checked above"),
                };
                normalized.map_err(CuError::from)?;
                continue;
            }

            let (_, numbers) = constant.numbers().map_err(CuError::from)?;
            for number in numbers {
                match constant.storage() {
                    ConstantStorage::F32 => {
                        if !(number.as_f64() as f32).is_finite() {
                            return Err(CuError::from(format!(
                                "Constant '{}' values must be finite",
                                constant.id()
                            )));
                        }
                    }
                    ConstantStorage::F64 => {
                        if !number.as_f64().is_finite() {
                            return Err(CuError::from(format!(
                                "Constant '{}' values must be finite",
                                constant.id()
                            )));
                        }
                    }
                    storage => validate_integer(constant.id(), storage, number)?,
                }
            }
        }
        Ok(())
    }

    /// Validate the logging configuration to ensure section pre-allocation sizes do not exceed slab sizes.
    /// This method is wrapper around [LoggingConfig::validate]
    pub fn validate_logging_config(&self) -> CuResult<()> {
        if let Some(logging) = &self.logging {
            return logging.validate();
        }
        Ok(())
    }

    /// Validate the runtime configuration.
    pub fn validate_runtime_config(&self) -> CuResult<()> {
        if let Some(runtime) = &self.runtime {
            runtime.validate()?;
            if let Some(planner) = &runtime.planner {
                let copperlist_count = self
                    .logging
                    .as_ref()
                    .and_then(|logging| logging.copperlist_count)
                    .unwrap_or(DEFAULT_COPPERLIST_COUNT);
                planner.max_in_flight(copperlist_count)?;
            }
        }
        Ok(())
    }

    fn validate_stateless_configs(&self) -> CuResult<()> {
        match &self.graphs {
            Simple(graph) => validate_stateless_graph(graph),
            Missions(graphs) => {
                for graph in graphs.values() {
                    validate_stateless_graph(graph)?;
                }
                Ok(())
            }
        }
    }

    /// Validates every `anytime:` policy in the resolved graphs.
    ///
    /// Runs at configuration-resolution time, the first point where both the
    /// resolved graphs and `runtime.rate_target_hz` are known:
    ///
    /// 1. node-local bounds and ranges (see [`AnytimeConfig`]);
    /// 2. `anytime:` is only supported on regular tasks — refinement needs both
    ///    an input and an output;
    /// 3. an anytime task has exactly one input connection (the runner anchors
    ///    the job on the input's Tov) and at most one output message type
    ///    (`base()` and every `refine()` write the same output slot);
    /// 4. a *foreground* anytime task needs `max_refines`: the execution plan
    ///    is static (the node compiles to a base step plus `max_refines` refine
    ///    steps, see `curuntime::expand_anytime_steps`), so the refine count
    ///    must be known at compile time;
    /// 5. fit the period: a *foreground* anytime task in a rate-limited config
    ///    must set a time bound (`time_budget_ms` or `max_age_ms`), and the
    ///    worst-case window — `min` of the ones set — must be smaller than the
    ///    loop period. Background nodes and configs without a rate target skip
    ///    this check.
    pub fn validate_anytime_configs(&self) -> CuResult<()> {
        let rate_target_hz = self.runtime.as_ref().and_then(|r| r.rate_target_hz);
        match &self.graphs {
            Simple(graph) => validate_anytime_graph(graph, rate_target_hz),
            Missions(graphs) => {
                for graph in graphs.values() {
                    validate_anytime_graph(graph, rate_target_hz)?;
                }
                Ok(())
            }
        }
    }
}

/// Checks every `anytime:` node of one graph: local bounds, regular-task kind,
/// single-input/single-output arity, and the foreground fit-the-period rule
/// (see [`CuConfig::validate_anytime_configs`]).
fn validate_anytime_graph(graph: &CuGraph, rate_target_hz: Option<u64>) -> CuResult<()> {
    for (node_id, node) in graph.get_all_nodes() {
        let Some(anytime) = node.anytime() else {
            continue;
        };
        anytime.validate(&node.id)?;

        let kind = resolve_task_kind_for_id(graph, node_id)?;
        if kind != TaskKind::Regular {
            return Err(CuError::from(format!(
                "Task '{}' is declared with an anytime: policy but resolves to kind '{}'. Anytime refinement needs both an input and an output, so it is only supported on regular tasks.",
                node.id,
                kind.as_str()
            )));
        }

        // Foreground and background alike: the runner reads the job anchor
        // from the single input's Tov, and base()/refine() write one stable
        // output slot. Zero declared outputs is fine when the kind is
        // declared — the macro synthesizes exactly one nc output.
        let input_count = graph.get_dst_edges(node_id)?.len();
        if input_count != 1 {
            return Err(CuError::from(format!(
                "Task '{}' is an anytime task and must have exactly one input connection (found {input_count}): the runner anchors the job on the input's Tov.",
                node.id
            )));
        }
        let output_count = graph.get_node_output_msg_types_by_id(node_id)?.len();
        if output_count > 1 {
            return Err(CuError::from(format!(
                "Task '{}' is an anytime task and must have exactly one output message type (found {output_count}): base() and every refine() write the same output slot.",
                node.id
            )));
        }

        // Background placement: the refinement window runs on a worker thread
        // and may exceed the copperlist period — that is the point of it.
        if node.is_background() {
            continue;
        }

        // Foreground placement compiles to a static plan: the node's step is
        // followed by exactly max_refines refine steps, so the count must be
        // known here — a time-only hard bound cannot produce a static plan.
        if anytime.max_refines.is_none() {
            return Err(CuError::from(format!(
                "Task '{}' is a foreground anytime task and needs anytime.max_refines: the execution plan is static, so the refine step count must be known at compile time. time_budget_ms/max_age_ms remain early-stop conditions within those quanta.",
                node.id
            )));
        }

        let Some(rate_target_hz) = rate_target_hz else {
            continue;
        };
        let window_ms = match (anytime.time_budget_ms, anytime.max_age_ms) {
            (Some(budget), Some(age)) => budget.min(age),
            (Some(budget), None) => budget,
            (None, Some(age)) => age,
            (None, None) => {
                return Err(CuError::from(format!(
                    "Task '{}' is a foreground anytime task in a rate-limited config and needs a time bound: set anytime.time_budget_ms or anytime.max_age_ms. max_refines alone gives the runtime no time quantity to check against the {rate_target_hz} Hz loop period, and one slow quantum would silently overrun it.",
                    node.id
                )));
            }
        };
        let period_ms = 1_000.0 / rate_target_hz as f64;
        if window_ms >= period_ms {
            return Err(CuError::from(format!(
                "Task '{}': the worst-case anytime window ({window_ms} ms) does not fit within the {rate_target_hz} Hz loop period ({period_ms} ms) with headroom for the rest of the copperlist. Tighten the time bound, lower runtime.rate_target_hz, or run the task with background: true.",
                node.id
            )));
        }
    }
    Ok(())
}

fn validate_stateless_graph(graph: &CuGraph) -> CuResult<()> {
    for (node_id, node) in graph.get_all_nodes() {
        if node.get_declared_task_kind() != Some(TaskKind::Stateless) {
            continue;
        }

        resolve_task_kind_for_id(graph, node_id)?;
        if node.is_background() {
            return Err(CuError::from(format!(
                "Task '{}' is declared as kind 'stateless_task' and cannot be backgrounded.",
                node.id
            )));
        }
        if node.is_anytime() {
            return Err(CuError::from(format!(
                "Task '{}' is declared as kind 'stateless_task' and cannot use an anytime policy.",
                node.id
            )));
        }
    }
    Ok(())
}

#[cfg(feature = "std")]
#[derive(Default)]
#[doc(hidden)]
pub struct PortLookup {
    pub inputs: HashMap<String, String>,
    pub outputs: HashMap<String, String>,
    pub default_input: Option<String>,
    pub default_output: Option<String>,
}

#[cfg(feature = "std")]
#[derive(Clone)]
#[doc(hidden)]
pub struct RenderNode {
    pub id: String,
    pub type_name: String,
    pub flavor: Flavor,
    pub inputs: Vec<String>,
    pub outputs: Vec<String>,
}

#[cfg(feature = "std")]
#[derive(Clone)]
#[doc(hidden)]
pub struct RenderConnection {
    pub src: String,
    pub src_port: Option<String>,
    #[allow(dead_code)]
    pub src_channel: Option<String>,
    pub dst: String,
    pub dst_port: Option<String>,
    #[allow(dead_code)]
    pub dst_channel: Option<String>,
    pub msg: String,
}

#[cfg(feature = "std")]
#[doc(hidden)]
pub struct RenderTopology {
    pub nodes: Vec<RenderNode>,
    pub connections: Vec<RenderConnection>,
}

#[cfg(feature = "std")]
impl RenderTopology {
    pub fn sort_connections(&mut self) {
        self.connections.sort_by(|a, b| {
            a.src
                .cmp(&b.src)
                .then(a.dst.cmp(&b.dst))
                .then(a.msg.cmp(&b.msg))
        });
    }
}

#[cfg(feature = "std")]
#[allow(dead_code)]
struct RenderSection<'a> {
    label: Option<String>,
    graph: &'a CuGraph,
}

#[cfg(feature = "std")]
impl CuConfig {
    #[allow(dead_code)]
    fn render_section(
        &self,
        output: &mut dyn std::io::Write,
        graph: &CuGraph,
        label: Option<&str>,
    ) -> CuResult<()> {
        use std::fmt::Write as FmtWrite;

        let mut topology = build_render_topology(graph, &self.bridges);
        topology.nodes.sort_by(|a, b| a.id.cmp(&b.id));
        topology.sort_connections();

        let cluster_id = label.map(|lbl| format!("cluster_{}", sanitize_identifier(lbl)));
        if let Some(ref cluster_id) = cluster_id {
            writeln!(output, "    subgraph \"{cluster_id}\" {{")
                .map_err(|e| CuError::new_with_cause("Failed to write render output", e))?;
            writeln!(
                output,
                "        label=<<B>Mission: {}</B>>;",
                encode_text(label.unwrap())
            )
            .map_err(|e| CuError::new_with_cause("Failed to write render output", e))?;
            writeln!(
                output,
                "        labelloc=t; labeljust=l; color=\"#bbbbbb\"; style=\"rounded\"; margin=20;"
            )
            .map_err(|e| CuError::new_with_cause("Failed to write render output", e))?;
        }
        let indent = if cluster_id.is_some() {
            "        "
        } else {
            "    "
        };
        let node_prefix = label
            .map(|lbl| format!("{}__", sanitize_identifier(lbl)))
            .unwrap_or_default();

        let mut port_lookup: HashMap<String, PortLookup> = HashMap::new();
        let mut id_lookup: HashMap<String, String> = HashMap::new();

        for node in &topology.nodes {
            let node_idx = graph
                .get_node_id_by_name(node.id.as_str())
                .ok_or_else(|| CuError::from(format!("Node '{}' missing from graph", node.id)))?;
            let node_weight = graph
                .get_node(node_idx)
                .ok_or_else(|| CuError::from(format!("Node '{}' missing weight", node.id)))?;

            let fillcolor = match node.flavor {
                Flavor::Bridge => "#faedcd",
                Flavor::Task => match resolve_task_kind_for_id(graph, node_idx)? {
                    TaskKind::Source => "#ddefc7",
                    TaskKind::Sink => "#cce0ff",
                    TaskKind::Regular => "#f2f2f2",
                    TaskKind::Stateless => "#cba6f7",
                },
            };

            let port_base = format!("{}{}", node_prefix, sanitize_identifier(&node.id));
            let (inputs_table, input_map, default_input) =
                build_port_table("Inputs", &node.inputs, &port_base, "in");
            let (outputs_table, output_map, default_output) =
                build_port_table("Outputs", &node.outputs, &port_base, "out");
            let config_html = node_weight.config.as_ref().and_then(build_config_table);

            let mut label_html = String::new();
            write!(
                label_html,
                "<TABLE BORDER=\"0\" CELLBORDER=\"1\" CELLSPACING=\"0\" CELLPADDING=\"6\" COLOR=\"gray\" BGCOLOR=\"white\">"
            )
            .unwrap();
            write!(
                label_html,
                "<TR><TD COLSPAN=\"2\" ALIGN=\"LEFT\" BGCOLOR=\"{fillcolor}\"><FONT POINT-SIZE=\"12\"><B>{}</B></FONT><BR/><FONT COLOR=\"dimgray\">[{}]</FONT></TD></TR>",
                encode_text(&node.id),
                encode_text(&node.type_name)
            )
            .unwrap();
            write!(
                label_html,
                "<TR><TD ALIGN=\"LEFT\" VALIGN=\"TOP\">{inputs_table}</TD><TD ALIGN=\"LEFT\" VALIGN=\"TOP\">{outputs_table}</TD></TR>"
            )
            .unwrap();

            if let Some(config_html) = config_html {
                write!(
                    label_html,
                    "<TR><TD COLSPAN=\"2\" ALIGN=\"LEFT\">{config_html}</TD></TR>"
                )
                .unwrap();
            }

            label_html.push_str("</TABLE>");

            let identifier_raw = if node_prefix.is_empty() {
                node.id.clone()
            } else {
                format!("{node_prefix}{}", node.id)
            };
            let identifier = escape_dot_id(&identifier_raw);
            writeln!(output, "{indent}\"{identifier}\" [label=<{label_html}>];")
                .map_err(|e| CuError::new_with_cause("Failed to write render output", e))?;

            id_lookup.insert(node.id.clone(), identifier);
            port_lookup.insert(
                node.id.clone(),
                PortLookup {
                    inputs: input_map,
                    outputs: output_map,
                    default_input,
                    default_output,
                },
            );
        }

        for cnx in &topology.connections {
            let src_id = id_lookup
                .get(&cnx.src)
                .ok_or_else(|| CuError::from(format!("Unknown node '{}'", cnx.src)))?;
            let dst_id = id_lookup
                .get(&cnx.dst)
                .ok_or_else(|| CuError::from(format!("Unknown node '{}'", cnx.dst)))?;
            let src_suffix = port_lookup
                .get(&cnx.src)
                .and_then(|lookup| lookup.resolve_output(cnx.src_port.as_deref()))
                .map(|port| format!(":\"{port}\":e"))
                .unwrap_or_default();
            let dst_suffix = port_lookup
                .get(&cnx.dst)
                .and_then(|lookup| lookup.resolve_input(cnx.dst_port.as_deref()))
                .map(|port| format!(":\"{port}\":w"))
                .unwrap_or_default();
            let msg = encode_text(&cnx.msg);
            writeln!(
                output,
                "{indent}\"{src_id}\"{src_suffix} -> \"{dst_id}\"{dst_suffix} [label=< <B><FONT COLOR=\"gray\">{msg}</FONT></B> >];"
            )
            .map_err(|e| CuError::new_with_cause("Failed to write render output", e))?;
        }

        if cluster_id.is_some() {
            writeln!(output, "    }}")
                .map_err(|e| CuError::new_with_cause("Failed to write render output", e))?;
        }

        Ok(())
    }
}

#[cfg(feature = "std")]
#[doc(hidden)]
pub fn build_render_topology(graph: &CuGraph, bridges: &[BridgeConfig]) -> RenderTopology {
    let mut bridge_lookup = HashMap::new();
    for bridge in bridges {
        bridge_lookup.insert(bridge.id.as_str(), bridge);
    }

    let mut nodes: Vec<RenderNode> = Vec::new();
    let mut node_lookup: HashMap<String, usize> = HashMap::new();
    for (node_idx, node) in graph.get_all_nodes() {
        let node_id = node.get_id();
        let mut inputs = Vec::new();
        let mut outputs = Vec::new();
        if node.get_flavor() == Flavor::Bridge
            && let Some(bridge) = bridge_lookup.get(node_id.as_str())
        {
            for channel in &bridge.channels {
                match channel {
                    // Rx brings data from the bridge into the graph, so treat it as an output.
                    BridgeChannelConfigRepresentation::Rx { id, .. } => outputs.push(id.clone()),
                    // Tx consumes data from the graph heading into the bridge, so show it on the input side.
                    BridgeChannelConfigRepresentation::Tx { id, .. } => inputs.push(id.clone()),
                }
            }
        } else if node.get_flavor() == Flavor::Task {
            for (idx, msg) in graph
                .get_node_output_msg_types_by_id(node_idx)
                .unwrap_or_default()
                .into_iter()
                .enumerate()
            {
                outputs.push(format!("out{idx}: {msg}"));
            }
        }

        node_lookup.insert(node_id.clone(), nodes.len());
        nodes.push(RenderNode {
            id: node_id,
            type_name: node.get_type().to_string(),
            flavor: node.get_flavor(),
            inputs,
            outputs,
        });
    }

    let mut output_port_lookup: Vec<HashMap<String, String>> = vec![HashMap::new(); nodes.len()];
    for (node_idx, node) in graph.get_all_nodes() {
        let Some(&idx) = node_lookup.get(&node.get_id()) else {
            continue;
        };
        if node.get_flavor() != Flavor::Task {
            continue;
        }
        for (port_idx, msg) in graph
            .get_node_output_msg_types_by_id(node_idx)
            .unwrap_or_default()
            .into_iter()
            .enumerate()
        {
            output_port_lookup[idx].insert(msg.clone(), format!("out{port_idx}: {msg}"));
        }
    }

    let mut auto_input_counts = vec![0usize; nodes.len()];
    for edge in graph.0.edge_references() {
        let cnx = edge.weight();
        if let Some(&idx) = node_lookup.get(&cnx.dst)
            && nodes[idx].flavor == Flavor::Task
            && cnx.dst_channel.is_none()
        {
            auto_input_counts[idx] += 1;
        }
    }

    let mut next_auto_input = vec![0usize; nodes.len()];
    let mut connections = Vec::new();
    for edge in graph.0.edge_references() {
        let cnx = edge.weight();
        let mut src_port = cnx.src_channel.clone();
        let mut dst_port = cnx.dst_channel.clone();

        if let Some(&idx) = node_lookup.get(&cnx.src) {
            let node = &mut nodes[idx];
            if node.flavor == Flavor::Task && src_port.is_none() {
                src_port = output_port_lookup[idx].get(&cnx.msg).cloned();
            }
        }
        if let Some(&idx) = node_lookup.get(&cnx.dst) {
            let node = &mut nodes[idx];
            if node.flavor == Flavor::Task && dst_port.is_none() {
                let count = auto_input_counts[idx];
                let next = if count <= 1 {
                    "in".to_string()
                } else {
                    let next = format!("in.{}", next_auto_input[idx]);
                    next_auto_input[idx] += 1;
                    next
                };
                node.inputs.push(next.clone());
                dst_port = Some(next);
            }
        }

        connections.push(RenderConnection {
            src: cnx.src.clone(),
            src_port,
            src_channel: cnx.src_channel.clone(),
            dst: cnx.dst.clone(),
            dst_port,
            dst_channel: cnx.dst_channel.clone(),
            msg: cnx.msg.clone(),
        });
    }

    RenderTopology { nodes, connections }
}

#[cfg(feature = "std")]
impl PortLookup {
    pub fn resolve_input(&self, name: Option<&str>) -> Option<&str> {
        if let Some(name) = name
            && let Some(port) = self.inputs.get(name)
        {
            return Some(port.as_str());
        }
        self.default_input.as_deref()
    }

    pub fn resolve_output(&self, name: Option<&str>) -> Option<&str> {
        if let Some(name) = name
            && let Some(port) = self.outputs.get(name)
        {
            return Some(port.as_str());
        }
        self.default_output.as_deref()
    }
}

#[cfg(feature = "std")]
#[allow(dead_code)]
fn build_port_table(
    title: &str,
    names: &[String],
    base_id: &str,
    prefix: &str,
) -> (String, HashMap<String, String>, Option<String>) {
    use std::fmt::Write as FmtWrite;

    let mut html = String::new();
    write!(
        html,
        "<TABLE BORDER=\"0\" CELLBORDER=\"0\" CELLSPACING=\"0\" CELLPADDING=\"1\">"
    )
    .unwrap();
    write!(
        html,
        "<TR><TD ALIGN=\"LEFT\"><FONT COLOR=\"dimgray\">{}</FONT></TD></TR>",
        encode_text(title)
    )
    .unwrap();

    let mut lookup = HashMap::new();
    let mut default_port = None;

    if names.is_empty() {
        html.push_str("<TR><TD ALIGN=\"LEFT\"><FONT COLOR=\"lightgray\">&mdash;</FONT></TD></TR>");
    } else {
        for (idx, name) in names.iter().enumerate() {
            let port_id = format!("{base_id}_{prefix}_{idx}");
            write!(
                html,
                "<TR><TD PORT=\"{port_id}\" ALIGN=\"LEFT\">{}</TD></TR>",
                encode_text(name)
            )
            .unwrap();
            lookup.insert(name.clone(), port_id.clone());
            if idx == 0 {
                default_port = Some(port_id);
            }
        }
    }

    html.push_str("</TABLE>");
    (html, lookup, default_port)
}

#[cfg(feature = "std")]
#[allow(dead_code)]
fn build_config_table(config: &ComponentConfig) -> Option<String> {
    use std::fmt::Write as FmtWrite;

    if config.0.is_empty() {
        return None;
    }

    let mut entries: Vec<_> = config.0.iter().collect();
    entries.sort_by(|a, b| a.0.cmp(b.0));

    let mut html = String::new();
    html.push_str("<TABLE BORDER=\"0\" CELLBORDER=\"0\" CELLSPACING=\"0\" CELLPADDING=\"1\">");
    for (key, value) in entries {
        let value_txt = format!("{value}");
        write!(
            html,
            "<TR><TD ALIGN=\"LEFT\"><FONT COLOR=\"dimgray\">{}</FONT> = {}</TD></TR>",
            encode_text(key),
            encode_text(&value_txt)
        )
        .unwrap();
    }
    html.push_str("</TABLE>");
    Some(html)
}

#[cfg(feature = "std")]
#[allow(dead_code)]
fn sanitize_identifier(value: &str) -> String {
    value
        .chars()
        .map(|c| if c.is_ascii_alphanumeric() { c } else { '_' })
        .collect()
}

#[cfg(feature = "std")]
#[allow(dead_code)]
fn escape_dot_id(value: &str) -> String {
    let mut escaped = String::with_capacity(value.len());
    for ch in value.chars() {
        match ch {
            '"' => escaped.push_str("\\\""),
            '\\' => escaped.push_str("\\\\"),
            _ => escaped.push(ch),
        }
    }
    escaped
}

impl LoggingConfig {
    /// Validate the logging configuration to ensure section pre-allocation sizes do not exceed slab sizes.
    pub fn validate(&self) -> CuResult<()> {
        if let Some(copperlist_count) = self.copperlist_count
            && copperlist_count == 0
        {
            return Err(CuError::from(
                "CopperList count cannot be zero. Set logging.copperlist_count to at least 1.",
            ));
        }

        if let Some(section_size_mib) = self.section_size_mib
            && let Some(slab_size_mib) = self.slab_size_mib
            && section_size_mib > slab_size_mib
        {
            return Err(CuError::from(format!(
                "Section size ({section_size_mib} MiB) cannot be larger than slab size ({slab_size_mib} MiB). Adjust the parameters accordingly."
            )));
        }

        let mut codec_ids = HashMap::new();
        for codec in &self.codecs {
            if codec_ids.insert(codec.id.as_str(), ()).is_some() {
                return Err(CuError::from(format!(
                    "Duplicate logging codec id '{}'. Codec ids must be unique.",
                    codec.id
                )));
            }
        }

        Ok(())
    }
}

impl RuntimeConfig {
    /// Validate runtime loop-rate settings.
    pub fn validate(&self) -> CuResult<()> {
        if let Some(rate_target_hz) = self.rate_target_hz {
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
        }

        Ok(())
    }
}

#[allow(dead_code)] // dead in no-std
fn substitute_parameters(content: &str, params: &HashMap<String, Value>) -> String {
    let mut result = content.to_string();

    for (key, value) in params {
        let pattern = format!("{{{{{key}}}}}");
        result = result.replace(&pattern, &value.to_string());
    }

    result
}

/// Returns a merged CuConfigRepresentation.
#[cfg(feature = "std")]
fn process_includes(
    file_path: &str,
    base_representation: CuConfigRepresentation,
    processed_files: &mut Vec<String>,
    active_features: &[&str],
) -> CuResult<CuConfigRepresentation> {
    // Note: Circular dependency detection removed
    processed_files.push(file_path.to_string());

    let mut result = base_representation;

    if let Some(includes) = result.includes.take() {
        for include in includes {
            if include
                .when
                .as_ref()
                .is_some_and(|predicate| !predicate.evaluate(active_features))
            {
                continue;
            }

            let include_path = if include.path.starts_with('/') {
                include.path.clone()
            } else {
                let current_dir = std::path::Path::new(file_path).parent();

                match current_dir.map(|path| path.to_string_lossy().to_string()) {
                    Some(current_dir) if !current_dir.is_empty() => {
                        format!("{}/{}", current_dir, include.path)
                    }
                    _ => include.path,
                }
            };

            let include_content = read_to_string(&include_path).map_err(|e| {
                CuError::from(format!("Failed to read include file: {include_path}"))
                    .add_cause(e.to_string().as_str())
            })?;

            let processed_content = substitute_parameters(&include_content, &include.params);

            let mut included_representation: CuConfigRepresentation = match Options::default()
                .with_default_extension(Extensions::IMPLICIT_SOME)
                .with_default_extension(Extensions::UNWRAP_NEWTYPES)
                .with_default_extension(Extensions::UNWRAP_VARIANT_NEWTYPES)
                .from_str(&processed_content)
            {
                Ok(rep) => rep,
                Err(e) => {
                    return Err(CuError::from(format!(
                        "Failed to parse include file: {} - Error: {} at position {}",
                        include_path, e.code, e.span
                    )));
                }
            };

            included_representation = process_includes(
                &include_path,
                included_representation,
                processed_files,
                active_features,
            )?;

            if let Some(included_constants) = included_representation.constants {
                if result.constants.is_none() {
                    result.constants = Some(included_constants);
                } else {
                    let mut constants = result.constants.take().unwrap();
                    for included_constant in included_constants {
                        if !constants.iter().any(|constant| {
                            constant.id == included_constant.id
                                && constant.module_path() == included_constant.module_path()
                        }) {
                            constants.push(included_constant);
                        }
                    }
                    result.constants = Some(constants);
                }
            }

            if let Some(included_tasks) = included_representation.tasks {
                if result.tasks.is_none() {
                    result.tasks = Some(included_tasks);
                } else {
                    let mut tasks = result.tasks.take().unwrap();
                    for included_task in included_tasks {
                        if !tasks.iter().any(|t| t.id == included_task.id) {
                            tasks.push(included_task);
                        }
                    }
                    result.tasks = Some(tasks);
                }
            }

            if let Some(included_bridges) = included_representation.bridges {
                if result.bridges.is_none() {
                    result.bridges = Some(included_bridges);
                } else {
                    let mut bridges = result.bridges.take().unwrap();
                    for included_bridge in included_bridges {
                        if !bridges.iter().any(|b| b.id == included_bridge.id) {
                            bridges.push(included_bridge);
                        }
                    }
                    result.bridges = Some(bridges);
                }
            }

            if let Some(included_resources) = included_representation.resources {
                if result.resources.is_none() {
                    result.resources = Some(included_resources);
                } else {
                    let mut resources = result.resources.take().unwrap();
                    for included_resource in included_resources {
                        if !resources.iter().any(|r| r.id == included_resource.id) {
                            resources.push(included_resource);
                        }
                    }
                    result.resources = Some(resources);
                }
            }

            if let Some(included_cnx) = included_representation.cnx {
                if result.cnx.is_none() {
                    result.cnx = Some(included_cnx);
                } else {
                    let mut cnx = result.cnx.take().unwrap();
                    for included_c in included_cnx {
                        if let Some(existing_cnx) = cnx.iter_mut().find(|c| {
                            c.src == included_c.src
                                && c.dst == included_c.dst
                                && c.msg == included_c.msg
                        }) {
                            merge_connection_missions(
                                &mut existing_cnx.missions,
                                &included_c.missions,
                            );
                        } else {
                            cnx.push(included_c);
                        }
                    }
                    result.cnx = Some(cnx);
                }
            }

            if let Some(included_monitors) = included_representation.monitors {
                if result.monitors.is_none() {
                    result.monitors = Some(included_monitors);
                } else {
                    let mut monitors = result.monitors.take().unwrap();
                    for included_monitor in included_monitors {
                        if !monitors.iter().any(|m| m.type_ == included_monitor.type_) {
                            monitors.push(included_monitor);
                        }
                    }
                    result.monitors = Some(monitors);
                }
            }

            if result.logging.is_none() {
                result.logging = included_representation.logging;
            }

            if result.runtime.is_none() {
                result.runtime = included_representation.runtime;
            }

            if result.log_streaming.is_none() {
                result.log_streaming = included_representation.log_streaming;
            }

            if let Some(included_missions) = included_representation.missions {
                if result.missions.is_none() {
                    result.missions = Some(included_missions);
                } else {
                    let mut missions = result.missions.take().unwrap();
                    for included_mission in included_missions {
                        if !missions.iter().any(|m| m.id == included_mission.id) {
                            missions.push(included_mission);
                        }
                    }
                    result.missions = Some(missions);
                }
            }
        }
    }

    Ok(result)
}

#[cfg(feature = "std")]
fn parse_instance_config_overrides_string(
    content: &str,
) -> CuResult<InstanceConfigOverridesRepresentation> {
    Options::default()
        .with_default_extension(Extensions::IMPLICIT_SOME)
        .with_default_extension(Extensions::UNWRAP_NEWTYPES)
        .with_default_extension(Extensions::UNWRAP_VARIANT_NEWTYPES)
        .from_str(content)
        .map_err(|e| {
            CuError::from(format!(
                "Failed to parse instance override file: Error: {} at position {}",
                e.code, e.span
            ))
        })
}

#[cfg(feature = "std")]
fn merge_component_config(target: &mut Option<ComponentConfig>, value: &ComponentConfig) {
    if let Some(existing) = target {
        existing.merge_from(value);
    } else {
        *target = Some(value.clone());
    }
}

#[cfg(feature = "std")]
fn apply_task_config_override_to_graph(
    graph: &mut CuGraph,
    task_id: &str,
    value: &ComponentConfig,
) -> usize {
    let mut matches = 0usize;
    let node_indices: Vec<_> = graph.0.node_indices().collect();
    for node_index in node_indices {
        let node = &mut graph.0[node_index];
        if node.get_flavor() == Flavor::Task && node.id == task_id {
            merge_component_config(&mut node.config, value);
            matches += 1;
        }
    }
    matches
}

#[cfg(feature = "std")]
fn apply_bridge_node_config_override_to_graph(
    graph: &mut CuGraph,
    bridge_id: &str,
    value: &ComponentConfig,
) {
    let node_indices: Vec<_> = graph.0.node_indices().collect();
    for node_index in node_indices {
        let node = &mut graph.0[node_index];
        if node.get_flavor() == Flavor::Bridge && node.id == bridge_id {
            merge_component_config(&mut node.config, value);
        }
    }
}

#[cfg(feature = "std")]
fn parse_instance_override_target(path: &str) -> CuResult<(InstanceConfigTargetKind, String)> {
    let mut parts = path.split('/');
    let scope = parts.next().unwrap_or_default();
    let id = parts.next().unwrap_or_default();
    let leaf = parts.next().unwrap_or_default();

    if scope.is_empty() || id.is_empty() || leaf.is_empty() || parts.next().is_some() {
        return Err(CuError::from(format!(
            "Invalid instance override path '{}'. Expected 'tasks/<id>/config', 'resources/<id>/config', or 'bridges/<id>/config'.",
            path
        )));
    }

    if leaf != "config" {
        return Err(CuError::from(format!(
            "Invalid instance override path '{}'. Only the '/config' leaf is supported.",
            path
        )));
    }

    let kind = match scope {
        "tasks" => InstanceConfigTargetKind::Task,
        "resources" => InstanceConfigTargetKind::Resource,
        "bridges" => InstanceConfigTargetKind::Bridge,
        _ => {
            return Err(CuError::from(format!(
                "Invalid instance override path '{}'. Supported roots are 'tasks', 'resources', and 'bridges'.",
                path
            )));
        }
    };

    Ok((kind, id.to_string()))
}

#[cfg(feature = "std")]
fn apply_instance_config_set_operation(
    config: &mut CuConfig,
    operation: &InstanceConfigSetOperation,
) -> CuResult<()> {
    let (target_kind, target_id) = parse_instance_override_target(&operation.path)?;

    match target_kind {
        InstanceConfigTargetKind::Task => {
            let matches = match &mut config.graphs {
                ConfigGraphs::Simple(graph) => {
                    apply_task_config_override_to_graph(graph, &target_id, &operation.value)
                }
                ConfigGraphs::Missions(graphs) => graphs
                    .values_mut()
                    .map(|graph| {
                        apply_task_config_override_to_graph(graph, &target_id, &operation.value)
                    })
                    .sum(),
            };

            if matches == 0 {
                return Err(CuError::from(format!(
                    "Instance override path '{}' targets unknown task '{}'.",
                    operation.path, target_id
                )));
            }
        }
        InstanceConfigTargetKind::Resource => {
            let mut matches = 0usize;
            for resource in &mut config.resources {
                if resource.id == target_id {
                    merge_component_config(&mut resource.config, &operation.value);
                    matches += 1;
                }
            }
            if matches == 0 {
                return Err(CuError::from(format!(
                    "Instance override path '{}' targets unknown resource '{}'.",
                    operation.path, target_id
                )));
            }
        }
        InstanceConfigTargetKind::Bridge => {
            let mut matches = 0usize;
            for bridge in &mut config.bridges {
                if bridge.id == target_id {
                    merge_component_config(&mut bridge.config, &operation.value);
                    matches += 1;
                }
            }
            if matches == 0 {
                return Err(CuError::from(format!(
                    "Instance override path '{}' targets unknown bridge '{}'.",
                    operation.path, target_id
                )));
            }

            match &mut config.graphs {
                ConfigGraphs::Simple(graph) => {
                    apply_bridge_node_config_override_to_graph(graph, &target_id, &operation.value);
                }
                ConfigGraphs::Missions(graphs) => {
                    for graph in graphs.values_mut() {
                        apply_bridge_node_config_override_to_graph(
                            graph,
                            &target_id,
                            &operation.value,
                        );
                    }
                }
            }
        }
    }

    Ok(())
}

#[cfg(feature = "std")]
fn apply_instance_overrides(
    config: &mut CuConfig,
    overrides: &InstanceConfigOverridesRepresentation,
) -> CuResult<()> {
    for operation in &overrides.set {
        apply_instance_config_set_operation(config, operation)?;
    }
    Ok(())
}

#[cfg(feature = "std")]
fn apply_instance_overrides_from_file(
    config: &mut CuConfig,
    override_path: &std::path::Path,
) -> CuResult<()> {
    let override_content = read_to_string(override_path).map_err(|e| {
        CuError::from(format!(
            "Failed to read instance override file '{}'",
            override_path.display()
        ))
        .add_cause(e.to_string().as_str())
    })?;
    let overrides = parse_instance_config_overrides_string(&override_content).map_err(|e| {
        CuError::from(format!(
            "Failed to parse instance override file '{}': {e}",
            override_path.display()
        ))
    })?;
    apply_instance_overrides(config, &overrides)
}

#[cfg(feature = "std")]
#[allow(dead_code)]
fn parse_multi_config_string(content: &str) -> CuResult<MultiCopperConfigRepresentation> {
    Options::default()
        .with_default_extension(Extensions::IMPLICIT_SOME)
        .with_default_extension(Extensions::UNWRAP_NEWTYPES)
        .with_default_extension(Extensions::UNWRAP_VARIANT_NEWTYPES)
        .from_str(content)
        .map_err(|e| {
            CuError::from(format!(
                "Failed to parse multi-Copper configuration: Error: {} at position {}",
                e.code, e.span
            ))
        })
}

#[cfg(feature = "std")]
#[allow(dead_code)]
fn resolve_relative_config_path(base_path: Option<&str>, referenced_path: &str) -> String {
    if referenced_path.starts_with('/') || base_path.is_none() {
        return referenced_path.to_string();
    }

    let current_dir = std::path::Path::new(base_path.expect("checked above"))
        .parent()
        .unwrap_or_else(|| std::path::Path::new(""))
        .to_path_buf();
    current_dir
        .join(referenced_path)
        .to_string_lossy()
        .to_string()
}

#[cfg(feature = "std")]
#[allow(dead_code)]
fn parse_multi_endpoint(endpoint: &str) -> CuResult<MultiCopperEndpoint> {
    let mut parts = endpoint.split('/');
    let subsystem_id = parts.next().unwrap_or_default();
    let bridge_id = parts.next().unwrap_or_default();
    let channel_id = parts.next().unwrap_or_default();

    if subsystem_id.is_empty()
        || bridge_id.is_empty()
        || channel_id.is_empty()
        || parts.next().is_some()
    {
        return Err(CuError::from(format!(
            "Invalid multi-Copper endpoint '{endpoint}'. Expected 'subsystem/bridge/channel'."
        )));
    }

    Ok(MultiCopperEndpoint {
        subsystem_id: subsystem_id.to_string(),
        bridge_id: bridge_id.to_string(),
        channel_id: channel_id.to_string(),
    })
}

#[cfg(feature = "std")]
#[allow(dead_code)]
fn multi_channel_key(bridge_id: &str, channel_id: &str) -> String {
    format!("{bridge_id}/{channel_id}")
}

#[cfg(feature = "std")]
#[allow(dead_code)]
fn register_multi_channel_msg(
    contracts: &mut HashMap<String, MultiCopperChannelContract>,
    bridge_id: &str,
    channel_id: &str,
    expected_direction: MultiCopperChannelDirection,
    msg: &str,
) -> CuResult<()> {
    let key = multi_channel_key(bridge_id, channel_id);
    let contract = contracts.get_mut(&key).ok_or_else(|| {
        CuError::from(format!(
            "Bridge channel '{bridge_id}/{channel_id}' is referenced by the graph but not declared in the bridge config."
        ))
    })?;

    if contract.direction != expected_direction {
        let expected = match expected_direction {
            MultiCopperChannelDirection::Rx => "Rx",
            MultiCopperChannelDirection::Tx => "Tx",
        };
        return Err(CuError::from(format!(
            "Bridge channel '{bridge_id}/{channel_id}' is used as {expected} in the graph but declared with the opposite direction."
        )));
    }

    match &contract.msg {
        Some(existing) if existing != msg => Err(CuError::from(format!(
            "Bridge channel '{bridge_id}/{channel_id}' carries inconsistent message types '{existing}' and '{msg}'."
        ))),
        Some(_) => Ok(()),
        None => {
            contract.msg = Some(msg.to_string());
            Ok(())
        }
    }
}

#[cfg(feature = "std")]
#[allow(dead_code)]
fn build_multi_bridge_channel_contracts(
    config: &CuConfig,
) -> CuResult<HashMap<String, MultiCopperChannelContract>> {
    let graph = config
        .graphs
        .get_graph(Some(DEFAULT_MISSION_ID))
        .map_err(|e| {
            CuError::from(format!(
                "Multi-Copper subsystem configs with missions must define a '{DEFAULT_MISSION_ID}' mission: {e}"
            ))
        })?;

    let mut contracts = HashMap::new();
    for bridge in &config.bridges {
        for channel in &bridge.channels {
            let (channel_id, direction) = match channel {
                BridgeChannelConfigRepresentation::Rx { id, .. } => {
                    (id.as_str(), MultiCopperChannelDirection::Rx)
                }
                BridgeChannelConfigRepresentation::Tx { id, .. } => {
                    (id.as_str(), MultiCopperChannelDirection::Tx)
                }
            };

            let key = multi_channel_key(&bridge.id, channel_id);
            if contracts.contains_key(&key) {
                return Err(CuError::from(format!(
                    "Duplicate bridge channel declaration for '{key}'."
                )));
            }

            contracts.insert(
                key,
                MultiCopperChannelContract {
                    bridge_type: bridge.type_.clone(),
                    direction,
                    msg: None,
                },
            );
        }
    }

    for edge in graph.edges() {
        if let Some(channel_id) = &edge.src_channel {
            register_multi_channel_msg(
                &mut contracts,
                &edge.src,
                channel_id,
                MultiCopperChannelDirection::Rx,
                &edge.msg,
            )?;
        }
        if let Some(channel_id) = &edge.dst_channel {
            register_multi_channel_msg(
                &mut contracts,
                &edge.dst,
                channel_id,
                MultiCopperChannelDirection::Tx,
                &edge.msg,
            )?;
        }
    }

    Ok(contracts)
}

#[cfg(feature = "std")]
#[allow(dead_code)]
fn validate_multi_config_representation(
    representation: MultiCopperConfigRepresentation,
    file_path: Option<&str>,
    active_features: &[&str],
) -> CuResult<MultiCopperConfig> {
    if representation
        .instance_overrides_root
        .as_ref()
        .is_some_and(|root| root.trim().is_empty())
    {
        return Err(CuError::from(
            "Multi-Copper instance_overrides_root must not be empty.",
        ));
    }

    if representation.subsystems.is_empty() {
        return Err(CuError::from(
            "Multi-Copper config must declare at least one subsystem.",
        ));
    }
    if representation.subsystems.len() > usize::from(u16::MAX) + 1 {
        return Err(CuError::from(
            "Multi-Copper config supports at most 65536 distinct subsystem ids.",
        ));
    }

    let mut seen_subsystems = std::collections::HashSet::new();
    for subsystem in &representation.subsystems {
        if subsystem.id.trim().is_empty() {
            return Err(CuError::from(
                "Multi-Copper subsystem ids must not be empty.",
            ));
        }
        if !seen_subsystems.insert(subsystem.id.clone()) {
            return Err(CuError::from(format!(
                "Duplicate multi-Copper subsystem id '{}'.",
                subsystem.id
            )));
        }
    }

    let mut sorted_ids: Vec<_> = representation
        .subsystems
        .iter()
        .map(|subsystem| subsystem.id.clone())
        .collect();
    sorted_ids.sort();
    let subsystem_code_map: HashMap<_, _> = sorted_ids
        .into_iter()
        .enumerate()
        .map(|(idx, id)| {
            (
                id,
                u16::try_from(idx).expect("subsystem count was validated against u16 range"),
            )
        })
        .collect();

    let mut subsystem_contracts: HashMap<String, HashMap<String, MultiCopperChannelContract>> =
        HashMap::new();
    let mut subsystems = Vec::with_capacity(representation.subsystems.len());

    for subsystem in representation.subsystems {
        let resolved_config_path = resolve_relative_config_path(file_path, &subsystem.config);
        let config = read_configuration_with_features(&resolved_config_path, active_features)
            .map_err(|e| {
                CuError::from(format!(
                    "Failed to read subsystem '{}' from '{}': {e}",
                    subsystem.id, resolved_config_path
                ))
            })?;
        let contracts = build_multi_bridge_channel_contracts(&config).map_err(|e| {
            CuError::from(format!(
                "Invalid subsystem '{}' for multi-Copper validation: {e}",
                subsystem.id
            ))
        })?;
        subsystem_contracts.insert(subsystem.id.clone(), contracts);
        subsystems.push(MultiCopperSubsystem {
            subsystem_code: *subsystem_code_map
                .get(&subsystem.id)
                .expect("subsystem code map must contain every subsystem"),
            id: subsystem.id,
            config_path: resolved_config_path,
            config,
        });
    }

    let mut interconnects = Vec::with_capacity(representation.interconnects.len());
    for interconnect in representation.interconnects {
        if interconnect
            .when
            .as_ref()
            .is_some_and(|predicate| !predicate.evaluate(active_features))
        {
            continue;
        }

        let from = parse_multi_endpoint(&interconnect.from).map_err(|e| {
            CuError::from(format!(
                "Invalid multi-Copper interconnect source '{}': {e}",
                interconnect.from
            ))
        })?;
        let to = parse_multi_endpoint(&interconnect.to).map_err(|e| {
            CuError::from(format!(
                "Invalid multi-Copper interconnect destination '{}': {e}",
                interconnect.to
            ))
        })?;

        let from_contracts = subsystem_contracts.get(&from.subsystem_id).ok_or_else(|| {
            CuError::from(format!(
                "Interconnect source '{}' references unknown subsystem '{}'.",
                from, from.subsystem_id
            ))
        })?;
        let to_contracts = subsystem_contracts.get(&to.subsystem_id).ok_or_else(|| {
            CuError::from(format!(
                "Interconnect destination '{}' references unknown subsystem '{}'.",
                to, to.subsystem_id
            ))
        })?;

        let from_contract = from_contracts
            .get(&multi_channel_key(&from.bridge_id, &from.channel_id))
            .ok_or_else(|| {
                CuError::from(format!(
                    "Interconnect source '{}' references unknown bridge channel.",
                    from
                ))
            })?;
        let to_contract = to_contracts
            .get(&multi_channel_key(&to.bridge_id, &to.channel_id))
            .ok_or_else(|| {
                CuError::from(format!(
                    "Interconnect destination '{}' references unknown bridge channel.",
                    to
                ))
            })?;

        if from_contract.direction != MultiCopperChannelDirection::Tx {
            return Err(CuError::from(format!(
                "Interconnect source '{}' must reference a Tx bridge channel.",
                from
            )));
        }
        if to_contract.direction != MultiCopperChannelDirection::Rx {
            return Err(CuError::from(format!(
                "Interconnect destination '{}' must reference an Rx bridge channel.",
                to
            )));
        }

        if from_contract.bridge_type != to_contract.bridge_type {
            return Err(CuError::from(format!(
                "Interconnect '{}' -> '{}' mixes incompatible bridge types '{}' and '{}'.",
                from, to, from_contract.bridge_type, to_contract.bridge_type
            )));
        }

        let from_msg = from_contract.msg.as_ref().ok_or_else(|| {
            CuError::from(format!(
                "Interconnect source '{}' is not wired inside subsystem '{}', so its message type cannot be inferred.",
                from, from.subsystem_id
            ))
        })?;
        let to_msg = to_contract.msg.as_ref().ok_or_else(|| {
            CuError::from(format!(
                "Interconnect destination '{}' is not wired inside subsystem '{}', so its message type cannot be inferred.",
                to, to.subsystem_id
            ))
        })?;

        if from_msg != to_msg {
            return Err(CuError::from(format!(
                "Interconnect '{}' -> '{}' connects incompatible message types '{}' and '{}'.",
                from, to, from_msg, to_msg
            )));
        }
        if interconnect.msg != *from_msg {
            return Err(CuError::from(format!(
                "Interconnect '{}' -> '{}' declares message type '{}' but subsystem graphs require '{}'.",
                from, to, interconnect.msg, from_msg
            )));
        }

        interconnects.push(MultiCopperInterconnect {
            from,
            to,
            msg: interconnect.msg,
            bridge_type: from_contract.bridge_type.clone(),
        });
    }

    let instance_overrides_root = representation
        .instance_overrides_root
        .as_ref()
        .map(|root| resolve_relative_config_path(file_path, root));

    Ok(MultiCopperConfig {
        subsystems,
        interconnects,
        instance_overrides_root,
    })
}

/// Read a copper configuration from a file.
#[cfg(feature = "std")]
pub fn read_configuration(config_filename: &str) -> CuResult<CuConfig> {
    read_configuration_with_features(config_filename, &[])
}

/// Read a Copper configuration using the supplied compile-time Cargo features.
#[cfg(feature = "std")]
pub fn read_configuration_with_features(
    config_filename: &str,
    active_features: &[&str],
) -> CuResult<CuConfig> {
    let config_content = read_configuration_content(config_filename)?;
    read_configuration_str_with_features(config_content, Some(config_filename), active_features)
}

#[cfg(feature = "std")]
fn read_configuration_content(config_filename: &str) -> CuResult<String> {
    read_to_string(config_filename).map_err(|e| {
        CuError::from(format!(
            "Failed to read configuration file: {:?}",
            config_filename
        ))
        .add_cause(e.to_string().as_str())
    })
}

/// Read a copper configuration from a String.
/// Parse a RON string into a CuConfigRepresentation, using the standard options.
/// Returns an error if the parsing fails.
fn parse_config_string(content: &str) -> CuResult<CuConfigRepresentation> {
    Options::default()
        .with_default_extension(Extensions::IMPLICIT_SOME)
        .with_default_extension(Extensions::UNWRAP_NEWTYPES)
        .with_default_extension(Extensions::UNWRAP_VARIANT_NEWTYPES)
        .from_str(content)
        .map_err(|e| {
            CuError::from(format!(
                "Failed to parse configuration: Error: {} at position {}",
                e.code, e.span
            ))
        })
}

/// Convert a CuConfigRepresentation to a CuConfig.
/// Uses the deserialize_impl method and validates the logging configuration.
fn config_representation_to_config(representation: CuConfigRepresentation) -> CuResult<CuConfig> {
    #[allow(unused_mut)]
    let mut cuconfig = CuConfig::deserialize_impl(representation)
        .map_err(|e| CuError::from(format!("Error deserializing configuration: {e}")))?;

    #[cfg(feature = "std")]
    cuconfig.ensure_default_background_pool();

    cuconfig.validate_logging_config()?;
    cuconfig.validate_runtime_config()?;
    cuconfig.validate_stateless_configs()?;
    cuconfig.validate_anytime_configs()?;
    cuconfig.validate_constants()?;
    cuconfig.validate_log_streaming_config()?;

    Ok(cuconfig)
}

#[allow(unused_variables)]
fn resolve_configuration_representation(
    config_content: &str,
    file_path: Option<&str>,
    active_features: &[&str],
) -> CuResult<CuConfigRepresentation> {
    // Parse the configuration string
    let representation = parse_config_string(config_content)?;

    // Process includes and generate a merged configuration if a file path is provided
    // includes are only available with std.
    #[cfg(feature = "std")]
    let representation = if let Some(path) = file_path {
        process_includes(path, representation, &mut Vec::new(), active_features)?
    } else {
        representation
    };

    Ok(representation)
}

/// Read a Copper configuration and return the include-expanded RON used by proc-macro bundling.
///
/// The RON is serialized from the ordered source representation before it is lowered into
/// mission graph hash maps. This keeps task ordering aligned with generated runtime code.
#[cfg(feature = "std")]
#[doc(hidden)]
#[allow(dead_code)]
pub fn read_configuration_with_resolved_ron(config_filename: &str) -> CuResult<(CuConfig, String)> {
    read_configuration_with_resolved_ron_and_features(config_filename, &[])
}

/// Read and expand a Copper configuration using the supplied compile-time Cargo features.
#[cfg(feature = "std")]
#[doc(hidden)]
pub fn read_configuration_with_resolved_ron_and_features(
    config_filename: &str,
    active_features: &[&str],
) -> CuResult<(CuConfig, String)> {
    let config_content = read_configuration_content(config_filename)?;
    let representation = resolve_configuration_representation(
        &config_content,
        Some(config_filename),
        active_features,
    )?;
    let resolved_ron = CuConfig::get_options()
        .to_string_pretty(&representation, ron::ser::PrettyConfig::default())
        .map_err(|e| CuError::from(format!("Error serializing configuration: {e}")))?;
    let config = config_representation_to_config(representation)?;
    Ok((config, resolved_ron))
}

#[allow(dead_code)]
pub fn read_configuration_str(
    config_content: String,
    file_path: Option<&str>,
) -> CuResult<CuConfig> {
    read_configuration_str_with_features(config_content, file_path, &[])
}

/// Read a Copper configuration string using the supplied compile-time Cargo features.
pub fn read_configuration_str_with_features(
    config_content: String,
    file_path: Option<&str>,
    active_features: &[&str],
) -> CuResult<CuConfig> {
    let representation =
        resolve_configuration_representation(&config_content, file_path, active_features)?;

    // Convert the representation to a CuConfig and validate
    config_representation_to_config(representation)
}

/// Read a strict multi-Copper umbrella configuration from a file.
#[cfg(feature = "std")]
#[allow(dead_code)]
pub fn read_multi_configuration(config_filename: &str) -> CuResult<MultiCopperConfig> {
    read_multi_configuration_with_features(config_filename, &[])
}

/// Read a multi-Copper configuration using the supplied compile-time Cargo features.
#[cfg(feature = "std")]
#[allow(dead_code)]
pub fn read_multi_configuration_with_features(
    config_filename: &str,
    active_features: &[&str],
) -> CuResult<MultiCopperConfig> {
    let config_content = read_to_string(config_filename).map_err(|e| {
        CuError::from(format!(
            "Failed to read multi-Copper configuration file: {:?}",
            config_filename
        ))
        .add_cause(e.to_string().as_str())
    })?;
    read_multi_configuration_str_with_features(
        config_content,
        Some(config_filename),
        active_features,
    )
}

/// Read a strict multi-Copper umbrella configuration from a string.
#[cfg(feature = "std")]
#[allow(dead_code)]
pub fn read_multi_configuration_str(
    config_content: String,
    file_path: Option<&str>,
) -> CuResult<MultiCopperConfig> {
    read_multi_configuration_str_with_features(config_content, file_path, &[])
}

/// Read a multi-Copper configuration string using the supplied compile-time Cargo features.
#[cfg(feature = "std")]
#[allow(dead_code)]
pub fn read_multi_configuration_str_with_features(
    config_content: String,
    file_path: Option<&str>,
    active_features: &[&str],
) -> CuResult<MultiCopperConfig> {
    let representation = parse_multi_config_string(&config_content)?;
    validate_multi_config_representation(representation, file_path, active_features)
}

#[cfg(test)]
mod tests;
