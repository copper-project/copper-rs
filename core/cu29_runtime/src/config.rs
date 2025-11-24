//! This module defines the configuration of the copper runtime.
//! The configuration is a directed graph where nodes are tasks and edges are connections between tasks.
//! The configuration is serialized in the RON format.
//! The configuration is used to generate the runtime code at compile time.
#[cfg(not(feature = "std"))]
extern crate alloc;

use core::fmt;
use core::fmt::Display;
use cu29_traits::{CuError, CuResult};
use hashbrown::HashMap;
use petgraph::stable_graph::{EdgeIndex, NodeIndex, StableDiGraph};
use petgraph::visit::EdgeRef;
#[cfg(feature = "std")]
use petgraph::visit::IntoEdgeReferences;
pub use petgraph::Direction::Incoming;
pub use petgraph::Direction::Outgoing;
use ron::extensions::Extensions;
use ron::value::Value as RonValue;
use ron::{Number, Options};
use serde::{Deserialize, Deserializer, Serialize, Serializer};
use ConfigGraphs::{Missions, Simple};

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

/// NodeId is the unique identifier of a node in the configuration graph for petgraph
/// and the code generation.
pub type NodeId = u32;

/// This is the configuration of a component (like a task config or a monitoring config):w
/// It is a map of key-value pairs.
/// It is given to the new method of the task implementation.
#[derive(Serialize, Deserialize, Debug, Clone, Default)]
pub struct ComponentConfig(pub HashMap<String, Value>);

impl Display for ComponentConfig {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut first = true;
        let ComponentConfig(config) = self;
        write!(f, "{{")?;
        for (key, value) in config.iter() {
            if !first {
                write!(f, ", ")?;
            }
            write!(f, "{key}: {value}")?;
            first = false;
        }
        write!(f, "}}")
    }
}

// forward map interface
impl ComponentConfig {
    #[allow(dead_code)]
    pub fn new() -> Self {
        ComponentConfig(HashMap::new())
    }

    #[allow(dead_code)]
    pub fn get<T: From<Value>>(&self, key: &str) -> Option<T> {
        let ComponentConfig(config) = self;
        config.get(key).map(|v| T::from(v.clone()))
    }

    #[allow(dead_code)]
    pub fn set<T: Into<Value>>(&mut self, key: &str, value: T) {
        let ComponentConfig(config) = self;
        config.insert(key.to_string(), value.into());
    }
}

// The configuration Serialization format is as follows:
// (
//   tasks : [ (id: "toto", type: "zorglub::MyType", config: {...}),
//             (id: "titi", type: "zorglub::MyType2", config: {...})]
//   cnx : [ (src: "toto", dst: "titi", msg: "zorglub::MyMsgType"),...]
// )

/// Wrapper around the ron::Value to allow for custom serialization.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct Value(RonValue);

// Macro for implementing From<T> for Value where T is a numeric type
macro_rules! impl_from_numeric_for_value {
    ($($source:ty),* $(,)?) => {
        $(impl From<$source> for Value {
            fn from(value: $source) -> Self {
                Value(RonValue::Number(value.into()))
            }
        })*
    };
}

// Implement From for common numeric types
impl_from_numeric_for_value!(i8, i16, i32, i64, u8, u16, u32, u64, f32, f64);

impl From<Value> for bool {
    fn from(value: Value) -> Self {
        if let Value(RonValue::Bool(v)) = value {
            v
        } else {
            panic!("Expected a Boolean variant but got {value:?}")
        }
    }
}
macro_rules! impl_from_value_for_int {
    ($($target:ty),* $(,)?) => {
        $(
            impl From<Value> for $target {
                fn from(value: Value) -> Self {
                    if let Value(RonValue::Number(num)) = value {
                        match num {
                            Number::I8(n) => n as $target,
                            Number::I16(n) => n as $target,
                            Number::I32(n) => n as $target,
                            Number::I64(n) => n as $target,
                            Number::U8(n) => n as $target,
                            Number::U16(n) => n as $target,
                            Number::U32(n) => n as $target,
                            Number::U64(n) => n as $target,
                            Number::F32(_) | Number::F64(_) | Number::__NonExhaustive(_) => {
                                panic!("Expected an integer Number variant but got {num:?}")
                            }
                        }
                    } else {
                        panic!("Expected a Number variant but got {value:?}")
                    }
                }
            }
        )*
    };
}

impl_from_value_for_int!(u8, i8, u16, i16, u32, i32, u64, i64);

impl From<Value> for f64 {
    fn from(value: Value) -> Self {
        if let Value(RonValue::Number(num)) = value {
            num.into_f64()
        } else {
            panic!("Expected a Number variant but got {value:?}")
        }
    }
}

impl From<String> for Value {
    fn from(value: String) -> Self {
        Value(RonValue::String(value))
    }
}

impl From<Value> for String {
    fn from(value: Value) -> Self {
        if let Value(RonValue::String(s)) = value {
            s
        } else {
            panic!("Expected a String variant")
        }
    }
}

impl Display for Value {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let Value(value) = self;
        match value {
            RonValue::Number(n) => {
                let s = match n {
                    Number::I8(n) => n.to_string(),
                    Number::I16(n) => n.to_string(),
                    Number::I32(n) => n.to_string(),
                    Number::I64(n) => n.to_string(),
                    Number::U8(n) => n.to_string(),
                    Number::U16(n) => n.to_string(),
                    Number::U32(n) => n.to_string(),
                    Number::U64(n) => n.to_string(),
                    Number::F32(n) => n.0.to_string(),
                    Number::F64(n) => n.0.to_string(),
                    _ => panic!("Expected a Number variant but got {value:?}"),
                };
                write!(f, "{s}")
            }
            RonValue::String(s) => write!(f, "{s}"),
            RonValue::Bool(b) => write!(f, "{b}"),
            RonValue::Map(m) => write!(f, "{m:?}"),
            RonValue::Char(c) => write!(f, "{c:?}"),
            RonValue::Unit => write!(f, "unit"),
            RonValue::Option(o) => write!(f, "{o:?}"),
            RonValue::Seq(s) => write!(f, "{s:?}"),
            RonValue::Bytes(bytes) => write!(f, "{bytes:?}"),
        }
    }
}

/// Configuration for logging in the node.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct NodeLogging {
    enabled: bool,
}

/// Distinguishes regular tasks from bridge nodes so downstream stages can apply
/// bridge-specific instantiation rules.
#[derive(Default, Debug, Copy, Clone, PartialEq, Eq)]
pub enum Flavor {
    #[default]
    Task,
    Bridge,
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

    /// Config passed to the task.
    #[serde(skip_serializing_if = "Option::is_none")]
    config: Option<ComponentConfig>,

    /// Missions for which this task is run.
    missions: Option<Vec<String>>,

    /// Run this task in the background:
    /// ie. Will be set to run on a background thread and until it is finished `CuTask::process` will return None.
    #[serde(skip_serializing_if = "Option::is_none")]
    background: Option<bool>,

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

    /// Node role in the runtime graph (normal task or bridge endpoint).
    #[serde(skip, default)]
    flavor: Flavor,
}

impl Node {
    #[allow(dead_code)]
    pub fn new(id: &str, ptype: &str) -> Self {
        Node {
            id: id.to_string(),
            type_: Some(ptype.to_string()),
            config: None,
            missions: None,
            background: None,
            run_in_sim: None,
            logging: None,
            flavor: Flavor::Task,
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
    pub fn is_background(&self) -> bool {
        self.background.unwrap_or(false)
    }

    #[allow(dead_code)]
    pub fn get_instance_config(&self) -> Option<&ComponentConfig> {
        self.config.as_ref()
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
            logging.enabled
        } else {
            true
        }
    }

    #[allow(dead_code)]
    pub fn get_param<T: From<Value>>(&self, key: &str) -> Option<T> {
        let pc = self.config.as_ref()?;
        let ComponentConfig(pc) = pc;
        let v = pc.get(key)?;
        Some(T::from(v.clone()))
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

/// Declarative definition of a bridge component with a list of channels.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct BridgeConfig {
    pub id: String,
    #[serde(rename = "type")]
    pub type_: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub config: Option<ComponentConfig>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub missions: Option<Vec<String>>,
    /// List of logical endpoints exposed by this bridge.
    pub channels: Vec<BridgeChannelConfigRepresentation>,
}

impl BridgeConfig {
    fn to_node(&self) -> Node {
        let mut node = Node::new_with_flavor(&self.id, &self.type_, Flavor::Bridge);
        node.config = self.config.clone();
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
        self.0.node_indices().find_map(|node_index| {
            if let Some(node) = self.0.node_weight(node_index) {
                if node.id != node_id {
                    return None;
                }
                let edges: Vec<_> = self
                    .0
                    .edges_directed(node_index, Outgoing)
                    .map(|edge| edge.id().index())
                    .collect();
                if edges.is_empty() {
                    return None;
                }
                let cnx = self
                    .0
                    .edge_weight(EdgeIndex::new(edges[0]))
                    .expect("Found an cnx id but could not retrieve it back");
                return Some(cnx.msg.clone());
            }
            None
        })
    }

    #[allow(dead_code)]
    pub fn get_node_input_msg_type(&self, node_id: &str) -> Option<String> {
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
                let cnx = self
                    .0
                    .edge_weight(EdgeIndex::new(edges[0]))
                    .expect("Found an cnx id but could not retrieve it back");
                return Some(cnx.msg.clone());
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
            Simple(graph) => {
                let mut map = HashMap::new();
                map.insert("default".to_string(), graph.clone());
                map
            }
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
            Simple(graph) => {
                if mission_id.is_none() || mission_id.unwrap() == "default" {
                    Ok(graph)
                } else {
                    Err("Cannot get mission graph from simple config".into())
                }
            }
            Missions(graphs) => {
                if let Some(id) = mission_id {
                    graphs
                        .get(id)
                        .ok_or_else(|| format!("Mission {id} not found").into())
                } else {
                    Err("Mission ID required for mission configs".into())
                }
            }
        }
    }

    #[allow(dead_code)]
    pub fn get_graph_mut(&mut self, mission_id: Option<&str>) -> CuResult<&mut CuGraph> {
        match self {
            Simple(ref mut graph) => {
                if mission_id.is_none() {
                    Ok(graph)
                } else {
                    Err("Cannot get mission graph from simple config".into())
                }
            }
            Missions(ref mut graphs) => {
                if let Some(id) = mission_id {
                    graphs
                        .get_mut(id)
                        .ok_or_else(|| format!("Mission {id} not found").into())
                } else {
                    Err("Mission ID required for mission configs".into())
                }
            }
        }
    }

    pub fn add_mission(&mut self, mission_id: &str) -> CuResult<&mut CuGraph> {
        match self {
            Simple(_) => Err("Cannot add mission to simple config".into()),
            Missions(graphs) => {
                if graphs.contains_key(mission_id) {
                    Err(format!("Mission {mission_id} already exists").into())
                } else {
                    let graph = CuGraph::default();
                    graphs.insert(mission_id.to_string(), graph);
                    // Get a mutable reference to the newly inserted graph
                    Ok(graphs.get_mut(mission_id).unwrap())
                }
            }
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
    /// Optional monitoring configuration
    pub monitor: Option<MonitorConfig>,
    /// Optional logging configuration
    pub logging: Option<LoggingConfig>,
    /// Optional runtime configuration
    pub runtime: Option<RuntimeConfig>,
    /// Declarative bridge definitions that are yet to be expanded into the graph
    pub bridges: Vec<BridgeConfig>,
    /// Graph structure - either a single graph or multiple mission-specific graphs
    pub graphs: ConfigGraphs,
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

#[derive(Serialize, Deserialize, Default, Debug, Clone)]
pub struct LoggingConfig {
    /// Enable task logging to the log file.
    #[serde(default = "default_as_true", skip_serializing_if = "Clone::clone")]
    pub enable_task_logging: bool,

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
}

/// Missions are used to generate alternative DAGs within the same configuration.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct MissionsConfig {
    pub id: String,
}

/// Includes are used to include other configuration files.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct IncludesConfig {
    pub path: String,
    pub params: HashMap<String, Value>,
    pub missions: Option<Vec<String>>,
}

/// This is the main Copper configuration representation.
#[derive(Serialize, Deserialize, Default)]
struct CuConfigRepresentation {
    tasks: Option<Vec<Node>>,
    bridges: Option<Vec<BridgeConfig>>,
    cnx: Option<Vec<SerializedCnx>>,
    monitor: Option<MonitorConfig>,
    logging: Option<LoggingConfig>,
    runtime: Option<RuntimeConfig>,
    missions: Option<Vec<MissionsConfig>>,
    includes: Option<Vec<IncludesConfig>>,
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
                for c in cnx {
                    if let Some(cnx_missions) = &c.missions {
                        // if there is a filter by mission on the connection, only add the connection to the mission if it matches the filter.
                        if cnx_missions.contains(&mission_id.to_owned()) {
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
                                .connect_ext(
                                    src,
                                    dst,
                                    &c.msg,
                                    Some(cnx_missions.clone()),
                                    src_channel,
                                    dst_channel,
                                )
                                .map_err(|e| E::from(e.to_string()))?;
                        }
                    } else {
                        // if there is no filter by mission on the connection, add the connection to the mission.
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
                            .connect_ext(src, dst, &c.msg, None, src_channel, dst_channel)
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
            for c in cnx {
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
                    .connect_ext(src, dst, &c.msg, None, src_channel, dst_channel)
                    .map_err(|e| E::from(e.to_string()))?;
            }
        }
        cuconfig.graphs = Simple(graph);
    }

    cuconfig.monitor = representation.monitor.clone();
    cuconfig.logging = representation.logging.clone();
    cuconfig.runtime = representation.runtime.clone();
    cuconfig.bridges = representation.bridges.clone().unwrap_or_default();

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
        match &self.graphs {
            Simple(graph) => {
                let tasks: Vec<Node> = graph
                    .0
                    .node_indices()
                    .map(|idx| graph.0[idx].clone())
                    .filter(|node| node.get_flavor() == Flavor::Task)
                    .collect();

                let cnx: Vec<SerializedCnx> = graph
                    .0
                    .edge_indices()
                    .map(|edge| SerializedCnx::from(&graph.0[edge]))
                    .collect();

                CuConfigRepresentation {
                    tasks: Some(tasks),
                    bridges: bridges.clone(),
                    cnx: Some(cnx),
                    monitor: self.monitor.clone(),
                    logging: self.logging.clone(),
                    runtime: self.runtime.clone(),
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
                let mut cnx = Vec::new();

                for graph in graphs.values() {
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
                        let serialized = SerializedCnx::from(edge);
                        if !cnx.iter().any(|c: &SerializedCnx| {
                            c.src == serialized.src
                                && c.dst == serialized.dst
                                && c.msg == serialized.msg
                        }) {
                            cnx.push(serialized);
                        }
                    }
                }

                CuConfigRepresentation {
                    tasks: Some(tasks),
                    bridges,
                    cnx: Some(cnx),
                    monitor: self.monitor.clone(),
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
            graphs: Simple(CuGraph(StableDiGraph::new())),
            monitor: None,
            logging: None,
            runtime: None,
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
            graphs: Missions(HashMap::new()),
            monitor: None,
            logging: None,
            runtime: None,
            bridges: Vec::new(),
        }
    }

    fn get_options() -> Options {
        Options::default()
            .with_default_extension(Extensions::IMPLICIT_SOME)
            .with_default_extension(Extensions::UNWRAP_NEWTYPES)
            .with_default_extension(Extensions::UNWRAP_VARIANT_NEWTYPES)
    }

    #[allow(dead_code)]
    pub fn serialize_ron(&self) -> String {
        let ron = Self::get_options();
        let pretty = ron::ser::PrettyConfig::default();
        ron.to_string_pretty(&self, pretty).unwrap()
    }

    #[allow(dead_code)]
    pub fn deserialize_ron(ron: &str) -> Self {
        match Self::get_options().from_str(ron) {
            Ok(representation) => Self::deserialize_impl(representation).unwrap_or_else(|e| {
                panic!("Error deserializing configuration: {e}");
            }),
            Err(e) => panic!("Syntax Error in config: {} at position {}", e.code, e.span),
        }
    }

    fn deserialize_impl(representation: CuConfigRepresentation) -> Result<Self, String> {
        deserialize_config_representation(&representation)
    }

    /// Render the configuration graph in the dot format.
    #[cfg(feature = "std")]
    pub fn render(
        &self,
        output: &mut dyn std::io::Write,
        mission_id: Option<&str>,
    ) -> CuResult<()> {
        writeln!(output, "digraph G {{").unwrap();
        writeln!(output, "    graph [rankdir=LR, nodesep=0.8, ranksep=1.2];").unwrap();
        writeln!(output, "    node [shape=plain, fontname=\"Noto Sans\"];").unwrap();
        writeln!(output, "    edge [fontname=\"Noto Sans\"];").unwrap();

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

        writeln!(output, "}}").unwrap();
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
        self.monitor.as_ref()
    }

    #[allow(dead_code)]
    pub fn get_runtime_config(&self) -> Option<&RuntimeConfig> {
        self.runtime.as_ref()
    }

    /// Validate the logging configuration to ensure section pre-allocation sizes do not exceed slab sizes.
    /// This method is wrapper around [LoggingConfig::validate]
    pub fn validate_logging_config(&self) -> CuResult<()> {
        if let Some(logging) = &self.logging {
            return logging.validate();
        }
        Ok(())
    }
}

#[cfg(feature = "std")]
struct PortLookup {
    inputs: HashMap<String, String>,
    outputs: HashMap<String, String>,
    default_input: Option<String>,
    default_output: Option<String>,
}

#[cfg(feature = "std")]
#[derive(Clone)]
struct RenderNode {
    id: String,
    type_name: String,
    flavor: Flavor,
    inputs: Vec<String>,
    outputs: Vec<String>,
}

#[cfg(feature = "std")]
#[derive(Clone)]
struct RenderConnection {
    src: String,
    src_port: Option<String>,
    dst: String,
    dst_port: Option<String>,
    msg: String,
}

#[cfg(feature = "std")]
struct RenderTopology {
    nodes: Vec<RenderNode>,
    connections: Vec<RenderConnection>,
}

#[cfg(feature = "std")]
struct RenderSection<'a> {
    label: Option<String>,
    graph: &'a CuGraph,
}

#[cfg(feature = "std")]
impl CuConfig {
    fn render_section(
        &self,
        output: &mut dyn std::io::Write,
        graph: &CuGraph,
        label: Option<&str>,
    ) -> CuResult<()> {
        use std::fmt::Write as FmtWrite;

        let mut topology = build_render_topology(graph, &self.bridges);
        topology.nodes.sort_by(|a, b| a.id.cmp(&b.id));
        topology.connections.sort_by(|a, b| {
            a.src
                .cmp(&b.src)
                .then(a.dst.cmp(&b.dst))
                .then(a.msg.cmp(&b.msg))
        });

        let cluster_id = label.map(|lbl| format!("cluster_{}", sanitize_identifier(lbl)));
        if let Some(ref cluster_id) = cluster_id {
            writeln!(output, "    subgraph \"{cluster_id}\" {{").unwrap();
            writeln!(
                output,
                "        label=<<B>Mission: {}</B>>;",
                encode_text(label.unwrap())
            )
            .unwrap();
            writeln!(
                output,
                "        labelloc=t; labeljust=l; color=\"#bbbbbb\"; style=\"rounded\"; margin=20;"
            )
            .unwrap();
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

            let is_src = graph.get_dst_edges(node_idx).unwrap_or_default().is_empty();
            let is_sink = graph.get_src_edges(node_idx).unwrap_or_default().is_empty();

            let fillcolor = match node.flavor {
                Flavor::Bridge => "#faedcd",
                Flavor::Task if is_src => "#ddefc7",
                Flavor::Task if is_sink => "#cce0ff",
                _ => "#f2f2f2",
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
            writeln!(output, "{indent}\"{identifier}\" [label=<{label_html}>];").unwrap();

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
            .unwrap();
        }

        if cluster_id.is_some() {
            writeln!(output, "    }}").unwrap();
        }

        Ok(())
    }
}

#[cfg(feature = "std")]
fn build_render_topology(graph: &CuGraph, bridges: &[BridgeConfig]) -> RenderTopology {
    let mut bridge_lookup = HashMap::new();
    for bridge in bridges {
        bridge_lookup.insert(bridge.id.as_str(), bridge);
    }

    let mut nodes: HashMap<String, RenderNode> = HashMap::new();
    for (_, node) in graph.get_all_nodes() {
        let node_id = node.get_id();
        let mut inputs = Vec::new();
        let mut outputs = Vec::new();
        if node.get_flavor() == Flavor::Bridge {
            if let Some(bridge) = bridge_lookup.get(node_id.as_str()) {
                for channel in &bridge.channels {
                    match channel {
                        // Rx brings data from the bridge into the graph, so treat it as an output.
                        BridgeChannelConfigRepresentation::Rx { id, .. } => {
                            outputs.push(id.clone())
                        }
                        // Tx consumes data from the graph heading into the bridge, so show it on the input side.
                        BridgeChannelConfigRepresentation::Tx { id, .. } => inputs.push(id.clone()),
                    }
                }
            }
        }

        nodes.insert(
            node_id.clone(),
            RenderNode {
                id: node_id,
                type_name: node.get_type().to_string(),
                flavor: node.get_flavor(),
                inputs,
                outputs,
            },
        );
    }

    let mut connections = Vec::new();
    for edge in graph.0.edge_references() {
        let cnx = edge.weight();
        if let Some(node) = nodes.get_mut(&cnx.src) {
            if node.flavor == Flavor::Task && cnx.src_channel.is_none() && node.outputs.is_empty() {
                node.outputs.push("out0".to_string());
            }
        }
        if let Some(node) = nodes.get_mut(&cnx.dst) {
            if node.flavor == Flavor::Task && cnx.dst_channel.is_none() {
                let next = format!("in{}", node.inputs.len());
                node.inputs.push(next);
            }
        }

        connections.push(RenderConnection {
            src: cnx.src.clone(),
            src_port: cnx.src_channel.clone(),
            dst: cnx.dst.clone(),
            dst_port: cnx.dst_channel.clone(),
            msg: cnx.msg.clone(),
        });
    }

    RenderTopology {
        nodes: nodes.into_values().collect(),
        connections,
    }
}

#[cfg(feature = "std")]
impl PortLookup {
    fn resolve_input(&self, name: Option<&str>) -> Option<&str> {
        if let Some(name) = name {
            if let Some(port) = self.inputs.get(name) {
                return Some(port.as_str());
            }
        }
        self.default_input.as_deref()
    }

    fn resolve_output(&self, name: Option<&str>) -> Option<&str> {
        if let Some(name) = name {
            if let Some(port) = self.outputs.get(name) {
                return Some(port.as_str());
            }
        }
        self.default_output.as_deref()
    }
}

#[cfg(feature = "std")]
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
fn sanitize_identifier(value: &str) -> String {
    value
        .chars()
        .map(|c| if c.is_ascii_alphanumeric() { c } else { '_' })
        .collect()
}

#[cfg(feature = "std")]
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
        if let Some(section_size_mib) = self.section_size_mib {
            if let Some(slab_size_mib) = self.slab_size_mib {
                if section_size_mib > slab_size_mib {
                    return Err(CuError::from(format!("Section size ({section_size_mib} MiB) cannot be larger than slab size ({slab_size_mib} MiB). Adjust the parameters accordingly.")));
                }
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
) -> CuResult<CuConfigRepresentation> {
    // Note: Circular dependency detection removed
    processed_files.push(file_path.to_string());

    let mut result = base_representation;

    if let Some(includes) = result.includes.take() {
        for include in includes {
            let include_path = if include.path.starts_with('/') {
                include.path.clone()
            } else {
                let current_dir = std::path::Path::new(file_path)
                    .parent()
                    .unwrap_or_else(|| std::path::Path::new(""))
                    .to_string_lossy()
                    .to_string();

                format!("{}/{}", current_dir, include.path)
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

            included_representation =
                process_includes(&include_path, included_representation, processed_files)?;

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

            if let Some(included_cnx) = included_representation.cnx {
                if result.cnx.is_none() {
                    result.cnx = Some(included_cnx);
                } else {
                    let mut cnx = result.cnx.take().unwrap();
                    for included_c in included_cnx {
                        if !cnx
                            .iter()
                            .any(|c| c.src == included_c.src && c.dst == included_c.dst)
                        {
                            cnx.push(included_c);
                        }
                    }
                    result.cnx = Some(cnx);
                }
            }

            if result.monitor.is_none() {
                result.monitor = included_representation.monitor;
            }

            if result.logging.is_none() {
                result.logging = included_representation.logging;
            }

            if result.runtime.is_none() {
                result.runtime = included_representation.runtime;
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

/// Read a copper configuration from a file.
#[cfg(feature = "std")]
pub fn read_configuration(config_filename: &str) -> CuResult<CuConfig> {
    let config_content = read_to_string(config_filename).map_err(|e| {
        CuError::from(format!(
            "Failed to read configuration file: {:?}",
            &config_filename
        ))
        .add_cause(e.to_string().as_str())
    })?;
    read_configuration_str(config_content, Some(config_filename))
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
    let cuconfig = CuConfig::deserialize_impl(representation)
        .map_err(|e| CuError::from(format!("Error deserializing configuration: {e}")))?;

    cuconfig.validate_logging_config()?;

    Ok(cuconfig)
}

#[allow(unused_variables)]
pub fn read_configuration_str(
    config_content: String,
    file_path: Option<&str>,
) -> CuResult<CuConfig> {
    // Parse the configuration string
    let representation = parse_config_string(&config_content)?;

    // Process includes and generate a merged configuration if a file path is provided
    // includes are only available with std.
    #[cfg(feature = "std")]
    let representation = if let Some(path) = file_path {
        process_includes(path, representation, &mut Vec::new())?
    } else {
        representation
    };

    // Convert the representation to a CuConfig and validate
    config_representation_to_config(representation)
}

// tests
#[cfg(test)]
mod tests {
    use super::*;
    #[cfg(not(feature = "std"))]
    use alloc::vec;

    #[test]
    fn test_plain_serialize() {
        let mut config = CuConfig::default();
        let graph = config.get_graph_mut(None).unwrap();
        let n1 = graph
            .add_node(Node::new("test1", "package::Plugin1"))
            .unwrap();
        let n2 = graph
            .add_node(Node::new("test2", "package::Plugin2"))
            .unwrap();
        graph.connect(n1, n2, "msgpkg::MsgType").unwrap();
        let serialized = config.serialize_ron();
        let deserialized = CuConfig::deserialize_ron(&serialized);
        let graph = config.graphs.get_graph(None).unwrap();
        let deserialized_graph = deserialized.graphs.get_graph(None).unwrap();
        assert_eq!(graph.node_count(), deserialized_graph.node_count());
        assert_eq!(graph.edge_count(), deserialized_graph.edge_count());
    }

    #[test]
    fn test_serialize_with_params() {
        let mut config = CuConfig::default();
        let graph = config.get_graph_mut(None).unwrap();
        let mut camera = Node::new("copper-camera", "camerapkg::Camera");
        camera.set_param::<Value>("resolution-height", 1080.into());
        graph.add_node(camera).unwrap();
        let serialized = config.serialize_ron();
        let config = CuConfig::deserialize_ron(&serialized);
        let deserialized = config.get_graph(None).unwrap();
        assert_eq!(
            deserialized
                .get_node(0)
                .unwrap()
                .get_param::<i32>("resolution-height")
                .unwrap(),
            1080
        );
    }

    #[test]
    #[should_panic(expected = "Syntax Error in config: Expected opening `[` at position 1:9-1:10")]
    fn test_deserialization_error() {
        // Task needs to be an array, but provided tuple wrongfully
        let txt = r#"( tasks: (), cnx: [], monitor: (type: "ExampleMonitor", ) ) "#;
        CuConfig::deserialize_ron(txt);
    }
    #[test]
    fn test_missions() {
        let txt = r#"( missions: [ (id: "data_collection"), (id: "autonomous")])"#;
        let config = CuConfig::deserialize_ron(txt);
        let graph = config.graphs.get_graph(Some("data_collection")).unwrap();
        assert!(graph.node_count() == 0);
        let graph = config.graphs.get_graph(Some("autonomous")).unwrap();
        assert!(graph.node_count() == 0);
    }

    #[test]
    fn test_monitor() {
        let txt = r#"( tasks: [], cnx: [], monitor: (type: "ExampleMonitor", ) ) "#;
        let config = CuConfig::deserialize_ron(txt);
        assert_eq!(config.monitor.as_ref().unwrap().type_, "ExampleMonitor");

        let txt =
            r#"( tasks: [], cnx: [], monitor: (type: "ExampleMonitor", config: { "toto": 4, } )) "#;
        let config = CuConfig::deserialize_ron(txt);
        assert_eq!(
            config.monitor.as_ref().unwrap().config.as_ref().unwrap().0["toto"].0,
            4u8.into()
        );
    }

    #[test]
    fn test_logging_parameters() {
        // Test with `enable_task_logging: false`
        let txt = r#"( tasks: [], cnx: [], logging: ( slab_size_mib: 1024, section_size_mib: 100, enable_task_logging: false ),) "#;

        let config = CuConfig::deserialize_ron(txt);
        assert!(config.logging.is_some());
        let logging_config = config.logging.unwrap();
        assert_eq!(logging_config.slab_size_mib.unwrap(), 1024);
        assert_eq!(logging_config.section_size_mib.unwrap(), 100);
        assert!(!logging_config.enable_task_logging);

        // Test with `enable_task_logging` not provided
        let txt =
            r#"( tasks: [], cnx: [], logging: ( slab_size_mib: 1024, section_size_mib: 100, ),) "#;
        let config = CuConfig::deserialize_ron(txt);
        assert!(config.logging.is_some());
        let logging_config = config.logging.unwrap();
        assert_eq!(logging_config.slab_size_mib.unwrap(), 1024);
        assert_eq!(logging_config.section_size_mib.unwrap(), 100);
        assert!(logging_config.enable_task_logging);
    }

    #[test]
    fn test_bridge_parsing() {
        let txt = r#"
        (
            tasks: [
                (id: "dst", type: "tasks::Destination"),
                (id: "src", type: "tasks::Source"),
            ],
            bridges: [
                (
                    id: "radio",
                    type: "tasks::SerialBridge",
                    config: { "path": "/dev/ttyACM0", "baud": 921600 },
                    channels: [
                        Rx ( id: "status", route: "sys/status" ),
                        Tx ( id: "motor", route: "motor/cmd" ),
                    ],
                ),
            ],
            cnx: [
                (src: "radio/status", dst: "dst", msg: "mymsgs::Status"),
                (src: "src", dst: "radio/motor", msg: "mymsgs::MotorCmd"),
            ],
        )
        "#;

        let config = CuConfig::deserialize_ron(txt);
        assert_eq!(config.bridges.len(), 1);
        let bridge = &config.bridges[0];
        assert_eq!(bridge.id, "radio");
        assert_eq!(bridge.channels.len(), 2);
        match &bridge.channels[0] {
            BridgeChannelConfigRepresentation::Rx { id, route, .. } => {
                assert_eq!(id, "status");
                assert_eq!(route.as_deref(), Some("sys/status"));
            }
            _ => panic!("expected Rx channel"),
        }
        match &bridge.channels[1] {
            BridgeChannelConfigRepresentation::Tx { id, route, .. } => {
                assert_eq!(id, "motor");
                assert_eq!(route.as_deref(), Some("motor/cmd"));
            }
            _ => panic!("expected Tx channel"),
        }
        let graph = config.graphs.get_graph(None).unwrap();
        let bridge_id = graph
            .get_node_id_by_name("radio")
            .expect("bridge node missing");
        let bridge_node = graph.get_node(bridge_id).unwrap();
        assert_eq!(bridge_node.get_flavor(), Flavor::Bridge);

        // Edges should retain channel metadata.
        let mut edges = Vec::new();
        for edge_idx in graph.0.edge_indices() {
            edges.push(graph.0[edge_idx].clone());
        }
        assert_eq!(edges.len(), 2);
        let status_edge = edges
            .iter()
            .find(|e| e.dst == "dst")
            .expect("status edge missing");
        assert_eq!(status_edge.src_channel.as_deref(), Some("status"));
        assert!(status_edge.dst_channel.is_none());
        let motor_edge = edges
            .iter()
            .find(|e| e.dst_channel.is_some())
            .expect("motor edge missing");
        assert_eq!(motor_edge.dst_channel.as_deref(), Some("motor"));
    }

    #[test]
    fn test_bridge_roundtrip() {
        let mut config = CuConfig::default();
        let mut bridge_config = ComponentConfig::default();
        bridge_config.set("port", "/dev/ttyACM0".to_string());
        config.bridges.push(BridgeConfig {
            id: "radio".to_string(),
            type_: "tasks::SerialBridge".to_string(),
            config: Some(bridge_config),
            missions: None,
            channels: vec![
                BridgeChannelConfigRepresentation::Rx {
                    id: "status".to_string(),
                    route: Some("sys/status".to_string()),
                    config: None,
                },
                BridgeChannelConfigRepresentation::Tx {
                    id: "motor".to_string(),
                    route: Some("motor/cmd".to_string()),
                    config: None,
                },
            ],
        });

        let serialized = config.serialize_ron();
        assert!(
            serialized.contains("bridges"),
            "bridges section missing from serialized config"
        );
        let deserialized = CuConfig::deserialize_ron(&serialized);
        assert_eq!(deserialized.bridges.len(), 1);
        let bridge = &deserialized.bridges[0];
        assert_eq!(bridge.channels.len(), 2);
        assert!(matches!(
            bridge.channels[0],
            BridgeChannelConfigRepresentation::Rx { .. }
        ));
        assert!(matches!(
            bridge.channels[1],
            BridgeChannelConfigRepresentation::Tx { .. }
        ));
    }

    #[test]
    fn test_bridge_channel_config() {
        let txt = r#"
        (
            tasks: [],
            bridges: [
                (
                    id: "radio",
                    type: "tasks::SerialBridge",
                    channels: [
                        Rx ( id: "status", route: "sys/status", config: { "filter": "fast" } ),
                        Tx ( id: "imu", route: "telemetry/imu", config: { "rate": 100 } ),
                    ],
                ),
            ],
            cnx: [],
        )
        "#;

        let config = CuConfig::deserialize_ron(txt);
        let bridge = &config.bridges[0];
        match &bridge.channels[0] {
            BridgeChannelConfigRepresentation::Rx {
                config: Some(cfg), ..
            } => {
                let val: String = cfg.get("filter").expect("filter missing");
                assert_eq!(val, "fast");
            }
            _ => panic!("expected Rx channel with config"),
        }
        match &bridge.channels[1] {
            BridgeChannelConfigRepresentation::Tx {
                config: Some(cfg), ..
            } => {
                let rate: i32 = cfg.get("rate").expect("rate missing");
                assert_eq!(rate, 100);
            }
            _ => panic!("expected Tx channel with config"),
        }
    }

    #[test]
    #[should_panic(expected = "channel 'motor' is Tx and cannot act as a source")]
    fn test_bridge_tx_cannot_be_source() {
        let txt = r#"
        (
            tasks: [
                (id: "dst", type: "tasks::Destination"),
            ],
            bridges: [
                (
                    id: "radio",
                    type: "tasks::SerialBridge",
                    channels: [
                        Tx ( id: "motor", route: "motor/cmd" ),
                    ],
                ),
            ],
            cnx: [
                (src: "radio/motor", dst: "dst", msg: "mymsgs::MotorCmd"),
            ],
        )
        "#;

        CuConfig::deserialize_ron(txt);
    }

    #[test]
    #[should_panic(expected = "channel 'status' is Rx and cannot act as a destination")]
    fn test_bridge_rx_cannot_be_destination() {
        let txt = r#"
        (
            tasks: [
                (id: "src", type: "tasks::Source"),
            ],
            bridges: [
                (
                    id: "radio",
                    type: "tasks::SerialBridge",
                    channels: [
                        Rx ( id: "status", route: "sys/status" ),
                    ],
                ),
            ],
            cnx: [
                (src: "src", dst: "radio/status", msg: "mymsgs::Status"),
            ],
        )
        "#;

        CuConfig::deserialize_ron(txt);
    }

    #[test]
    fn test_validate_logging_config() {
        // Test with valid logging configuration
        let txt =
            r#"( tasks: [], cnx: [], logging: ( slab_size_mib: 1024, section_size_mib: 100 ) )"#;
        let config = CuConfig::deserialize_ron(txt);
        assert!(config.validate_logging_config().is_ok());

        // Test with invalid logging configuration
        let txt =
            r#"( tasks: [], cnx: [], logging: ( slab_size_mib: 100, section_size_mib: 1024 ) )"#;
        let config = CuConfig::deserialize_ron(txt);
        assert!(config.validate_logging_config().is_err());
    }

    // this test makes sure the edge id is suitable to be used to sort the inputs of a task
    #[test]
    fn test_deserialization_edge_id_assignment() {
        // note here that the src1 task is added before src2 in the tasks array,
        // however, src1 connection is added AFTER src2 in the cnx array
        let txt = r#"(
            tasks: [(id: "src1", type: "a"), (id: "src2", type: "b"), (id: "sink", type: "c")],
            cnx: [(src: "src2", dst: "sink", msg: "msg1"), (src: "src1", dst: "sink", msg: "msg2")]
        )"#;
        let config = CuConfig::deserialize_ron(txt);
        let graph = config.graphs.get_graph(None).unwrap();
        assert!(config.validate_logging_config().is_ok());

        // the node id depends on the order in which the tasks are added
        let src1_id = 0;
        assert_eq!(graph.get_node(src1_id).unwrap().id, "src1");
        let src2_id = 1;
        assert_eq!(graph.get_node(src2_id).unwrap().id, "src2");

        // the edge id depends on the order the connection is created
        // the src2 was added second in the tasks, but the connection was added first
        let src1_edge_id = *graph.get_src_edges(src1_id).unwrap().first().unwrap();
        assert_eq!(src1_edge_id, 1);
        let src2_edge_id = *graph.get_src_edges(src2_id).unwrap().first().unwrap();
        assert_eq!(src2_edge_id, 0);
    }

    #[test]
    fn test_simple_missions() {
        // A simple config that selection a source depending on the mission it is in.
        let txt = r#"(
                    missions: [ (id: "m1"),
                                (id: "m2"),
                                ],
                    tasks: [(id: "src1", type: "a", missions: ["m1"]),
                            (id: "src2", type: "b", missions: ["m2"]),
                            (id: "sink", type: "c")],

                    cnx: [
                            (src: "src1", dst: "sink", msg: "u32", missions: ["m1"]),
                            (src: "src2", dst: "sink", msg: "u32", missions: ["m2"]),
                         ],
              )
              "#;

        let config = CuConfig::deserialize_ron(txt);
        let m1_graph = config.graphs.get_graph(Some("m1")).unwrap();
        assert_eq!(m1_graph.edge_count(), 1);
        assert_eq!(m1_graph.node_count(), 2);
        let index = 0;
        let cnx = m1_graph.get_edge_weight(index).unwrap();

        assert_eq!(cnx.src, "src1");
        assert_eq!(cnx.dst, "sink");
        assert_eq!(cnx.msg, "u32");
        assert_eq!(cnx.missions, Some(vec!["m1".to_string()]));

        let m2_graph = config.graphs.get_graph(Some("m2")).unwrap();
        assert_eq!(m2_graph.edge_count(), 1);
        assert_eq!(m2_graph.node_count(), 2);
        let index = 0;
        let cnx = m2_graph.get_edge_weight(index).unwrap();
        assert_eq!(cnx.src, "src2");
        assert_eq!(cnx.dst, "sink");
        assert_eq!(cnx.msg, "u32");
        assert_eq!(cnx.missions, Some(vec!["m2".to_string()]));
    }
    #[test]
    fn test_mission_serde() {
        // A simple config that selection a source depending on the mission it is in.
        let txt = r#"(
                    missions: [ (id: "m1"),
                                (id: "m2"),
                                ],
                    tasks: [(id: "src1", type: "a", missions: ["m1"]),
                            (id: "src2", type: "b", missions: ["m2"]),
                            (id: "sink", type: "c")],

                    cnx: [
                            (src: "src1", dst: "sink", msg: "u32", missions: ["m1"]),
                            (src: "src2", dst: "sink", msg: "u32", missions: ["m2"]),
                         ],
              )
              "#;

        let config = CuConfig::deserialize_ron(txt);
        let serialized = config.serialize_ron();
        let deserialized = CuConfig::deserialize_ron(&serialized);
        let m1_graph = deserialized.graphs.get_graph(Some("m1")).unwrap();
        assert_eq!(m1_graph.edge_count(), 1);
        assert_eq!(m1_graph.node_count(), 2);
        let index = 0;
        let cnx = m1_graph.get_edge_weight(index).unwrap();
        assert_eq!(cnx.src, "src1");
        assert_eq!(cnx.dst, "sink");
        assert_eq!(cnx.msg, "u32");
        assert_eq!(cnx.missions, Some(vec!["m1".to_string()]));
    }

    #[test]
    fn test_keyframe_interval() {
        // note here that the src1 task is added before src2 in the tasks array,
        // however, src1 connection is added AFTER src2 in the cnx array
        let txt = r#"(
            tasks: [(id: "src1", type: "a"), (id: "src2", type: "b"), (id: "sink", type: "c")],
            cnx: [(src: "src2", dst: "sink", msg: "msg1"), (src: "src1", dst: "sink", msg: "msg2")],
            logging: ( keyframe_interval: 314 )
        )"#;
        let config = CuConfig::deserialize_ron(txt);
        let logging_config = config.logging.unwrap();
        assert_eq!(logging_config.keyframe_interval.unwrap(), 314);
    }

    #[test]
    fn test_default_keyframe_interval() {
        // note here that the src1 task is added before src2 in the tasks array,
        // however, src1 connection is added AFTER src2 in the cnx array
        let txt = r#"(
            tasks: [(id: "src1", type: "a"), (id: "src2", type: "b"), (id: "sink", type: "c")],
            cnx: [(src: "src2", dst: "sink", msg: "msg1"), (src: "src1", dst: "sink", msg: "msg2")],
            logging: ( slab_size_mib: 200, section_size_mib: 1024, )
        )"#;
        let config = CuConfig::deserialize_ron(txt);
        let logging_config = config.logging.unwrap();
        assert_eq!(logging_config.keyframe_interval.unwrap(), 100);
    }
}
