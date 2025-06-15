//! This module defines the configuration of the copper runtime.
//! The configuration is a directed graph where nodes are tasks and edges are connections between tasks.
//! The configuration is serialized in the RON format.
//! The configuration is used to generate the runtime code at compile time.

use cu29_traits::{CuError, CuResult};
use html_escape::encode_text;
use petgraph::stable_graph::{EdgeIndex, NodeIndex, StableDiGraph};
use petgraph::visit::EdgeRef;
pub use petgraph::Direction::Incoming;
pub use petgraph::Direction::Outgoing;
use ron::extensions::Extensions;
use ron::value::Value as RonValue;
use ron::{Number, Options};
use serde::{Deserialize, Deserializer, Serialize, Serializer};
use std::collections::HashMap;
use std::fmt;
use std::fmt::Display;
use std::fs::read_to_string;
use ConfigGraphs::{Missions, Simple};

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
                            Number::F32(_) | Number::F64(_) => {
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

/// A node in the configuration graph.
/// A node represents a Task in the system Graph.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct Node {
    id: String,

    #[serde(rename = "type", skip_serializing_if = "Option::is_none")]
    type_: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    config: Option<ComponentConfig>,

    missions: Option<Vec<String>>,
}

impl Node {
    #[allow(dead_code)]
    pub fn new(id: &str, ptype: &str) -> Self {
        Node {
            id: id.to_string(),
            type_: Some(ptype.to_string()),
            // base_period_ns: None,
            config: None,
            missions: None,
        }
    }

    #[allow(dead_code)]
    pub fn get_id(&self) -> String {
        self.id.clone()
    }

    #[allow(dead_code)]
    pub fn set_type(mut self, name: Option<String>) -> Self {
        self.type_ = name;
        self
    }

    #[allow(dead_code)]
    pub fn get_type(&self) -> &str {
        self.type_.as_ref().unwrap()
    }

    #[allow(dead_code)]
    pub fn get_instance_config(&self) -> Option<&ComponentConfig> {
        self.config.as_ref()
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
}

/// This represents a connection between 2 tasks (nodes) in the configuration graph.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct Cnx {
    /// Source node id.
    src: String,

    // Destination node id.
    dst: String,

    /// Message type exchanged between src and dst.
    pub msg: String,

    /// Restrict this connection for this list of missions.
    pub missions: Option<Vec<String>>,

    /// Tells Copper if it needs to log the messages.
    pub store: Option<bool>,
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

    pub fn node_indices(&self) -> Vec<petgraph::stable_graph::NodeIndex> {
        self.0.node_indices().collect()
    }

    pub fn add_node(&mut self, node: Node) -> CuResult<NodeId> {
        Ok(self.0.add_node(node).index() as NodeId)
    }

    pub fn connect_ext(
        &mut self,
        source: NodeId,
        target: NodeId,
        msg_type: &str,
        store: Option<bool>,
        missions: Option<Vec<String>>,
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
                store,
            },
        );
        Ok(())
    }
    /// Get the node with the given id.
    /// If mission_id is provided, get the node from that mission's graph.
    /// Otherwise get the node from the simple graph.
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
                    panic!("A CuSrcTask is configured with no task connected to it.")
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

    /// Adds an edge between two nodes/tasks in the configuration graph.
    /// msg_type is the type of message exchanged between the two nodes/tasks.
    #[allow(dead_code)]
    pub fn connect(&mut self, source: NodeId, target: NodeId, msg_type: &str) -> CuResult<()> {
        self.connect_ext(source, target, msg_type, None, None)
    }
}

impl std::ops::Index<NodeIndex> for CuGraph {
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

#[derive(Serialize, Deserialize, Default, Debug, Clone)]
pub struct LoggingConfig {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub slab_size_mib: Option<u64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub section_size_mib: Option<u64>,
    #[serde(default = "default_as_true", skip_serializing_if = "Clone::clone")]
    pub enable_task_logging: bool,
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
    cnx: Option<Vec<Cnx>>,
    monitor: Option<MonitorConfig>,
    logging: Option<LoggingConfig>,
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

            if let Some(cnx) = &representation.cnx {
                for c in cnx {
                    if let Some(cnx_missions) = &c.missions {
                        // if there is a filter by mission on the connection, only add the connection to the mission if it matches the filter.
                        if cnx_missions.contains(&mission_id.to_owned()) {
                            let src = graph
                                .node_indices()
                                .into_iter()
                                .find(|i| graph.get_node(i.index() as NodeId).unwrap().id == c.src)
                                .ok_or_else(|| {
                                    E::from(format!("Source node not found: {}", c.src))
                                })?;
                            let dst = graph
                                .node_indices()
                                .into_iter()
                                .find(|i| graph.get_node(i.index() as NodeId).unwrap().id == c.dst)
                                .ok_or_else(|| {
                                    E::from(format!("Destination node not found: {}", c.dst))
                                })?;
                            graph
                                .connect_ext(
                                    src.index() as NodeId,
                                    dst.index() as NodeId,
                                    &c.msg,
                                    c.store,
                                    Some(cnx_missions.clone()),
                                )
                                .map_err(|e| E::from(e.to_string()))?;
                        }
                    } else {
                        // if there is no filter by mission on the connection, add the connection to the mission.
                        let src = graph
                            .node_indices()
                            .into_iter()
                            .find(|i| graph.get_node(i.index() as NodeId).unwrap().id == c.src)
                            .ok_or_else(|| E::from(format!("Source node not found: {}", c.src)))?;
                        let dst = graph
                            .node_indices()
                            .into_iter()
                            .find(|i| graph.get_node(i.index() as NodeId).unwrap().id == c.dst)
                            .ok_or_else(|| {
                                E::from(format!("Destination node not found: {}", c.dst))
                            })?;
                        graph
                            .connect_ext(
                                src.index() as NodeId,
                                dst.index() as NodeId,
                                &c.msg,
                                c.store,
                                None,
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

        if let Some(cnx) = &representation.cnx {
            for c in cnx {
                let src = graph
                    .node_indices()
                    .into_iter()
                    .find(|i| graph.get_node(i.index() as NodeId).unwrap().id == c.src)
                    .ok_or_else(|| E::from(format!("Source node not found: {}", c.src)))?;
                let dst = graph
                    .node_indices()
                    .into_iter()
                    .find(|i| graph.get_node(i.index() as NodeId).unwrap().id == c.dst)
                    .ok_or_else(|| E::from(format!("Destination node not found: {}", c.dst)))?;
                graph
                    .connect_ext(
                        src.index() as NodeId,
                        dst.index() as NodeId,
                        &c.msg,
                        c.store,
                        None,
                    )
                    .map_err(|e| E::from(e.to_string()))?;
            }
        }
        cuconfig.graphs = Simple(graph);
    }

    cuconfig.monitor = representation.monitor.clone();
    cuconfig.logging = representation.logging.clone();

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
        match &self.graphs {
            Simple(graph) => {
                let tasks: Vec<Node> = graph
                    .0
                    .node_indices()
                    .map(|idx| graph.0[idx].clone())
                    .collect();

                let cnx: Vec<Cnx> = graph
                    .0
                    .edge_indices()
                    .map(|edge| graph.0[edge].clone())
                    .collect();

                CuConfigRepresentation {
                    tasks: Some(tasks),
                    cnx: Some(cnx),
                    monitor: self.monitor.clone(),
                    logging: self.logging.clone(),
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
                        if !tasks.iter().any(|n: &Node| n.id == node.id) {
                            tasks.push(node.clone());
                        }
                    }

                    // Add all edges from this mission
                    for edge_idx in graph.0.edge_indices() {
                        let edge = &graph.0[edge_idx];
                        if !cnx.iter().any(|c: &Cnx| {
                            c.src == edge.src && c.dst == edge.dst && c.msg == edge.msg
                        }) {
                            cnx.push(edge.clone());
                        }
                    }
                }

                CuConfigRepresentation {
                    tasks: Some(tasks),
                    cnx: Some(cnx),
                    monitor: self.monitor.clone(),
                    logging: self.logging.clone(),
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
            Err(e) => panic!(
                "Syntax Error in config: {} at position {}",
                e.code, e.position
            ),
        }
    }

    fn deserialize_impl(representation: CuConfigRepresentation) -> Result<Self, String> {
        deserialize_config_representation(&representation)
    }

    /// Render the configuration graph in the dot format.
    pub fn render(
        &self,
        output: &mut dyn std::io::Write,
        mission_id: Option<&str>,
    ) -> CuResult<()> {
        writeln!(output, "digraph G {{").unwrap();

        let graph = self.get_graph(mission_id)?;

        for index in graph.node_indices() {
            let node = &graph[index];
            let config_str = match &node.config {
                Some(config) => {
                    let config_str = config
                        .0
                        .iter()
                        .map(|(k, v)| format!("<B>{k}</B> = {v}<BR ALIGN=\"LEFT\"/>"))
                        .collect::<Vec<String>>()
                        .join("\n");
                    format!("____________<BR/><BR ALIGN=\"LEFT\"/>{config_str}")
                }
                None => String::new(),
            };
            writeln!(output, "{} [", index.index()).unwrap();
            writeln!(output, "shape=box,").unwrap();
            writeln!(output, "style=\"rounded, filled\",").unwrap();
            writeln!(output, "fontname=\"Noto Sans\"").unwrap();

            let is_src = graph
                .get_dst_edges(index.index() as NodeId)
                .unwrap_or_default()
                .is_empty();
            let is_sink = graph
                .get_src_edges(index.index() as NodeId)
                .unwrap_or_default()
                .is_empty();
            if is_src {
                writeln!(output, "fillcolor=lightgreen,").unwrap();
            } else if is_sink {
                writeln!(output, "fillcolor=lightblue,").unwrap();
            } else {
                writeln!(output, "fillcolor=lightgrey,").unwrap();
            }
            writeln!(output, "color=grey,").unwrap();

            writeln!(output, "labeljust=l,").unwrap();
            writeln!(
                output,
                "label=< <FONT COLOR=\"red\"><B>{}</B></FONT> <FONT COLOR=\"dimgray\">[{}]</FONT><BR ALIGN=\"LEFT\"/>{} >",
                node.id,
                node.get_type(),
                config_str
            )
                .unwrap();

            writeln!(output, "];").unwrap();
        }
        for edge in graph.0.edge_indices() {
            let (src, dst) = graph.0.edge_endpoints(edge).unwrap();

            let cnx = &graph.0[edge];
            let msg = encode_text(&cnx.msg);
            writeln!(
                output,
                "{} -> {} [label=< <B><FONT COLOR=\"gray\">{}</FONT></B> >];",
                src.index(),
                dst.index(),
                msg
            )
            .unwrap();
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

    /// Validate the logging configuration to ensure section pre-allocation sizes do not exceed slab sizes.
    /// This method is wrapper around [LoggingConfig::validate]
    pub fn validate_logging_config(&self) -> CuResult<()> {
        if let Some(logging) = &self.logging {
            return logging.validate();
        }
        Ok(())
    }
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

fn substitute_parameters(content: &str, params: &HashMap<String, Value>) -> String {
    let mut result = content.to_string();

    for (key, value) in params {
        let pattern = format!("{{{{{key}}}}}");
        result = result.replace(&pattern, &value.to_string());
    }

    result
}

/// Returns a merged CuConfigRepresentation.
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
                if current_dir.is_empty() {
                    include.path
                } else {
                    format!("{}/{}", current_dir, include.path)
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
                        include_path, e.code, e.position
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
                e.code, e.position
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

pub fn read_configuration_str(
    config_content: String,
    file_path: Option<&str>,
) -> CuResult<CuConfig> {
    // Parse the configuration string
    let representation = parse_config_string(&config_content)?;

    // Process includes and generate a merged configuration if a file path is provided
    let processed_representation = if let Some(path) = file_path {
        process_includes(path, representation, &mut Vec::new())?
    } else {
        representation
    };

    // Convert the representation to a CuConfig and validate
    config_representation_to_config(processed_representation)
}

// tests
#[cfg(test)]
mod tests {
    use super::*;

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
        assert_eq!(graph.0.node_count(), deserialized_graph.0.node_count());
        assert_eq!(graph.0.edge_count(), deserialized_graph.0.edge_count());
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
    #[should_panic(expected = "Syntax Error in config: Expected opening `[` at position 1:10")]
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
        assert!(graph.0.node_count() == 0);
        let graph = config.graphs.get_graph(Some("autonomous")).unwrap();
        assert!(graph.0.node_count() == 0);
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
        assert_eq!(m1_graph.0.edge_count(), 1);
        assert_eq!(m1_graph.0.node_count(), 2);
        let index = EdgeIndex::new(0);
        let cnx = m1_graph.0.edge_weight(index).unwrap();

        assert_eq!(cnx.src, "src1");
        assert_eq!(cnx.dst, "sink");
        assert_eq!(cnx.msg, "u32");
        assert_eq!(cnx.missions, Some(vec!["m1".to_string()]));

        let m2_graph = config.graphs.get_graph(Some("m2")).unwrap();
        assert_eq!(m2_graph.0.edge_count(), 1);
        assert_eq!(m2_graph.0.node_count(), 2);
        let index = EdgeIndex::new(0);
        let cnx = m2_graph.0.edge_weight(index).unwrap();
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
        assert_eq!(m1_graph.0.edge_count(), 1);
        assert_eq!(m1_graph.0.node_count(), 2);
        let index = EdgeIndex::new(0);
        let cnx = m1_graph.0.edge_weight(index).unwrap();
        assert_eq!(cnx.src, "src1");
        assert_eq!(cnx.dst, "sink");
        assert_eq!(cnx.msg, "u32");
        assert_eq!(cnx.missions, Some(vec!["m1".to_string()]));
    }
}
