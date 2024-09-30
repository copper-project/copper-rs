//! This module defines the configuration of the copper runtime.
//! The configuration is a directed graph where nodes are tasks and edges are connections between tasks.
//! The configuration is serialized in the RON format.
//! The configuration is used to generate the runtime code at compile time.

use crate::{CuError, CuResult};
use petgraph::stable_graph::{EdgeIndex, StableDiGraph};
use petgraph::visit::EdgeRef;
use ron::extensions::Extensions;
use ron::value::Value as RonValue;
use ron::Options;
use serde::{Deserialize, Deserializer, Serialize, Serializer};
use std::collections::HashMap;
use std::fmt;
use std::fmt::Display;
use std::fs::read_to_string;

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
        write!(f, "{{")?;
        for (key, value) in self.0.iter() {
            if !first {
                write!(f, ", ")?;
            }
            write!(f, "{}: {}", key, value)?;
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
        self.0.get(key).map(|v| T::from(v.clone()))
    }

    #[allow(dead_code)]
    pub fn set<T: Into<Value>>(&mut self, key: &str, value: T) {
        self.0.insert(key.to_string(), value.into());
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

impl From<i32> for Value {
    fn from(value: i32) -> Self {
        Value(RonValue::Number(value.into()))
    }
}

impl From<u32> for Value {
    fn from(value: u32) -> Self {
        Value(RonValue::Number((value as u64).into()))
    }
}

impl From<u16> for Value {
    fn from(value: u16) -> Self {
        Value(RonValue::Number((value as u64).into()))
    }
}

impl From<u8> for Value {
    fn from(value: u8) -> Self {
        Value(RonValue::Number((value as u64).into()))
    }
}

impl From<f64> for Value {
    fn from(value: f64) -> Self {
        Value(RonValue::Number(value.into()))
    }
}

impl From<Value> for u8 {
    fn from(value: Value) -> Self {
        if let RonValue::Number(num) = value.0 {
            if let Some(i) = num.as_i64() {
                i as u8
            } else {
                panic!("Expected an integer value but got {:?}", value)
            }
        } else {
            panic!("Expected a Number variant but got {:?}", value)
        }
    }
}

impl From<Value> for u32 {
    fn from(value: Value) -> Self {
        if let RonValue::Number(num) = value.0 {
            if let Some(i) = num.as_i64() {
                i as u32
            } else {
                panic!("Expected an integer value but got {:?}", value)
            }
        } else {
            panic!("Expected a Number variant but got {:?}", value)
        }
    }
}

impl From<Value> for i32 {
    fn from(value: Value) -> Self {
        if let RonValue::Number(num) = value.0 {
            if let Some(i) = num.as_i64() {
                i as i32
            } else {
                panic!("Expected an integer value but got {:?}", value)
            }
        } else {
            panic!("Expected a Number variant but got {:?}", value)
        }
    }
}

impl From<Value> for f64 {
    fn from(value: Value) -> Self {
        if let RonValue::Number(num) = value.0 {
            if let Some(f) = num.as_f64() {
                f
            } else {
                panic!("Expected a float value but got {:?}", value)
            }
        } else {
            panic!("Expected a Number variant but got {:?}", value)
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
        if let RonValue::String(s) = value.0 {
            s
        } else {
            panic!("Expected a String variant")
        }
    }
}

impl Display for Value {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match &self.0 {
            RonValue::Number(n) => write!(f, "{}", n.as_i64().unwrap()),
            RonValue::String(s) => write!(f, "{}", s),
            RonValue::Bool(b) => write!(f, "{}", b),
            RonValue::Map(m) => write!(f, "{:?}", m),
            RonValue::Char(c) => write!(f, "{:?}", c),
            RonValue::Unit => write!(f, "unit"),
            RonValue::Option(o) => write!(f, "{:?}", o),
            RonValue::Seq(s) => write!(f, "{:?}", s),
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
}

impl Node {
    #[allow(dead_code)]
    pub fn new(id: &str, ptype: &str) -> Self {
        Node {
            id: id.to_string(),
            type_: Some(ptype.to_string()),
            // base_period_ns: None,
            config: None,
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
        let v = pc.0.get(key)?;
        Some(T::from(v.clone()))
    }

    #[allow(dead_code)]
    pub fn set_param<T: Into<Value>>(&mut self, key: &str, value: T) {
        if self.config.is_none() {
            self.config = Some(ComponentConfig(HashMap::new()));
        }
        self.config
            .as_mut()
            .unwrap()
            .0
            .insert(key.to_string(), value.into());
    }
}

/// This represents a connection between 2 tasks (nodes) in the configuration graph.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct Cnx {
    /// Source node id.
    src: String,

    // Destination node id.
    dst: String,

    /// Message type exchanged betwee src and dst.
    pub msg: String,

    /// Tells Copper to batch messages before sending the buffer to the next node.
    /// If None, Copper will just send 1 message at a time.
    /// If Some(n), Copper will batch n messages before sending the buffer.
    pub batch: Option<u32>,

    /// Tells Copper if it needs to log the messages.
    pub store: Option<bool>,
}

/// CuConfig is the programmatic representation of the configuration graph.
/// It is a directed graph where nodes are tasks and edges are connections between tasks.
#[derive(Debug)]
pub struct CuConfig {
    // This is not what is directly serialized, see the custom serialization below.
    pub graph: StableDiGraph<Node, Cnx, NodeId>,
    monitor: Option<MonitorConfig>,
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

/// The config is a list of tasks and their connections.
#[derive(Serialize, Deserialize, Default)]
struct CuConfigRepresentation {
    tasks: Vec<Node>,
    cnx: Vec<Cnx>,
    monitor: Option<MonitorConfig>,
}

impl<'de> Deserialize<'de> for CuConfig {
    /// This is a custom serialization to make this implementation independent of petgraph.
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let representation = CuConfigRepresentation::deserialize(deserializer)
            .expect("Failed to deserialize config");

        let mut cuconfig = CuConfig::default();
        for task in representation.tasks {
            cuconfig.add_node(task);
        }

        for c in representation.cnx {
            let src = cuconfig
                .graph
                .node_indices()
                .find(|i| cuconfig.graph[*i].id == c.src)
                .expect("Source node not found");
            let dst = cuconfig
                .graph
                .node_indices()
                .find(|i| cuconfig.graph[*i].id == c.dst)
                .expect("Destination node not found");
            cuconfig.connect_ext(
                src.index() as NodeId,
                dst.index() as NodeId,
                &c.msg,
                c.batch,
                c.store,
            );
        }
        cuconfig.monitor = representation.monitor;
        Ok(cuconfig)
    }
}

impl Serialize for CuConfig {
    /// This is a custom serialization to make this implementation independent of petgraph.
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let tasks: Vec<Node> = self
            .graph
            .node_indices()
            .map(|idx| self.graph[idx].clone())
            .collect();

        let cnx: Vec<Cnx> = self
            .graph
            .edge_indices()
            .map(|edge| self.graph[edge].clone())
            .collect();

        CuConfigRepresentation {
            tasks,
            cnx,
            monitor: self.monitor.clone(),
        }
        .serialize(serializer)
    }
}

impl Default for CuConfig {
    fn default() -> Self {
        CuConfig {
            graph: StableDiGraph::new(),
            monitor: None,
        }
    }
}

/// The implementation has a lot of conveinence methods to manipulate
/// the configuration to give some flexibility into programmatically creating the configuration.
impl CuConfig {
    /// Add a new node to the configuration graph.
    pub fn add_node(&mut self, node: Node) -> NodeId {
        self.graph.add_node(node).index() as NodeId
    }

    /// Get the node with the given id.
    #[allow(dead_code)] // Used in proc macro
    pub fn get_node(&self, node_id: NodeId) -> Option<&Node> {
        self.graph.node_weight(node_id.into())
    }

    /// Get the list of edges that are connected to the given node as a source.
    pub fn get_src_edges(&self, node_id: NodeId) -> Vec<usize> {
        self.graph
            .edges_directed(node_id.into(), petgraph::Direction::Outgoing)
            .map(|edge| edge.id().index())
            .collect()
    }

    /// Get the list of edges that are connected to the given node as a destination.
    pub fn get_dst_edges(&self, node_id: NodeId) -> Vec<usize> {
        self.graph
            .edges_directed(node_id.into(), petgraph::Direction::Incoming)
            .map(|edge| edge.id().index())
            .collect()
    }

    #[allow(dead_code)]
    pub fn get_edge_weight(&self, index: usize) -> Option<Cnx> {
        self.graph.edge_weight(EdgeIndex::new(index)).cloned()
    }

    /// Convenience method to get all nodes in the configuration graph.
    #[allow(dead_code)]
    pub fn get_all_nodes(&self) -> Vec<&Node> {
        self.graph
            .node_indices()
            .map(|node| &self.graph[node])
            .collect()
    }

    /// Adds an edge between two nodes/tasks in the configuration graph.
    /// msg_type is the type of message exchanged between the two nodes/tasks.
    /// batch is the number of messages to batch before sending the buffer.
    /// store tells Copper if it needs to log the messages.
    pub fn connect_ext(
        &mut self,
        source: NodeId,
        target: NodeId,
        msg_type: &str,
        batch: Option<u32>,
        store: Option<bool>,
    ) {
        self.graph.add_edge(
            source.into(),
            target.into(),
            Cnx {
                src: self
                    .get_node(source)
                    .expect("Source node not found")
                    .id
                    .clone(),
                dst: self
                    .get_node(target)
                    .expect("Target node not found")
                    .id
                    .clone(),
                msg: msg_type.to_string(),
                batch,
                store,
            },
        );
    }

    /// Adds an edge between two nodes/tasks in the configuration graph.
    /// msg_type is the type of message exchanged between the two nodes/tasks.
    #[allow(dead_code)]
    pub fn connect(&mut self, source: NodeId, target: NodeId, msg_type: &str) {
        self.connect_ext(source, target, msg_type, None, None);
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

    pub fn deserialize_ron(ron: &str) -> Self {
        Self::get_options()
            .from_str(ron)
            .expect("Syntax Error in config")
    }

    /// Render the configuration graph in the dot format.
    pub fn render(&self, output: &mut dyn std::io::Write) {
        writeln!(output, "digraph G {{").unwrap();

        for index in self.graph.node_indices() {
            let node = &self.graph[index];
            let config_str = match &node.config {
                Some(config) => {
                    let config_str = config
                        .0
                        .iter()
                        .map(|(k, v)| format!("<B>{}</B> = {}<BR ALIGN=\"LEFT\"/>", k, v))
                        .collect::<Vec<String>>()
                        .join("\n");
                    format!("<BR/>____________<BR ALIGN=\"LEFT\"/>{}", config_str)
                }
                None => String::new(),
            };
            writeln!(output, "{} [", index.index()).unwrap();
            writeln!(output, "shape=box,").unwrap();
            writeln!(output, "style=\"rounded, filled\",").unwrap();
            writeln!(output, "fontname=\"Noto Sans\"").unwrap();

            let is_src = self.get_dst_edges(index.index() as NodeId).is_empty();
            let is_sink = self.get_src_edges(index.index() as NodeId).is_empty();
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
                "label=< <FONT COLOR=\"red\"><B>{}</B></FONT><BR ALIGN=\"LEFT\"/><BR ALIGN=\"RIGHT\"/><FONT COLOR=\"dimgray\">{}</FONT><BR ALIGN=\"LEFT\"/>{} >",
                node.id,
                node.get_type(),
                config_str
            )
                .unwrap();

            writeln!(output, "];").unwrap();
        }
        for edge in self.graph.edge_indices() {
            let (src, dst) = self.graph.edge_endpoints(edge).unwrap();

            let cnx = &self.graph[edge];
            writeln!(
                output,
                "{} -> {} [label=< <B><FONT COLOR=\"gray\">{}/{}/{}</FONT></B> >];",
                src.index(),
                dst.index(),
                cnx.msg,
                cnx.batch.unwrap_or(1),
                cnx.store.unwrap_or(false)
            )
            .unwrap();
        }
        writeln!(output, "}}").unwrap();
    }

    #[allow(dead_code)]
    pub fn get_all_instances_configs(&self) -> Vec<Option<&ComponentConfig>> {
        self.get_all_nodes()
            .iter()
            .map(|node_config| node_config.get_instance_config())
            .collect()
    }

    #[allow(dead_code)]
    pub fn get_monitor_config(&self) -> Option<&MonitorConfig> {
        self.monitor.as_ref()
    }
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
    Ok(CuConfig::deserialize_ron(&config_content))
}

// tests
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_plain_serialize() {
        let mut config = CuConfig::default();
        let n1 = config.add_node(Node::new("test1", "package::Plugin1"));
        let n2 = config.add_node(Node::new("test2", "package::Plugin2"));
        config.connect(n1, n2, "msgpkg::MsgType");
        let serialized = config.serialize_ron();
        println!("{}", serialized);
        let deserialized = CuConfig::deserialize_ron(&serialized);
        assert_eq!(config.graph.node_count(), deserialized.graph.node_count());
        assert_eq!(config.graph.edge_count(), deserialized.graph.edge_count());
    }

    #[test]
    fn test_serialize_with_params() {
        let mut config = CuConfig::default();
        let mut camera = Node::new("copper-camera", "camerapkg::Camera");
        camera.set_param::<Value>("resolution-height", 1080.into());
        config.add_node(camera);
        let serialized = config.serialize_ron();
        println!("{}", serialized);
        let deserialized = CuConfig::deserialize_ron(&serialized);
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
    fn test_monitor() {
        let txt = r#"( tasks: [], cnx: [], monitor: (type: "ExampleMonitor", ) ) "#;
        let config = CuConfig::deserialize_ron(txt);
        assert_eq!(config.monitor.as_ref().unwrap().type_, "ExampleMonitor");

        let txt =
            r#"( tasks: [], cnx: [], monitor: (type: "ExampleMonitor", config: { "toto": 4, } )) "#;
        let config = CuConfig::deserialize_ron(txt);
        assert_eq!(
            config.monitor.as_ref().unwrap().config.as_ref().unwrap().0["toto"],
            4.into()
        );
    }
}
