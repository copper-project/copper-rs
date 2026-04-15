//! This module defines the configuration of the copper runtime.
//! The configuration is a directed graph where nodes are tasks and edges are connections between tasks.
//! The configuration is serialized in the RON format.
//! The configuration is used to generate the runtime code at compile time.
#[cfg(not(feature = "std"))]
extern crate alloc;

use ConfigGraphs::{Missions, Simple};
use core::any::type_name;
use core::fmt;
use core::fmt::Display;
use cu29_traits::{CuError, CuResult};
use cu29_value::Value as CuValue;
use hashbrown::HashMap;
pub use petgraph::Direction::Incoming;
pub use petgraph::Direction::Outgoing;
use petgraph::stable_graph::{EdgeIndex, NodeIndex, StableDiGraph};
#[cfg(feature = "std")]
use petgraph::visit::IntoEdgeReferences;
use petgraph::visit::{Bfs, EdgeRef};
use ron::extensions::Extensions;
use ron::value::Value as RonValue;
use ron::{Number, Options};
use serde::de::DeserializeOwned;
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

/// NodeId is the unique identifier of a node in the configuration graph for petgraph
/// and the code generation.
pub type NodeId = u32;
pub const DEFAULT_MISSION_ID: &str = "default";

/// This is the configuration of a component (like a task config or a monitoring config):w
/// It is a map of key-value pairs.
/// It is given to the new method of the task implementation.
#[derive(Serialize, Deserialize, Debug, Clone, Default)]
pub struct ComponentConfig(pub HashMap<String, Value>);

/// Mapping between resource binding names and bundle-scoped resource ids.
#[allow(dead_code)]
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
    pub fn get<T>(&self, key: &str) -> Result<Option<T>, ConfigError>
    where
        T: for<'a> TryFrom<&'a Value, Error = ConfigError>,
    {
        let ComponentConfig(config) = self;
        match config.get(key) {
            Some(value) => T::try_from(value).map(Some),
            None => Ok(None),
        }
    }

    #[allow(dead_code)]
    /// Retrieve a structured config value by deserializing it with cu29-value.
    ///
    /// Example RON:
    /// `{ "calibration": { "matrix": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], "enabled": true } }`
    ///
    /// ```rust,ignore
    /// #[derive(serde::Deserialize)]
    /// struct CalibrationCfg {
    ///     matrix: [[f32; 3]; 3],
    ///     enabled: bool,
    /// }
    /// let cfg: CalibrationCfg = config.get_value("calibration")?.unwrap();
    /// ```
    pub fn get_value<T>(&self, key: &str) -> Result<Option<T>, ConfigError>
    where
        T: DeserializeOwned,
    {
        let ComponentConfig(config) = self;
        let Some(value) = config.get(key) else {
            return Ok(None);
        };
        let cu_value = ron_value_to_cu_value(&value.0).map_err(|err| err.with_key(key))?;
        cu_value
            .deserialize_into::<T>()
            .map(Some)
            .map_err(|err| ConfigError {
                message: format!(
                    "Config key '{key}' failed to deserialize as {}: {err}",
                    type_name::<T>()
                ),
            })
    }

    #[allow(dead_code)]
    pub fn deserialize_into<T>(&self) -> Result<T, ConfigError>
    where
        T: DeserializeOwned,
    {
        let mut map = BTreeMap::new();
        for (key, value) in &self.0 {
            let mapped_value = ron_value_to_cu_value(&value.0).map_err(|err| err.with_key(key))?;
            map.insert(CuValue::String(key.clone()), mapped_value);
        }

        CuValue::Map(map)
            .deserialize_into::<T>()
            .map_err(|err| ConfigError {
                message: format!(
                    "Config failed to deserialize as {}: {err}",
                    type_name::<T>()
                ),
            })
    }

    #[allow(dead_code)]
    pub fn set<T: Into<Value>>(&mut self, key: &str, value: T) {
        let ComponentConfig(config) = self;
        config.insert(key.to_string(), value.into());
    }

    #[allow(dead_code)]
    pub fn merge_from(&mut self, other: &ComponentConfig) {
        let ComponentConfig(config) = self;
        for (key, value) in &other.0 {
            config.insert(key.clone(), value.clone());
        }
    }
}

fn ron_value_to_cu_value(value: &RonValue) -> Result<CuValue, ConfigError> {
    match value {
        RonValue::Bool(v) => Ok(CuValue::Bool(*v)),
        RonValue::Char(v) => Ok(CuValue::Char(*v)),
        RonValue::String(v) => Ok(CuValue::String(v.clone())),
        RonValue::Bytes(v) => Ok(CuValue::Bytes(v.clone())),
        RonValue::Unit => Ok(CuValue::Unit),
        RonValue::Option(v) => {
            let mapped = match v {
                Some(inner) => Some(Box::new(ron_value_to_cu_value(inner)?)),
                None => None,
            };
            Ok(CuValue::Option(mapped))
        }
        RonValue::Seq(seq) => {
            let mut mapped = Vec::with_capacity(seq.len());
            for item in seq {
                mapped.push(ron_value_to_cu_value(item)?);
            }
            Ok(CuValue::Seq(mapped))
        }
        RonValue::Map(map) => {
            let mut mapped = BTreeMap::new();
            for (key, value) in map.iter() {
                let mapped_key = ron_value_to_cu_value(key)?;
                let mapped_value = ron_value_to_cu_value(value)?;
                mapped.insert(mapped_key, mapped_value);
            }
            Ok(CuValue::Map(mapped))
        }
        RonValue::Number(num) => match num {
            Number::I8(v) => Ok(CuValue::I8(*v)),
            Number::I16(v) => Ok(CuValue::I16(*v)),
            Number::I32(v) => Ok(CuValue::I32(*v)),
            Number::I64(v) => Ok(CuValue::I64(*v)),
            Number::U8(v) => Ok(CuValue::U8(*v)),
            Number::U16(v) => Ok(CuValue::U16(*v)),
            Number::U32(v) => Ok(CuValue::U32(*v)),
            Number::U64(v) => Ok(CuValue::U64(*v)),
            Number::F32(v) => Ok(CuValue::F32(v.0)),
            Number::F64(v) => Ok(CuValue::F64(v.0)),
            _ => Err(ConfigError {
                message: "Unsupported RON number variant".to_string(),
            }),
        },
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

#[derive(Debug, Clone, PartialEq)]
pub struct ConfigError {
    message: String,
}

impl ConfigError {
    fn type_mismatch(expected: &'static str, value: &Value) -> Self {
        ConfigError {
            message: format!("Expected {expected} but got {value:?}"),
        }
    }

    fn with_key(self, key: &str) -> Self {
        ConfigError {
            message: format!("Config key '{key}': {}", self.message),
        }
    }
}

impl Display for ConfigError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.message)
    }
}

#[cfg(feature = "std")]
impl std::error::Error for ConfigError {}

#[cfg(not(feature = "std"))]
impl core::error::Error for ConfigError {}

impl From<ConfigError> for CuError {
    fn from(err: ConfigError) -> Self {
        CuError::from(err.to_string())
    }
}

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

impl TryFrom<&Value> for bool {
    type Error = ConfigError;

    fn try_from(value: &Value) -> Result<Self, Self::Error> {
        if let Value(RonValue::Bool(v)) = value {
            Ok(*v)
        } else {
            Err(ConfigError::type_mismatch("bool", value))
        }
    }
}

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
                            _ => {
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

macro_rules! impl_try_from_value_for_int {
    ($($target:ty),* $(,)?) => {
        $(
            impl TryFrom<&Value> for $target {
                type Error = ConfigError;

                fn try_from(value: &Value) -> Result<Self, Self::Error> {
                    if let Value(RonValue::Number(num)) = value {
                        match num {
                            Number::I8(n) => Ok(*n as $target),
                            Number::I16(n) => Ok(*n as $target),
                            Number::I32(n) => Ok(*n as $target),
                            Number::I64(n) => Ok(*n as $target),
                            Number::U8(n) => Ok(*n as $target),
                            Number::U16(n) => Ok(*n as $target),
                            Number::U32(n) => Ok(*n as $target),
                            Number::U64(n) => Ok(*n as $target),
                            Number::F32(_) | Number::F64(_) => {
                                Err(ConfigError::type_mismatch("integer", value))
                            }
                            _ => {
                                Err(ConfigError::type_mismatch("integer", value))
                            }
                        }
                    } else {
                        Err(ConfigError::type_mismatch("integer", value))
                    }
                }
            }
        )*
    };
}

impl_try_from_value_for_int!(u8, i8, u16, i16, u32, i32, u64, i64);

impl TryFrom<&Value> for f64 {
    type Error = ConfigError;

    fn try_from(value: &Value) -> Result<Self, Self::Error> {
        if let Value(RonValue::Number(num)) = value {
            let number = match num {
                Number::I8(n) => *n as f64,
                Number::I16(n) => *n as f64,
                Number::I32(n) => *n as f64,
                Number::I64(n) => *n as f64,
                Number::U8(n) => *n as f64,
                Number::U16(n) => *n as f64,
                Number::U32(n) => *n as f64,
                Number::U64(n) => *n as f64,
                Number::F32(n) => n.0 as f64,
                Number::F64(n) => n.0,
                _ => {
                    return Err(ConfigError::type_mismatch("number", value));
                }
            };
            Ok(number)
        } else {
            Err(ConfigError::type_mismatch("number", value))
        }
    }
}

impl From<Value> for f64 {
    fn from(value: Value) -> Self {
        if let Value(RonValue::Number(num)) = value {
            num.into_f64()
        } else {
            panic!("Expected a Number variant but got {value:?}")
        }
    }
}

//Basically just a copy of the From<Value> for f64.
impl TryFrom<&Value> for f32 {
    type Error = ConfigError;

    fn try_from(value: &Value) -> Result<Self, Self::Error> {
        if let Value(RonValue::Number(num)) = value {
            let number = match num {
                Number::I8(n) => *n as f32,
                Number::I16(n) => *n as f32,
                Number::I32(n) => *n as f32,
                Number::I64(n) => *n as f32,
                Number::U8(n) => *n as f32,
                Number::U16(n) => *n as f32,
                Number::U32(n) => *n as f32,
                Number::U64(n) => *n as f32,
                Number::F32(n) => n.0,
                Number::F64(n) => n.0 as f32,
                _ => {
                    return Err(ConfigError::type_mismatch("number", value));
                }
            };
            Ok(number)
        } else {
            Err(ConfigError::type_mismatch("number", value))
        }
    }
}

impl From<Value> for f32 {
    fn from(value: Value) -> Self {
        if let Value(RonValue::Number(num)) = value {
            num.into_f64() as f32
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

impl TryFrom<&Value> for String {
    type Error = ConfigError;

    fn try_from(value: &Value) -> Result<Self, Self::Error> {
        if let Value(RonValue::String(s)) = value {
            Ok(s.clone())
        } else {
            Err(ConfigError::type_mismatch("string", value))
        }
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
    #[serde(default = "default_as_true")]
    enabled: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    codec: Option<String>,
    #[serde(default, skip_serializing_if = "HashMap::is_empty")]
    codecs: HashMap<String, String>,
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
}

impl Default for NodeLogging {
    fn default() -> Self {
        Self {
            enabled: true,
            codec: None,
            codecs: HashMap::new(),
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

    /// Resources requested by the task.
    #[serde(skip_serializing_if = "Option::is_none")]
    resources: Option<HashMap<String, String>>,

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
            config: None,
            resources: None,
            missions: None,
            background: None,
            run_in_sim: None,
            logging: None,
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
    pub fn set_resources<I>(&mut self, resources: Option<I>)
    where
        I: IntoIterator<Item = (String, String)>,
    {
        self.resources = resources.map(|iter| iter.into_iter().collect());
    }

    #[allow(dead_code)]
    pub fn is_background(&self) -> bool {
        self.background.unwrap_or(false)
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
    pub id: String,
    #[serde(rename = "provider")]
    pub provider: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub config: Option<ComponentConfig>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub missions: Option<Vec<String>>,
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
    /// Monitoring configuration list.
    pub monitors: Vec<MonitorConfig>,
    /// Optional logging configuration
    pub logging: Option<LoggingConfig>,
    /// Optional runtime configuration
    pub runtime: Option<RuntimeConfig>,
    /// Declarative resource bundle definitions
    pub resources: Vec<ResourceBundleConfig>,
    /// Declarative bridge definitions that are yet to be expanded into the graph
    pub bridges: Vec<BridgeConfig>,
    /// Graph structure - either a single graph or multiple mission-specific graphs
    pub graphs: ConfigGraphs,
}

impl CuConfig {
    #[cfg(feature = "std")]
    fn ensure_threadpool_bundle(&mut self) {
        if !self.has_background_tasks() {
            return;
        }
        if self
            .resources
            .iter()
            .any(|bundle| bundle.id == "threadpool")
        {
            return;
        }

        let mut config = ComponentConfig::default();
        config.set("threads", 2u64);
        self.resources.push(ResourceBundleConfig {
            id: "threadpool".to_string(),
            provider: "cu29::resource::ThreadPoolBundle".to_string(),
            config: Some(config),
            missions: None,
        });
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

/// Includes are used to include other configuration files.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct IncludesConfig {
    pub path: String,
    pub params: HashMap<String, Value>,
    pub missions: Option<Vec<String>>,
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
    tasks: Option<Vec<Node>>,
    resources: Option<Vec<ResourceBundleConfig>>,
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
    cuconfig.logging = representation.logging.clone();
    cuconfig.runtime = representation.runtime.clone();
    cuconfig.resources = representation.resources.clone().unwrap_or_default();
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
                    tasks: Some(tasks),
                    bridges: bridges.clone(),
                    cnx: Some(cnx),
                    monitors: monitors.clone(),
                    logging: self.logging.clone(),
                    runtime: self.runtime.clone(),
                    resources: resources.clone(),
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
                    tasks: Some(tasks),
                    resources: resources.clone(),
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
            graphs: Simple(CuGraph(StableDiGraph::new())),
            monitors: Vec::new(),
            logging: None,
            runtime: None,
            resources: Vec::new(),
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
            monitors: Vec::new(),
            logging: None,
            runtime: None,
            resources: Vec::new(),
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
            return runtime.validate();
        }
        Ok(())
    }
}

#[cfg(feature = "std")]
#[derive(Default)]
pub(crate) struct PortLookup {
    pub inputs: HashMap<String, String>,
    pub outputs: HashMap<String, String>,
    pub default_input: Option<String>,
    pub default_output: Option<String>,
}

#[cfg(feature = "std")]
#[derive(Clone)]
pub(crate) struct RenderNode {
    pub id: String,
    pub type_name: String,
    pub flavor: Flavor,
    pub inputs: Vec<String>,
    pub outputs: Vec<String>,
}

#[cfg(feature = "std")]
#[derive(Clone)]
pub(crate) struct RenderConnection {
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
pub(crate) struct RenderTopology {
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
pub(crate) fn build_render_topology(graph: &CuGraph, bridges: &[BridgeConfig]) -> RenderTopology {
    let mut bridge_lookup = HashMap::new();
    for bridge in bridges {
        bridge_lookup.insert(bridge.id.as_str(), bridge);
    }

    let mut nodes: Vec<RenderNode> = Vec::new();
    let mut node_lookup: HashMap<String, usize> = HashMap::new();
    for (_, node) in graph.get_all_nodes() {
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
    let mut output_edges: Vec<_> = graph.0.edge_references().collect();
    output_edges.sort_by_key(|edge| edge.id().index());
    for edge in output_edges {
        let cnx = edge.weight();
        if let Some(&idx) = node_lookup.get(&cnx.src)
            && nodes[idx].flavor == Flavor::Task
            && cnx.src_channel.is_none()
        {
            let port_map = &mut output_port_lookup[idx];
            if !port_map.contains_key(&cnx.msg) {
                let label = format!("out{}: {}", port_map.len(), cnx.msg);
                port_map.insert(cnx.msg.clone(), label.clone());
                nodes[idx].outputs.push(label);
            }
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
) -> CuResult<CuConfigRepresentation> {
    // Note: Circular dependency detection removed
    processed_files.push(file_path.to_string());

    let mut result = base_representation;

    if let Some(includes) = result.includes.take() {
        for include in includes {
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
    let graph = config.graphs.get_default_mission_graph().map_err(|e| {
        CuError::from(format!(
            "Multi-Copper subsystem configs currently require exactly one local graph: {e}"
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
        let config = read_configuration(&resolved_config_path).map_err(|e| {
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
    #[allow(unused_mut)]
    let mut cuconfig = CuConfig::deserialize_impl(representation)
        .map_err(|e| CuError::from(format!("Error deserializing configuration: {e}")))?;

    #[cfg(feature = "std")]
    cuconfig.ensure_threadpool_bundle();

    cuconfig.validate_logging_config()?;
    cuconfig.validate_runtime_config()?;

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

/// Read a strict multi-Copper umbrella configuration from a file.
#[cfg(feature = "std")]
#[allow(dead_code)]
pub fn read_multi_configuration(config_filename: &str) -> CuResult<MultiCopperConfig> {
    let config_content = read_to_string(config_filename).map_err(|e| {
        CuError::from(format!(
            "Failed to read multi-Copper configuration file: {:?}",
            &config_filename
        ))
        .add_cause(e.to_string().as_str())
    })?;
    read_multi_configuration_str(config_content, Some(config_filename))
}

/// Read a strict multi-Copper umbrella configuration from a string.
#[cfg(feature = "std")]
#[allow(dead_code)]
pub fn read_multi_configuration_str(
    config_content: String,
    file_path: Option<&str>,
) -> CuResult<MultiCopperConfig> {
    let representation = parse_multi_config_string(&config_content)?;
    validate_multi_config_representation(representation, file_path)
}

// tests
#[cfg(test)]
mod tests {
    use super::*;
    #[cfg(not(feature = "std"))]
    use alloc::vec;
    use serde::Deserialize;
    #[cfg(feature = "std")]
    use std::path::{Path, PathBuf};

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
        let serialized = config.serialize_ron().unwrap();
        let deserialized = CuConfig::deserialize_ron(&serialized).unwrap();
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
        let serialized = config.serialize_ron().unwrap();
        let config = CuConfig::deserialize_ron(&serialized).unwrap();
        let deserialized = config.get_graph(None).unwrap();
        let resolution = deserialized
            .get_node(0)
            .unwrap()
            .get_param::<i32>("resolution-height")
            .expect("resolution-height lookup failed");
        assert_eq!(resolution, Some(1080));
    }

    #[derive(Debug, Deserialize, PartialEq)]
    struct InnerSettings {
        threshold: u32,
        flags: Option<bool>,
    }

    #[derive(Debug, Deserialize, PartialEq)]
    struct SettingsConfig {
        gain: f32,
        matrix: [[f32; 3]; 3],
        inner: InnerSettings,
        tags: Vec<String>,
    }

    #[test]
    fn test_component_config_get_value_structured() {
        let txt = r#"
            (
                tasks: [
                    (
                        id: "task",
                        type: "pkg::Task",
                        config: {
                            "settings": {
                                "gain": 1.5,
                                "matrix": [
                                    [1.0, 0.0, 0.0],
                                    [0.0, 1.0, 0.0],
                                    [0.0, 0.0, 1.0],
                                ],
                                "inner": { "threshold": 42, "flags": Some(true) },
                                "tags": ["alpha", "beta"],
                            },
                        },
                    ),
                ],
                cnx: [],
            )
        "#;
        let config = CuConfig::deserialize_ron(txt).unwrap();
        let graph = config.graphs.get_graph(None).unwrap();
        let node = graph.get_node(0).unwrap();
        let component = node.get_instance_config().expect("missing config");
        let settings = component
            .get_value::<SettingsConfig>("settings")
            .expect("settings lookup failed")
            .expect("missing settings");
        let expected = SettingsConfig {
            gain: 1.5,
            matrix: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            inner: InnerSettings {
                threshold: 42,
                flags: Some(true),
            },
            tags: vec!["alpha".to_string(), "beta".to_string()],
        };
        assert_eq!(settings, expected);
    }

    #[test]
    fn test_component_config_get_value_scalar_compatibility() {
        let txt = r#"
            (
                tasks: [
                    (id: "task", type: "pkg::Task", config: { "scalar": 7 }),
                ],
                cnx: [],
            )
        "#;
        let config = CuConfig::deserialize_ron(txt).unwrap();
        let graph = config.graphs.get_graph(None).unwrap();
        let node = graph.get_node(0).unwrap();
        let component = node.get_instance_config().expect("missing config");
        let scalar = component
            .get::<u32>("scalar")
            .expect("scalar lookup failed");
        assert_eq!(scalar, Some(7));
    }

    #[test]
    fn test_component_config_get_value_mixed_usage() {
        let txt = r#"
            (
                tasks: [
                    (
                        id: "task",
                        type: "pkg::Task",
                        config: {
                            "scalar": 12,
                            "settings": {
                                "gain": 2.5,
                                "matrix": [
                                    [1.0, 2.0, 3.0],
                                    [4.0, 5.0, 6.0],
                                    [7.0, 8.0, 9.0],
                                ],
                                "inner": { "threshold": 7, "flags": None },
                                "tags": ["gamma"],
                            },
                        },
                    ),
                ],
                cnx: [],
            )
        "#;
        let config = CuConfig::deserialize_ron(txt).unwrap();
        let graph = config.graphs.get_graph(None).unwrap();
        let node = graph.get_node(0).unwrap();
        let component = node.get_instance_config().expect("missing config");
        let scalar = component
            .get::<u32>("scalar")
            .expect("scalar lookup failed");
        let settings = component
            .get_value::<SettingsConfig>("settings")
            .expect("settings lookup failed");
        assert_eq!(scalar, Some(12));
        assert!(settings.is_some());
    }

    #[test]
    fn test_component_config_get_value_error_includes_key() {
        let txt = r#"
            (
                tasks: [
                    (
                        id: "task",
                        type: "pkg::Task",
                        config: { "settings": { "gain": 1.0 } },
                    ),
                ],
                cnx: [],
            )
        "#;
        let config = CuConfig::deserialize_ron(txt).unwrap();
        let graph = config.graphs.get_graph(None).unwrap();
        let node = graph.get_node(0).unwrap();
        let component = node.get_instance_config().expect("missing config");
        let err = component
            .get_value::<u32>("settings")
            .expect_err("expected type mismatch");
        assert!(err.to_string().contains("settings"));
    }

    #[test]
    fn test_deserialization_error() {
        // Task needs to be an array, but provided tuple wrongfully
        let txt = r#"( tasks: (), cnx: [], monitors: [(type: "ExampleMonitor", )] ) "#;
        let err = CuConfig::deserialize_ron(txt).expect_err("expected deserialization error");
        assert!(
            err.to_string()
                .contains("Syntax Error in config: Expected opening `[` at position 1:9-1:10")
        );
    }
    #[test]
    fn test_missions() {
        let txt = r#"( missions: [ (id: "data_collection"), (id: "autonomous")])"#;
        let config = CuConfig::deserialize_ron(txt).unwrap();
        let graph = config.graphs.get_graph(Some("data_collection")).unwrap();
        assert!(graph.node_count() == 0);
        let graph = config.graphs.get_graph(Some("autonomous")).unwrap();
        assert!(graph.node_count() == 0);
    }

    #[test]
    fn test_monitor_plural_syntax() {
        let txt = r#"( tasks: [], cnx: [], monitors: [(type: "ExampleMonitor", )] ) "#;
        let config = CuConfig::deserialize_ron(txt).unwrap();
        assert_eq!(config.get_monitor_config().unwrap().type_, "ExampleMonitor");

        let txt = r#"( tasks: [], cnx: [], monitors: [(type: "ExampleMonitor", config: { "toto": 4, } )] ) "#;
        let config = CuConfig::deserialize_ron(txt).unwrap();
        assert_eq!(
            config
                .get_monitor_config()
                .unwrap()
                .config
                .as_ref()
                .unwrap()
                .0["toto"]
                .0,
            4u8.into()
        );
    }

    #[test]
    fn test_monitor_singular_syntax() {
        let txt = r#"( tasks: [], cnx: [], monitor: (type: "ExampleMonitor", config: { "toto": 4, } ) ) "#;
        let config = CuConfig::deserialize_ron(txt).unwrap();
        assert_eq!(config.get_monitor_configs().len(), 1);
        assert_eq!(config.get_monitor_config().unwrap().type_, "ExampleMonitor");
        assert_eq!(
            config
                .get_monitor_config()
                .unwrap()
                .config
                .as_ref()
                .unwrap()
                .0["toto"]
                .0,
            4u8.into()
        );
    }

    #[test]
    #[cfg(feature = "std")]
    fn test_render_topology_multi_input_ports() {
        let mut config = CuConfig::default();
        let graph = config.get_graph_mut(None).unwrap();
        let src1 = graph.add_node(Node::new("src1", "tasks::Source1")).unwrap();
        let src2 = graph.add_node(Node::new("src2", "tasks::Source2")).unwrap();
        let dst = graph.add_node(Node::new("dst", "tasks::Dst")).unwrap();
        graph.connect(src1, dst, "msg::A").unwrap();
        graph.connect(src2, dst, "msg::B").unwrap();

        let topology = build_render_topology(graph, &[]);
        let dst_node = topology
            .nodes
            .iter()
            .find(|node| node.id == "dst")
            .expect("missing dst node");
        assert_eq!(dst_node.inputs.len(), 2);

        let mut dst_ports: Vec<_> = topology
            .connections
            .iter()
            .filter(|cnx| cnx.dst == "dst")
            .map(|cnx| cnx.dst_port.as_deref().expect("missing dst port"))
            .collect();
        dst_ports.sort();
        assert_eq!(dst_ports, vec!["in.0", "in.1"]);
    }

    #[test]
    fn test_logging_parameters() {
        // Test with `enable_task_logging: false`
        let txt = r#"( tasks: [], cnx: [], logging: ( slab_size_mib: 1024, section_size_mib: 100, enable_task_logging: false ),) "#;

        let config = CuConfig::deserialize_ron(txt).unwrap();
        assert!(config.logging.is_some());
        let logging_config = config.logging.unwrap();
        assert_eq!(logging_config.slab_size_mib.unwrap(), 1024);
        assert_eq!(logging_config.section_size_mib.unwrap(), 100);
        assert!(!logging_config.enable_task_logging);

        // Test with `enable_task_logging` not provided
        let txt =
            r#"( tasks: [], cnx: [], logging: ( slab_size_mib: 1024, section_size_mib: 100, ),) "#;
        let config = CuConfig::deserialize_ron(txt).unwrap();
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

        let config = CuConfig::deserialize_ron(txt).unwrap();
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
            resources: None,
            missions: None,
            run_in_sim: None,
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

        let serialized = config.serialize_ron().unwrap();
        assert!(
            serialized.contains("bridges"),
            "bridges section missing from serialized config"
        );
        let deserialized = CuConfig::deserialize_ron(&serialized).unwrap();
        assert_eq!(deserialized.bridges.len(), 1);
        let bridge = &deserialized.bridges[0];
        assert!(bridge.is_run_in_sim());
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
    fn test_resource_parsing() {
        let txt = r#"
        (
            resources: [
                (
                    id: "fc",
                    provider: "copper_board_px4::Px4Bundle",
                    config: { "baud": 921600 },
                    missions: ["m1"],
                ),
                (
                    id: "misc",
                    provider: "cu29_runtime::StdClockBundle",
                ),
            ],
        )
        "#;

        let config = CuConfig::deserialize_ron(txt).unwrap();
        assert_eq!(config.resources.len(), 2);
        let fc = &config.resources[0];
        assert_eq!(fc.id, "fc");
        assert_eq!(fc.provider, "copper_board_px4::Px4Bundle");
        assert_eq!(fc.missions.as_deref(), Some(&["m1".to_string()][..]));
        let baud: u32 = fc
            .config
            .as_ref()
            .expect("missing config")
            .get::<u32>("baud")
            .expect("baud lookup failed")
            .expect("missing baud");
        assert_eq!(baud, 921_600);
        let misc = &config.resources[1];
        assert_eq!(misc.id, "misc");
        assert_eq!(misc.provider, "cu29_runtime::StdClockBundle");
        assert!(misc.config.is_none());
    }

    #[test]
    fn test_resource_roundtrip() {
        let mut config = CuConfig::default();
        let mut bundle_cfg = ComponentConfig::default();
        bundle_cfg.set("path", "/dev/ttyACM0".to_string());
        config.resources.push(ResourceBundleConfig {
            id: "fc".to_string(),
            provider: "copper_board_px4::Px4Bundle".to_string(),
            config: Some(bundle_cfg),
            missions: Some(vec!["m1".to_string()]),
        });

        let serialized = config.serialize_ron().unwrap();
        let deserialized = CuConfig::deserialize_ron(&serialized).unwrap();
        assert_eq!(deserialized.resources.len(), 1);
        let res = &deserialized.resources[0];
        assert_eq!(res.id, "fc");
        assert_eq!(res.provider, "copper_board_px4::Px4Bundle");
        assert_eq!(res.missions.as_deref(), Some(&["m1".to_string()][..]));
        let path: String = res
            .config
            .as_ref()
            .expect("missing config")
            .get::<String>("path")
            .expect("path lookup failed")
            .expect("missing path");
        assert_eq!(path, "/dev/ttyACM0");
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

        let config = CuConfig::deserialize_ron(txt).unwrap();
        let bridge = &config.bridges[0];
        match &bridge.channels[0] {
            BridgeChannelConfigRepresentation::Rx {
                config: Some(cfg), ..
            } => {
                let val = cfg
                    .get::<String>("filter")
                    .expect("filter lookup failed")
                    .expect("filter missing");
                assert_eq!(val, "fast");
            }
            _ => panic!("expected Rx channel with config"),
        }
        match &bridge.channels[1] {
            BridgeChannelConfigRepresentation::Tx {
                config: Some(cfg), ..
            } => {
                let rate = cfg
                    .get::<i32>("rate")
                    .expect("rate lookup failed")
                    .expect("rate missing");
                assert_eq!(rate, 100);
            }
            _ => panic!("expected Tx channel with config"),
        }
    }

    #[test]
    fn test_task_resources_roundtrip() {
        let txt = r#"
        (
            tasks: [
                (
                    id: "imu",
                    type: "tasks::ImuDriver",
                    resources: { "bus": "fc.spi_1", "irq": "fc.gpio_imu" },
                ),
            ],
            cnx: [],
        )
        "#;

        let config = CuConfig::deserialize_ron(txt).unwrap();
        let graph = config.graphs.get_graph(None).unwrap();
        let node = graph.get_node(0).expect("missing task node");
        let resources = node.get_resources().expect("missing resources map");
        assert_eq!(resources.get("bus").map(String::as_str), Some("fc.spi_1"));
        assert_eq!(
            resources.get("irq").map(String::as_str),
            Some("fc.gpio_imu")
        );

        let serialized = config.serialize_ron().unwrap();
        let deserialized = CuConfig::deserialize_ron(&serialized).unwrap();
        let graph = deserialized.graphs.get_graph(None).unwrap();
        let node = graph.get_node(0).expect("missing task node");
        let resources = node
            .get_resources()
            .expect("missing resources map after roundtrip");
        assert_eq!(resources.get("bus").map(String::as_str), Some("fc.spi_1"));
        assert_eq!(
            resources.get("irq").map(String::as_str),
            Some("fc.gpio_imu")
        );
    }

    #[test]
    fn test_bridge_resources_preserved() {
        let mut config = CuConfig::default();
        config.resources.push(ResourceBundleConfig {
            id: "fc".to_string(),
            provider: "board::Bundle".to_string(),
            config: None,
            missions: None,
        });
        let bridge_resources = HashMap::from([("serial".to_string(), "fc.serial0".to_string())]);
        config.bridges.push(BridgeConfig {
            id: "radio".to_string(),
            type_: "tasks::SerialBridge".to_string(),
            config: None,
            resources: Some(bridge_resources),
            missions: None,
            run_in_sim: None,
            channels: vec![BridgeChannelConfigRepresentation::Tx {
                id: "uplink".to_string(),
                route: None,
                config: None,
            }],
        });

        let serialized = config.serialize_ron().unwrap();
        let deserialized = CuConfig::deserialize_ron(&serialized).unwrap();
        let graph = deserialized.graphs.get_graph(None).expect("missing graph");
        let bridge_id = graph
            .get_node_id_by_name("radio")
            .expect("bridge node missing");
        let node = graph.get_node(bridge_id).expect("missing bridge node");
        let resources = node
            .get_resources()
            .expect("bridge resources were not preserved");
        assert_eq!(
            resources.get("serial").map(String::as_str),
            Some("fc.serial0")
        );
    }

    #[test]
    fn test_demo_config_parses() {
        let txt = r#"(
    resources: [
        (
            id: "fc",
            provider: "crate::resources::RadioBundle",
        ),
    ],
    tasks: [
        (id: "thr", type: "tasks::ThrottleControl"),
        (id: "tele0", type: "tasks::TelemetrySink0"),
        (id: "tele1", type: "tasks::TelemetrySink1"),
        (id: "tele2", type: "tasks::TelemetrySink2"),
        (id: "tele3", type: "tasks::TelemetrySink3"),
    ],
    bridges: [
        (  id: "crsf",
           type: "cu_crsf::CrsfBridge<SerialResource, SerialPortError>",
           resources: { "serial": "fc.serial" },
           channels: [
                Rx ( id: "rc_rx" ),  // receiving RC Channels
                Tx ( id: "lq_tx" ),  // Sending LineQuality back
            ],
        ),
        (
            id: "bdshot",
            type: "cu_bdshot::RpBdshotBridge",
            channels: [
                Tx ( id: "esc0_tx" ),
                Tx ( id: "esc1_tx" ),
                Tx ( id: "esc2_tx" ),
                Tx ( id: "esc3_tx" ),
                Rx ( id: "esc0_rx" ),
                Rx ( id: "esc1_rx" ),
                Rx ( id: "esc2_rx" ),
                Rx ( id: "esc3_rx" ),
            ],
        ),
    ],
    cnx: [
        (src: "crsf/rc_rx", dst: "thr", msg: "cu_crsf::messages::RcChannelsPayload"),
        (src: "thr", dst: "bdshot/esc0_tx", msg: "cu_bdshot::EscCommand"),
        (src: "thr", dst: "bdshot/esc1_tx", msg: "cu_bdshot::EscCommand"),
        (src: "thr", dst: "bdshot/esc2_tx", msg: "cu_bdshot::EscCommand"),
        (src: "thr", dst: "bdshot/esc3_tx", msg: "cu_bdshot::EscCommand"),
        (src: "bdshot/esc0_rx", dst: "tele0", msg: "cu_bdshot::EscTelemetry"),
        (src: "bdshot/esc1_rx", dst: "tele1", msg: "cu_bdshot::EscTelemetry"),
        (src: "bdshot/esc2_rx", dst: "tele2", msg: "cu_bdshot::EscTelemetry"),
        (src: "bdshot/esc3_rx", dst: "tele3", msg: "cu_bdshot::EscTelemetry"),
    ],
)"#;
        let config = CuConfig::deserialize_ron(txt).unwrap();
        assert_eq!(config.resources.len(), 1);
        assert_eq!(config.bridges.len(), 2);
    }

    #[test]
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

        let err = CuConfig::deserialize_ron(txt).expect_err("expected bridge source error");
        assert!(
            err.to_string()
                .contains("channel 'motor' is Tx and cannot act as a source")
        );
    }

    #[test]
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

        let err = CuConfig::deserialize_ron(txt).expect_err("expected bridge destination error");
        assert!(
            err.to_string()
                .contains("channel 'status' is Rx and cannot act as a destination")
        );
    }

    #[test]
    fn test_validate_logging_config() {
        // Test with valid logging configuration
        let txt =
            r#"( tasks: [], cnx: [], logging: ( slab_size_mib: 1024, section_size_mib: 100 ) )"#;
        let config = CuConfig::deserialize_ron(txt).unwrap();
        assert!(config.validate_logging_config().is_ok());

        // Test with invalid logging configuration
        let txt =
            r#"( tasks: [], cnx: [], logging: ( slab_size_mib: 100, section_size_mib: 1024 ) )"#;
        let config = CuConfig::deserialize_ron(txt).unwrap();
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
        let config = CuConfig::deserialize_ron(txt).unwrap();
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

        let config = CuConfig::deserialize_ron(txt).unwrap();
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

        let config = CuConfig::deserialize_ron(txt).unwrap();
        let serialized = config.serialize_ron().unwrap();
        let deserialized = CuConfig::deserialize_ron(&serialized).unwrap();
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
    fn test_mission_scoped_nc_connection_survives_serialize_roundtrip() {
        let txt = r#"(
            missions: [(id: "m1"), (id: "m2")],
            tasks: [
                (id: "src_m1", type: "a", missions: ["m1"]),
                (id: "src_m2", type: "b", missions: ["m2"]),
            ],
            cnx: [
                (src: "src_m1", dst: "__nc__", msg: "msg::A", missions: ["m1"]),
                (src: "src_m2", dst: "__nc__", msg: "msg::B", missions: ["m2"]),
            ]
        )"#;

        let config = CuConfig::deserialize_ron(txt).unwrap();
        let serialized = config.serialize_ron().unwrap();
        let deserialized = CuConfig::deserialize_ron(&serialized).unwrap();

        let m1_graph = deserialized.graphs.get_graph(Some("m1")).unwrap();
        let src_m1_id = m1_graph.get_node_id_by_name("src_m1").unwrap();
        let src_m1 = m1_graph.get_node(src_m1_id).unwrap();
        assert_eq!(src_m1.nc_outputs(), &["msg::A".to_string()]);

        let m2_graph = deserialized.graphs.get_graph(Some("m2")).unwrap();
        let src_m2_id = m2_graph.get_node_id_by_name("src_m2").unwrap();
        let src_m2 = m2_graph.get_node(src_m2_id).unwrap();
        assert_eq!(src_m2.nc_outputs(), &["msg::B".to_string()]);
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
        let config = CuConfig::deserialize_ron(txt).unwrap();
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
        let config = CuConfig::deserialize_ron(txt).unwrap();
        let logging_config = config.logging.unwrap();
        assert_eq!(logging_config.keyframe_interval.unwrap(), 100);
    }

    #[test]
    fn test_runtime_rate_target_rejects_zero() {
        let txt = r#"(
            tasks: [(id: "src", type: "a"), (id: "sink", type: "b")],
            cnx: [(src: "src", dst: "sink", msg: "msg::A")],
            runtime: (rate_target_hz: 0)
        )"#;

        let err =
            read_configuration_str(txt.to_string(), None).expect_err("runtime config should fail");
        assert!(
            err.to_string()
                .contains("Runtime rate target cannot be zero"),
            "unexpected error: {err}"
        );
    }

    #[test]
    fn test_runtime_rate_target_rejects_above_nanosecond_resolution() {
        let txt = format!(
            r#"(
                tasks: [(id: "src", type: "a"), (id: "sink", type: "b")],
                cnx: [(src: "src", dst: "sink", msg: "msg::A")],
                runtime: (rate_target_hz: {})
            )"#,
            MAX_RATE_TARGET_HZ + 1
        );

        let err = read_configuration_str(txt, None).expect_err("runtime config should fail");
        assert!(
            err.to_string().contains("exceeds the supported maximum"),
            "unexpected error: {err}"
        );
    }

    #[test]
    fn test_nc_connection_marks_source_output_without_creating_edge() {
        let txt = r#"(
            tasks: [(id: "src", type: "a"), (id: "sink", type: "b")],
            cnx: [
                (src: "src", dst: "sink", msg: "msg::A"),
                (src: "src", dst: "__nc__", msg: "msg::B"),
            ]
        )"#;
        let config = CuConfig::deserialize_ron(txt).unwrap();
        let graph = config.get_graph(None).unwrap();
        let src_id = graph.get_node_id_by_name("src").unwrap();
        let src_node = graph.get_node(src_id).unwrap();

        assert_eq!(graph.edge_count(), 1);
        assert_eq!(src_node.nc_outputs(), &["msg::B".to_string()]);
    }

    #[test]
    fn test_nc_connection_survives_serialize_roundtrip() {
        let txt = r#"(
            tasks: [(id: "src", type: "a"), (id: "sink", type: "b")],
            cnx: [
                (src: "src", dst: "sink", msg: "msg::A"),
                (src: "src", dst: "__nc__", msg: "msg::B"),
            ]
        )"#;
        let config = CuConfig::deserialize_ron(txt).unwrap();
        let serialized = config.serialize_ron().unwrap();
        let deserialized = CuConfig::deserialize_ron(&serialized).unwrap();
        let graph = deserialized.get_graph(None).unwrap();
        let src_id = graph.get_node_id_by_name("src").unwrap();
        let src_node = graph.get_node(src_id).unwrap();

        assert_eq!(graph.edge_count(), 1);
        assert_eq!(src_node.nc_outputs(), &["msg::B".to_string()]);
    }

    #[test]
    fn test_nc_connection_preserves_original_connection_order() {
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
        let src_node = graph.get_node(src_id).unwrap();
        let edge_id = graph.get_src_edges(src_id).unwrap()[0];
        let edge = graph.edge(edge_id).unwrap();

        assert_eq!(edge.msg, "msg::B");
        assert_eq!(edge.order, 1);
        assert_eq!(
            src_node
                .nc_outputs_with_order()
                .map(|(msg, order)| (msg.as_str(), order))
                .collect::<Vec<_>>(),
            vec![("msg::A", 0)]
        );
    }

    #[cfg(feature = "std")]
    fn multi_config_test_dir(name: &str) -> PathBuf {
        let unique = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .expect("system time before unix epoch")
            .as_nanos();
        let dir = std::env::temp_dir().join(format!("cu29_multi_config_{name}_{unique}"));
        std::fs::create_dir_all(&dir).expect("create temp test dir");
        dir
    }

    #[cfg(feature = "std")]
    fn write_multi_config_file(dir: &Path, name: &str, contents: &str) -> PathBuf {
        let path = dir.join(name);
        std::fs::write(&path, contents).expect("write temp config file");
        path
    }

    #[cfg(feature = "std")]
    fn alpha_subsystem_config() -> &'static str {
        r#"(
            tasks: [
                (id: "src", type: "demo::Src"),
                (id: "sink", type: "demo::Sink"),
            ],
            bridges: [
                (
                    id: "zenoh",
                    type: "demo::ZenohBridge",
                    channels: [
                        Tx(id: "ping"),
                        Rx(id: "pong"),
                    ],
                ),
            ],
            cnx: [
                (src: "src", dst: "zenoh/ping", msg: "demo::Ping"),
                (src: "zenoh/pong", dst: "sink", msg: "demo::Pong"),
            ],
        )"#
    }

    #[cfg(feature = "std")]
    fn beta_subsystem_config() -> &'static str {
        r#"(
            tasks: [
                (id: "responder", type: "demo::Responder"),
            ],
            bridges: [
                (
                    id: "zenoh",
                    type: "demo::ZenohBridge",
                    channels: [
                        Rx(id: "ping"),
                        Tx(id: "pong"),
                    ],
                ),
            ],
            cnx: [
                (src: "zenoh/ping", dst: "responder", msg: "demo::Ping"),
                (src: "responder", dst: "zenoh/pong", msg: "demo::Pong"),
            ],
        )"#
    }

    #[cfg(feature = "std")]
    fn instance_override_subsystem_config() -> &'static str {
        r#"(
            tasks: [
                (
                    id: "imu",
                    type: "demo::ImuTask",
                    config: {
                        "sample_hz": 200,
                    },
                ),
            ],
            resources: [
                (
                    id: "board",
                    provider: "demo::BoardBundle",
                    config: {
                        "bus": "i2c-1",
                    },
                ),
            ],
            bridges: [
                (
                    id: "radio",
                    type: "demo::RadioBridge",
                    config: {
                        "mtu": 32,
                    },
                    channels: [
                        Tx(id: "tx"),
                        Rx(id: "rx"),
                    ],
                ),
            ],
            cnx: [
                (src: "imu", dst: "radio/tx", msg: "demo::Packet"),
                (src: "radio/rx", dst: "imu", msg: "demo::Packet"),
            ],
        )"#
    }

    #[cfg(feature = "std")]
    #[test]
    fn test_read_multi_configuration_assigns_stable_subsystem_codes() {
        let dir = multi_config_test_dir("stable_ids");
        write_multi_config_file(&dir, "alpha.ron", alpha_subsystem_config());
        write_multi_config_file(&dir, "beta.ron", beta_subsystem_config());
        let network_path = write_multi_config_file(
            &dir,
            "network.ron",
            r#"(
                subsystems: [
                    (id: "beta", config: "beta.ron"),
                    (id: "alpha", config: "alpha.ron"),
                ],
                interconnects: [
                    (from: "alpha/zenoh/ping", to: "beta/zenoh/ping", msg: "demo::Ping"),
                    (from: "beta/zenoh/pong", to: "alpha/zenoh/pong", msg: "demo::Pong"),
                ],
            )"#,
        );

        let config =
            read_multi_configuration(network_path.to_str().expect("network path utf8")).unwrap();

        let alpha = config.subsystem("alpha").expect("alpha subsystem missing");
        let beta = config.subsystem("beta").expect("beta subsystem missing");
        assert_eq!(alpha.subsystem_code, 0);
        assert_eq!(beta.subsystem_code, 1);
        assert_eq!(config.interconnects.len(), 2);
        assert_eq!(config.interconnects[0].bridge_type, "demo::ZenohBridge");
    }

    #[cfg(feature = "std")]
    #[test]
    fn test_read_multi_configuration_rejects_wrong_direction() {
        let dir = multi_config_test_dir("wrong_direction");
        write_multi_config_file(&dir, "alpha.ron", alpha_subsystem_config());
        write_multi_config_file(&dir, "beta.ron", beta_subsystem_config());
        let network_path = write_multi_config_file(
            &dir,
            "network.ron",
            r#"(
                subsystems: [
                    (id: "alpha", config: "alpha.ron"),
                    (id: "beta", config: "beta.ron"),
                ],
                interconnects: [
                    (from: "alpha/zenoh/pong", to: "beta/zenoh/ping", msg: "demo::Pong"),
                ],
            )"#,
        );

        let err = read_multi_configuration(network_path.to_str().expect("network path utf8"))
            .expect_err("direction mismatch should fail");

        assert!(
            err.to_string()
                .contains("must reference a Tx bridge channel"),
            "unexpected error: {err}"
        );
    }

    #[cfg(feature = "std")]
    #[test]
    fn test_read_multi_configuration_rejects_declared_message_mismatch() {
        let dir = multi_config_test_dir("msg_mismatch");
        write_multi_config_file(&dir, "alpha.ron", alpha_subsystem_config());
        write_multi_config_file(&dir, "beta.ron", beta_subsystem_config());
        let network_path = write_multi_config_file(
            &dir,
            "network.ron",
            r#"(
                subsystems: [
                    (id: "alpha", config: "alpha.ron"),
                    (id: "beta", config: "beta.ron"),
                ],
                interconnects: [
                    (from: "alpha/zenoh/ping", to: "beta/zenoh/ping", msg: "demo::Wrong"),
                ],
            )"#,
        );

        let err = read_multi_configuration(network_path.to_str().expect("network path utf8"))
            .expect_err("message mismatch should fail");

        assert!(
            err.to_string()
                .contains("declares message type 'demo::Wrong'"),
            "unexpected error: {err}"
        );
    }

    #[cfg(feature = "std")]
    #[test]
    fn test_read_multi_configuration_resolves_instance_override_root() {
        let dir = multi_config_test_dir("instance_root");
        write_multi_config_file(&dir, "robot.ron", instance_override_subsystem_config());
        let network_path = write_multi_config_file(
            &dir,
            "multi_copper.ron",
            r#"(
                subsystems: [
                    (id: "robot", config: "robot.ron"),
                ],
                interconnects: [],
                instance_overrides_root: "instances",
            )"#,
        );

        let config =
            read_multi_configuration(network_path.to_str().expect("network path utf8")).unwrap();

        assert_eq!(
            config.instance_overrides_root.as_deref().map(Path::new),
            Some(dir.join("instances").as_path())
        );
    }

    #[cfg(feature = "std")]
    #[test]
    fn test_resolve_subsystem_config_for_instance_applies_overrides() {
        let dir = multi_config_test_dir("instance_apply");
        write_multi_config_file(&dir, "robot.ron", instance_override_subsystem_config());
        let instances_dir = dir.join("instances").join("17");
        std::fs::create_dir_all(&instances_dir).expect("create instance dir");
        write_multi_config_file(
            &instances_dir,
            "robot.ron",
            r#"(
                set: [
                    (
                        path: "tasks/imu/config",
                        value: {
                            "gyro_bias": [0.1, -0.2, 0.3],
                        },
                    ),
                    (
                        path: "resources/board/config",
                        value: {
                            "bus": "robot17-imu",
                        },
                    ),
                    (
                        path: "bridges/radio/config",
                        value: {
                            "mtu": 64,
                        },
                    ),
                ],
            )"#,
        );
        let network_path = write_multi_config_file(
            &dir,
            "multi_copper.ron",
            r#"(
                subsystems: [
                    (id: "robot", config: "robot.ron"),
                ],
                interconnects: [],
                instance_overrides_root: "instances",
            )"#,
        );

        let multi =
            read_multi_configuration(network_path.to_str().expect("network path utf8")).unwrap();
        let effective = multi
            .resolve_subsystem_config_for_instance("robot", 17)
            .expect("effective config");

        let graph = effective.get_graph(None).expect("graph");
        let imu_id = graph.get_node_id_by_name("imu").expect("imu node");
        let imu = graph.get_node(imu_id).expect("imu weight");
        let imu_cfg = imu.get_instance_config().expect("imu config");
        assert_eq!(imu_cfg.get::<u64>("sample_hz").unwrap(), Some(200));
        let gyro_bias: Vec<f64> = imu_cfg
            .get_value("gyro_bias")
            .expect("gyro_bias deserialize")
            .expect("gyro_bias value");
        assert_eq!(gyro_bias, vec![0.1, -0.2, 0.3]);

        let board = effective
            .resources
            .iter()
            .find(|resource| resource.id == "board")
            .expect("board resource");
        assert_eq!(
            board.config.as_ref().unwrap().get::<String>("bus").unwrap(),
            Some("robot17-imu".to_string())
        );

        let radio = effective
            .bridges
            .iter()
            .find(|bridge| bridge.id == "radio")
            .expect("radio bridge");
        assert_eq!(
            radio.config.as_ref().unwrap().get::<u64>("mtu").unwrap(),
            Some(64)
        );

        let radio_id = graph.get_node_id_by_name("radio").expect("radio node");
        let radio_node = graph.get_node(radio_id).expect("radio weight");
        assert_eq!(
            radio_node
                .get_instance_config()
                .unwrap()
                .get::<u64>("mtu")
                .unwrap(),
            Some(64)
        );
    }

    #[cfg(feature = "std")]
    #[test]
    fn test_resolve_subsystem_config_for_instance_rejects_unknown_path() {
        let dir = multi_config_test_dir("instance_unknown");
        write_multi_config_file(&dir, "robot.ron", instance_override_subsystem_config());
        let instances_dir = dir.join("instances").join("17");
        std::fs::create_dir_all(&instances_dir).expect("create instance dir");
        write_multi_config_file(
            &instances_dir,
            "robot.ron",
            r#"(
                set: [
                    (
                        path: "tasks/missing/config",
                        value: {
                            "gyro_bias": [1.0, 2.0, 3.0],
                        },
                    ),
                ],
            )"#,
        );
        let network_path = write_multi_config_file(
            &dir,
            "multi_copper.ron",
            r#"(
                subsystems: [
                    (id: "robot", config: "robot.ron"),
                ],
                interconnects: [],
                instance_overrides_root: "instances",
            )"#,
        );

        let multi =
            read_multi_configuration(network_path.to_str().expect("network path utf8")).unwrap();
        let err = multi
            .resolve_subsystem_config_for_instance("robot", 17)
            .expect_err("unknown task override should fail");

        assert!(
            err.to_string().contains("targets unknown task 'missing'"),
            "unexpected error: {err}"
        );
    }
}
