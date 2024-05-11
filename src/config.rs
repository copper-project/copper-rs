use petgraph::dot::Dot;
use petgraph::dot::Config as PetConfig;
use petgraph::stable_graph::StableDiGraph;
use serde::{Serialize, Deserialize};
use uom::si::rational::Time;
use uom::si::time::nanosecond;
use ron::Options;
use ron::extensions::Extensions;
use std::collections::HashMap;
use ron::value::Value as RonValue;


pub type ConfigNodeId = u32;
pub type NodeConfig = HashMap<String, Value>;

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct Value(RonValue);

impl From<i32> for Value {
    fn from(value: i32) -> Self {
        Value(RonValue::Number(value.into()))
    }
}
impl From<f64> for Value {
    fn from(value: f64) -> Self {
        Value(RonValue::Number(value.into()))
    }
}

impl From<Value> for i32 {
    fn from(value: Value) -> Self {
        if let RonValue::Number(num) = value.0 {
            if let Some(i) = num.as_i64() {
                i as i32
            } else {
                panic!("Expected an integer value")
            }
        } else {
            panic!("Expected a Number variant")
        }
    }
}

impl From<Value> for f64 {
    fn from(value: Value) -> Self {
        if let RonValue::Number(num) = value.0 {
            if let Some(f) = num.as_f64() {
                f
            } else {
                panic!("Expected a float value")
            }
        } else {
            panic!("Expected a Number variant")
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


#[derive(Serialize, Deserialize, Debug)]
pub struct ConfigNode {
    name: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    ptype: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    base_period_ns: Option<isize>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pconfig:Option<NodeConfig>, 
}

impl ConfigNode {
    pub fn new(name: &str, ptype: &str) -> Self {
        ConfigNode {
            name: name.to_string(),
            ptype: Some(ptype.to_string()),
            base_period_ns: None,
            pconfig: None,
        }
    }

    #[allow(dead_code)]
    pub fn set_plugin_package(mut self, package: Option<String>) -> Self {
        self.ptype = package;
        self
    }

    #[allow(dead_code)]
    pub fn base_period(&self) -> Option<Time> {
        self.base_period_ns.map(|frequency| Time::new::<nanosecond>(frequency.into()))
    }

    pub fn set_base_period(mut self, period: Time) -> Self {
        let as_ns = &period.get::<nanosecond>();
        debug_assert_eq!(*as_ns.denom(), 1isize); // We normalize to the ns
        self.base_period_ns = Some(*as_ns.numer());
        self
    }

    pub fn get_param<T: From<Value>>(&self, key: &str) -> Option<T> {
        let pc = self.pconfig.as_ref();
        if pc.is_none() {
            return None;
        }
        let pc = pc.unwrap();
        let v = pc.get(key);
        if v.is_none() {
            return None;
        }
        Some(T::from(v.unwrap().clone()))
    }

    pub fn set_param<T: Into<Value>>(&mut self, key: &str, value: T) {
        if self.pconfig.is_none() {
            self.pconfig = Some(HashMap::new());
        }
        self.pconfig.as_mut().unwrap().insert(key.to_string(), value.into());
    }

}


#[derive(Serialize, Deserialize, Debug)]
pub struct CopperConfig {
    pub graph: StableDiGraph<ConfigNode, String, ConfigNodeId>,
}

impl CopperConfig {
    pub fn new() -> Self {
        CopperConfig {
            graph: StableDiGraph::new(),
        }
    }
    pub fn add_node(&mut self, node: ConfigNode) -> ConfigNodeId {
        self.graph.add_node(node).index() as ConfigNodeId
    }
    pub fn connect(&mut self, source: ConfigNodeId, target: ConfigNodeId, msg_type: &str) {
        self.graph.add_edge(source.into(), target.into(), msg_type.to_string());
    }

    #[allow(dead_code)]
    pub fn serialize(&self) -> String {
        let ron = Options::default()
            .with_default_extension(Extensions::IMPLICIT_SOME)
            .with_default_extension(Extensions::UNWRAP_NEWTYPES)
            .with_default_extension(Extensions::UNWRAP_VARIANT_NEWTYPES);
        let pretty = ron::ser::PrettyConfig::default();
        let answer = ron.to_string_pretty(&self, pretty).unwrap();
        // RON doesn't put its own options in the serialization format, so we have to add it manually
        format!("#![enable(implicit_some)]\n{}\n", answer)

    }

    #[allow(dead_code)]
    pub fn deserialize(ron: &str) -> Self {
        ron::de::from_str(ron).expect("Syntax Error in config")
    }

    pub fn render(&self, output: &mut dyn std::io::Write) {
        let dot = Dot::with_config(&self.graph, &[PetConfig::EdgeNoLabel]);
        write!(output, "{:?}", dot).unwrap();
    }
}

// tests
#[cfg(test)]
mod tests {
    use super::*;
    use uom::si::time::second;
    use uom::si::time::millisecond;

    #[test]
    fn test_base_period() {
        let node = ConfigNode {
            name: "test".to_string(),
            ptype: Some("test".to_string()),
            pconfig: Some(HashMap::new()),
            base_period_ns: Some(1_000_000_000),
        };
        assert_eq!(node.base_period(), Some(Time::new::<second>(1.into())));
        
        let node = ConfigNode::new("test2", "package::Plugin").set_base_period(Time::new::<millisecond>(500.into()));
        assert_eq!(node.base_period_ns, Some(500_000_000));
        assert_eq!(node.base_period(), Some(Time::new::<nanosecond>(500_000_000.into())));
    }

    #[test]
    fn test_serialize() {
        let mut config = CopperConfig::new();
        let n1 = config.add_node(ConfigNode::new("test1", "package::Plugin1"));
        let n2 = config.add_node(ConfigNode::new("test2", "package::Plugin2"));
        config.connect(n1, n2, "msgpkg::MsgType");
        let serialized = config.serialize();
        println!("{}", serialized);
        let deserialized = CopperConfig::deserialize(&serialized);
        assert_eq!(config.graph.node_count(), deserialized.graph.node_count());
        assert_eq!(config.graph.edge_count(), deserialized.graph.edge_count());
    }
}
