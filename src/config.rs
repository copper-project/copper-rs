use petgraph::dot::Dot;
use petgraph::dot::Config as PetConfig;
use petgraph::stable_graph::StableDiGraph;
use serde::{Serialize, Deserialize};
use uom::si::rational::Time;
use uom::si::time::nanosecond;
use ron::Options;
use ron::extensions::Extensions;


pub type ConfigNodeId = u32;

#[derive(Serialize, Deserialize, Debug)]
pub enum LinkType {
    Local,
    Network
}

#[derive(Serialize, Deserialize, Debug)]
pub struct ConfigNode {
    name: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    plugin_package: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    base_period_ns: Option<isize>,
}

impl ConfigNode {
    pub fn new(name: &str) -> Self {
        ConfigNode {
            name: name.to_string(),
            plugin_package:None,
            base_period_ns: None,
        }
    }

    #[allow(dead_code)]
    pub fn set_plugin_package(mut self, package: Option<String>) -> Self {
        self.plugin_package = package;
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

}


#[derive(Serialize, Deserialize, Debug)]
pub struct CopperConfig {
    pub graph: StableDiGraph<ConfigNode, LinkType, ConfigNodeId>,
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
    pub fn connect(&mut self, source: ConfigNodeId, target: ConfigNodeId) {
        self.graph.add_edge(source.into(), target.into(), LinkType::Local);
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
            plugin_package: Some("test".to_string()),
            base_period_ns: Some(1_000_000_000),
        };
        assert_eq!(node.base_period(), Some(Time::new::<second>(1.into())));
        
        let node = ConfigNode::new("test2").set_base_period(Time::new::<millisecond>(500.into()));
        assert_eq!(node.base_period_ns, Some(500_000_000));
        assert_eq!(node.base_period(), Some(Time::new::<nanosecond>(500_000_000.into())));
    }

    #[test]
    fn test_serialize() {
        let mut config = CopperConfig::new();
        let n1 = config.add_node(ConfigNode::new("test1"));
        let n2 = config.add_node(ConfigNode::new("test2"));
        config.connect(n1, n2);
        let serialized = config.serialize();
        println!("{}", serialized);
        let deserialized = CopperConfig::deserialize(&serialized);
        assert_eq!(config.graph.node_count(), deserialized.graph.node_count());
        assert_eq!(config.graph.edge_count(), deserialized.graph.edge_count());
    }
}
