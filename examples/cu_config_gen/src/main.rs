use cu29::config::CuConfig;
use cu29::config::Node;
use cu29::config::Value;

/// This is a small example to show how to programmatically generate a configuration.
/// This is useful for making variations of your robot more easily.
fn main() {
    // Generate a config
    let mut copperconfig = CuConfig::default();
    let graph = copperconfig.get_graph_mut(None).unwrap();
    let mut camera = Node::new("camera", "camerapkg::Camera");
    camera.set_param::<Value>("resolution-height", 1080.into());
    let isp = Node::new("copper-isp", "isppkg::Isp");
    let algo = Node::new("copper-algo", "algopkg::Algo");
    let n1 = graph.add_node(isp).unwrap();
    let n2 = graph.add_node(camera).unwrap();
    let n3 = graph.add_node(algo).unwrap();

    graph.connect(n2, n1, "imgmsgpkg::Image").unwrap();
    graph.connect(n1, n3, "imgmsgpkg::Image").unwrap();
    println!("{}", copperconfig.serialize_ron());
}
