use copper::register_plugin;
use copper::config::CopperConfig;
use copper::config::ConfigNode;
use copper::core::Plugin;
use copper::core::PluginType;
use uom::si::rational::Time;
use uom::si::time::second;

#[derive(Default)]
struct MyPlugin;

impl Plugin for MyPlugin {
    fn name(&self) -> &'static str {
        "MyPlugin"
    }

    fn initialize(&self) {
        println!("Initializing MyPlugin");
    }

    fn plugin_type(&self) -> PluginType {
        PluginType::Algorithm
    }
}

register_plugin!(MyPlugin);

fn main() {

    let mut copperconfig = CopperConfig::new();
    let camera = ConfigNode::new("copper-camera").set_base_period(Time::new::<second>(60.into()));
    let isp = ConfigNode::new("copper-isp").set_base_period(Time::new::<second>(1.into()));
    let algo = ConfigNode::new("copper-algo").set_base_period(Time::new::<second>(5.into()));
    let n1 = copperconfig.add_node(isp);
    let n2 = copperconfig.add_node(camera);
    let n3 = copperconfig.add_node(algo);

    copperconfig.connect(n1, n2);
    copperconfig.connect(n2, n3);
    println!("{}", copperconfig.serialize());
    

    {
        let mut file = std::fs::File::create("/tmp/copperconfig.dot").unwrap();
        copperconfig.render(&mut file);
    }

}
