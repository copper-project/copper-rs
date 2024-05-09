mod config;

use config::ConfigNode;
use uom::si::rational::Time;
use uom::si::time::second;

fn main() {

    let mut copperconfig = config::CopperConfig::new();
    let isp = ConfigNode::new("copper-isp").set_base_period(Time::new::<second>(1.into()));
    let camera = ConfigNode::new("copper-camera").set_base_period(Time::new::<second>(60.into()));
    let algo = ConfigNode::new("copper-algo").set_base_period(Time::new::<second>(5.into()));
    let n1 = copperconfig.add_node(isp);
    let n2 = copperconfig.add_node(camera);
    let n3 = copperconfig.add_node(algo);

    copperconfig.connect(n1, n2);
    copperconfig.connect(n2, n3);
    println!("{}", copperconfig.serialize());
}
