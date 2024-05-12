use std::env;
use std::io::Write;
use std::path::PathBuf;
use copper::config::CopperConfig;
use copper::config::ConfigNode;
use copper::config::Value;
use uom::si::time::second;
use uom::si::time::Time;

fn main() {

    // Generate a config
    let mut copperconfig = CopperConfig::new();
    let mut camera = ConfigNode::new("copper-camera", "camerapkg::Camera").set_base_period(Time::new::<second>(60.into()));
    camera.set_param::<Value>("resolution-height", 1080.into());
    camera.set_param::<Value>("resolution-width", 1920.into());
    let mut isp = ConfigNode::new("copper-isp", "isppkg::Isp").set_base_period(Time::new::<second>(1.into()));
    isp.set_param::<Value>("tone", 1.3.into());
    let algo = ConfigNode::new("copper-algo", "algopkg::Algo").set_base_period(Time::new::<second>(5.into()));
    let n1 = copperconfig.add_node(isp);
    let n2 = copperconfig.add_node(camera);
    let n3 = copperconfig.add_node(algo);

    copperconfig.connect(n2, n1, "imgmsgpkg::Image");
    copperconfig.connect(n1, n3, "imgmsgpkg::Image");
    {
        let mut dest = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
        dest.push("copperconfig.ron");
        let mut file = std::fs::File::create(dest).unwrap();
        file.write_all(copperconfig.serialize().as_bytes()).unwrap();
    }
}
