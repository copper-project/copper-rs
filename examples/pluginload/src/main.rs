use uom::si::rational::Time;
use uom::si::time::second;

use copper::config::CopperConfig;
use copper::config::Node;

fn main() {
    let mut copperconfig = CopperConfig::default();
    let mut camera =
        Node::new("copper-camera", "camerapkg::Camera")
            .set_base_period(Time::new::<second>(60.into()));
    camera.set_param::<i32>("resolution-height", 1080);
    camera.set_param::<i32>("resolution-width", 1920);
    let mut isp =
        Node::new("copper-isp", "isppkg::Isp").set_base_period(Time::new::<second>(1.into()));
    isp.set_param::<f64>("tone", 1.3);
    let algo =
        Node::new("copper-algo", "algopkg::Algo").set_base_period(Time::new::<second>(5.into()));
    let n1 = copperconfig.add_node(isp);
    let n2 = copperconfig.add_node(camera);
    let n3 = copperconfig.add_node(algo);

    copperconfig.connect(n2, n1, "imgmsgpkg::Image");
    copperconfig.connect(n1, n3, "imgmsgpkg::Image");
    println!("{}", copperconfig.serialize());

    {
        let mut file = std::fs::File::create("/tmp/copperconfig.dot").unwrap();
        copperconfig.render(&mut file);
    }
}
