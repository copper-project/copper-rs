fn main() {
    cu29_build::setup();

    println!("cargo::rerun-if-changed=copperconfig.ron");
    println!("cargo::rerun-if-changed=graph/base.ron");
    println!("cargo::rerun-if-changed=graph/jetson_mipi.ron");
    println!("cargo::rerun-if-changed=tests/forwarding");
}
