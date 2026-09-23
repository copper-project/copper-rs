fn main() {
    cu29_build::setup();
    println!("cargo:rerun-if-changed=target/pgs/selected.config.ron");
}
