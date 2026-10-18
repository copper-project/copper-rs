fn main() {
    cu29_build::setup();
    println!("cargo::rerun-if-changed=build.rs");
    println!("cargo::rerun-if-changed=copperconfig.ron");
    println!("cargo::rerun-if-changed=src");
}
