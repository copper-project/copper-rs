fn main() {
    cu29_build::setup();
    println!("cargo::rustc-link-search={}", env!("CARGO_MANIFEST_DIR"));
    println!("cargo::rerun-if-changed=memory.x");
}
