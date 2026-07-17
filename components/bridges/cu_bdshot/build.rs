fn main() {
    // Struct logging for Copper
    cu29_build::setup();

    println!("cargo:rerun-if-changed=src/dshot.pio");
}
