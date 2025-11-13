fn main() {
    // Struct logging for Copper
    println!(
        "cargo:rustc-env=LOG_INDEX_DIR={}",
        std::env::var("OUT_DIR").unwrap()
    );

    println!("cargo:rerun-if-changed=src/dshot.pio");
}
