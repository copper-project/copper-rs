#![doc = include_str!("../README.md")]

/// Emit the standard Cargo build-script configuration required by Copper.
pub fn setup() {
    println!(
        "cargo::rustc-env=LOG_INDEX_DIR={}",
        std::env::var("OUT_DIR").expect("Cargo must provide OUT_DIR")
    );
}
