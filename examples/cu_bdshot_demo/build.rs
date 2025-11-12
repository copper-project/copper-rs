use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    // Struct logging for Copper
    println!(
        "cargo:rustc-env=LOG_INDEX_DIR={}",
        std::env::var("OUT_DIR").unwrap()
    );
    //
    // Put the linker script somewhere the linker can find it
    let out = PathBuf::from(std::env::var_os("OUT_DIR").unwrap());
    println!("cargo:rustc-link-search={}", out.display());

    let memory_x = include_bytes!("memory.x");
    let mut f = File::create(out.join("memory.x")).unwrap();
    f.write_all(memory_x).unwrap();
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=build.rs");
}
