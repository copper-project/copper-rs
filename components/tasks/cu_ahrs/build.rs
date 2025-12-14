use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    // Allow consumers (e.g., alternative MCU targets) to opt out of the bundled
    // RP2350 memory.x to avoid clashes with their own linker scripts.
    let skip_memory_x = std::env::var("CU_AHRS_SKIP_MEMORY_X").is_ok();

    // Put the linker script somewhere the linker can find it
    let out = PathBuf::from(std::env::var_os("OUT_DIR").unwrap());
    println!("cargo:rustc-link-search={}", out.display());

    if !skip_memory_x {
        let memory_x = include_bytes!("memory.x");
        let mut f = File::create(out.join("memory.x")).unwrap();
        f.write_all(memory_x).unwrap();
        println!("cargo:rerun-if-changed=memory.x");
    }
    println!("cargo:rerun-if-changed=build.rs");

    // Needed by cu29 logging macros (LOG_INDEX_DIR env var)
    println!("cargo:rustc-env=LOG_INDEX_DIR={}", out.display());
}
