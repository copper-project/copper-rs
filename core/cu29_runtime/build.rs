use std::path::Path;
fn main() {
    println!(
        "cargo:rustc-env=LOG_INDEX_DIR={}",
        std::env::var("OUT_DIR").unwrap()
    );

    println!("cargo:rustc-check-cfg=cfg(has_nvidia_gpu)");
    if Path::new("/proc/driver/nvidia/gpus").exists() {
        println!("cargo:rustc-cfg=has_nvidia_gpu");
    }
}
