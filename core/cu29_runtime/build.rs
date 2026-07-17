use std::path::Path;
fn main() {
    cu29_build::setup();

    println!("cargo:rustc-check-cfg=cfg(has_nvidia_gpu)");
    if Path::new("/proc/driver/nvidia/gpus").exists() {
        println!("cargo:rustc-cfg=has_nvidia_gpu");
    }
}
