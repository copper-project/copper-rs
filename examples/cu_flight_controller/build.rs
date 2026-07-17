fn main() {
    // Make sure the linker picks up the STM32H743 memory layout.
    let out_dir = std::path::PathBuf::from(std::env::var("OUT_DIR").unwrap());
    println!("cargo:rustc-link-search={}", out_dir.display());

    let memory_x = include_bytes!("memory.x");
    std::fs::write(out_dir.join("memory.x"), memory_x).unwrap();
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=build.rs");
    for config in [
        "flight_controller.ron",
        "mcu_config.ron",
        "mcu_graph.ron",
        "mcu_autonomy.ron",
        "compute_config.ron",
        "compute_preview.ron",
        "compute_autonomy.ron",
    ] {
        println!("cargo:rerun-if-changed={config}");
    }

    cu29_build::setup();
}
