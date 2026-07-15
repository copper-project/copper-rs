fn main() {
    // Make sure the linker picks up the STM32H743 memory layout.
    let out_dir = std::path::PathBuf::from(std::env::var("OUT_DIR").unwrap());
    println!("cargo:rustc-link-search={}", out_dir.display());

    let memory_x = include_bytes!("memory.x");
    std::fs::write(out_dir.join("memory.x"), memory_x).unwrap();
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=build.rs");
    for config in [
        "copperconfig.ron",
        "mcu_graph.ron",
        "mcu_config.ron",
        "mcu_autonomy_config.ron",
        "compute_bevymon_config.ron",
        "compute_end2end_config.ron",
        "compute_vitfly_config.ron",
        "multi_copper.ron",
        "multi_copper_udp.ron",
        "multi_copper_vitfly_sim.ron",
    ] {
        println!("cargo:rerun-if-changed={config}");
    }

    println!(
        "cargo:rustc-env=LOG_INDEX_DIR={}",
        std::env::var("OUT_DIR").unwrap()
    );
}
