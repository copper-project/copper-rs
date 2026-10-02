fn main() {
    cu29_build::setup();
    for config in [
        "mcu_config.ron",
        "mcu_graph.ron",
        "mcu_autonomy.ron",
        "compute_config.ron",
        "compute_preview.ron",
        "compute_autonomy.ron",
    ] {
        println!("cargo::rerun-if-changed=../{config}");
    }
}
