fn main() {
    let gst_enabled = std::env::var("CARGO_FEATURE_GST").is_ok();
    if !gst_enabled {
        println!(
            "cargo:warning=GStreamer feature (gst) is not enabled. Skipping cu_dynthreshold build."
        );
    }
    cu29_build::setup();
}
