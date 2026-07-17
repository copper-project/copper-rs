use cfg_aliases::cfg_aliases;
fn main() {
    let gst_enabled = std::env::var("CARGO_FEATURE_GST").is_ok();
    if !gst_enabled {
        println!(
            "cargo:warning=GStreamer feature (gst) is not enabled. Skipping cu_gstreamer build."
        );
    }
    cu29_build::setup();
    cfg_aliases! {
        hardware: { all(target_os = "linux", not(feature = "mock")) },
        mock: { any(not(target_os = "linux"), feature = "mock") },
    }
}
