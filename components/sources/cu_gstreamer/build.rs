use cfg_aliases::cfg_aliases;
fn main() {
    let gst_enabled = std::env::var("CARGO_FEATURE_GST").is_ok();
    if !gst_enabled {
        println!("cargo:warning=GStreamer feature is not enabled. Skipping cu_gstreamer build.");
    }
    println!(
        "cargo:rustc-env=LOG_INDEX_DIR={}",
        std::env::var("OUT_DIR").unwrap()
    );
    cfg_aliases! {
        hardware: { all(target_os = "linux", not(feature = "mock")) },
        mock: { any(not(target_os = "linux"), feature = "mock") },
    }
}
