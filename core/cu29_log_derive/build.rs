fn main() {
    println!("cargo:rustc-check-cfg=cfg(cu29_default_log_level_debug)");
    println!("cargo:rustc-check-cfg=cfg(cu29_default_log_level_info)");
    let has_manual_level = std::env::vars().any(|(k, _)| {
        k == "CARGO_FEATURE_LOG_LEVEL_DEBUG"
            || k == "CARGO_FEATURE_LOG_LEVEL_INFO"
            || k == "CARGO_FEATURE_LOG_LEVEL_WARNING"
            || k == "CARGO_FEATURE_LOG_LEVEL_ERROR"
            || k == "CARGO_FEATURE_LOG_LEVEL_CRITICAL"
    });

    if !has_manual_level {
        match std::env::var("PROFILE").as_deref() {
            Ok("release") => println!("cargo:rustc-cfg=cu29_default_log_level_info"),
            _ => println!("cargo:rustc-cfg=cu29_default_log_level_debug"),
        }
    }

    if cfg!(target_os = "windows") {
        println!("cargo:rustc-link-lib=advapi32");
    }
    println!(
        "cargo:rustc-env=LOG_INDEX_DIR={}",
        std::env::var("OUT_DIR").unwrap()
    );
}
