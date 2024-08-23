fn main() {
    println!(
        "cargo:rustc-env=LOG_INDEX_DIR={}",
        std::env::var("OUT_DIR").unwrap()
    );
    /// completely fail if the platform is macos
    if cfg!(target_os = "macos") {
        panic!("macos is not supported");
    };
}
