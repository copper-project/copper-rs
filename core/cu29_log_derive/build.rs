fn main() {
    if cfg!(target_os = "windows") {
        println!("cargo:rustc-link-lib=advapi32");
    }
    println!(
        "cargo:rustc-env=LOG_INDEX_DIR={}",
        std::env::var("OUT_DIR").unwrap()
    );
}
