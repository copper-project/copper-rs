fn main() {
    println!(
        "cargo:rustc-env=LOG_INDEX_DIR={}",
        std::env::var("OUT_DIR").expect("OUT_DIR is always set by cargo"),
    );
}
