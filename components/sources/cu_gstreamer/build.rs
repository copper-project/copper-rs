use cfg_aliases::cfg_aliases;
fn main() {
    println!(
        "cargo:rustc-env=LOG_INDEX_DIR={}",
        std::env::var("OUT_DIR").unwrap()
    );
    cfg_aliases! {
        hardware: { all(target_os = "linux", not(feature = "mock")) },
        mock: { any(not(target_os = "linux"), feature = "mock") },
    }
}
