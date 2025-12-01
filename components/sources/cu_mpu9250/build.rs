use cfg_aliases::cfg_aliases;

fn main() {
    println!(
        "cargo:rustc-env=LOG_INDEX_DIR={}",
        std::env::var("OUT_DIR").expect("OUT_DIR not set")
    );

    cfg_aliases! {
        hardware: { all(target_os = "linux", feature = "linux-embedded") },
    }
}
