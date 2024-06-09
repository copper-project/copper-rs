use std::env;

const INDEX_DIR_NAME: &str = "copper_log_index";

fn main() {
    let out_dir = env::var("OUT_DIR").expect("OUT_DIR is not defined");
    println!("cargo:rustc-env=OUT_DIR={}", out_dir);
}
