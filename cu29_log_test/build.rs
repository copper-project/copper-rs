use std::env;

fn main() {
    let out_dir = env::var("OUT_DIR").expect("OUT_DIR is not defined");
    println!("cargo:rustc-env=OUT_DIR={}", out_dir);
}
