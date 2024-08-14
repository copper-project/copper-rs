use std::env;
fn main() {
    // This is essential to be able to generate the structure log index within the project out directory.
    let out_dir = env::var("OUT_DIR").unwrap();
    println!("cargo:rustc-env=LOGIN_INDEX_DIR={}", out_dir);
}
