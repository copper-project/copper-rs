use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
    println!("cargo:rustc-env=LOG_INDEX_DIR={}", out_dir.display());
    println!("cargo:rustc-link-search={}", out_dir.display());

    let memory_x = include_bytes!("memory.x");
    let mut f = File::create(out_dir.join("memory.x")).unwrap();
    f.write_all(memory_x).unwrap();
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=build.rs");
}
