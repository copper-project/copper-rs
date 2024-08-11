use std::env;
use std::path::{Path, PathBuf};

fn main() {

    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());

    let target_dir =  out_dir.join("../../..");
    let lib_name = "libcu29_export.so";
    let new_name = "cu29_export.so";

    let lib_path = target_dir.join(lib_name);
    let new_path = target_dir.join(new_name);

    if !new_path.exists() {
        std::os::unix::fs::symlink(&lib_path, &new_path).expect("Failed to create symlink");
    }
}

