use std::path::{Path, PathBuf};
use std::{fs, io};

fn main() {
    println!("cargo::rerun-if-changed=tests/config");
    let trybuild_dir = PathBuf::from(std::env::var("OUT_DIR").unwrap())
        .ancestors()
        .nth(4)
        .unwrap()
        .to_path_buf()
        .join("tests/trybuild/cu29-derive/config");
    let config_dir =
        PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").unwrap()).join("tests/config");
    copy_dir_all(config_dir, trybuild_dir).unwrap();
}

fn copy_dir_all(src: impl AsRef<Path>, dst: impl AsRef<Path>) -> io::Result<()> {
    fs::create_dir_all(&dst)?;
    for entry in fs::read_dir(src)? {
        let entry = entry?;
        let ty = entry.file_type()?;
        if ty.is_dir() {
            copy_dir_all(entry.path(), dst.as_ref().join(entry.file_name()))?;
        } else {
            fs::copy(entry.path(), dst.as_ref().join(entry.file_name()))?;
        }
    }
    Ok(())
}
