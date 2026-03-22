//! Shared std-only helpers for unified-log slab files.

use std::fs::{File, OpenOptions};
use std::io;
use std::path::{Path, PathBuf};

pub(crate) fn build_slab_path(base_file_path: &Path, slab_index: usize) -> io::Result<PathBuf> {
    let mut file_path = base_file_path.to_path_buf();
    let stem = file_path.file_stem().ok_or_else(|| {
        io::Error::new(
            io::ErrorKind::InvalidInput,
            "Base file path has no file name",
        )
    })?;
    let stem = stem.to_str().ok_or_else(|| {
        io::Error::new(
            io::ErrorKind::InvalidInput,
            "Base file name is not valid UTF-8",
        )
    })?;
    let extension = file_path.extension().ok_or_else(|| {
        io::Error::new(
            io::ErrorKind::InvalidInput,
            "Base file path has no extension",
        )
    })?;
    let extension = extension.to_str().ok_or_else(|| {
        io::Error::new(
            io::ErrorKind::InvalidInput,
            "Base file extension is not valid UTF-8",
        )
    })?;
    if stem.is_empty() {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "Base file name is empty",
        ));
    }
    let file_name = format!("{stem}_{slab_index}.{extension}");
    file_path.set_file_name(file_name);
    Ok(file_path)
}

pub(crate) fn make_slab_file(
    base_file_path: &Path,
    slab_size: usize,
    slab_suffix: usize,
) -> io::Result<File> {
    let file_path = build_slab_path(base_file_path, slab_suffix)?;
    let file = OpenOptions::new()
        .read(true)
        .write(true)
        .create(true)
        .truncate(true)
        .open(&file_path)
        .map_err(|e| {
            io::Error::new(
                e.kind(),
                format!("Failed to open file {}: {e}", file_path.display()),
            )
        })?;
    file.set_len(slab_size as u64).map_err(|e| {
        io::Error::new(
            e.kind(),
            format!("Failed to set file length for {}: {e}", file_path.display()),
        )
    })?;
    Ok(file)
}

fn remove_existing_alias(base_file_path: &Path) -> io::Result<()> {
    match std::fs::symlink_metadata(base_file_path) {
        Ok(meta) => {
            if meta.is_dir() {
                return Err(io::Error::new(
                    io::ErrorKind::AlreadyExists,
                    format!(
                        "Cannot create base log alias at {} because a directory already exists there",
                        base_file_path.display()
                    ),
                ));
            }
            std::fs::remove_file(base_file_path).map_err(|e| {
                io::Error::new(
                    e.kind(),
                    format!(
                        "Failed to remove existing base log alias {}: {e}",
                        base_file_path.display()
                    ),
                )
            })
        }
        Err(e) if e.kind() == io::ErrorKind::NotFound => Ok(()),
        Err(e) => Err(io::Error::new(
            e.kind(),
            format!(
                "Failed to inspect existing base log alias {}: {e}",
                base_file_path.display()
            ),
        )),
    }
}

pub(crate) fn create_base_alias_link(base_file_path: &Path) -> io::Result<()> {
    let first_slab_path = build_slab_path(base_file_path, 0)?;
    remove_existing_alias(base_file_path)?;

    #[cfg(unix)]
    {
        use std::os::unix::fs::symlink;
        let relative_target = Path::new(first_slab_path.file_name().ok_or_else(|| {
            io::Error::new(
                io::ErrorKind::InvalidInput,
                "First slab file has no name component",
            )
        })?);
        symlink(relative_target, base_file_path).map_err(|e| {
            io::Error::new(
                e.kind(),
                format!(
                    "Failed to create base log alias {} -> {}: {e}",
                    base_file_path.display(),
                    first_slab_path.display()
                ),
            )
        })
    }

    #[cfg(windows)]
    {
        use std::os::windows::fs::symlink_file;
        let relative_target = Path::new(first_slab_path.file_name().ok_or_else(|| {
            io::Error::new(
                io::ErrorKind::InvalidInput,
                "First slab file has no name component",
            )
        })?);
        match symlink_file(relative_target, base_file_path) {
            Ok(()) => Ok(()),
            Err(symlink_err) => std::fs::hard_link(&first_slab_path, base_file_path).map_err(
                |hard_link_err| {
                    io::Error::other(format!(
                        "Failed to create base log alias {}. Symlink error: {symlink_err}. Hard-link fallback error: {hard_link_err}",
                        base_file_path.display()
                    ))
                },
            ),
        }?;
        Ok(())
    }

    #[cfg(not(any(unix, windows)))]
    {
        std::fs::hard_link(&first_slab_path, base_file_path).map_err(|e| {
            io::Error::new(
                e.kind(),
                format!(
                    "Failed to create base log alias {} -> {}: {e}",
                    base_file_path.display(),
                    first_slab_path.display()
                ),
            )
        })
    }
}
