use crate::cached_path::error::Error;
use flate2::read::GzDecoder;
use std::fs::{self, File};
use std::io::Read;
use std::path::Path;
use tempfile::tempdir_in;

/// Supported archive types.
pub(crate) enum ArchiveFormat {
    TarGz,
    Zip,
}

// see https://github.com/bojand/infer/issues/91
#[allow(clippy::nonminimal_bool)]
fn is_lzma(buf: &[u8]) -> bool {
    buf.len() > 4
        && buf[0] == 0x5D
        && buf[1] == 0x00
        && buf[2] == 0x00
        && (buf[3] == 0x80
            || buf[3] == 0x01
            || buf[3] == 0x10
            || buf[3] == 0x08
            || buf[3] == 0x20
            || buf[3] == 0x40
            || buf[3] == 0x80
            || buf[3] == 0x00)
        && (buf[4] == 0x00 || buf[4] == 0x01 || buf[4] == 0x02)
}

fn infer() -> infer::Infer {
    let mut infer = infer::Infer::new();
    infer.add("application/x-lzma", "lzma", is_lzma);
    infer
}

impl ArchiveFormat {
    fn is_tar<R: Read>(read: &mut R) -> bool {
        let mut buf = [0; 262];
        read.read_exact(&mut buf)
            .is_ok_and(|_| infer::archive::is_tar(&buf))
    }

    /// Parse archive type from resource extension.
    pub(crate) fn parse_from_extension(resource: &Path) -> Result<Self, Error> {
        if let Some(file_type) = infer().get_from_path(resource)? {
            let archive_type = match file_type.mime_type() {
                "application/gzip" if Self::is_tar(&mut GzDecoder::new(File::open(resource)?)) => {
                    Self::TarGz
                }
                "application/zip" => Self::Zip,
                tpe => {
                    return Err(Error::ExtractionError(format!(
                        "unsupported file format: {tpe}"
                    )))
                }
            };
            Ok(archive_type)
        } else {
            Err(Error::ExtractionError(
                "cannot determine archive file type".into(),
            ))
        }
    }
}

pub(crate) fn extract_archive<P: AsRef<Path>>(
    path: P,
    target: P,
    format: &ArchiveFormat,
) -> Result<(), Error> {
    // We'll first extract to a temp directory in the same parent as the target directory.
    let target_parent_dir = target.as_ref().parent().unwrap();
    let temp_target = tempdir_in(target_parent_dir)?;

    match format {
        ArchiveFormat::TarGz => {
            let tar_gz = File::open(path)?;
            let tar = GzDecoder::new(tar_gz);
            let mut archive = tar::Archive::new(tar);
            archive.unpack(&temp_target)?;
        }
        ArchiveFormat::Zip => {
            let file = File::open(path)?;
            let mut archive =
                zip::ZipArchive::new(file).map_err(|e| Error::ExtractionError(e.to_string()))?;
            archive
                .extract(temp_target.path())
                .map_err(|e| Error::ExtractionError(e.to_string()))?;
        }
    };

    // Now rename the temp directory to the final target directory.
    fs::rename(temp_target, target)?;

    Ok(())
}
