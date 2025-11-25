use anyhow::{Context, Result};
use bincode::config::standard;
use bincode::{decode_from_slice, encode_to_vec, Decode, Encode};
use fs2::FileExt;
use std::collections::HashMap;
use std::fs::{self, OpenOptions};
use std::io::{Read, Seek, SeekFrom, Write};
use std::path::{Path, PathBuf};

type IndexType = u32;

#[cfg(feature = "macro_debug")]
const COLORED_PREFIX_BUILD_LOG: &str = "\x1b[32mCLog:\x1b[0m";

#[cfg(feature = "macro_debug")]
macro_rules! build_log {
    ($($arg:tt)*) => {
        eprintln!("{} {}", COLORED_PREFIX_BUILD_LOG, format!($($arg)*));
    };
}

/// The name of the directory where the log index is stored.
const INDEX_DIR_NAME: &str = "cu29_log_index";
const DB_FILE_NAME: &str = "strings.bin";

#[derive(Encode, Decode, Default)]
struct InternDb {
    next_index: IndexType,
    strings: Vec<String>,
    string_to_index: HashMap<String, IndexType>,
}

impl InternDb {
    fn new() -> Self {
        Self {
            next_index: 1, // keep 0 reserved as before
            ..Default::default()
        }
    }
}

fn parent_n_times(path: &Path, n: usize) -> Option<PathBuf> {
    let mut result = Some(path.to_path_buf());
    for _ in 0..n {
        result = result?.parent().map(PathBuf::from);
    }
    result
}

/// Convenience function to returns the default path for the log index directory.
pub fn default_log_index_dir() -> PathBuf {
    let outdir = std::env::var("LOG_INDEX_DIR").expect("no LOG_INDEX_DIR system variable set, be sure build.rs sets it, see cu29_log/build.rs for example.");
    let outdir_path = Path::new(&outdir);
    parent_n_times(outdir_path, 3).unwrap().join(INDEX_DIR_NAME)
}

fn database_path(base: &Path) -> PathBuf {
    match base.extension() {
        Some(_) => base.to_path_buf(),
        None => base.join(DB_FILE_NAME),
    }
}

/// Reads all interned strings from the index at the specified path.
/// The index is created at compile time within your project output directory.
pub fn read_interned_strings(index: &Path) -> Result<Vec<String>> {
    let db_path = database_path(index);
    let db =
        load_db_shared(&db_path).context("Could not open the string index. Check the path.")?;
    Ok(db.strings)
}

pub fn intern_string(s: &str) -> Option<IndexType> {
    let base_dir = default_log_index_dir();
    let db_path = database_path(&base_dir);
    if let Some(parent) = db_path.parent() {
        fs::create_dir_all(parent).ok()?;
    }

    #[cfg(feature = "macro_debug")]
    log_db_info_once(&db_path);

    let mut file = OpenOptions::new()
        .read(true)
        .write(true)
        .create(true)
        .truncate(false)
        .open(&db_path)
        .ok()?;
    file.lock_exclusive().ok()?;

    let mut buf = Vec::new();
    file.read_to_end(&mut buf).ok()?;
    file.seek(SeekFrom::Start(0)).ok()?;

    let mut db = if buf.is_empty() {
        InternDb::new()
    } else {
        decode_from_slice(&buf, standard()).ok()?.0
    };

    if let Some(&idx) = db.string_to_index.get(s) {
        #[cfg(feature = "macro_debug")]
        {
            build_log!("#{:0>3} [r] -> {}.", idx, s);
        }
        return Some(idx);
    }

    let idx = db.next_index;
    let idx_usize = idx as usize;
    if db.strings.len() <= idx_usize {
        db.strings.resize(idx_usize + 1, String::new());
    }
    db.strings[idx_usize] = s.to_string();
    db.string_to_index.insert(s.to_string(), idx);
    db.next_index = db.next_index.checked_add(1)?;

    let encoded = encode_to_vec(&db, standard()).ok()?;
    file.set_len(0).ok()?;
    file.write_all(&encoded).ok()?;
    file.flush().ok()?;
    let _ = file.unlock();

    #[cfg(feature = "macro_debug")]
    {
        build_log!("#{:0>3} [n] -> {}.", idx, s);
    }

    Some(idx)
}

#[cfg(feature = "macro_debug")]
fn log_db_info_once(db_path: &Path) {
    use std::sync::OnceLock;
    static ONCE: OnceLock<()> = OnceLock::new();
    ONCE.get_or_init(|| {
        build_log!(
            "=================================================================================="
        );
        build_log!("Interned strings are stored in: {:?}", db_path);
        build_log!("   [r] is reused index and [n] is new index.");
        build_log!(
            "=================================================================================="
        );
    });
}

fn load_db_shared(path: &Path) -> std::result::Result<InternDb, anyhow::Error> {
    let mut file = match OpenOptions::new().read(true).open(path) {
        Ok(f) => f,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(InternDb::new()),
        Err(e) => return Err(e).context("Failed to open index file"),
    };
    file.lock_shared()
        .context("Failed to lock index for read")?;

    let mut buf = Vec::new();
    file.read_to_end(&mut buf)
        .context("Failed to read index file")?;
    let _ = file.unlock();

    if buf.is_empty() {
        return Ok(InternDb::new());
    }

    let (db, _): (InternDb, _) =
        decode_from_slice(&buf, standard()).context("Failed to decode index")?;
    Ok(db)
}

#[allow(dead_code)]
pub fn record_callsite(filename: &str, line_number: u32) -> Option<IndexType> {
    intern_string(format!("{filename}:{line_number}").as_str())
}
