use anyhow::{Context, Result};
use redb::{Database, ReadableDatabase, ReadableTable, TableDefinition};
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::OnceLock;

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
const DB_FILE_NAME: &str = "strings.redb";

const COUNTER_TABLE: TableDefinition<&str, IndexType> = TableDefinition::new("counter");
const STRING_TO_INDEX_TABLE: TableDefinition<&str, IndexType> =
    TableDefinition::new("string_to_index");
const INDEX_TO_STRING_TABLE: TableDefinition<IndexType, &str> =
    TableDefinition::new("index_to_string");
const COUNTER_KEY: &str = "__counter__";

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
    // If the caller passes a directory (old LMDB layout), create/read the redb file inside it.
    // If a file path is provided (e.g. custom .redb path), use it directly.
    match base.extension() {
        Some(_) => base.to_path_buf(),
        None => base.join(DB_FILE_NAME),
    }
}

fn database() -> &'static Database {
    static DB: OnceLock<Database> = OnceLock::new();
    DB.get_or_init(|| {
        let base_dir = default_log_index_dir();
        let db_path = database_path(&base_dir);
        if let Some(parent) = db_path.parent() {
            fs::create_dir_all(parent).unwrap();
        }

        #[cfg(feature = "macro_debug")]
        {
            build_log!(
                "=================================================================================="
            );
            build_log!("Interned strings are stored in: {:?}", db_path);
            build_log!("   [r] is reused index and [n] is new index.");
            build_log!(
                "=================================================================================="
            );
        }

        Database::create(db_path).expect("Failed to open interned strings database")
    })
}

/// Reads all interned strings from the index at the specified path.
/// The index is created at compile time within your project output directory.
pub fn read_interned_strings(index: &Path) -> Result<Vec<String>> {
    let mut all_strings = Vec::<String>::new();
    let db_path = database_path(index);
    let db = Database::open(db_path).context("Could not open the string index. Check the path.")?;

    let read_txn = db.begin_read()?;
    let index_to_string = read_txn
        .open_table(INDEX_TO_STRING_TABLE)
        .context("Could not open the index_to_string table")?;
    for entry in index_to_string.iter()? {
        let (index, value) = entry?;
        let idx = index.value() as usize;
        if all_strings.len() <= idx {
            all_strings.resize(idx + 1, String::new());
        }
        all_strings[idx] = value.value().to_string();
    }
    Ok(all_strings)
}

pub fn intern_string(s: &str) -> Option<IndexType> {
    let db = database();

    // Fast path: see if the string already exists.
    if let Ok(read_txn) = db.begin_read() {
        if let Ok(string_to_index) = read_txn.open_table(STRING_TO_INDEX_TABLE) {
            if let Ok(Some(index)) = string_to_index.get(s) {
                let index = index.value();
                #[cfg(feature = "macro_debug")]
                {
                    build_log!("#{:0>3} [r] -> {}.", index, s);
                }
                return Some(index);
            }
        }
    }

    let write_txn = db.begin_write().ok()?;
    let next_index = {
        let mut counter_table = write_txn.open_table(COUNTER_TABLE).ok()?;
        let mut string_to_index = write_txn.open_table(STRING_TO_INDEX_TABLE).ok()?;
        let mut index_to_string = write_txn.open_table(INDEX_TO_STRING_TABLE).ok()?;

        let current_counter = counter_table
            .get(COUNTER_KEY)
            .ok()?
            .map(|v| v.value())
            .unwrap_or(0);
        let next_index = current_counter + 1;

        counter_table.insert(COUNTER_KEY, next_index).ok()?;
        index_to_string.insert(next_index, s).ok()?;
        string_to_index.insert(s, next_index).ok()?;
        next_index
    };

    write_txn.commit().ok()?;

    #[cfg(feature = "macro_debug")]
    {
        build_log!("#{:0>3} [n] -> {}.", next_index, s);
    }

    Some(next_index)
}

#[allow(dead_code)]
pub fn record_callsite(filename: &str, line_number: u32) -> Option<IndexType> {
    intern_string(format!("{filename}:{line_number}").as_str())
}
