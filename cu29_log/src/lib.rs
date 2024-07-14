use bincode::{Decode, Encode};
use cu29_clock::CuTime;
use cu29_traits::{CuError, CuResult};
pub use cu29_value as value;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::fmt::Display;
use std::path::{Path, PathBuf};
use strfmt::strfmt;
use value::Value;

/// The name of the directory where the log index is stored.
const INDEX_DIR_NAME: &str = "cu29_log_index";

#[allow(dead_code)]
pub const ANONYMOUS: u32 = 0;

/// This is the basic structure for a log entry in Copper.
#[derive(Debug, Encode, Decode, Serialize, Deserialize, PartialEq)]
pub struct CuLogEntry {
    // Approximate time when the log entry was created.
    pub time: CuTime,

    // interned index of the message
    pub msg_index: u32,
    // interned indexes of the parameter names
    pub paramname_indexes: Vec<u32>,
    // Serializable values for the parameters (Values are acting like an Any Value).
    pub params: Vec<Value>,
}

// This is for internal debug purposes.
impl Display for CuLogEntry {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "CuLogEntry {{ msg_index: {}, paramname_indexes: {:?}, params: {:?} }}",
            self.msg_index, self.paramname_indexes, self.params
        )
    }
}

impl CuLogEntry {
    /// msg_index is the interned index of the message.
    pub fn new(msg_index: u32) -> Self {
        CuLogEntry {
            time: 0.into(), // We have no clock at that point it is called from random places
            // the clock will be set at actual log time from clock source provided
            msg_index,
            paramname_indexes: Vec::new(),
            params: Vec::new(),
        }
    }

    /// Add a parameter to the log entry.
    /// paramname_index is the interned index of the parameter name.
    pub fn add_param(&mut self, paramname_index: u32, param: Value) {
        self.paramname_indexes.push(paramname_index);
        self.params.push(param);
    }
}

/// Rebuild a log line from the interned strings and the CuLogEntry.
/// This basically translates the world of copper logs to text logs.
pub fn rebuild_logline(all_interned_strings: &Vec<String>, entry: &CuLogEntry) -> CuResult<String> {
    let mut format_string = all_interned_strings[entry.msg_index as usize].clone();
    let mut vars = HashMap::new();

    for (i, param) in entry.params.iter().enumerate() {
        let param_as_string = format!("{}", param);
        if entry.paramname_indexes[i] == 0 {
            // Anonymous parameter
            format_string = format_string.replacen("{}", &param_as_string, 1);
        } else {
            // Named parameter
            let name = all_interned_strings[entry.paramname_indexes[i] as usize].clone();
            vars.insert(name, param_as_string);
        }
    }

    if vars.is_empty() {
        return Ok(format_string);
    }

    // Use strfmt to replace named parameters
    strfmt(&format_string, &vars).map_err(|e| {
        CuError::new_with_cause(
            format!(
                "Failed to format log line: {:?} with variables [{:?}]",
                format_string, vars
            )
            .as_str(),
            e,
        )
    })
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
    let outdir = std::env::var("OUT_DIR").expect("no OUT_DIR set, build.rs must be broken");
    let outdir_path = Path::new(&outdir);
    let target_dir = parent_n_times(&outdir_path, 3)
        .unwrap()
        .join(INDEX_DIR_NAME);
    target_dir
}
