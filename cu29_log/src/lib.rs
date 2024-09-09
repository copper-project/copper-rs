use bincode::{Decode, Encode};
use cu29_clock::CuTime;
use cu29_traits::{CuError, CuResult};
pub use cu29_value as value;
use serde::{Deserialize, Serialize};
use smallvec::SmallVec;
use std::collections::HashMap;
use std::fmt::Display;
use std::path::{Path, PathBuf};
use strfmt::strfmt;
use value::Value;

/// The name of the directory where the log index is stored.
const INDEX_DIR_NAME: &str = "cu29_log_index";

#[allow(dead_code)]
pub const ANONYMOUS: u32 = 0;

pub const MAX_LOG_PARAMS_ON_STACK: usize = 10;

/// This is the basic structure for a log entry in Copper.
#[derive(Debug, Serialize, Deserialize, PartialEq)]
pub struct CuLogEntry {
    // Approximate time when the log entry was created.
    pub time: CuTime,

    // interned index of the message
    pub msg_index: u32,

    // interned indexes of the parameter names
    pub paramname_indexes: SmallVec<[u32; MAX_LOG_PARAMS_ON_STACK]>,

    // Serializable values for the parameters (Values are acting like an Any Value).
    pub params: SmallVec<[Value; MAX_LOG_PARAMS_ON_STACK]>,
}

impl Encode for CuLogEntry {
    fn encode<E: bincode::enc::Encoder>(
        &self,
        encoder: &mut E,
    ) -> Result<(), bincode::error::EncodeError> {
        self.time.encode(encoder)?;
        self.msg_index.encode(encoder)?;

        (self.paramname_indexes.len() as u64).encode(encoder)?;
        for &index in &self.paramname_indexes {
            index.encode(encoder)?;
        }

        (self.params.len() as u64).encode(encoder)?;
        for param in &self.params {
            param.encode(encoder)?;
        }

        Ok(())
    }
}

impl Decode for CuLogEntry {
    fn decode<D: bincode::de::Decoder>(
        decoder: &mut D,
    ) -> Result<Self, bincode::error::DecodeError> {
        let time = CuTime::decode(decoder)?;
        let msg_index = u32::decode(decoder)?;

        let paramname_len = u64::decode(decoder)? as usize;
        let mut paramname_indexes = SmallVec::with_capacity(paramname_len);
        for _ in 0..paramname_len {
            paramname_indexes.push(u32::decode(decoder)?);
        }

        let params_len = u64::decode(decoder)? as usize;
        let mut params = SmallVec::with_capacity(params_len);
        for _ in 0..params_len {
            params.push(Value::decode(decoder)?);
        }

        Ok(CuLogEntry {
            time,
            msg_index,
            paramname_indexes,
            params,
        })
    }
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
            paramname_indexes: SmallVec::new(),
            params: SmallVec::new(),
        }
    }

    /// Add a parameter to the log entry.
    /// paramname_index is the interned index of the parameter name.
    pub fn add_param(&mut self, paramname_index: u32, param: Value) {
        self.paramname_indexes.push(paramname_index);
        self.params.push(param);
    }
}

/// Text log line formatter.
#[inline]
pub fn format_logline(
    time: CuTime,
    format_str: &str,
    params: &[String],
    named_params: &HashMap<String, String>,
) -> CuResult<String> {
    let mut format_str = format_str.to_string();

    for param in params.iter() {
        format_str = format_str.replacen("{}", &param, 1);
    }

    if named_params.is_empty() {
        return Ok(format_str);
    }

    let logline = strfmt(&format_str, &named_params).map_err(|e| {
        CuError::new_with_cause(
            format!(
                "Failed to format log line: {:?} with variables [{:?}]",
                format_str, named_params
            )
            .as_str(),
            e,
        )
    })?;
    Ok(format!("{}: {}", time, logline))
}

/// Rebuild a log line from the interned strings and the CuLogEntry.
/// This basically translates the world of copper logs to text logs.
pub fn rebuild_logline(all_interned_strings: &Vec<String>, entry: &CuLogEntry) -> CuResult<String> {
    let format_string = &all_interned_strings[entry.msg_index as usize];
    let mut anon_params: Vec<String> = Vec::new();
    let mut named_params = HashMap::new();

    for (i, param) in entry.params.iter().enumerate() {
        let param_as_string = format!("{}", param);
        if entry.paramname_indexes[i] == 0 {
            // Anonymous parameter
            anon_params.push(param_as_string);
        } else {
            // Named parameter
            let name = all_interned_strings[entry.paramname_indexes[i] as usize].clone();
            named_params.insert(name, param_as_string);
        }
    }
    format_logline(entry.time, format_string, &anon_params, &named_params)
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
    let target_dir = parent_n_times(&outdir_path, 3)
        .unwrap()
        .join(INDEX_DIR_NAME);
    target_dir
}
