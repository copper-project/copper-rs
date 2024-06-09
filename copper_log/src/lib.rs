use std::collections::HashMap;
use std::fmt::Display;
use bincode_derive::{Decode, Encode};
use serde::{Deserialize, Serialize};
use strfmt::strfmt;
pub use copper_value as value;
use value::Value;
use copper_traits::{CuError, CuResult};

#[allow(dead_code)]
pub const ANONYMOUS: u32 = 0;

/// This is the basic structure for a log entry in Copper.
#[derive(Debug, Encode, Decode, Serialize, Deserialize, PartialEq)]
pub struct CuLogEntry {
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
pub fn rebuild_logline(all_interned_strings: &Vec<String>, entry: CuLogEntry) -> CuResult<String> {
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

    // Use strfmt to replace named parameters
    strfmt(&format_string, &vars).map_err(|e| CuError::new_with_cause(format!("Failed to format log line: {:?} with {:?}", format_string, vars).as_str(), e))
}

