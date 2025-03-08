use bincode::{Decode as dDecode, Encode, Encode as dEncode};
use serde::{Deserialize, Serialize};
use std::error::Error;
use std::fmt::{Debug, Display, Formatter};

/// Common copper Error type.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CuError {
    message: String,
    cause: Option<String>,
}

impl Display for CuError {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        let context_str = match &self.cause {
            Some(c) => c.to_string(),
            None => "None".to_string(),
        };
        write!(f, "{}\n   context:{}", self.message, context_str)?;
        Ok(())
    }
}

impl Error for CuError {}

impl From<&str> for CuError {
    fn from(s: &str) -> CuError {
        CuError {
            message: s.to_string(),
            cause: None,
        }
    }
}

impl From<String> for CuError {
    fn from(s: String) -> CuError {
        CuError {
            message: s,
            cause: None,
        }
    }
}

impl CuError {
    pub fn new_with_cause(message: &str, cause: impl Error) -> CuError {
        CuError {
            message: message.to_string(),
            cause: Some(cause.to_string()),
        }
    }

    pub fn add_cause(mut self, context: &str) -> CuError {
        self.cause = Some(context.into());
        self
    }
}

// Generic Result type for copper.
pub type CuResult<T> = Result<T, CuError>;

/// Defines a basic write, append only stream trait to be able to log or send serializable objects.
pub trait WriteStream<E: Encode>: Sync + Send + Debug {
    fn log(&mut self, obj: &E) -> CuResult<()>;
    fn flush(&mut self) -> CuResult<()> {
        Ok(())
    }
}

/// Defines the types of what can be logged in the unified logger.
#[derive(dEncode, dDecode, Copy, Clone, Debug, PartialEq)]
pub enum UnifiedLogType {
    Empty,             // Dummy default used as a debug marker
    StructuredLogLine, // This is for the structured logs (ie. debug! etc..)
    CopperList,        // This is the actual data log storing activities between tasks.
    LastEntry,         // This is a special entry that is used to signal the end of the log.
}

/// A CopperListTuple needs to be encodable, decodable and fixed size in memory.
pub trait CopperListTuple: bincode::Encode + bincode::Decode<()> + Debug {} // Decode is Sized

// Also anything that follows this contract can be a payload (blanket implementation)
impl<T> CopperListTuple for T where T: bincode::Encode + bincode::Decode<()> + Debug {} // Decode is Sized
