use bincode::{Decode as dDecode, Encode, Encode as dEncode};
use std::error::Error;
use std::fmt::{Display, Formatter};

/// Common copper Error type.
#[derive(Debug)]
pub struct CuError {
    message: String,
    cause: Option<String>,
}

impl Display for CuError {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        let context_str = match &self.cause {
            Some(c) => format!("{}", c),
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
pub trait WriteStream: Sync + Send {
    fn log(&mut self, obj: &impl Encode) -> CuResult<()>;
}

#[derive(dEncode, dDecode, Copy, Clone, Debug, PartialEq)]
pub enum DataLogType {
    StructuredLogLine,
    CopperList,
    LastEntry, // This is a special entry that is used to signal the end of the log.
}
