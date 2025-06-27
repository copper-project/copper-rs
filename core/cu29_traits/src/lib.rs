use bincode::{Decode as dDecode, Encode, Encode as dEncode};

use anyhow::Error as AnyhowError;
use core::error::Error;
use core::fmt::{Debug, Display};
use std::fmt::Formatter;

#[derive(Debug)]
pub struct CuError {
    id: u32,
    cause: Option<AnyhowError>,
}

impl Display for CuError {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        if let Some(cause) = &self.cause {
            write!(f, "CuError {}: {}", self.id, cause)
        } else {
            write!(f, "CuError {}", self.id)
        }
    }
}

impl Error for CuError {}

impl CuError {
    pub fn new(id: u32) -> Self {
        Self { id, cause: None }
    }

    pub fn with_cause(mut self, cause: impl Into<AnyhowError>) -> Self {
        self.cause = Some(cause.into());
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
    FrozenTasks,       // Log of all frozen state of the tasks.
    LastEntry,         // This is a special entry that is used to signal the end of the log.
}

/// A CopperListTuple needs to be encodable, decodable and fixed size in memory.
pub trait CopperListTuple: bincode::Encode + bincode::Decode<()> + Debug {} // Decode is Sized

// Also anything that follows this contract can be a payload (blanket implementation)
impl<T> CopperListTuple for T where T: bincode::Encode + bincode::Decode<()> + Debug {} // Decode is Sized
