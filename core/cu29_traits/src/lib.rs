#![cfg_attr(not(feature = "std"), no_std)]
extern crate alloc;

use alloc::string::ToString;
use arrayvec::ArrayString;
use bincode::{Decode as dDecode, Encode, Encode as dEncode};
use core::error::Error;
use core::fmt::{Debug, Display, Formatter, Result};
use core::result;
use serde::{Deserialize, Serialize};

/// Most structures in Copper are fixed size. This is a wrapper for the strings.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CuString<const N: usize>(pub ArrayString<N>);

impl<const N: usize> CuString<N> {
    #[allow(dead_code)]
    fn new() -> Self {
        Self(ArrayString::new())
    }
}

pub type CuStr16 = CuString<16>;
pub type CuStr32 = CuString<32>;
pub type CuStr64 = CuString<64>;
pub type CuStr128 = CuString<128>;

impl From<&str> for CuStr64 {
    /// This will be a safe operation by truncating to the capacity
    fn from(s: &str) -> Self {
        let mut out = ArrayString::new();
        out.push_str(&s[..s.len().min(64)]);
        CuString::<64>(out)
    }
}

#[cfg(feature = "std")]
impl From<String> for CuStr64 {
    fn from(value: String) -> Self {
        value.as_str().into()
    }
}

impl Display for CuStr64 {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        write!(f, "{}", self.0)
    }
}

impl From<&dyn Error> for CuStr64 {
    fn from(value: &dyn Error) -> Self {
        value.to_string().as_str().into()
    }
}

/// Common copper Error type.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CuError {
    message: CuStr64,
    cause: Option<CuStr64>,
}

impl Display for CuError {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        let context_str = match &self.cause {
            Some(c) => c,
            None => &"None".into(),
        };
        write!(f, "{}\n   context:{}", self.message, context_str)?;
        Ok(())
    }
}

impl Error for CuError {}

impl From<&str> for CuError {
    fn from(s: &str) -> CuError {
        CuError {
            message: s.into(),
            cause: None,
        }
    }
}

impl From<CuStr64> for CuError {
    fn from(s: CuStr64) -> CuError {
        CuError {
            message: s,
            cause: None,
        }
    }
}

#[cfg(feature = "std")]
impl From<String> for CuError {
    fn from(s: String) -> CuError {
        CuError {
            message: s.into(),
            cause: None,
        }
    }
}

impl CuError {
    pub fn new_with_cause(message: &str, cause: impl Error) -> CuError {
        CuError {
            message: message.into(),
            cause: Some(cause.to_string().as_str().into()),
        }
    }

    pub fn add_cause(mut self, context: &str) -> CuError {
        self.cause = Some(context.into());
        self
    }
}

// Generic Result type for copper.
pub type CuResult<T> = result::Result<T, CuError>;

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
