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
    /// Returns the string and number of truncated characters
    #[allow(dead_code)]
    fn new_truncated(s: &str) -> (Self, usize) {
        let cu_string = Self::from(s);

        if s.len() > N {
            // Input string is longer than our fixed size of N
            (cu_string, s.len() - N)
        } else {
            // Our fixed size of N is equal to or longer than the input string
            (cu_string, 0)
        }
    }
}

pub type CuStr16 = CuString<16>;
pub type CuStr32 = CuString<32>;
pub type CuStr64 = CuString<64>;
pub type CuStr128 = CuString<128>;

impl<const N: usize> From<&str> for CuString<N> {
    /// This will be a safe operation by truncating to the capacity
    fn from(s: &str) -> Self {
        let mut out = ArrayString::new();
        out.push_str(&s[..s.len().min(N)]);
        CuString::<N>(out)
    }
}

impl<const N: usize> Display for CuString<N> {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        write!(f, "{}", self.0)
    }
}

#[cfg(feature = "std")]
impl From<String> for CuStr64 {
    fn from(value: String) -> Self {
        value.as_str().into()
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

#[cfg(test)]
mod tests {

    use super::*;
    #[test]
    fn test_cu_string() {
        let my_str = CuString::<5>::from("hello");
        assert!(format!("{}", my_str) == "hello");
    }
    #[test]
    fn test_new_truncated_lost_chars() {
        let (my_str, n_truncated_chars) = CuString::<1>::new_truncated("hello");
        assert!(format!("{}", my_str) == "h");
        assert!(n_truncated_chars == 4);
    }
    #[test]
    fn test_new_truncated_no_lost_chars() {
        let (my_str, n_truncated_chars) = CuString::<50>::new_truncated("hello");
        assert!(format!("{}", my_str) == "hello");
        assert!(n_truncated_chars == 0);
    }
    #[test]
    fn test_new_truncated_equal_size() {
        let (my_str, n_truncated_chars) = CuString::<1>::new_truncated("h");
        assert!(format!("{}", my_str) == "h");
        assert!(n_truncated_chars == 0);
    }
}
