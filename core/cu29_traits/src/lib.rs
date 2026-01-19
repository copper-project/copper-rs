//! Common copper traits and types for robotics systems.
//!
//! This crate is no_std compatible by default. Enable the "std" feature for additional
//! functionality like implementing `std::error::Error` for `CuError` and the
//! `new_with_cause` method that accepts types implementing `std::error::Error`.
//!
//! # Features
//!
//! - `std` (default): Enables standard library support
//!   - Implements `std::error::Error` for `CuError`
//!   - Adds `CuError::new_with_cause()` method for interop with std error types
//!
//! # no_std Usage
//!
//! To use without the standard library:
//!
//! ```toml
//! [dependencies]
//! cu29-traits = { version = "0.9", default-features = false }
//! ```

#![cfg_attr(not(feature = "std"), no_std)]
extern crate alloc;

use bincode::de::{BorrowDecoder, Decoder};
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{BorrowDecode, Decode as dDecode, Decode, Encode, Encode as dEncode};
use compact_str::CompactString;
use cu29_clock::{PartialCuTimeRange, Tov};
use serde::{Deserialize, Serialize};

use alloc::boxed::Box;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::vec::Vec;
#[cfg(not(feature = "std"))]
use core::error::Error as CoreError;
use core::fmt::{Debug, Display, Formatter};
#[cfg(feature = "std")]
use std::error::Error;

// Type alias for the boxed error type to simplify conditional compilation
#[cfg(feature = "std")]
type DynError = dyn std::error::Error + Send + Sync + 'static;
#[cfg(not(feature = "std"))]
type DynError = dyn core::error::Error + Send + Sync + 'static;

/// A simple wrapper around String that implements Error trait.
/// Used for cloning and deserializing CuError causes.
#[derive(Debug)]
struct StringError(String);

impl Display for StringError {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        write!(f, "{}", self.0)
    }
}

#[cfg(feature = "std")]
impl std::error::Error for StringError {}

#[cfg(not(feature = "std"))]
impl core::error::Error for StringError {}

/// Common copper Error type.
///
/// This error type stores an optional cause as a boxed dynamic error,
/// allowing for proper error chaining while maintaining Clone and
/// Serialize/Deserialize support through custom implementations.
pub struct CuError {
    message: String,
    cause: Option<Box<DynError>>,
}

// Custom Debug implementation that formats cause as string
impl Debug for CuError {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("CuError")
            .field("message", &self.message)
            .field("cause", &self.cause.as_ref().map(|e| e.to_string()))
            .finish()
    }
}

// Custom Clone implementation - clones cause as StringError wrapper
impl Clone for CuError {
    fn clone(&self) -> Self {
        CuError {
            message: self.message.clone(),
            cause: self
                .cause
                .as_ref()
                .map(|e| Box::new(StringError(e.to_string())) as Box<DynError>),
        }
    }
}

// Custom Serialize - serializes cause as Option<String>
impl Serialize for CuError {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        use serde::ser::SerializeStruct;
        let mut state = serializer.serialize_struct("CuError", 2)?;
        state.serialize_field("message", &self.message)?;
        state.serialize_field("cause", &self.cause.as_ref().map(|e| e.to_string()))?;
        state.end()
    }
}

// Custom Deserialize - deserializes cause as StringError wrapper
impl<'de> Deserialize<'de> for CuError {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct CuErrorHelper {
            message: String,
            cause: Option<String>,
        }

        let helper = CuErrorHelper::deserialize(deserializer)?;
        Ok(CuError {
            message: helper.message,
            cause: helper
                .cause
                .map(|s| Box::new(StringError(s)) as Box<DynError>),
        })
    }
}

impl Display for CuError {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        let context_str = match &self.cause {
            Some(c) => c.to_string(),
            None => "None".to_string(),
        };
        write!(f, "{}\n   context:{}", self.message, context_str)?;
        Ok(())
    }
}

#[cfg(not(feature = "std"))]
impl CoreError for CuError {
    fn source(&self) -> Option<&(dyn CoreError + 'static)> {
        self.cause
            .as_deref()
            .map(|e| e as &(dyn CoreError + 'static))
    }
}

#[cfg(feature = "std")]
impl Error for CuError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        self.cause.as_deref().map(|e| e as &(dyn Error + 'static))
    }
}

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
    /// Creates a new CuError from an interned string index.
    /// Used by the cu_error! macro.
    ///
    /// The index is stored as a placeholder string `[interned:{index}]`.
    /// Actual string resolution happens at logging time via the unified logger.
    pub fn new(message_index: usize) -> CuError {
        CuError {
            message: format!("[interned:{}]", message_index),
            cause: None,
        }
    }

    /// Creates a new CuError with a message and an underlying cause.
    ///
    /// # Example
    /// ```
    /// use cu29_traits::CuError;
    ///
    /// let io_err = std::io::Error::other("io error");
    /// let err = CuError::new_with_cause("Failed to read file", io_err);
    /// ```
    #[cfg(feature = "std")]
    pub fn new_with_cause<E>(message: &str, cause: E) -> CuError
    where
        E: std::error::Error + Send + Sync + 'static,
    {
        CuError {
            message: message.to_string(),
            cause: Some(Box::new(cause)),
        }
    }

    /// Creates a new CuError with a message and an underlying cause.
    #[cfg(not(feature = "std"))]
    pub fn new_with_cause<E>(message: &str, cause: E) -> CuError
    where
        E: core::error::Error + Send + Sync + 'static,
    {
        CuError {
            message: message.to_string(),
            cause: Some(Box::new(cause)),
        }
    }

    /// Adds or replaces the cause with a context string.
    ///
    /// This is useful for adding context to errors during propagation.
    ///
    /// # Example
    /// ```
    /// use cu29_traits::CuError;
    ///
    /// let err = CuError::from("base error").add_cause("additional context");
    /// ```
    pub fn add_cause(mut self, context: &str) -> CuError {
        self.cause = Some(Box::new(StringError(context.to_string())));
        self
    }

    /// Adds a cause error to this CuError (builder pattern).
    ///
    /// # Example
    /// ```
    /// use cu29_traits::CuError;
    ///
    /// let io_err = std::io::Error::other("io error");
    /// let err = CuError::from("Operation failed").with_cause(io_err);
    /// ```
    #[cfg(feature = "std")]
    pub fn with_cause<E>(mut self, cause: E) -> CuError
    where
        E: std::error::Error + Send + Sync + 'static,
    {
        self.cause = Some(Box::new(cause));
        self
    }

    /// Adds a cause error to this CuError (builder pattern).
    #[cfg(not(feature = "std"))]
    pub fn with_cause<E>(mut self, cause: E) -> CuError
    where
        E: core::error::Error + Send + Sync + 'static,
    {
        self.cause = Some(Box::new(cause));
        self
    }

    /// Returns a reference to the underlying cause, if any.
    pub fn cause(&self) -> Option<&(dyn core::error::Error + Send + Sync + 'static)> {
        self.cause.as_deref()
    }

    /// Returns the error message.
    pub fn message(&self) -> &str {
        &self.message
    }
}

/// Creates a CuError with a message and cause in a single call.
///
/// This is a convenience function for use with `.map_err()`.
///
/// # Example
/// ```
/// use cu29_traits::with_cause;
///
/// let result: Result<(), std::io::Error> = Err(std::io::Error::other("io error"));
/// let cu_result = result.map_err(|e| with_cause("Failed to read file", e));
/// ```
#[cfg(feature = "std")]
pub fn with_cause<E>(message: &str, cause: E) -> CuError
where
    E: std::error::Error + Send + Sync + 'static,
{
    CuError::new_with_cause(message, cause)
}

/// Creates a CuError with a message and cause in a single call.
#[cfg(not(feature = "std"))]
pub fn with_cause<E>(message: &str, cause: E) -> CuError
where
    E: core::error::Error + Send + Sync + 'static,
{
    CuError::new_with_cause(message, cause)
}

// Generic Result type for copper.
pub type CuResult<T> = Result<T, CuError>;

/// Defines a basic write, append only stream trait to be able to log or send serializable objects.
pub trait WriteStream<E: Encode>: Debug + Send + Sync {
    fn log(&mut self, obj: &E) -> CuResult<()>;
    fn flush(&mut self) -> CuResult<()> {
        Ok(())
    }
    /// Optional byte count of the last successful `log` call, if the implementation can report it.
    fn last_log_bytes(&self) -> Option<usize> {
        None
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
/// Represent the minimum set of traits to be usable as Metadata in Copper.
pub trait Metadata: Default + Debug + Clone + Encode + Decode<()> + Serialize {}

impl Metadata for () {}

/// Key metadata piece attached to every message in Copper.
pub trait CuMsgMetadataTrait {
    /// The time range used for the processing of this message
    fn process_time(&self) -> PartialCuTimeRange;

    /// Small status text for user UI to get the realtime state of task (max 24 chrs)
    fn status_txt(&self) -> &CuCompactString;
}

/// A generic trait to expose the generated CuStampedDataSet from the task graph.
pub trait ErasedCuStampedData {
    fn payload(&self) -> Option<&dyn erased_serde::Serialize>;
    fn tov(&self) -> Tov;
    fn metadata(&self) -> &dyn CuMsgMetadataTrait;
}

/// Trait to get a vector of type-erased CuStampedDataSet
/// This is used for generic serialization of the copperlists
pub trait ErasedCuStampedDataSet {
    fn cumsgs(&self) -> Vec<&dyn ErasedCuStampedData>;
}

/// Trait to trace back from the CopperList the origin of the messages
pub trait MatchingTasks {
    fn get_all_task_ids() -> &'static [&'static str];
}

/// Trait for providing JSON schemas for CopperList payload types.
///
/// This trait is implemented by the generated CuMsgs type via the `gen_cumsgs!` macro
/// when MCAP export support is enabled. It provides compile-time schema information
/// for each task's payload type, enabling proper schema generation for Foxglove.
///
/// The default implementation returns an empty vector for backwards compatibility
/// with code that doesn't need MCAP export support.
pub trait PayloadSchemas {
    /// Returns a vector of (task_id, schema_json) pairs.
    ///
    /// Each entry corresponds to a task in the CopperList, in order.
    /// The schema is a JSON Schema string generated from the payload type.
    fn get_payload_schemas() -> Vec<(&'static str, String)> {
        Vec::new()
    }
}

/// A CopperListTuple needs to be encodable, decodable and fixed size in memory.
pub trait CopperListTuple:
    bincode::Encode
    + bincode::Decode<()>
    + Debug
    + Serialize
    + ErasedCuStampedDataSet
    + MatchingTasks
    + Default
{
} // Decode forces Sized already

// Also anything that follows this contract can be a payload (blanket implementation)
impl<T> CopperListTuple for T where
    T: bincode::Encode
        + bincode::Decode<()>
        + Debug
        + Serialize
        + ErasedCuStampedDataSet
        + MatchingTasks
        + Default
{
}

// We use this type to convey very small status messages.
// MAX_SIZE from their repr module is not accessible so we need to copy paste their definition for 24
// which is the maximum size for inline allocation (no heap)
pub const COMPACT_STRING_CAPACITY: usize = size_of::<String>();

#[derive(Clone, Default, Serialize, Deserialize, PartialEq, Eq)]
pub struct CuCompactString(pub CompactString);

impl Encode for CuCompactString {
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        let CuCompactString(compact_string) = self;
        let bytes = &compact_string.as_bytes();
        bytes.encode(encoder)
    }
}

impl Debug for CuCompactString {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        if self.0.is_empty() {
            return write!(f, "CuCompactString(Empty)");
        }
        write!(f, "CuCompactString({})", self.0)
    }
}

impl<Context> Decode<Context> for CuCompactString {
    fn decode<D: Decoder>(decoder: &mut D) -> Result<Self, DecodeError> {
        let bytes = <Vec<u8> as Decode<D::Context>>::decode(decoder)?; // Decode into a byte buffer
        let compact_string =
            CompactString::from_utf8(bytes).map_err(|e| DecodeError::Utf8 { inner: e })?;
        Ok(CuCompactString(compact_string))
    }
}

impl<'de, Context> BorrowDecode<'de, Context> for CuCompactString {
    fn borrow_decode<D: BorrowDecoder<'de>>(decoder: &mut D) -> Result<Self, DecodeError> {
        CuCompactString::decode(decoder)
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for CuError {
    fn format(&self, f: defmt::Formatter) {
        match &self.cause {
            Some(c) => {
                let cause_str = c.to_string();
                defmt::write!(
                    f,
                    "CuError {{ message: {}, cause: {} }}",
                    defmt::Display2Format(&self.message),
                    defmt::Display2Format(&cause_str),
                )
            }
            None => defmt::write!(
                f,
                "CuError {{ message: {}, cause: None }}",
                defmt::Display2Format(&self.message),
            ),
        }
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for CuCompactString {
    fn format(&self, f: defmt::Formatter) {
        if self.0.is_empty() {
            defmt::write!(f, "CuCompactString(Empty)");
        } else {
            defmt::write!(f, "CuCompactString({})", defmt::Display2Format(&self.0));
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::CuCompactString;
    use bincode::{config, decode_from_slice, encode_to_vec};
    use compact_str::CompactString;

    #[test]
    fn test_cucompactstr_encode_decode_empty() {
        let cstr = CuCompactString(CompactString::from(""));
        let config = config::standard();
        let encoded = encode_to_vec(&cstr, config).expect("Encoding failed");
        assert_eq!(encoded.len(), 1); // This encodes the usize 0 in variable encoding so 1 byte which is 0.
        let (decoded, _): (CuCompactString, usize) =
            decode_from_slice(&encoded, config).expect("Decoding failed");
        assert_eq!(cstr.0, decoded.0);
    }

    #[test]
    fn test_cucompactstr_encode_decode_small() {
        let cstr = CuCompactString(CompactString::from("test"));
        let config = config::standard();
        let encoded = encode_to_vec(&cstr, config).expect("Encoding failed");
        assert_eq!(encoded.len(), 5); // This encodes a 4-byte string "test" plus 1 byte for the length prefix.
        let (decoded, _): (CuCompactString, usize) =
            decode_from_slice(&encoded, config).expect("Decoding failed");
        assert_eq!(cstr.0, decoded.0);
    }
}

// Tests that require std feature
#[cfg(all(test, feature = "std"))]
mod std_tests {
    use crate::{CuError, with_cause};

    #[test]
    fn test_cuerror_from_str() {
        let err = CuError::from("test error");
        assert_eq!(err.message(), "test error");
        assert!(err.cause().is_none());
    }

    #[test]
    fn test_cuerror_from_string() {
        let err = CuError::from(String::from("test error"));
        assert_eq!(err.message(), "test error");
        assert!(err.cause().is_none());
    }

    #[test]
    fn test_cuerror_new_index() {
        let err = CuError::new(42);
        assert_eq!(err.message(), "[interned:42]");
        assert!(err.cause().is_none());
    }

    #[test]
    fn test_cuerror_new_with_cause() {
        let io_err = std::io::Error::other("io error");
        let err = CuError::new_with_cause("wrapped error", io_err);
        assert_eq!(err.message(), "wrapped error");
        assert!(err.cause().is_some());
        assert!(err.cause().unwrap().to_string().contains("io error"));
    }

    #[test]
    fn test_cuerror_add_cause() {
        let err = CuError::from("base error").add_cause("additional context");
        assert_eq!(err.message(), "base error");
        assert!(err.cause().is_some());
        assert_eq!(err.cause().unwrap().to_string(), "additional context");
    }

    #[test]
    fn test_cuerror_with_cause_method() {
        let io_err = std::io::Error::other("io error");
        let err = CuError::from("base error").with_cause(io_err);
        assert_eq!(err.message(), "base error");
        assert!(err.cause().is_some());
    }

    #[test]
    fn test_cuerror_with_cause_free_function() {
        let io_err = std::io::Error::other("io error");
        let err = with_cause("wrapped", io_err);
        assert_eq!(err.message(), "wrapped");
        assert!(err.cause().is_some());
    }

    #[test]
    fn test_cuerror_clone() {
        let io_err = std::io::Error::other("io error");
        let err = CuError::new_with_cause("test", io_err);
        let cloned = err.clone();
        assert_eq!(err.message(), cloned.message());
        // Cause string representation should match
        assert_eq!(
            err.cause().map(|c| c.to_string()),
            cloned.cause().map(|c| c.to_string())
        );
    }

    #[test]
    fn test_cuerror_serialize_deserialize_json() {
        let io_err = std::io::Error::other("io error");
        let err = CuError::new_with_cause("test", io_err);

        let serialized = serde_json::to_string(&err).unwrap();
        let deserialized: CuError = serde_json::from_str(&serialized).unwrap();

        assert_eq!(err.message(), deserialized.message());
        // Cause should be preserved as string
        assert!(deserialized.cause().is_some());
    }

    #[test]
    fn test_cuerror_serialize_deserialize_no_cause() {
        let err = CuError::from("simple error");

        let serialized = serde_json::to_string(&err).unwrap();
        let deserialized: CuError = serde_json::from_str(&serialized).unwrap();

        assert_eq!(err.message(), deserialized.message());
        assert!(deserialized.cause().is_none());
    }

    #[test]
    fn test_cuerror_display() {
        let err = CuError::from("test error").add_cause("some context");
        let display = format!("{}", err);
        assert!(display.contains("test error"));
        assert!(display.contains("some context"));
    }

    #[test]
    fn test_cuerror_debug() {
        let err = CuError::from("test error").add_cause("some context");
        let debug = format!("{:?}", err);
        assert!(debug.contains("test error"));
        assert!(debug.contains("some context"));
    }
}
