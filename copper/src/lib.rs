pub mod config;
pub mod cutask;
pub mod serde;

// Copper common error type.
pub type CuError = String;
pub type CuResult<T> = std::result::Result<T, CuError>;
