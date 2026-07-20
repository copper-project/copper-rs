//! # Zenoh Protocol Codec
//!
//! This module provides the core encoding and decoding infrastructure for the Zenoh protocol.
//! It defines traits and utilities for serializing and deserializing protocol messages
//! in a zero-copy, `no_std` compatible manner.
//!
//! ## Overview
//!
//! The codec system is built around several key concepts:
//!
//! ### Core Traits
//!
//! - [`ZEncode`] / [`ZDecode`]: Main traits for encoding/decoding complete structures
//! - [`ZBodyEncode`] / [`ZBodyDecode`]: Traits for encoding/decoding just the body (without header)
//! - [`ZLen`] / [`ZBodyLen`]: Traits for calculating the encoded length of data
//! - [`ZHeader`]: Trait for types that have a header byte
//! - [`ZExt`]: Trait for protocol extensions with specific kinds (Unit, U64, or ZStruct)
//!
//! ### Extension System
//!
//! The extension system ([`ext`] module) allows protocol messages to include optional
//! extensions identified by an ID and kind. Extensions can be:
//! - **Unit**: No associated data (presence flag only)
//! - **U64**: Contains a variable-length encoded 64-bit integer
//! - **ZStruct**: Contains a structured type with its own encoding
//!
//! ## Usage with Derive Macros
//!
//! Types can automatically implement the codec traits using derive macros:
//!
//! ```ignore
//! use zenoh_proto::*;
//!
//! // Derive ZStruct for a protocol message
//! #[derive(ZStruct, Debug, PartialEq)]
//! #[zenoh(header = "_:3|ID:5=0x01")]
//! pub struct MyMessage {
//!     pub field1: u32,
//!     pub field2: u64,
//! }
//!
//! // Derive ZExt for a protocol extension
//! #[derive(ZExt, Debug, PartialEq)]
//! pub struct MyExtension {
//!     pub value: u64,
//! }
//! ```
//!
//! ## Encoding/Decoding Flow
//!
//! 1. **Length Calculation**: Use [`ZLen::z_len()`] to determine required buffer size
//! 2. **Encoding**: Use [`ZEncode::z_encode()`] to write data to a [`ZWriteable`] buffer
//! 3. **Decoding**: Use [`ZDecode::z_decode()`] to read data from a [`ZReadable`] buffer
//!
//! ## Example
//!
//! ```ignore
//! let msg = MyMessage { field1: 42, field2: 100 };
//!
//! // Calculate required size
//! let len = msg.z_len();
//!
//! // Encode
//! let mut buffer = vec![0u8; len];
//! msg.z_encode(&mut &mut buffer[..]).unwrap();
//!
//! // Decode
//! let decoded = MyMessage::z_decode(&mut &buffer[..]).unwrap();
//! assert_eq!(msg, decoded);
//! ```

pub mod ext;
pub mod r#struct;

pub use ext::*;
pub use r#struct::*;
