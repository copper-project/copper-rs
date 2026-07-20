#![cfg_attr(not(feature = "std"), no_std)]

pub(crate) use zenoh_derive::*;

mod bytes;
mod codec;
mod endpoint;
mod ke;
mod zerror;

pub mod logging;
pub mod msgs;

pub use bytes::*;
pub use codec::*;
pub use endpoint::*;
pub use ke::*;
pub use msgs::{exts, fields};
pub use zerror::*;

#[cfg(test)]
mod tests;

/// Current version of the Zenoh protocol implementation.
pub const VERSION: u8 = 9;
