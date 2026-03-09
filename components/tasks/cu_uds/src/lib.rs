//! # cu_uds — Unified Diagnostic Services (ISO 14229) for Copper
//!
//! Provides UDS server and client tasks that sit on top of ISO-TP.
//!
//! ## Architecture
//! ```text
//! CanSource → IsotpCodec → UdsServer → IsotpCodec → CanSink
//! ```

#![cfg_attr(not(feature = "std"), no_std)]
extern crate alloc;

mod server;
mod client;

pub use server::UdsServer;
pub use client::UdsClient;
