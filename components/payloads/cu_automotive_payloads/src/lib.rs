//! # cu_automotive_payloads — Copper Automotive Payload Types
//!
//! Shared message payload types for automotive communication protocols used within the
//! Copper runtime. Every type in this crate satisfies [`CuMsgPayload`] so it can flow
//! through CopperLists and be serialized to the unified log.
//!
//! ## Protocols covered
//!
//! | Module | Standard |
//! |--------|----------|
//! | [`can`] | ISO 11898 — Classical CAN & CAN FD frames |
//! | [`isotp`] | ISO 15765-2 — CAN Transport Protocol (PDU) |
//! | [`uds`] | ISO 14229 — Unified Diagnostic Services |
//! | [`someip`] | AUTOSAR SOME/IP header, messages, SD entries |

#![cfg_attr(not(feature = "std"), no_std)]
extern crate alloc;

pub mod can;
pub mod isotp;
pub mod someip;
pub mod uds;

pub use can::*;
pub use isotp::*;
pub use someip::*;
pub use uds::*;
