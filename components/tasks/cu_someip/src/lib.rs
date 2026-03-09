//! # cu_someip — SOME/IP Tasks for Copper
//!
//! Provides SOME/IP UDP source, sink, and service discovery tasks
//! for the Copper runtime.
//!
//! ## Architecture
//!
//! ```text
//! SomeIpSource (UDP RX) → SomeIpRouter → SomeIpSink (UDP TX)
//! SomeIpSdMonitor (multicast listener) → notifies availability
//! ```
//!
//! ## Configuration (RON)
//! ```ron
//! (id: "someip_rx", type: "cu_someip::SomeIpSource", config: {
//!     "bind_addr": "0.0.0.0",
//!     "bind_port": 30509
//! })
//! (id: "someip_tx", type: "cu_someip::SomeIpSink", config: {
//!     "remote_addr": "192.168.1.100",
//!     "remote_port": 30509
//! })
//! ```

#![cfg_attr(not(feature = "std"), no_std)]
extern crate alloc;

mod source;
mod sink;
mod router;
mod sd;

pub use source::SomeIpSource;
pub use sink::SomeIpSink;
pub use router::SomeIpRouter;
pub use sd::SomeIpSdMonitor;
