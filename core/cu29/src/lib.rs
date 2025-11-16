//! # Copper Runtime & SDK
//!
//! Think of Copper as a robotics game engine: define a task graph, compile once,
//! and get deterministic execution, unified logging, and sub-microsecond
//! latency from Linux workstations all the way down to bare-metal MPU builds.
//!
//! ## Quick start
//!
//! ```bash
//! cargo install cargo-generate
//! git clone https://github.com/copper-project/copper-rs
//! cd copper-rs/templates
//! cargo cunew /path/to/my_robot   # scaffolds a full Copper project
//! ```
//!
//! It will generate a minimal Copper robot project at `/path/to/my_robot`. `cargo build` should
//! compile it out of the box.
//!
//! ## Concepts behind Copper
//!
//! Check out the [Copper Wiki](https://github.com/copper-project/copper-rs/wiki) to understand the
//! deployments concepts, task lifecycle, available components, etc ...
//!
//! ## More examples to get you started
//!
//! - `examples/cu_caterpillar`: a minimal running example passing around booleans.  
//! - `examples/cu_rp_balancebot`: a more complete example try Copper without hardware via
//!   `cargo install cu-rp-balancebot` + `balancebot-sim` (Bevy + Avian3d).
//!
//! ## Key traits and structs to check out
//!
//! - `cu29_runtime::app::CuApp`: the main trait the copper runtime will expose to run your application. (when run() etc .. is coming from)
//! - `cu29_runtime::config::CuConfig`: the configuration of your runtime
//! - `cu29_runtime::cutask::CuTask`: the core trait and helpers to implement your own tasks.
//! - `cu29_runtime::cubridge::CuBridge`: the trait to implement bridges to hardware or other software.
//! - `cu29_runtime::curuntime::CuRuntime`: the runtime that manages task execution.
//! - `cu29_runtime::simulation`: This will explain how to hook up your tasks to a simulation environment.
//!
//! Need help or want to show what you're building? Join
//! [Discord](https://discord.gg/VkCG7Sb9Kw) and hop into the #general channel.
//!

#![cfg_attr(not(feature = "std"), no_std)]
#[cfg(not(feature = "std"))]
extern crate alloc;

pub use cu29_runtime::config;
pub use cu29_runtime::copperlist;
#[cfg(feature = "std")]
pub use cu29_runtime::cuasynctask;
pub use cu29_runtime::cubridge;
pub use cu29_runtime::curuntime;
pub use cu29_runtime::cutask;
pub use cu29_runtime::input_msg;
pub use cu29_runtime::monitoring;
pub use cu29_runtime::output_msg;
pub use cu29_runtime::payload;
pub use cu29_runtime::rx_channels;
#[cfg(feature = "std")]
pub use cu29_runtime::simulation;
pub use cu29_runtime::tx_channels;

pub use bincode;
pub use cu29_clock as clock;
#[cfg(feature = "std")]
pub use cu29_runtime::config::read_configuration;
pub use cu29_traits::*;

#[cfg(feature = "std")]
pub use rayon;

pub mod prelude {
    #[cfg(feature = "std")]
    pub use ctrlc;
    pub use cu29_clock::*;
    pub use cu29_derive::*;
    pub use cu29_log::*;
    pub use cu29_log_derive::*;
    pub use cu29_log_runtime::*;
    pub use cu29_runtime::app::*;
    pub use cu29_runtime::config::*;
    pub use cu29_runtime::copperlist::*;
    pub use cu29_runtime::cubridge::*;
    pub use cu29_runtime::curuntime::*;
    pub use cu29_runtime::cutask::*;
    pub use cu29_runtime::input_msg;
    pub use cu29_runtime::monitoring::*;
    pub use cu29_runtime::output_msg;
    pub use cu29_runtime::payload::*;
    pub use cu29_runtime::rx_channels;
    #[cfg(feature = "std")]
    pub use cu29_runtime::simulation::*;
    pub use cu29_runtime::tx_channels;
    pub use cu29_runtime::*;
    pub use cu29_traits::*;
    pub use cu29_unifiedlog::*;
    pub use cu29_value::to_value;
    pub use cu29_value::Value;
    #[cfg(feature = "std")]
    pub use pool::*;
    pub use serde::Serialize;
}
