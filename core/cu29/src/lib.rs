#![doc = include_str!("../README.md")]
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
#[cfg(feature = "std")]
pub use cu29_runtime::simulation;

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
    pub use cu29_runtime::curuntime::*;
    pub use cu29_runtime::cutask::*;
    pub use cu29_runtime::input_msg;
    pub use cu29_runtime::monitoring::*;
    pub use cu29_runtime::output_msg;
    pub use cu29_runtime::payload::*;
    #[cfg(feature = "std")]
    pub use cu29_runtime::simulation::*;
    pub use cu29_runtime::*;
    pub use cu29_traits::*;
    pub use cu29_unifiedlog::*;
    pub use cu29_value::to_value;
    pub use cu29_value::Value;
    #[cfg(feature = "std")]
    pub use pool::*;
    pub use serde::Serialize;
}
