#![doc = include_str!("../README.md")]

// backward compatibility
pub use cu29_runtime::config;
pub use cu29_runtime::copperlist;
pub use cu29_runtime::curuntime;
pub use cu29_runtime::cutask;
pub use cu29_runtime::input_msg;
pub use cu29_runtime::monitoring;
pub use cu29_runtime::output_msg;
pub use cu29_runtime::payload;
pub use cu29_runtime::simulation;

pub use cu29_clock as clock;
pub use cu29_runtime::config::read_configuration;
pub use cu29_traits::*;

pub mod prelude {
    pub use cu29_clock as clock;
    pub use cu29_log::*;
    pub use cu29_log_derive::*;
    pub use cu29_log_runtime::*;
    pub use cu29_runtime::config::*;
    pub use cu29_runtime::copperlist::*;
    pub use cu29_runtime::curuntime::*;
    pub use cu29_runtime::cutask::*;
    pub use cu29_runtime::input_msg;
    pub use cu29_runtime::monitoring::*;
    pub use cu29_runtime::output_msg;
    pub use cu29_runtime::payload::*;
    pub use cu29_runtime::simulation::*;
    pub use cu29_runtime::*;
    pub use cu29_traits::*;
}
