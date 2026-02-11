#![doc = include_str!("../README.md")]
#![cfg_attr(not(feature = "std"), no_std)]

extern crate alloc;

#[doc(hidden)]
pub use paste::paste as __cu29_paste;

pub mod app;
#[cfg(feature = "std")]
mod app_sim;
pub mod config;
pub mod copperlist;
#[cfg(feature = "std")]
pub mod cuasynctask; // no no-std version yet
pub mod cubridge;
pub mod curuntime;
pub mod cutask;
#[cfg(feature = "std")]
pub mod debug;
pub(crate) mod log;
pub mod monitoring;
pub mod payload;
#[cfg(feature = "std")]
pub mod pool;
pub mod reflect;
#[cfg(feature = "remote-debug")]
pub mod remote_debug;
pub mod resource;
#[cfg(feature = "std")]
pub mod simulation;
