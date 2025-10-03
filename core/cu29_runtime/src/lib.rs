#![doc = include_str!("../README.md")]
#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(not(feature = "std"))]
extern crate alloc;

pub mod app;
pub mod config;
pub mod copperlist;
#[cfg(feature = "std")]
pub mod cuasynctask; // no no-std version yet
pub mod curuntime;
pub mod cutask;
pub(crate) mod log;
pub mod monitoring;
pub mod payload;
#[cfg(feature = "std")]
pub mod pool;
#[cfg(feature = "std")]
pub mod simulation;
