#![doc = include_str!("../README.md")]
#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(all(feature = "parallel-rt", not(feature = "std")))]
compile_error!("feature `parallel-rt` requires `std`");

extern crate alloc;

#[doc(hidden)]
pub use paste::paste as __cu29_paste;

pub mod app;
#[cfg(feature = "std")]
mod app_sim;
pub mod config;
pub mod context;
pub mod copperlist;
#[cfg(feature = "std")]
pub mod cuasynctask; // no no-std version yet
pub mod cubridge;
pub mod curuntime;
pub mod cutask;
#[cfg(feature = "std")]
pub mod debug;
#[cfg(feature = "std")]
pub mod distributed_replay;
pub(crate) mod log;
pub mod logcodec;
pub mod monitoring;
#[cfg(all(feature = "std", feature = "parallel-rt"))]
pub mod parallel_queue;
#[cfg(all(feature = "std", feature = "parallel-rt"))]
pub mod parallel_rt;
pub mod payload;
#[cfg(feature = "std")]
pub mod pool;
pub mod reflect;
#[cfg(feature = "remote-debug")]
pub mod remote_debug;
#[cfg(feature = "std")]
pub mod replay;
pub mod resource;
#[cfg(feature = "std")]
pub mod simulation;
