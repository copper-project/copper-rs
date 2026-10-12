//! Experimental PTP reference adapters for Copper's shared execution clock.
//!
//! Linux users configure `LinuxPtpBundle` with `linux-phc`. Board providers
//! implement [`cu29::clock_sync::ClockReference`] and export an owned reference
//! through [`cu29::clock_sync::ClockReferenceBundle`].

#![cfg_attr(not(feature = "std"), no_std)]

mod board;
pub use board::{BoardPtp, BoardPtpHooks};

#[cfg(all(feature = "linux-phc", target_os = "linux"))]
mod linux;
#[cfg(all(feature = "linux-phc", target_os = "linux"))]
pub use linux::{LinuxPhc, LinuxPtpBundle, LinuxSystemPtp, LinuxSystemPtpBundle};
