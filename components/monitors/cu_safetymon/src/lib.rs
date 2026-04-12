#![cfg_attr(not(feature = "std"), no_std)]
//! Safety monitor for fault latching and fail-fast process termination.
//!
//! `CuSafetyMon` is designed for safety-critical deployments where "keep running"
//! is less important than deterministic fault containment.
//!
//! Behavior:
//! - Maintains a CopperList heartbeat (`process_copperlist`) and exits if no progress
//!   is observed within `copperlist_deadline_ms`.
//! - Consumes the runtime execution probe to report the last known
//!   `(component_id, step, culistid)` when a lock is detected.
//! - Registers with Copper's shared panic hook and turns panics into explicit process exit codes.
//! - Latches runtime errors (`process_error`) as shutdown faults with their own exit code.
//!   The `process_error` index is a monitored component index into
//!   `CuMonitoringMetadata::components()`.
//!
//! Config (monitor `config` map keys):
//! - `copperlist_deadline_ms` (u64, default: 1000)
//! - `watchdog_period_ms` (u64, default: `max(deadline/4, 10)`)
//! - `exit_code_shutdown` (i32, default: 70)
//! - `exit_code_lock` (i32, default: 71)
//! - `exit_code_panic` (i32, default: 72)
//!
//! Implementation is split by target profile:
//! - `std_impl`: full watchdog/panic/exit behavior.
//! - `nostd_impl`: compile-only placeholder (behavior intentionally deferred).

extern crate alloc;

#[cfg(not(feature = "std"))]
mod nostd_impl;
#[cfg(feature = "std")]
mod std_impl;

#[cfg(not(feature = "std"))]
pub use nostd_impl::CuSafetyMon;
#[cfg(feature = "std")]
pub use std_impl::CuSafetyMon;
