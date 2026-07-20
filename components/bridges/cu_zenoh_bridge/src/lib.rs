#![cfg_attr(feature = "nostd", no_std)]

#[cfg(all(feature = "std", feature = "nostd"))]
compile_error!("cu-zenoh-bridge features `std` and `nostd` are mutually exclusive");
#[cfg(not(any(feature = "std", feature = "nostd")))]
compile_error!("enable either the `std` or `nostd` feature of cu-zenoh-bridge");

#[cfg(feature = "nostd")]
extern crate alloc;

#[cfg(feature = "std")]
mod host;
#[cfg(feature = "std")]
pub use host::*;

#[cfg(feature = "nostd")]
mod nostd;
#[cfg(feature = "nostd")]
pub use nostd::*;
