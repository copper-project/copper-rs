#![cfg_attr(not(feature = "std"), no_std)]

extern crate alloc;

#[cfg(all(feature = "eh1", feature = "eh02"))]
compile_error!("Features \"eh1\" and \"eh02\" are mutually exclusive; disable default features when enabling \"eh1\".");

#[cfg(feature = "eh02")]
pub use embedded_sdmmc as sdmmc;
#[cfg(feature = "eh1")]
pub use embedded_sdmmc_eh1 as sdmmc;

mod gpt;
mod logger;

pub use gpt::find_copper_partition;
pub use logger::{EMMCLogger, EMMCSectionStorage, ForceSyncSend};
