#![cfg_attr(not(feature = "std"), no_std)]

extern crate alloc;

mod gpt;
mod logger;

pub use gpt::find_copper_partition;
pub use logger::{EMMCLogger, EMMCSectionStorage, ForceSyncSend};
