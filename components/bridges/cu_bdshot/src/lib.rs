#![cfg_attr(not(test), no_std)]

#[cfg(feature = "stm32")]
extern crate alloc;

pub mod board;
pub mod bridge;
pub mod messages;

mod decode;
#[cfg(feature = "rp2350")]
mod esc_channel;

pub use board::*;
pub use bridge::{BdshotBoardProvider, CuBdshotBridge};
pub use messages::{DShotTelemetry, EscCommand, EscTelemetry};

#[cfg(feature = "stm32")]
pub type Stm32BdshotBridge = CuBdshotBridge<board::Stm32BoardProvider>;

#[cfg(feature = "rp2350")]
pub type RpBdshotBridge = CuBdshotBridge<board::Rp2350BoardProvider>;
