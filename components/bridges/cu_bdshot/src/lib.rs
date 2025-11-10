#![cfg_attr(not(test), no_std)]

pub mod board;
pub mod bridge;
pub mod messages;

mod decode;
mod esc_channel;

pub use board::Rp2350Board;
pub use bridge::CuBdshotBridge;
pub use messages::{DShotTelemetry, EscCommand, EscTelemetry};

pub type RpBdshotBridge = CuBdshotBridge<Rp2350Board>;
