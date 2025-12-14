#![cfg_attr(not(test), no_std)]

pub mod board;
pub mod bridge;
pub mod messages;

mod decode;
mod esc_channel;

pub use board::{
    Rp2350Board, Rp2350BoardConfig, Rp2350BoardProvider, Rp2350BoardResources,
    register_rp2350_board,
};
pub use bridge::{BdshotBoardProvider, CuBdshotBridge};
pub use messages::{DShotTelemetry, EscCommand, EscTelemetry};

pub type RpBdshotBridge = CuBdshotBridge<Rp2350BoardProvider>;
