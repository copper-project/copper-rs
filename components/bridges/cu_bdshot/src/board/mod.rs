use crate::messages::DShotTelemetry;

#[cfg(feature = "rp2350")]
pub mod rp2350;

#[cfg(feature = "stm32h7")]
pub mod stm32;

pub trait BdshotBoard {
    const CHANNEL_COUNT: usize;

    fn exchange(&mut self, channel: usize, frame: u32) -> Option<DShotTelemetry>;

    fn delay(&mut self, micros: u32);
}

#[cfg(feature = "rp2350")]
pub use rp2350::{
    Rp2350Board, Rp2350BoardConfig, Rp2350BoardProvider, Rp2350BoardResources, encode_frame,
    register_rp2350_board,
};

#[cfg(feature = "stm32h7")]
pub use stm32::{
    Stm32H7Board, Stm32H7BoardProvider, Stm32H7BoardResources, encode_frame, register_stm32h7_board,
};
