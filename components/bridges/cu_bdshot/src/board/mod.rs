use crate::messages::DShotTelemetry;

pub mod rp2350;

pub trait BdshotBoard {
    const CHANNEL_COUNT: usize;

    fn exchange(&mut self, channel: usize, frame: u32) -> Option<DShotTelemetry>;

    fn delay(&mut self, micros: u32);
}

pub use rp2350::{
    encode_frame, register_rp2350_board, Rp2350Board, Rp2350BoardConfig, Rp2350BoardProvider,
    Rp2350BoardResources,
};
