use crate::messages::{DShotTelemetry, EscCommand};

pub trait BdshotBoard {
    const CHANNEL_COUNT: usize;

    fn exchange(&mut self, channel: usize, frame: u32) -> Option<DShotTelemetry>;

    fn delay(&mut self, micros: u32);
}

/// Construct an encoded DSHOT frame from payload (MCU -> ESC)
fn bdshot_frame(payload: u16) -> u32 {
    let d = ((payload & 0x07FF) << 1) | 1;
    let inv = (!d) & 0x0FFF;
    let crc = !(!((inv ^ (inv >> 4) ^ (inv >> 8)) & 0x0F)) & 0x0F;
    !((d << 4) | crc) as u32
}

/// Encodes a throttle command and telemetry request into a DSHOT frame
pub fn encode_frame(command: EscCommand) -> u32 {
    let mut bits = (command.throttle & 0x07FF) << 1;
    if command.request_telemetry {
        bits |= 0x01;
    }
    bdshot_frame(bits)
}

#[cfg(feature = "rp2350")]
pub mod rp2350;
#[cfg(feature = "stm32")]
pub mod stm32;

#[cfg(feature = "stm32")]
pub use stm32::{
    DshotDmaTx, H7Dshot600Backend, Stm32BdshotBackend, Stm32BdshotBoard, Stm32BoardProvider,
    TelemetryCapture, decode_oversampled_telemetry, encode_dshot600_dma,
    register_stm32_bdshot_backend, DSHOT_BIT_0_TICKS, DSHOT_BIT_1_TICKS, DSHOT_DMA_BUFFER_SIZE,
};
#[cfg(feature = "rp2350")]
pub use rp2350::{
    Rp2350Board, Rp2350BoardConfig, Rp2350BoardProvider, Rp2350BoardResources,
    register_rp2350_board,
};
