use crate::board::BdshotBoard;
use crate::bridge::BdshotBoardProvider;
use crate::decode::{decode_telemetry_packet, extract_telemetry_payload_with_crc_test, fold_gcr};
use crate::messages::DShotTelemetry;
use alloc::boxed::Box;
use cortex_m::asm;
use cu29::CuResult;
use cu29::prelude::CuError;
use spin::Mutex;

/// Minimal trait a hardware backend must implement to drive bidirectional DShot600 on STM32/H7.
///
/// A backend is responsible for emitting the encoded 32-bit DShot frame on the selected
/// timer/DMAR channel and, after the ~30us turnaround, capturing the 21 raw telemetry bits
/// coming back from the ESC (NRZI/GCR encoded as per the spec).
pub trait Stm32BdshotBackend {
    /// Number of ESC channels exposed by this backend (e.g. 4 on the MICOAIR743).
    fn channels(&self) -> usize;

    /// Emit one encoded DShot frame to the ESC.
    fn send_frame(&mut self, channel: usize, frame: u32);

    /// Return the raw 21-bit telemetry word captured from the wire (NRZI bitstream).
    /// If nothing was captured, return `None`.
    fn read_raw_telemetry(&mut self, channel: usize) -> Option<u32>;

    /// Busy-wait delay in microseconds between frames.
    fn delay_us(&mut self, micros: u32);
}

impl<T: Stm32BdshotBackend + ?Sized> Stm32BdshotBackend for Box<T> {
    fn channels(&self) -> usize {
        (**self).channels()
    }

    fn send_frame(&mut self, channel: usize, frame: u32) {
        (**self).send_frame(channel, frame);
    }

    fn read_raw_telemetry(&mut self, channel: usize) -> Option<u32> {
        (**self).read_raw_telemetry(channel)
    }

    fn delay_us(&mut self, micros: u32) {
        (**self).delay_us(micros);
    }
}

pub struct Stm32BdshotBoard<B: Stm32BdshotBackend> {
    backend: B,
    channel_count: usize,
}

impl<B: Stm32BdshotBackend> Stm32BdshotBoard<B> {
    pub fn new(backend: B) -> Self {
        let channel_count = backend.channels();
        Self {
            backend,
            channel_count,
        }
    }

    /// Helper to turn a captured 21-bit NRZI payload into decoded telemetry.
    fn decode_telemetry(raw21: u32) -> Option<DShotTelemetry> {
        let raw20 = fold_gcr(raw21);
        let data = crate::decode::gcr_to_16bit(raw20)?;
        let payload = extract_telemetry_payload_with_crc_test(data)?;
        Some(decode_telemetry_packet(payload))
    }
}

impl<B: Stm32BdshotBackend> BdshotBoard for Stm32BdshotBoard<B> {
    // MICOAIR743 exposes 10 PWM outputs; we cap the bridge at 4 ESCs for parity with RP.
    const CHANNEL_COUNT: usize = 4;

    fn exchange(&mut self, channel: usize, frame: u32) -> Option<DShotTelemetry> {
        if channel >= self.channel_count {
            return None;
        }
        self.backend.send_frame(channel, frame);
        if let Some(raw21) = self.backend.read_raw_telemetry(channel) {
            Self::decode_telemetry(raw21)
        } else {
            None
        }
    }

    fn delay(&mut self, micros: u32) {
        self.backend.delay_us(micros);
    }
}

/// Lightweight helper to decode a telemetry response captured at 3x oversample.
/// The buffer should contain at least 63 samples (21 bits * 3), where each entry
/// is 0/1 representing the wire level. For noisy captures, only the first 63
/// samples are used.
pub fn decode_oversampled_telemetry(samples: &[u8]) -> Option<u32> {
    if samples.len() < 63 {
        return None;
    }
    let mut raw21: u32 = 0;
    for bit_idx in 0..21 {
        let base = bit_idx * 3;
        let ones = samples[base..base + 3].iter().filter(|b| **b != 0).count();
        let bit = ones >= 2;
        raw21 = (raw21 << 1) | u32::from(bit);
    }
    Some(raw21)
}

/// Global slot used to pass the concrete backend (built in firmware init) into the bridge.
static STM32_BOARD_SLOT: Mutex<Option<Stm32BdshotBoard<Box<dyn Stm32BdshotBackend + Send>>>> =
    Mutex::new(None);

/// Register the platform-specific backend once at startup.
pub fn register_stm32_bdshot_backend<B>(backend: B) -> CuResult<()>
where
    B: Stm32BdshotBackend + Send + 'static,
{
    let mut slot = STM32_BOARD_SLOT.lock();
    if slot.is_some() {
        return Err(CuError::from("STM32 BDShot backend already registered"));
    }
    *slot = Some(Stm32BdshotBoard::new(Box::new(backend)));
    Ok(())
}

pub struct Stm32BoardProvider;

impl BdshotBoardProvider for Stm32BoardProvider {
    type Board = Stm32BdshotBoard<Box<dyn Stm32BdshotBackend + Send>>;

    fn create_board() -> CuResult<Self::Board> {
        STM32_BOARD_SLOT
            .lock()
            .take()
            .ok_or_else(|| CuError::from("No STM32 BDShot backend registered"))
    }
}

/// Constants matching Betaflight's DShot600 DMA encoding (timing ticks).
pub const DSHOT_DMA_BUFFER_SIZE: usize = 18;
pub const DSHOT_BIT_0_TICKS: u16 = 7;
pub const DSHOT_BIT_1_TICKS: u16 = 14;

/// Encode a 16-bit DShot packet into a DMA buffer (duty ticks) with a reset tail.
pub fn encode_dshot600_dma(frame: u32) -> [u16; DSHOT_DMA_BUFFER_SIZE] {
    let mut buf = [0u16; DSHOT_DMA_BUFFER_SIZE];
    let mut packet = frame as u16;
    for i in 0..16 {
        buf[i] = if (packet & 0x8000) != 0 {
            DSHOT_BIT_1_TICKS
        } else {
            DSHOT_BIT_0_TICKS
        };
        packet <<= 1;
    }
    // last two entries are low-level frame reset (~2us)
    buf[16] = 0;
    buf[17] = 0;
    buf
}

/// Caller implements this to push a filled DMA buffer to the appropriate timer channel.
pub trait DshotDmaTx {
    fn submit(&mut self, channel: usize, buffer: &[u16; DSHOT_DMA_BUFFER_SIZE]);
}

/// Caller implements this to supply 3x oversampled telemetry samples for a channel.
pub trait TelemetryCapture {
    fn take_samples(&mut self, channel: usize) -> Option<[u8; 63]>;
}

/// Generic H7 backend that converts encoded frames into DMA duty buffers and decodes telemetry
/// from 3x oversampled samples provided by the caller. Wiring to timers/DMA happens in the
/// user-provided Tx/Rx adapters.
pub struct H7Dshot600Backend<Tx, Rx> {
    tx: Tx,
    rx: Rx,
    channels: usize,
    cycles_per_us: u32,
}

impl<Tx, Rx> H7Dshot600Backend<Tx, Rx> {
    pub fn new(tx: Tx, rx: Rx, sysclk_hz: u32, channels: usize) -> Self {
        Self {
            tx,
            rx,
            channels,
            cycles_per_us: sysclk_hz / 1_000_000,
        }
    }
}

impl<Tx, Rx> Stm32BdshotBackend for H7Dshot600Backend<Tx, Rx>
where
    Tx: DshotDmaTx,
    Rx: TelemetryCapture,
{
    fn channels(&self) -> usize {
        self.channels
    }

    fn send_frame(&mut self, channel: usize, frame: u32) {
        let buf = encode_dshot600_dma(frame);
        self.tx.submit(channel, &buf);
    }

    fn read_raw_telemetry(&mut self, channel: usize) -> Option<u32> {
        let samples = self.rx.take_samples(channel)?;
        decode_oversampled_telemetry(&samples)
    }

    fn delay_us(&mut self, micros: u32) {
        // Conservative chunking to keep the asm delay argument in range.
        let cycles = self.cycles_per_us.max(1) as u64;
        let mut rem = micros as u64;
        while rem > 0 {
            let step = rem.min(1000);
            let total = cycles * step;
            asm::delay(total as u32);
            rem -= step;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{
        DSHOT_BIT_0_TICKS, DSHOT_BIT_1_TICKS, Stm32BdshotBackend, Stm32BdshotBoard,
        decode_oversampled_telemetry, encode_dshot600_dma,
    };
    use crate::board::BdshotBoard;
    use crate::messages::EscCommand;
    use crate::{DShotTelemetry, encode_frame};

    #[derive(Default)]
    struct DummyBackend {
        sent: alloc::vec::Vec<u32>,
        telemetry: Option<u32>,
    }

    impl Stm32BdshotBackend for DummyBackend {
        fn channels(&self) -> usize {
            1
        }

        fn send_frame(&mut self, channel: usize, frame: u32) {
            assert_eq!(channel, 0);
            self.sent.push(frame);
        }

        fn read_raw_telemetry(&mut self, channel: usize) -> Option<u32> {
            assert_eq!(channel, 0);
            self.telemetry.take()
        }

        fn delay_us(&mut self, _micros: u32) {}
    }

    #[test]
    fn majority_decode_fixed_pattern() {
        // Raw NRZI pattern for zero RPM telemetry (same as decode::tests) with 3x oversampling.
        let pattern: [u8; 63] = {
            let bits = [
                0, 0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1,
            ];
            let mut buf = [0u8; 63];
            for (i, bit) in bits.into_iter().enumerate() {
                let base = i * 3;
                buf[base] = bit;
                buf[base + 1] = bit;
                buf[base + 2] = bit;
            }
            buf
        };
        let raw21 = decode_oversampled_telemetry(&pattern).unwrap();
        assert_eq!(raw21, 0b001010010100101010001);
    }

    #[test]
    fn encode_dshot600_dma_matches_bits() {
        // 0xA5A5 has alternating bits for easy inspection
        let buf = encode_dshot600_dma(0xA5A5);
        // First bit is 1
        assert_eq!(buf[0], DSHOT_BIT_1_TICKS);
        // Second bit is 0
        assert_eq!(buf[1], DSHOT_BIT_0_TICKS);
        // Last two are reset padding
        assert_eq!(buf[16], 0);
        assert_eq!(buf[17], 0);
    }

    #[test]
    fn board_exchange_decodes_telemetry() {
        let raw21 = 0b001010010100101010001; // zero RPM telemetry sample
        let backend = DummyBackend {
            telemetry: Some(raw21),
            ..Default::default()
        };
        let mut board = Stm32BdshotBoard::new(backend);
        let cmd = EscCommand {
            throttle: 0,
            request_telemetry: true,
        };
        let frame = encode_frame(cmd);
        let telem = board.exchange(0, frame).unwrap();
        assert_eq!(telem, DShotTelemetry::Erpm(0));
    }
}
