use crate::decode::{decode_telemetry_packet, extract_telemetry_payload_with_crc_test, fold_gcr};
use crate::esc_channel::EscChannel;
use crate::messages::{DShotTelemetry, EscCommand};
use cortex_m::asm;
use cu29::prelude::CuError;
use cu29::CuResult;
use hal::dma::Word;
use hal::pac;
use hal::pio::{PIOBuilder, StateMachineIndex, ValidStateMachine};
use pio_proc::pio_file;
use rp235x_hal as hal;
use spin::Mutex;

use crate::bridge::BdshotBoardProvider;

pub trait BdshotBoard {
    const CHANNEL_COUNT: usize;

    fn exchange(&mut self, channel: usize, frame: u32) -> Option<DShotTelemetry>;

    fn delay(&mut self, micros: u32);
}

pub struct Rp2350Board {
    cycles_per_micro: u32,
    ch0: EscChannel<pac::PIO0, hal::pio::SM0, Word, Word>,
    ch1: EscChannel<pac::PIO0, hal::pio::SM1, Word, Word>,
    ch2: EscChannel<pac::PIO0, hal::pio::SM2, Word, Word>,
    ch3: EscChannel<pac::PIO0, hal::pio::SM3, Word, Word>,
}

#[derive(Clone, Copy)]
pub struct Rp2350BoardConfig {
    pub pins: [u8; 4],
    pub target_pio_clock_hz: u32,
}

impl Default for Rp2350BoardConfig {
    fn default() -> Self {
        Self {
            pins: [6, 7, 8, 9],
            target_pio_clock_hz: 15_300_000,
        }
    }
}

pub struct Rp2350BoardResources {
    pub pio: hal::pio::PIO<pac::PIO0>,
    pub sm0: hal::pio::UninitStateMachine<(pac::PIO0, hal::pio::SM0)>,
    pub sm1: hal::pio::UninitStateMachine<(pac::PIO0, hal::pio::SM1)>,
    pub sm2: hal::pio::UninitStateMachine<(pac::PIO0, hal::pio::SM2)>,
    pub sm3: hal::pio::UninitStateMachine<(pac::PIO0, hal::pio::SM3)>,
}

impl Rp2350BoardResources {
    pub fn new(
        pio: hal::pio::PIO<pac::PIO0>,
        sm0: hal::pio::UninitStateMachine<(pac::PIO0, hal::pio::SM0)>,
        sm1: hal::pio::UninitStateMachine<(pac::PIO0, hal::pio::SM1)>,
        sm2: hal::pio::UninitStateMachine<(pac::PIO0, hal::pio::SM2)>,
        sm3: hal::pio::UninitStateMachine<(pac::PIO0, hal::pio::SM3)>,
    ) -> Self {
        Self {
            pio,
            sm0,
            sm1,
            sm2,
            sm3,
        }
    }
}

impl Rp2350Board {
    pub fn new(
        resources: Rp2350BoardResources,
        system_clock_hz: u32,
        config: Rp2350BoardConfig,
    ) -> CuResult<Self> {
        let Rp2350BoardResources {
            mut pio,
            sm0,
            sm1,
            sm2,
            sm3,
        } = resources;
        let pins = config.pins;
        let (div_int, div_frac) = pio_clkdiv_8p8(system_clock_hz, config.target_pio_clock_hz);
        let program = pio_file!("src/dshot.pio", select_program("bdshot_300"));
        let installed = pio
            .install(&program.program)
            .map_err(|_| CuError::from("Failed to install BDShot PIO program"))?;

        let ch0 = crate::build_ch!(installed, sm0, pins[0], div_int, div_frac);
        let ch1 = crate::build_ch!(installed, sm1, pins[1], div_int, div_frac);
        let ch2 = crate::build_ch!(installed, sm2, pins[2], div_int, div_frac);
        let ch3 = crate::build_ch!(installed, sm3, pins[3], div_int, div_frac);

        Ok(Self {
            cycles_per_micro: system_clock_hz / 1_000_000,
            ch0,
            ch1,
            ch2,
            ch3,
        })
    }
}

impl BdshotBoard for Rp2350Board {
    const CHANNEL_COUNT: usize = 4;

    fn exchange(&mut self, channel: usize, frame: u32) -> Option<DShotTelemetry> {
        let cycles = self.cycles_per_micro;
        match channel {
            0 => Self::exchange_impl(cycles, &mut self.ch0, frame),
            1 => Self::exchange_impl(cycles, &mut self.ch1, frame),
            2 => Self::exchange_impl(cycles, &mut self.ch2, frame),
            3 => Self::exchange_impl(cycles, &mut self.ch3, frame),
            _ => None,
        }
    }

    fn delay(&mut self, micros: u32) {
        Self::delay_ticks(self.cycles_per_micro, micros as u64);
    }
}

impl Rp2350Board {
    fn exchange_impl<SM>(
        cycles_per_micro: u32,
        ch: &mut EscChannel<pac::PIO0, SM, Word, Word>,
        frame: u32,
    ) -> Option<DShotTelemetry>
    where
        SM: StateMachineIndex,
        (pac::PIO0, SM): ValidStateMachine,
    {
        if ch.is_full() {
            return None;
        }
        ch.write(frame);
        if let Some(resp) = ch.read() {
            let raw20 = fold_gcr(resp);
            let data = crate::decode::gcr_to_16bit(raw20)?;
            let payload = extract_telemetry_payload_with_crc_test(data)?;
            return Some(decode_telemetry_packet(payload));
        }
        Self::delay_ticks(cycles_per_micro, 150);
        ch.restart();
        None
    }

    fn delay_ticks(cycles_per_micro: u32, mut micros: u64) {
        let cycles = cycles_per_micro.max(1) as u64;
        while micros > 0 {
            let chunk = micros.min(1000);
            let total = cycles * chunk;
            asm::delay(total as u32);
            micros -= chunk;
        }
    }
}

fn pio_clkdiv_8p8(sys_hz: u32, target_hz: u32) -> (u16, u8) {
    let div_fixed = ((sys_hz as f32 / target_hz as f32) * 256.0 + 0.5) as u32;
    ((div_fixed >> 8) as u16, (div_fixed & 0xFF) as u8)
}

fn bdshot_frame(payload: u16) -> u32 {
    let d = ((payload & 0x07FF) << 1) | 1;
    let inv = (!d) & 0x0FFF;
    let crc = !(!((inv ^ (inv >> 4) ^ (inv >> 8)) & 0x0F)) & 0x0F;
    !((d << 4) | crc) as u32
}

pub fn encode_frame(command: EscCommand) -> u32 {
    let mut bits = (command.throttle & 0x07FF) << 1;
    if command.request_telemetry {
        bits |= 0x01;
    }
    bdshot_frame(bits)
}

static RP_BOARD_SLOT: Mutex<Option<Rp2350Board>> = Mutex::new(None);

pub fn register_rp2350_board(board: Rp2350Board) -> CuResult<()> {
    let mut slot = RP_BOARD_SLOT.lock();
    if slot.is_some() {
        return Err(CuError::from("RP2350 BDShot board already registered"));
    }
    *slot = Some(board);
    Ok(())
}

pub struct Rp2350BoardProvider;

impl BdshotBoardProvider for Rp2350BoardProvider {
    type Board = Rp2350Board;

    fn create_board() -> CuResult<Self::Board> {
        RP_BOARD_SLOT
            .lock()
            .take()
            .ok_or_else(|| CuError::from("No RP2350 BDShot board registered"))
    }
}
