use super::BdshotBoard;
use crate::bridge::BdshotBoardProvider;
use crate::dshot::{TelemetryDecode, decode_raw21, dshot_frame};
use crate::messages::{DShotTelemetry, EscCommand};
use cortex_m::peripheral::DWT;
use cu29::CuResult;
use cu29::prelude::CuError;
use spin::Mutex;
use stm32h7xx_hal as hal;

use hal::gpio::{Analog, Output, Pin, PushPull};

// Slightly faster than nominal DSHOT600 (matching BF timing).
const DSHOT_FREQUENCY: u32 = 650_000;
const INVERT_SIGNAL: bool = true;
const INVERT_CSUM_ON_TELEM: bool = true;

const LA_SAMPLES: usize = 100;
const TELE_OVERSAMPLE: u32 = 3;

const MOTOR_TELE_MASKS: [u32; 4] = [1 << 14, 1 << 13, 1 << 11, 1 << 9]; // PE14/13/11/9
const MOTOR_MODER_MASKS: [u32; 4] = [
    0b11 << (14 * 2),
    0b11 << (13 * 2),
    0b11 << (11 * 2),
    0b11 << (9 * 2),
];
const MOTOR_MODER_OUTPUTS: [u32; 4] = [
    0b01 << (14 * 2),
    0b01 << (13 * 2),
    0b01 << (11 * 2),
    0b01 << (9 * 2),
];
const MOTOR_PUPDR_MASKS: [u32; 4] = [
    0b11 << (14 * 2),
    0b11 << (13 * 2),
    0b11 << (11 * 2),
    0b11 << (9 * 2),
];

pub struct Stm32H7BoardResources {
    pub m1: hal::gpio::gpioe::PE14<Analog>,
    pub m2: hal::gpio::gpioe::PE13<Analog>,
    pub m3: hal::gpio::gpioe::PE11<Analog>,
    pub m4: hal::gpio::gpioe::PE9<Analog>,
    pub dwt: DWT,
    pub sysclk_hz: u32,
}

pub struct Stm32H7Board {
    _motors: Motors,
    gpioe: *const hal::pac::gpioe::RegisterBlock,
    dwt: DWT,
    cycles_per_us: u32,
    bit_cycles: u32,
    t0_high: u32,
    t1_high: u32,
}

// SAFETY: The board is registered once and used on a single core with no concurrent access.
unsafe impl Send for Stm32H7Board {}

struct Motors {
    _m1: Pin<'E', 14, Output<PushPull>>,
    _m2: Pin<'E', 13, Output<PushPull>>,
    _m3: Pin<'E', 11, Output<PushPull>>,
    _m4: Pin<'E', 9, Output<PushPull>>,
}

impl Motors {
    fn new(
        m1: hal::gpio::gpioe::PE14<Analog>,
        m2: hal::gpio::gpioe::PE13<Analog>,
        m3: hal::gpio::gpioe::PE11<Analog>,
        m4: hal::gpio::gpioe::PE9<Analog>,
    ) -> Self {
        Self {
            _m1: m1.into_push_pull_output(),
            _m2: m2.into_push_pull_output(),
            _m3: m3.into_push_pull_output(),
            _m4: m4.into_push_pull_output(),
        }
    }

    fn all_low(&mut self) {
        self._m1.set_low();
        self._m2.set_low();
        self._m3.set_low();
        self._m4.set_low();
    }
}

impl Stm32H7Board {
    pub fn new(resources: Stm32H7BoardResources) -> CuResult<Self> {
        let bit_cycles = (resources.sysclk_hz / DSHOT_FREQUENCY).max(1);
        let cycles_per_us = (resources.sysclk_hz / 1_000_000).max(1);
        let t1_high = (bit_cycles * 2) / 3;
        let t0_high = bit_cycles / 3;
        let mut motors = Motors::new(resources.m1, resources.m2, resources.m3, resources.m4);
        motors.all_low();
        Ok(Self {
            _motors: motors,
            gpioe: hal::pac::GPIOE::ptr(),
            dwt: resources.dwt,
            cycles_per_us,
            bit_cycles,
            t0_high,
            t1_high,
        })
    }
}

impl BdshotBoard for Stm32H7Board {
    const CHANNEL_COUNT: usize = 4;

    fn exchange(&mut self, channel: usize, frame: u32) -> Option<DShotTelemetry> {
        // SAFETY: gpioe points to the GPIOE register block for the lifetime of the board.
        let gpioe = unsafe { &*self.gpioe };
        let mask = *MOTOR_TELE_MASKS.get(channel)?;
        let moder_mask = MOTOR_MODER_MASKS[channel];
        let moder_output = MOTOR_MODER_OUTPUTS[channel];
        let pupdr_mask = MOTOR_PUPDR_MASKS[channel];
        set_mode_output(gpioe, moder_mask, moder_output, pupdr_mask);
        drive_signal_level(gpioe, mask, false);
        let frame_start = self.dwt.cyccnt.read();
        send_frame(
            gpioe,
            &self.dwt,
            self.bit_cycles,
            self.t0_high,
            self.t1_high,
            mask,
            frame as u16,
        );
        set_mode_input(gpioe, moder_mask, pupdr_mask);
        let wants_telemetry = (frame as u16) & 0x10 != 0;
        if wants_telemetry {
            decode_telemetry(gpioe, &self.dwt, frame_start, self.bit_cycles, mask)
        } else {
            None
        }
    }

    fn delay(&mut self, micros: u32) {
        let cycles = self.cycles_per_us as u64;
        let mut remaining = micros as u64;
        while remaining > 0 {
            let chunk = remaining.min(1000);
            let start = self.dwt.cyccnt.read();
            busy_wait(start, (cycles * chunk) as u32, &self.dwt);
            remaining -= chunk;
        }
    }
}

pub fn encode_frame(command: EscCommand) -> u32 {
    dshot_frame(
        command.throttle,
        command.request_telemetry,
        INVERT_CSUM_ON_TELEM,
    ) as u32
}

/// Global to be able to send the actual HW interface to the bridge.
static STM32H7_BOARD_SLOT: Mutex<Option<Stm32H7Board>> = Mutex::new(None);

/// Global to be able to send the actual HW interface to the bridge.
pub fn register_stm32h7_board(board: Stm32H7Board) -> CuResult<()> {
    let mut slot = STM32H7_BOARD_SLOT.lock();
    if slot.is_some() {
        return Err(CuError::from("STM32H7 BDShot board already registered"));
    }
    *slot = Some(board);
    Ok(())
}

pub struct Stm32H7BoardProvider;

impl BdshotBoardProvider for Stm32H7BoardProvider {
    type Board = Stm32H7Board;

    fn create_board() -> CuResult<Self::Board> {
        STM32H7_BOARD_SLOT
            .lock()
            .take()
            .ok_or_else(|| CuError::from("No STM32H7 BDShot board registered"))
    }
}

fn busy_wait(start: u32, cycles: u32, dwt: &DWT) {
    while dwt.cyccnt.read().wrapping_sub(start) < cycles {}
}

fn drive_signal_level(gpioe: &hal::pac::gpioe::RegisterBlock, mask: u32, high: bool) {
    let (set_mask, reset_mask) = if INVERT_SIGNAL {
        if high { (0, mask) } else { (mask, 0) }
    } else if high {
        (mask, 0)
    } else {
        (0, mask)
    };
    gpioe.bsrr.write(|w| {
        // SAFETY: Writing raw bits to BSRR is the expected HAL usage.
        unsafe { w.bits(set_mask | (reset_mask << 16)) }
    });
}

fn send_frame(
    gpioe: &hal::pac::gpioe::RegisterBlock,
    dwt: &DWT,
    bit_cycles: u32,
    t0_high: u32,
    t1_high: u32,
    mask: u32,
    frame: u16,
) {
    for bit in (0..16).rev() {
        let bit_start = dwt.cyccnt.read();
        drive_signal_level(gpioe, mask, true);
        let high_cycles = if (frame >> bit) & 1 == 1 {
            t1_high
        } else {
            t0_high
        };
        busy_wait(bit_start, high_cycles, dwt);
        drive_signal_level(gpioe, mask, false);
        busy_wait(bit_start, bit_cycles, dwt);
    }
    // Required for ESC arming on this setup; removing breaks telemetry/arming.
    let hold_start = dwt.cyccnt.read();
    drive_signal_level(gpioe, mask, false);
    busy_wait(hold_start, bit_cycles, dwt);
}

fn decode_with_slip(raw21: u32) -> Option<(TelemetryDecode, i8)> {
    let candidates: [(u32, i8); 5] = [
        (raw21, 0),
        ((raw21 << 1) & 0x1f_ffff, 1),
        (raw21 >> 1, -1),
        ((raw21 << 2) & 0x1f_ffff, 2),
        (raw21 >> 2, -2),
    ];
    for (value, slip) in candidates.iter() {
        if let Some(decoded) = decode_raw21(*value) {
            return Some((decoded, *slip));
        }
    }
    None
}

fn decode_telemetry(
    gpioe: &hal::pac::gpioe::RegisterBlock,
    dwt: &DWT,
    frame_start: u32,
    bit_cycles: u32,
    motor_mask: u32,
) -> Option<DShotTelemetry> {
    let tele_bit_nom = (bit_cycles * 4) / 5;
    let la_step_cyc = core::cmp::max(1, tele_bit_nom / TELE_OVERSAMPLE);
    let guard_samples = 19 * TELE_OVERSAMPLE as usize;
    let min_valid_samples = (21 - 2) * TELE_OVERSAMPLE as usize;
    let frame_end = frame_start.wrapping_add(bit_cycles * 16);
    let start_scan = frame_end.wrapping_add(la_step_cyc * guard_samples as u32);

    let mut samples = [0u32; LA_SAMPLES];
    let mut t = start_scan;
    for slot in samples.iter_mut().take(LA_SAMPLES) {
        while dwt.cyccnt.read().wrapping_sub(t) as i32 as u32 & 0x8000_0000 != 0 {}
        *slot = gpioe.idr.read().bits();
        t = t.wrapping_add(la_step_cyc);
    }

    let end_limit = LA_SAMPLES.saturating_sub(min_valid_samples);
    let mut start = 0usize;
    while start < end_limit {
        if (samples[start] & motor_mask) == 0 {
            break;
        }
        start += 1;
    }
    if start >= end_limit {
        return None;
    }

    let remaining = core::cmp::min(LA_SAMPLES - start, LA_SAMPLES);
    if remaining < min_valid_samples {
        return None;
    }

    let mut level = if (samples[start] & motor_mask) != 0 {
        1u8
    } else {
        0u8
    };
    let mut run_len: u32 = 1;
    let mut bits: i32 = 0;
    let mut value: u32 = 0;
    let end = start.saturating_add(remaining);
    let mut i = start + 1;
    while i < end {
        let bit = if (samples[i] & motor_mask) != 0 {
            1u8
        } else {
            0u8
        };
        if bit == level {
            run_len += 1;
        } else {
            let len_bits = core::cmp::max(1, ((run_len + 1) / TELE_OVERSAMPLE) as i32);
            let len_bits_u = len_bits as u32;
            bits += len_bits;
            value <<= len_bits_u;
            if level != 0 {
                value |= (1u32 << len_bits_u) - 1;
            }
            level = bit;
            run_len = 1;
            if bits >= 21 {
                break;
            }
        }
        i += 1;
    }

    if bits < 18 {
        return None;
    }
    let nlen = 21 - bits;
    if nlen <= 0 {
        return None;
    }
    let nlen_u = nlen as u32;
    value <<= nlen_u;
    value |= (1u32 << nlen_u) - 1;
    let raw21 = value & 0x1f_ffff;

    decode_with_slip(raw21).map(|(decoded, _)| decoded.decoded)
}

fn set_mode_input(gpioe: &hal::pac::gpioe::RegisterBlock, moder_mask: u32, pupdr_mask: u32) {
    let moder = gpioe.moder.read().bits() & !moder_mask;
    gpioe.moder.write(|w| {
        // SAFETY: Writing raw bits to MODER is required for GPIO mode configuration.
        unsafe { w.bits(moder) }
    });

    let pupdr = gpioe.pupdr.read().bits() & !pupdr_mask;
    gpioe.pupdr.write(|w| {
        // SAFETY: Writing raw bits to PUPDR is required for GPIO pull configuration.
        unsafe { w.bits(pupdr) }
    });
}

fn set_mode_output(
    gpioe: &hal::pac::gpioe::RegisterBlock,
    moder_mask: u32,
    moder_output: u32,
    pupdr_mask: u32,
) {
    let moder = gpioe.moder.read().bits() & !moder_mask;
    gpioe.moder.write(|w| {
        // SAFETY: Writing raw bits to MODER is required for GPIO mode configuration.
        unsafe { w.bits(moder | moder_output) }
    });

    let pupdr = gpioe.pupdr.read().bits() & !pupdr_mask;
    gpioe.pupdr.write(|w| {
        // SAFETY: Writing raw bits to PUPDR is required for GPIO pull configuration.
        unsafe { w.bits(pupdr) }
    });
}
