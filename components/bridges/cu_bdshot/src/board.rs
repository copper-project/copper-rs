use crate::decode::{decode_telemetry_packet, extract_telemetry_payload_with_crc_test, fold_gcr};
use crate::esc_channel::EscChannel;
use crate::messages::{DShotTelemetry, EscCommand};
use cu29::CuResult;
use fugit::MicrosDurationU64;
use hal::clocks::init_clocks_and_plls;
use hal::gpio::bank0::Pins;
use hal::gpio::FunctionPio0;
use hal::gpio::PullNone;
use hal::pio::PIOBuilder;
use hal::pio::PIOExt;
use hal::timer::{CopyableTimer0, Timer, TimerDevice};
use hal::{pac, Clock, Sio, Watchdog};
use nb::block;
use pio_proc::pio_file;
use rp235x_hal as hal;

#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

pub trait BdshotBoard {
    const CHANNEL_COUNT: usize;

    fn init() -> CuResult<Self>
    where
        Self: Sized;

    fn exchange(&mut self, channel: usize, frame: u32) -> Option<DShotTelemetry>;

    fn delay(&mut self, micros: u64);
}

pub struct Rp2350Board {
    timer: Timer<CopyableTimer0>,
    channels: [EscChannel<
        hal::pio::PIO0,
        hal::pio::SM0,
        hal::dma::SingleBuffering,
        hal::dma::SingleBuffering,
    >; 4],
}

impl BdshotBoard for Rp2350Board {
    const CHANNEL_COUNT: usize = 4;

    fn init() -> CuResult<Self> {
        let mut p = pac::Peripherals::take().ok_or_else(|| cu29::CuError::from("PAC busy"))?;
        let mut watchdog = Watchdog::new(p.WATCHDOG);
        let clocks = init_clocks_and_plls(
            XOSC_CRYSTAL_FREQ,
            p.XOSC,
            p.CLOCKS,
            p.PLL_SYS,
            p.PLL_USB,
            &mut p.RESETS,
            &mut watchdog,
        )
        .map_err(|_| cu29::CuError::from("Clock init failed"))?;

        let sys_hz = clocks.system_clock.freq().to_Hz();
        let sio = Sio::new(p.SIO);
        let pins = Pins::new(p.IO_BANK0, p.PADS_BANK0, sio.gpio_bank0, &mut p.RESETS);

        let _gp6 = pins
            .gpio6
            .into_pull_type::<PullNone>()
            .into_function::<FunctionPio0>();
        let _gp7 = pins
            .gpio7
            .into_pull_type::<PullNone>()
            .into_function::<FunctionPio0>();
        let _gp8 = pins
            .gpio8
            .into_pull_type::<PullNone>()
            .into_function::<FunctionPio0>();
        let _gp9 = pins
            .gpio9
            .into_pull_type::<PullNone>()
            .into_function::<FunctionPio0>();

        let timer0 = Timer::<CopyableTimer0>::new_timer0(p.TIMER0, &mut p.RESETS, &clocks);
        const TARGET_PIO_CLOCK: u64 = 15_300_000;
        let (d, f) = pio_clkdiv_8p8(sys_hz, TARGET_PIO_CLOCK as u32);
        let pio_normal_prg = pio_proc::pio_file!("src/dshot.pio", select_program("bdshot_300"));
        let (mut pio0, sm0, sm1, sm2, sm3) = p.PIO0.split(&mut p.RESETS);
        let bdshot_prg = pio0.install(&pio_normal_prg.program).unwrap();

        let ch0 = crate::build_ch!(bdshot_prg, sm0, 6, d, f);
        let ch1 = crate::build_ch!(bdshot_prg, sm1, 7, d, f);
        let ch2 = crate::build_ch!(bdshot_prg, sm2, 8, d, f);
        let ch3 = crate::build_ch!(bdshot_prg, sm3, 9, d, f);

        Ok(Self {
            timer: timer0,
            channels: [ch0, ch1, ch2, ch3],
        })
    }

    fn exchange(&mut self, channel: usize, frame: u32) -> Option<DShotTelemetry> {
        let ch = match self.channels.get_mut(channel) {
            Some(ch) => ch,
            None => return None,
        };
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
        delay_us(&self.timer, 150);
        ch.restart();
        None
    }

    fn delay(&mut self, micros: u64) {
        delay_us(&self.timer, micros);
    }
}

const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;

fn delay_us<D: TimerDevice>(timer: &Timer<D>, us: u64) {
    let mut cd = timer.count_down();
    cd.start(MicrosDurationU64::micros(us));
    let _ = block!(cd.wait());
}

fn gcr_to_16bit(raw20: u32) -> Option<u16> {
    crate::decode::gcr_to_16bit(raw20)
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

#[macro_export]
macro_rules! build_ch {
    ($prog:expr, $sm:ident, $pin:expr, $d:expr, $f:expr) => {{
        let (sm, rx, tx) = PIOBuilder::from_installed_program(unsafe { $prog.share() })
            .set_pins($pin, 1)
            .out_pins($pin, 1)
            .out_shift_direction(hal::pio::ShiftDirection::Left)
            .in_shift_direction(hal::pio::ShiftDirection::Left)
            .in_pin_base($pin)
            .jmp_pin($pin)
            .clock_divisor_fixed_point($d, $f)
            .autopush(true)
            .push_threshold(21)
            .side_set_pin_base($pin)
            .build($sm);
        EscChannel {
            tx,
            rx,
            run: sm.start(),
        }
    }};
}

pub fn encode_frame(command: EscCommand) -> u32 {
    let mut bits = (command.throttle & 0x07FF) << 1;
    if command.request_telemetry {
        bits |= 0x01;
    }
    bdshot_frame(bits)
}
