#![no_std]
#![no_main]

use cortex_m_rt::entry;
use embedded_hal::delay::DelayNs;
use embedded_hal::pwm::SetDutyCycle;
use hal::{
    clocks::init_clocks_and_plls,
    gpio::{FunctionPwm, Pins},
    pac,
    pwm::Slices,
    sio::Sio,
    timer::{CopyableTimer0, Timer},
    watchdog::Watchdog,
};
use rp235x_hal as hal;
use static_cell::StaticCell;

use defmt_serial as _;
// pulls in #[global_logger]
use hal::gpio;
use panic_probe as _;
use rp235x_hal::clocks::ClockSource;
use rp235x_hal::uart::{Enabled, UartConfig, UartPeripheral};
// pulls in #[panic_handler]

const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;

#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

const HEAP_SIZE: usize = 128 * 1024;

use embedded_alloc::Heap;
#[global_allocator]
static ALLOC: Heap = Heap::empty();

// Define type for actual pins used by the UART
type U0Pinout = (
    gpio::Pin<gpio::bank0::Gpio0, gpio::FunctionUart, gpio::PullNone>,
    gpio::Pin<gpio::bank0::Gpio1, gpio::FunctionUart, gpio::PullNone>,
);

static SERIAL: StaticCell<UartPeripheral<Enabled, pac::UART0, U0Pinout>> = StaticCell::new();

#[entry]
fn main() -> ! {
    static mut HEAP: [u8; HEAP_SIZE] = [0; HEAP_SIZE];
    unsafe {
        ALLOC.init(HEAP.as_ptr() as usize, HEAP.len());
    }

    let mut p = pac::Peripherals::take().unwrap();

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
    .unwrap();

    let mut timer: Timer<CopyableTimer0> = Timer::new_timer0(p.TIMER0, &mut p.RESETS, &clocks);

    let sio = Sio::new(p.SIO);
    let pins = Pins::new(p.IO_BANK0, p.PADS_BANK0, sio.gpio_bank0, &mut p.RESETS);

    let r = pins.gpio18.into_function::<FunctionPwm>();
    let g = pins.gpio19.into_function::<FunctionPwm>();
    let b = pins.gpio20.into_function::<FunctionPwm>();

    let tx = pins
        .gpio0
        .into_function::<gpio::FunctionUart>()
        .into_pull_type::<gpio::PullNone>();
    let rx = pins
        .gpio1
        .into_function::<gpio::FunctionUart>()
        .into_pull_type::<gpio::PullNone>();

    let pwms = Slices::new(p.PWM, &mut p.RESETS);
    let uconfig = UartConfig::default(); // 115200 8N1

    let uart = UartPeripheral::new(p.UART0, (tx, rx), &mut p.RESETS)
        .enable(uconfig, clocks.peripheral_clock.get_freq())
        .unwrap();
    defmt_serial::defmt_serial(SERIAL.init(uart));

    // GPIO18 is slice PWM1 channel A, GPIO19 is PWM1 channel B, GPIO20 is PWM2 channel A
    let mut s1 = pwms.pwm1;
    s1.set_div_int(4);
    s1.enable();
    let mut rch = s1.channel_a;
    rch.output_to(r);
    let mut gch = s1.channel_b;
    gch.output_to(g);

    let mut s2 = pwms.pwm2;
    s2.set_div_int(4);
    s2.enable();
    let mut bch = s2.channel_a;
    bch.output_to(b);

    let mut set_rgb8 = |rv: u8, gv: u8, bv: u8| {
        rch.set_duty_cycle((rv as u16) << 8).unwrap();
        gch.set_duty_cycle((gv as u16) << 8).unwrap();
        bch.set_duty_cycle((bv as u16) << 8).unwrap();
    };

    let one_s: u32 = 1_000_000_000;
    let two_s: u32 = 2 * one_s;

    loop {
        defmt::debug!("1");
        set_rgb8(255, 0, 0);
        timer.delay_ns(one_s);
        defmt::debug!("2");
        set_rgb8(0, 255, 0);
        timer.delay_ns(one_s);
        defmt::info!("3");
        set_rgb8(0, 0, 255);
        timer.delay_ns(one_s);
        defmt::info!("4");
        set_rgb8(255, 255, 255);
        timer.delay_ns(two_s);
        defmt::info!("5");

        set_rgb8(255, 255, 0);
        timer.delay_ns(one_s);
        defmt::info!("6");
        set_rgb8(0, 255, 255);
        timer.delay_ns(one_s);
        defmt::info!("7");
        set_rgb8(255, 0, 255);
        timer.delay_ns(one_s);
        defmt::info!("8");
        set_rgb8(255, 255, 255);
        timer.delay_ns(two_s);
    }
}
