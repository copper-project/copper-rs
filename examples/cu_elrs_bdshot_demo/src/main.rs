#![no_std]
#![no_main]

extern crate alloc;

use alloc::sync::Arc;
use alloc::vec;
use alloc::vec::Vec;
use buddy_system_allocator::LockedHeap as Heap;
use cortex_m_rt::entry;
use cu29::cubridge::CuBridge;
use cu29::prelude::*;
use cu_bdshot::{register_rp2350_board, Rp2350Board, Rp2350BoardConfig, Rp2350BoardResources};
use cu_sdlogger::{find_copper_partition, EMMCLogger, EMMCSectionStorage, ForceSyncSend};
use embedded_hal::spi::MODE_0;
use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_sdmmc::SdCard;
use rp235x_hal as hal;
use rp235x_hal::clocks::Clock;
use rp235x_hal::fugit::RateExtU32;
use rp235x_hal::gpio::bank0::{Gpio15, Gpio16, Gpio17, Gpio18, Gpio19, Gpio2, Gpio3};
use rp235x_hal::gpio::{
    Function, FunctionPio0, FunctionSio, FunctionSpi, FunctionUartAux, FunctionXipCs1, Pin, PinId,
    PullDown, PullNone, PullType, PullUp, SioInput, SioOutput, ValidFunction,
};
use rp235x_hal::pac::{SPI0, UART0};
use rp235x_hal::pio::PIOExt;
use rp235x_hal::spi::{Enabled, FrameFormat};
use rp235x_hal::timer::{CopyableTimer0, CopyableTimer1};
use rp235x_hal::uart::{DataBits, StopBits, UartConfig, UartPeripheral};
use rp235x_hal::{clocks::init_clocks_and_plls, pac, sio::Sio, watchdog::Watchdog, Spi, Timer};
use spin::Mutex;

#[allow(unused_imports)]
use defmt_rtt as _;
#[allow(unused_imports)]
use panic_probe as _;

mod tasks;

#[copper_runtime(config = "copperconfig.ron")]
struct BdshotDemoApp {}

#[unsafe(link_section = ".start_block")]
#[used]
static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

#[global_allocator]
static ALLOC: Heap<32> = Heap::empty();

const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;

// SDCard Pins
type Mosi = Pin<Gpio19, FunctionSpi, PullDown>;
type Miso = Pin<Gpio16, FunctionSpi, PullDown>;
type Sck = Pin<Gpio18, FunctionSpi, PullDown>;
type Cs = Pin<Gpio17, FunctionSio<SioOutput>, PullDown>;
type Det = Pin<Gpio15, FunctionSio<SioInput>, PullUp>;

// SDCard types
type Spi0 = Spi<Enabled, SPI0, (Mosi, Miso, Sck)>;
type SdDev = ExclusiveDevice<Spi0, Cs, Timer0>;
type PimodoriSdCard = SdCard<SdDev, Timer1>;
type TSPimodoriSdCard = ForceSyncSend<SdCard<SdDev, Timer1>>;

// Timers
type Timer0 = Timer<CopyableTimer0>;
type Timer1 = Timer<CopyableTimer1>;

// Associated types for serial port
type ElrsTx = Pin<Gpio2, FunctionUartAux, PullDown>;
type ElrsRx = Pin<Gpio3, FunctionUartAux, PullUp>;
type SerialPort = UartPeripheral<rp235x_hal::uart::Enabled, UART0, (ElrsTx, ElrsRx)>;
type SerialPortError = rp235x_hal::uart::ReadErrorType;

#[entry]
fn main() -> ! {
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

    // Init Timers
    let timer0: Timer0 = Timer::new_timer0(p.TIMER0, &mut p.RESETS, &clocks);
    let timer_for_clock = timer0;

    // Init SDCard Pins
    let sio = Sio::new(p.SIO);
    let pins = hal::gpio::Pins::new(p.IO_BANK0, p.PADS_BANK0, sio.gpio_bank0, &mut p.RESETS);
    let (pio, sm0, sm1, sm2, sm3) = p.PIO0.split(&mut p.RESETS);

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

    // Init Serial Port pins
    let tx = pins
        .gpio2
        .into_push_pull_output()
        .into_function::<FunctionUartAux>();
    let rx = pins
        .gpio3
        .into_pull_up_input()
        .into_function::<FunctionUartAux>();

    // Init PIOs
    let resources = Rp2350BoardResources::new(pio, sm0, sm1, sm2, sm3);
    let board_cfg = Rp2350BoardConfig::default();
    let board = Rp2350Board::new(resources, clocks.system_clock.freq().to_Hz(), board_cfg)
        .expect("Failed to initialize BDShot board");
    register_rp2350_board(board).expect("BDShot board already registered");

    init_psram_and_allocator(pins.gpio47);
    quick_memory_test();

    info!("Setting up the SDCard...");

    let miso: Miso = pins.gpio16.into_function();
    let cs: Cs = pins.gpio17.into_push_pull_output();
    let sck: Sck = pins.gpio18.into_function();
    let mosi: Mosi = pins.gpio19.into_function();
    let _det: Det = pins.gpio15.into_pull_up_input();

    let sd_spi: Spi0 = Spi::<_, _, _, 8>::new(p.SPI0, (mosi, miso, sck)).init(
        &mut p.RESETS,
        clocks.peripheral_clock.freq(),
        16_000_000u32.Hz(),
        FrameFormat::MotorolaSpi(MODE_0),
    );

    let sd_dev: SdDev = ExclusiveDevice::new(sd_spi, cs, timer0).unwrap();
    let delay1: Timer1 = Timer::new_timer1(p.TIMER1, &mut p.RESETS, &clocks);
    info!("Creating Sdcard...");
    let sd: PimodoriSdCard = SdCard::new(sd_dev, delay1);

    if let Err(e) = sd.num_bytes() {
        defmt::panic!("SD Card error: {}", e);
    }

    info!("Creating TSPimodoriSdCard...");
    let sd = TSPimodoriSdCard::new(sd);

    // Init Serial
    // CrossFire is spec at 418_000 but all implementations use 420_000.
    let csrf_uart_cfg = UartConfig::new(420_000.Hz(), DataBits::Eight, None, StopBits::One);

    let csrf_uart: SerialPort = UartPeripheral::new(p.UART0, (tx, rx), &mut p.RESETS)
        .enable(csrf_uart_cfg, clocks.peripheral_clock.freq())
        .expect("Could not create UART peripheral");

    cu_embedded_registry::register(0, csrf_uart)
        .expect("Failed to register UART as CRSF serial port");

    info!("Setting up Copper...");

    let clock = build_clock(timer_for_clock);

    let Ok(Some((blk_id, blk_len))) = find_copper_partition(&sd) else {
        panic!(
            "Could not find the copper partition on the SDCard. Format it with the Copper SD script."
        );
    };

    info!(
        "Found copper partition at block {} with length {}",
        blk_id.0, blk_len.0
    );

    let writer = Arc::new(Mutex::new(
        EMMCLogger::new(sd, blk_id, blk_len).expect("Could not create EMMC logger"),
    ));

    let mut app = BdshotDemoApp::new(clock, writer).unwrap();
    info!("Starting cu-bdshot-demo...");

    let _ = <BdshotDemoApp as CuApplication<
        EMMCSectionStorage<TSPimodoriSdCard>,
        EMMCLogger<TSPimodoriSdCard>,
    >>::run(&mut app);
    error!("Copper crashed.");
    #[allow(clippy::empty_loop)]
    loop {}
}

fn build_clock(timer: Timer0) -> RobotClock {
    RobotClock::new_with_rtc(
        {
            let timer_for_counter = timer;
            move || timer_for_counter.get_counter().ticks() * 1_000
        },
        {
            let timer_for_wait = timer;
            move |ns| {
                let start = timer_for_wait.get_counter().ticks();
                let wait_us = ns / 1_000;
                while timer_for_wait.get_counter().ticks().wrapping_sub(start) < wait_us {
                    core::hint::spin_loop();
                }
            }
        },
    )
}

fn init_psram_and_allocator<I, F, P>(cs1: Pin<I, F, P>)
where
    I: PinId + ValidFunction<FunctionXipCs1>,
    F: Function,
    P: PullType,
{
    unsafe extern "C" {
        static mut __psram_heap_start__: u8;
        static mut __psram_heap_end__: u8;
    }

    let _ = cs1.into_function::<FunctionXipCs1>();

    unsafe {
        let xip = &*pac::XIP_CTRL::ptr();
        xip.ctrl().modify(|_, w| w.writable_m1().set_bit());

        let start = &raw mut __psram_heap_start__ as *mut _ as usize;
        let end = &raw mut __psram_heap_end__ as *mut _ as usize;
        ALLOC.lock().init(start, end - start);
    }
}

fn quick_memory_test() {
    info!("Testing Memory...");
    mem_stats();
    let mut v: Vec<u8> = vec![0u8; 4 * 1024];
    for (i, item) in v.iter_mut().enumerate() {
        *item = (i % 256) as u8;
    }
    info!("After 4kB allocation...");
    mem_stats();
    for (i, item) in v.iter().enumerate() {
        assert_eq!(*item, (i % 256) as u8);
    }
    info!("Memory test passed.");
}

fn mem_stats() {
    let current = ALLOC.lock().stats_alloc_actual();
    let total = ALLOC.lock().stats_total_bytes();
    info!("heap: current={}", current);
    info!("heap: total={}", total);
}
