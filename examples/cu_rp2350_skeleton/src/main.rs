#![no_std]
#![no_main]

extern crate alloc;

use alloc::sync::Arc;
use alloc::vec;
use alloc::vec::Vec;
use cortex_m_rt::entry;
use hal::{clocks::init_clocks_and_plls, gpio::Pins, pac, sio::Sio, watchdog::Watchdog};
use rp235x_hal as hal;
use rp235x_hal::gpio::bank0::{Gpio15, Gpio16, Gpio17, Gpio18, Gpio19};

use crate::bmlogger::{EMMCSectionStorage, ForceSyncSend};
use bmlogger::EMMCLogger;
use buddy_system_allocator::LockedHeap as Heap;
use cu29::prelude::*;
use defmt_rtt as _;
use embedded_hal::spi::MODE_0;
use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_sdmmc::SdCard;
use panic_probe as _;
use rp235x_hal::fugit::RateExtU32;
use rp235x_hal::gpio::{
    Function, FunctionSio, FunctionSpi, FunctionXipCs1, Pin, PinId, PullDown, PullType,
    PullUp, SioInput, SioOutput, ValidFunction,
};
use rp235x_hal::pac::SPI0;
use rp235x_hal::spi::{Enabled, FrameFormat};
use rp235x_hal::timer::{CopyableTimer0, CopyableTimer1};
use rp235x_hal::Clock;
use rp235x_hal::{Spi, Timer};
use spin::Mutex;

// --- Copper runtime
mod bmlogger;
pub mod tasks;

#[copper_runtime(config = "copperconfig.ron")]
struct BlinkyApp {}

// --- embedded setup
const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;

#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

#[global_allocator]
static ALLOC: Heap<32> = Heap::empty();

/// Initialize the PSRAM and maps the global allocator to it.
fn init_psram_and_allocator<I, F, P>(cs1: Pin<I, F, P>)
where
    I: PinId + ValidFunction<FunctionXipCs1>, // need to be able to convert to XIP_CS1
    F: Function,
    P: PullType,
{
    unsafe extern "C" {
        static mut __psram_heap_start__: u8;
        static mut __psram_heap_end__: u8;
    }

    // This is the dance to enable the external PSRAM on the rp235xh
    // 1) GPIO47 function => XIP_CS1
    let _ = cs1.into_function::<FunctionXipCs1>();

    unsafe {
        // 2) Enable writes to M1 (cached @0x1100_0000 and mapped by memory.x)
        let xip = &*pac::XIP_CTRL::ptr();
        xip.ctrl().modify(|_, w| w.writable_m1().set_bit());

        let start = &raw mut __psram_heap_start__ as *mut _ as usize;
        let end = &raw mut __psram_heap_end__ as *mut _ as usize;
        ALLOC.lock().init(start, end - start);
    }
}

// Validates that the allocator over PSRAM is working correctly.
fn quick_memory_test() {
    defmt::info!("Testing Memory... ");

    mem_stats();
    let mut v: Vec<u8> = vec![0u8; 4 * 1024];
    for (i , item) in v.iter_mut().enumerate() {
        *item = (i % 256) as u8;
    }

    defmt::info!("After 1 4kB allocation... ");
    mem_stats();
    for (i, item) in v.iter().enumerate() {
        assert_eq!(*item, (i % 256) as u8);
    }
    defmt::info!("Memory test passed.");
}

// Types to bind to the Pimodori pico plus SDCard Cowbell.
// I am not sure why embedded does that to us.
// Pinout
type Mosi = Pin<Gpio19, FunctionSpi, PullDown>;
type Miso = Pin<Gpio16, FunctionSpi, PullDown>;
type Sck = Pin<Gpio18, FunctionSpi, PullDown>;
type Cs = Pin<Gpio17, FunctionSio<SioOutput>, PullDown>;
type Det = Pin<Gpio15, FunctionSio<SioInput>, PullUp>;

// Timers
type Timer0 = Timer<CopyableTimer0>;
type Timer1 = Timer<CopyableTimer1>;

// Buses
type Spi0 = Spi<Enabled, SPI0, (Mosi, Miso, Sck)>;

type SdDev = ExclusiveDevice<Spi0, Cs, Timer0>;
type PimodoriSdCard = SdCard<SdDev, Timer1>;
type TSPimodoriSdCard = ForceSyncSend<SdCard<SdDev, Timer1>>;

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

    let timer: Timer0 = Timer::new_timer0(p.TIMER0, &mut p.RESETS, &clocks);

    let sio = Sio::new(p.SIO);
    let pins = Pins::new(p.IO_BANK0, p.PADS_BANK0, sio.gpio_bank0, &mut p.RESETS);

    // Now we have a comfortable heap to play with.
    init_psram_and_allocator(pins.gpio47);
    quick_memory_test();

    // Set up the SD card
    info!("Setting up the SDCard...");

    let miso: Miso = pins.gpio16.into_function();
    let cs: Cs = pins.gpio17.into_push_pull_output();
    let sck: Sck = pins.gpio18.into_function();
    let mosi: Mosi = pins.gpio19.into_function();
    let det: Det = pins.gpio15.into_pull_up_input();

    let sd_spi: Spi0 = Spi::<_, _, _, 8>::new(p.SPI0, (mosi, miso, sck)).init(
        &mut p.RESETS,
        clocks.peripheral_clock.freq(),
        16_000_000u32.Hz(),
        FrameFormat::MotorolaSpi(MODE_0),
    );

    let sd_dev: SdDev = ExclusiveDevice::new(sd_spi, cs, timer).unwrap();
    let _sd_det = Some(det);
    let delay1: Timer1 = Timer::new_timer1(p.TIMER1, &mut p.RESETS, &clocks);
    let sd: PimodoriSdCard = SdCard::new(sd_dev, delay1);

    if let Err(e) = sd.num_bytes() {
        defmt::panic!("SD Card error: {}", e);
    }

    let sd = TSPimodoriSdCard::new(sd);

    info!("Setting up Copper...");

    let led = pins.gpio20.into_push_pull_output();
    tasks::register_led(led);

    // start copper
    let clock = RobotClock::new();

    let Ok(Some((blk_id, blk_len))) = bmlogger::find_copper_partition(&sd) else {
        panic!("Could not find the copper partition on the SDCard. Be sure to format the sdcard with the Copper formatting script.");
    };

    info!(
        "Found copper partition at block {} with length {}",
        blk_id.0, blk_len.0
    );

    let writer = Arc::new(Mutex::new(
        EMMCLogger::new(sd, blk_id, blk_len).expect("Could not create EMMCLogger"),
    ));

    let mut app = BlinkyApp::new(clock, writer).unwrap();
    info!("Starting Copper...");

    let _ = <BlinkyApp as CuApplication<
        EMMCSectionStorage<TSPimodoriSdCard>,
        EMMCLogger<TSPimodoriSdCard>,
    >>::run(&mut app);
    defmt::error!("Copper crashed.");
    #[allow(clippy::empty_loop)]
    loop {}
}

fn mem_stats() {
    let current = ALLOC.lock().stats_alloc_actual();
    let total = ALLOC.lock().stats_total_bytes();

    defmt::info!("heap: current={}", current);
    defmt::info!("heap: total={}", total);
}
