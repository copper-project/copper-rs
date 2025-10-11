#![no_std]
#![no_main]

extern crate alloc;

use alloc::boxed::Box;
use alloc::sync::Arc;
use alloc::vec;
use alloc::vec::Vec;
use bincode::config::standard;
use bincode::encode_into_slice;
use cortex_m_rt::entry;
use hal::{clocks::init_clocks_and_plls, gpio::Pins, pac, sio::Sio, watchdog::Watchdog};
use rp235x_hal as hal;

use buddy_system_allocator::LockedHeap as Heap;
use cu29::prelude::*;
use defmt_rtt as _;
use panic_probe as _;
use rp235x_hal::gpio::{Function, FunctionXipCs1, Pin, PinId, PullType, ValidFunction};
use spin::Mutex;

// --- Copper runtime
pub mod tasks;
#[copper_runtime(config = "copperconfig.ron")]
struct EmbeddedApp {}

// FIXME(gbin): Actually implement the SDCARD logger.
// this is just a placeholder.
struct MyEmbeddedLogger {}

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
    for i in 0..v.len() {
        v[i] = (i % 256) as u8;
    }

    defmt::info!("After 1 4kB allocation... ");
    mem_stats();
    for i in 0..v.len() {
        assert_eq!(v[i], (i % 256) as u8);
    }
    defmt::info!("Memory test passed.");
}

impl UnifiedLogWrite for MyEmbeddedLogger {
    fn add_section(
        &mut self,
        entry_type: UnifiedLogType,
        _requested_section_size: usize,
    ) -> SectionHandle {
        use alloc::vec::Vec;
        // FIXME(gbin): this is just a hack to make it compile, it needs to be correctly implemented.
        // It minimalistically mimic the std implementation over the mmap file but with a memory leak.
        let buf: Vec<u8> = vec![0u8; 10000];
        let boxed: Box<[u8]> = buf.into_boxed_slice();
        let slice_static: &'static mut [u8] = Box::leak(boxed);

        let section_header = SectionHeader {
            magic: SECTION_MAGIC,
            entry_type,
            section_size: 10000u32,
            filled_size: 0u32,
        };

        encode_into_slice(&section_header, slice_static, standard())
            .expect("Failed to encode section header");

        SectionHandle::create(section_header, slice_static)
    }

    fn flush_section(&mut self, _section: &mut SectionHandle) {
        // no op for now
    }

    fn status(&self) -> UnifiedLogStatus {
        // no op for now
        UnifiedLogStatus {
            total_used_space: 0,
            total_allocated_space: 0,
        }
    }
}

#[entry]
fn main() -> ! {
    let mut p = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(p.WATCHDOG);
    let _clocks = init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        p.XOSC,
        p.CLOCKS,
        p.PLL_SYS,
        p.PLL_USB,
        &mut p.RESETS,
        &mut watchdog,
    )
    .unwrap();
    // let timer: Timer<CopyableTimer0> = Timer::new_timer0(p.TIMER0, &mut p.RESETS, &clocks);

    let sio = Sio::new(p.SIO);
    let pins = Pins::new(p.IO_BANK0, p.PADS_BANK0, sio.gpio_bank0, &mut p.RESETS);

    // Now we have a comfortable heap to play with.
    init_psram_and_allocator(pins.gpio47);
    quick_memory_test();

    info!("Setting up Copper...");

    let led = pins.gpio20.into_push_pull_output();
    tasks::register_led(led);

    // start copper
    let clock = RobotClock::new();
    let writer = Arc::new(Mutex::new(MyEmbeddedLogger {}));

    let mut app = EmbeddedApp::new(clock, writer).unwrap();
    info!("Starting Copper...");

    let _ = <EmbeddedApp as CuApplication<MyEmbeddedLogger>>::run(&mut app);
    defmt::error!("Copper crashed.");
    loop {}
}

fn mem_stats() {
    let current = ALLOC.lock().stats_alloc_actual();
    let total = ALLOC.lock().stats_total_bytes();

    defmt::info!("heap: current={}", current);
    defmt::info!("heap: total={}", total);
}
