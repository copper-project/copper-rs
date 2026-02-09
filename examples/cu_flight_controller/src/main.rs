#![no_std]
#![no_main]

extern crate alloc;
extern crate cu29 as bevy;

use alloc::sync::Arc;
use alloc::vec;
use buddy_system_allocator::LockedHeap as StdHeap;
use cortex_m::{asm, peripheral::SCB};
use cortex_m_rt::{ExceptionFrame, entry, exception};
use cu_micoairh743::{LogStorage, Logger, MicoAirH743Id};
use cu29::prelude::*;
use defmt_rtt as _;
use panic_probe as _;
use spin::Mutex;

mod messages;
mod tasks;

#[global_allocator]
static ALLOC: StdHeap<32> = StdHeap::empty();
static mut HEAP_MEM: [u8; 256 * 1024] = [0; 256 * 1024];
const HARDFAULT_MAGIC: u32 = 0xDEAD_BEEF;

#[repr(C)]
struct HardFaultSnapshot {
    magic: u32,
    pc: u32,
    lr: u32,
    sp: u32,
    cfsr: u32,
    hfsr: u32,
    dfsr: u32,
    mmfar: u32,
    bfar: u32,
    afsr: u32,
}

// SAFETY: Keep the last hard fault snapshot in a no-init section across resets.
#[unsafe(link_section = ".uninit.cu_fault")]
static mut LAST_HARDFAULT: HardFaultSnapshot = HardFaultSnapshot {
    magic: 0,
    pc: 0,
    lr: 0,
    sp: 0,
    cfsr: 0,
    hfsr: 0,
    dfsr: 0,
    mmfar: 0,
    bfar: 0,
    afsr: 0,
};

#[copper_runtime(config = "copperconfig.ron")]
struct FlightControllerApp {}

#[defmt::panic_handler]
fn defmt_panic() -> ! {
    panic_probe::hard_fault()
}

#[entry]
fn main() -> ! {
    report_last_fault();
    init_heap();

    let mut resources = FlightControllerApp::init_resources().unwrap_or_else(|e| {
        defmt::error!("Resource init failed: {}", e); // defmt here because we don't have a copper logger yet.
        loop {
            asm::wfi();
        }
    });

    let logger_key = app_resources::bundles::FC.key::<Logger, _>(MicoAirH743Id::Logger);
    let logger = resources.resources.take(logger_key).unwrap_or_else(|_e| {
        error!("Logger resource missing");
        loop {
            asm::wfi();
        }
    });
    let logger = logger.0;

    // Spin up Copper runtime with CRSF -> mapper -> sink graph.
    let clock = build_clock();
    let writer = Arc::new(Mutex::new(logger));

    if let Ok(mut app) = FlightControllerApp::new_with_resources(clock, writer, resources) {
        log_heap_stats("before-run");
        let _ = <FlightControllerApp as CuApplication<LogStorage, Logger>>::run(&mut app);
    } else {
        error!("App init failed");
    }
    loop {
        asm::wfi();
    }
}

const SYSCLK_HZ: u64 = 400_000_000;

fn read_rtc_ns() -> u64 {
    let cycles = cu29::clock::read_raw_counter() as u128;
    ((cycles * 1_000_000_000u128) / SYSCLK_HZ as u128) as u64
}

fn sleep_ns(ns: u64) {
    let cycles = ((ns as u128 * SYSCLK_HZ as u128) / 1_000_000_000u128) as u32;
    asm::delay(cycles.max(1));
}

fn build_clock() -> RobotClock {
    cu29::clock::initialize();
    RobotClock::new_with_rtc(read_rtc_ns, sleep_ns)
}

fn report_last_fault() {
    // SAFETY: Single-core firmware; only this function mutates the fault snapshot.
    unsafe {
        let ptr = core::ptr::addr_of_mut!(LAST_HARDFAULT);
        if (*ptr).magic == HARDFAULT_MAGIC {
            defmt::error!(
                "Last HardFault: pc=0x{:x} lr=0x{:x} sp=0x{:x}",
                (*ptr).pc,
                (*ptr).lr,
                (*ptr).sp
            );
            defmt::error!(
                "Last Fault status: CFSR=0x{:x} HFSR=0x{:x} DFSR=0x{:x} MMFAR=0x{:x} BFAR=0x{:x} AFSR=0x{:x}",
                (*ptr).cfsr,
                (*ptr).hfsr,
                (*ptr).dfsr,
                (*ptr).mmfar,
                (*ptr).bfar,
                (*ptr).afsr
            );
            (*ptr).magic = 0;
        }
    }
}

fn init_heap() {
    // SAFETY: HEAP_MEM is a dedicated static buffer for the allocator.
    unsafe {
        let start = core::ptr::addr_of_mut!(HEAP_MEM) as usize;
        let size = core::mem::size_of::<[u8; 256 * 1024]>();
        ALLOC.lock().init(start, size);
        defmt::info!("Heap region start={} size={}", start, size);
    }
}

pub(crate) fn heap_stats() -> (usize, usize, usize, usize) {
    let heap = ALLOC.lock();
    let user = heap.stats_alloc_user();
    let allocated = heap.stats_alloc_actual();
    let total = heap.stats_total_bytes();
    let free = total.saturating_sub(allocated);
    (user, allocated, total, free)
}

fn log_heap_stats(tag: &str) {
    let (user, allocated, total, free) = heap_stats();
    info!(
        "Heap stats({}): user={} alloc={} total={} free={}",
        tag, user, allocated, total, free
    );
}

#[exception]
// SAFETY: Only invoked by the MCU on a hard fault; must not return.
unsafe fn HardFault(ef: &ExceptionFrame) -> ! {
    // SAFETY: SCB is a valid core peripheral register block.
    let scb = unsafe { &*SCB::PTR };
    let cfsr = scb.cfsr.read();
    let hfsr = scb.hfsr.read();
    let dfsr = scb.dfsr.read();
    let mmfar = scb.mmfar.read();
    let bfar = scb.bfar.read();
    let afsr = scb.afsr.read();
    // SAFETY: We record the fault snapshot into a reserved static buffer.
    unsafe {
        LAST_HARDFAULT = HardFaultSnapshot {
            magic: HARDFAULT_MAGIC,
            pc: ef.pc(),
            lr: ef.lr(),
            sp: cortex_m::register::msp::read(),
            cfsr,
            hfsr,
            dfsr,
            mmfar,
            bfar,
            afsr,
        };
    }
    loop {
        asm::bkpt();
    }
}
