#![no_std]
#![no_main]

extern crate alloc;

use alloc::sync::Arc;
use alloc::vec;
use buddy_system_allocator::LockedHeap as StdHeap;
use cortex_m::{asm, peripheral::SCB};
use cortex_m_rt::{ExceptionFrame, entry, exception};
use cu_sensor_payloads::ImuPayload;
use cu29::prelude::*;
use cu_micoairh743::{LogStorage, Logger, MicoAirH743Id};
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

// Dummy IMU source placeholder.
pub struct DummyImu;
pub type RpMpu9250Source = DummyImu;
impl Freezable for DummyImu {}

impl CuTask for DummyImu {
    type Resources<'r> = ();
    type Input<'m> = ();
    type Output<'m> = CuMsg<ImuPayload>;

    fn new_with(
        _config: Option<&ComponentConfig>,
        _resources: Self::Resources<'_>,
    ) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self)
    }

    fn process<'i, 'o>(
        &mut self,
        _clock: &RobotClock,
        _inputs: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> CuResult<()> {
        output.clear_payload();
        Ok(())
    }
}

#[entry]
fn main() -> ! {
    report_last_fault();
    init_heap();
    defmt::info!("Heap initialized");
    log_heap_stats("after-init");

    // Smoke-test allocator.
    let mut buf = alloc::vec![0u8; 1024];
    for (i, b) in buf.iter_mut().enumerate() {
        *b = (i & 0xFF) as u8;
    }
    defmt::info!("Heap alloc test passed: len={}", buf.len());

    let mut resources = match FlightControllerApp::init_resources() {
        Ok(resources) => resources,
        Err(e) => {
            let _ = e;
            defmt::error!("Resource init failed");
            loop {
                asm::wfi();
            }
        }
    };
    defmt::info!("Board resources initialized");

    let logger_key = app_resources::bundles::FC.key::<Logger, _>(MicoAirH743Id::Logger);
    let logger = match resources.resources.take(logger_key) {
        Ok(logger) => logger.0,
        Err(e) => {
            let _ = e;
            defmt::error!("Logger resource missing");
            loop {
                asm::wfi();
            }
        }
    };
    defmt::info!("Logger resource acquired");

    // Spin up Copper runtime with CRSF -> mapper -> sink graph.
    defmt::info!("Building Copper runtime clock");
    let clock = build_clock();
    defmt::info!("Clock ready, constructing app");
    let writer = Arc::new(Mutex::new(logger));

    match FlightControllerApp::new_with_resources(clock, writer, resources) {
        Ok(mut app) => {
            defmt::info!("App constructed, starting Copper FC (H743, CRSF on UART6)...");
            log_heap_stats("before-run");
            let _ = <FlightControllerApp as CuApplication<LogStorage, Logger>>::run(&mut app);
            defmt::info!("Copper runtime returned unexpectedly");
        }
        Err(e) => {
            let _ = e;
            defmt::error!("App init failed");
        }
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
    unsafe {
        if LAST_HARDFAULT.magic == HARDFAULT_MAGIC {
            defmt::error!(
                "Last HardFault: pc=0x{:x} lr=0x{:x} sp=0x{:x}",
                LAST_HARDFAULT.pc,
                LAST_HARDFAULT.lr,
                LAST_HARDFAULT.sp
            );
            defmt::error!(
                "Last Fault status: CFSR=0x{:x} HFSR=0x{:x} DFSR=0x{:x} MMFAR=0x{:x} BFAR=0x{:x} AFSR=0x{:x}",
                LAST_HARDFAULT.cfsr,
                LAST_HARDFAULT.hfsr,
                LAST_HARDFAULT.dfsr,
                LAST_HARDFAULT.mmfar,
                LAST_HARDFAULT.bfar,
                LAST_HARDFAULT.afsr
            );
            LAST_HARDFAULT.magic = 0;
        }
    }
}

fn init_heap() {
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
    defmt::info!(
        "Heap stats({}): user={} alloc={} total={} free={}",
        tag,
        user,
        allocated,
        total,
        free
    );
}

#[exception]
unsafe fn HardFault(ef: &ExceptionFrame) -> ! {
    let scb = unsafe { &*SCB::PTR };
    let cfsr = scb.cfsr.read();
    let hfsr = scb.hfsr.read();
    let dfsr = scb.dfsr.read();
    let mmfar = scb.mmfar.read();
    let bfar = scb.bfar.read();
    let afsr = scb.afsr.read();
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
