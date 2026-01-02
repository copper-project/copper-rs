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
use defmt::info;
use defmt_rtt as _;
use panic_probe as _;
use spin::Mutex;

mod messages;
mod resources;
mod tasks;

pub use resources::{
    GreenLed, LogStorage, Logger, MicoAirH743Id, SerialPort, SerialPortError, Uart2Port, Uart6Port,
};

#[global_allocator]
static ALLOC: StdHeap<32> = StdHeap::empty();
static mut HEAP_MEM: [u8; 256 * 1024] = [0; 256 * 1024];

#[copper_runtime(config = "copperconfig.ron")]
struct FlightControllerApp {}

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
            defmt::error!("Resource init failed: {:?}", e);
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
            defmt::error!("Logger resource missing: {:?}", e);
            loop {
                asm::wfi();
            }
        }
    };
    defmt::info!("Logger resource acquired");

    // Spin up Copper runtime with CRSF -> mapper -> sink graph.
    info!("Building Copper runtime clock");
    let clock = build_clock();
    info!("Clock ready, constructing app");
    let writer = Arc::new(Mutex::new(logger));

    match FlightControllerApp::new_with_resources(clock, writer, resources) {
        Ok(mut app) => {
            info!("App constructed, starting Copper FC (H743, CRSF on UART6)...");
            log_heap_stats("before-run");
            let _ = <FlightControllerApp as CuApplication<LogStorage, Logger>>::run(&mut app);
            info!("Copper runtime returned unexpectedly");
        }
        Err(e) => {
            defmt::error!("App init failed: {:?}", e);
        }
    }
    loop {
        asm::wfi();
    }
}

fn build_clock() -> RobotClock {
    // Use a mock clock to avoid calibration/division at startup.
    let (clock, _mock) = RobotClock::mock();
    clock
}

fn init_heap() {
    unsafe {
        let start = core::ptr::addr_of_mut!(HEAP_MEM) as usize;
        let size = core::mem::size_of::<[u8; 256 * 1024]>();
        defmt::info!("Heap region start=0x{:x} size={}", start, size);
        ALLOC.lock().init(start, size);
    }
}

fn log_heap_stats(tag: &str) {
    let heap = ALLOC.lock();
    let user = heap.stats_alloc_user();
    let allocated = heap.stats_alloc_actual();
    let total = heap.stats_total_bytes();
    let free = total.saturating_sub(allocated);
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
    defmt::error!(
        "HardFault: pc=0x{:x} lr=0x{:x} sp=0x{:x}",
        ef.pc(),
        ef.lr(),
        cortex_m::register::msp::read()
    );
    defmt::error!(
        "Fault status: CFSR=0x{:x} HFSR=0x{:x} DFSR=0x{:x} MMFAR=0x{:x} BFAR=0x{:x} AFSR=0x{:x}",
        cfsr,
        hfsr,
        dfsr,
        mmfar,
        bfar,
        afsr
    );
    loop {
        asm::bkpt();
    }
}
