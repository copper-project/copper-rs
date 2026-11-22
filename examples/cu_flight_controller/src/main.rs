#![no_std]
#![no_main]

extern crate alloc;
extern crate cu29 as bevy;

#[cfg(feature = "sdcard-logging")]
use alloc::sync::Arc;
use alloc::vec;
use buddy_system_allocator::LockedHeap as StdHeap;
use cortex_m::{asm, peripheral::SCB};
use cortex_m_rt::{ExceptionFrame, entry, exception};
#[cfg(feature = "sdcard-logging")]
use cu_micoairh743::{Logger, MicoAirH743Id};
#[cfg(feature = "sdcard-logging")]
mod deferred_sd;
use cu29::prelude::*;
use defmt_rtt as _;
use panic_probe as _;
#[cfg(feature = "sdcard-logging")]
use spin::Mutex;

mod messages;
mod tasks;

#[global_allocator]
static ALLOC: StdHeap<32> = StdHeap::empty();
const HEAP_BYTES: usize = 256 * 1024;
static mut HEAP_MEM: [u8; HEAP_BYTES] = [0; HEAP_BYTES];
#[cfg(feature = "sdcard-logging")]
#[unsafe(link_section = ".sd_heap")]
static mut SD_HEAP: core::mem::MaybeUninit<[u8; deferred_sd::POOL_BYTES]> =
    core::mem::MaybeUninit::uninit();
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

#[copper_runtime(config = "flight_controller.ron", subsystem = "mcu")]
struct FlightControllerApp {}

#[defmt::panic_handler]
fn defmt_panic() -> ! {
    panic_probe::hard_fault()
}

#[entry]
fn main() -> ! {
    report_last_fault();
    init_heap();

    if let Err(err) = run_controller() {
        defmt::error!("Controller failed: {}", &err);
    }
    loop {
        asm::wfi();
    }
}

const SYSCLK_HZ: u64 = 400_000_000;

fn run_controller() -> CuResult<()> {
    #[cfg(feature = "sdcard-logging")]
    let (queue, output) = deferred_sd::queue();
    // The board bundle enables the DWT cycle counter used by RobotClock.
    #[allow(unused_mut)]
    let mut resources = FlightControllerApp::prepare_resources()?.resources;
    let clock = RobotClock::from_counter_frequency(core::num::NonZeroU64::new(SYSCLK_HZ).unwrap());
    let builder = FlightControllerApp::builder().with_clock(clock);
    #[cfg(feature = "sdcard-logging")]
    {
        let key = app_resources::bundles::FC.key::<Logger, _>(MicoAirH743Id::Logger);
        let recorder = Arc::new(Mutex::new(deferred_sd::Recorder::new(
            resources.take(key)?.0,
            queue,
        )));
        let app = builder
            .with_logger::<deferred_sd::PendingSection, _>(recorder.clone())
            .with_resources(move |_| Ok(resources))
            .build()?
            .start()?;
        let mut worker = recorder.lock().worker(output);
        type RunningController = CuAppLifecycle<
            deferred_sd::PendingSection,
            deferred_sd::Recorder,
            FlightControllerApp,
            Running,
        >;
        static CONTROL: Mutex<Option<RunningController>> = Mutex::new(None);
        static CONTROL_ERROR: Mutex<Option<CuError>> = Mutex::new(None);
        fn control() {
            let mut controller = CONTROL.lock();
            let Some(app) = controller.as_mut() else {
                return;
            };
            if let Err(error) = app.run_one_iteration() {
                // Stop the tasks immediately, including when the worker yielded
                // from inside a card transfer. Shutdown records go into the queue.
                if let Some(app) = controller.take() {
                    drop(app.stop());
                }
                *CONTROL_ERROR.lock() = Some(error);
            }
        }
        *CONTROL.lock() = Some(app);

        #[cfg(feature = "textlogs")]
        log_heap_stats("before-run");
        let mut iterations = 0u32;
        loop {
            control();
            if let Some(error) = CONTROL_ERROR.lock().take() {
                if let Err(flush) = worker.finish() {
                    defmt::error!("SD shutdown: {}", flush);
                }
                return Err(error);
            }
            cu_micoairh743::sd_dma::service_with(
                || {
                    for _ in 0..4 {
                        worker.service();
                    }
                },
                control,
            );
            iterations = iterations.wrapping_add(1);
            if iterations.is_multiple_of(16384) {
                let (consumed, dropped) = worker.counts();
                defmt::info!("SD drained={} dropped={}", consumed, dropped);
            }
        }
    }
    #[cfg(not(feature = "sdcard-logging"))]
    {
        let app = builder.with_resources(move |_| Ok(resources)).build()?;
        #[cfg(feature = "textlogs")]
        log_heap_stats("before-run");
        app.run_until_shutdown()?;
        Ok(())
    }
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
        let size = HEAP_BYTES;
        ALLOC.lock().init(start, size);
        #[cfg(feature = "sdcard-logging")]
        {
            // Startup owns RCC. Enable both D2 SRAM banks before the allocator
            // writes their free list. Leave AXI RAM available for the main stack.
            let rcc = &*stm32h7xx_hal::pac::RCC::ptr();
            rcc.ahb2enr
                .modify(|_, w| w.sram1en().set_bit().sram2en().set_bit());
            let _ = rcc.ahb2enr.read();
            cortex_m::asm::dsb();
            let sd_start = core::ptr::addr_of_mut!(SD_HEAP) as usize;
            ALLOC
                .lock()
                .add_to_heap(sd_start, sd_start + deferred_sd::POOL_BYTES);
        }

        defmt::info!("Heap region start={} size={}", start, size);
    }
}

#[cfg(feature = "textlogs")]
fn heap_stats() -> (usize, usize, usize, usize) {
    let heap = ALLOC.lock();
    let user = heap.stats_alloc_user();
    let allocated = heap.stats_alloc_actual();
    let total = heap.stats_total_bytes();
    let free = total.saturating_sub(allocated);
    (user, allocated, total, free)
}

#[cfg(feature = "textlogs")]
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
