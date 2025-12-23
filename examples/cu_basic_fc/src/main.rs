#![no_std]
#![no_main]

extern crate alloc;

use alloc::sync::Arc;
use alloc::vec;
use buddy_system_allocator::LockedHeap as StdHeap;
use cortex_m::{asm, peripheral::DWT};
use cortex_m_rt::{ExceptionFrame, entry, exception};
use cu_bdshot::{Stm32H7Board, Stm32H7BoardResources, register_stm32h7_board};
use cu_sdlogger::{EMMCLogger, EMMCSectionStorage, ForceSyncSend, find_copper_partition};
use cu_sensor_payloads::ImuPayload;
use cu29::prelude::*;
use defmt::info;
use defmt_rtt as _;
use panic_probe as _;
use spin::Mutex;
use stm32h7xx_hal::{
    gpio::{
        Speed,
    },
    nb, pac,
    prelude::*,
    rcc,
    sdmmc::{self, SdCard, Sdmmc, SdmmcBlockDevice},
    serial::Error as UartError,
    serial::{Serial, config::Config},
};

mod tasks;

#[global_allocator]
static ALLOC: StdHeap<32> = StdHeap::empty();
static mut HEAP_MEM: [u8; 256 * 1024] = [0; 256 * 1024];

#[copper_runtime(config = "copperconfig.ron")]
struct FlightControllerApp {}

// UART6 pins: PC6 (TX) PC7 (RX), AF7.
type SerialPort = SerialWrapper;
type SerialPortError = embedded_io::ErrorKind;

type SdBlockDev = ForceSyncSend<SdmmcBlockDevice<Sdmmc<pac::SDMMC1, SdCard>>>;
type LogStorage = EMMCSectionStorage<SdBlockDev>;
type Logger = EMMCLogger<SdBlockDev>;

// Dummy IMU source placeholder.
pub struct DummyImu;
pub type RpMpu9250Source = DummyImu;
impl Freezable for DummyImu {}

static GREEN_LED: Mutex<
    Option<
        stm32h7xx_hal::gpio::gpioe::PE6<stm32h7xx_hal::gpio::Output<stm32h7xx_hal::gpio::PushPull>>,
    >,
> = Mutex::new(None);

struct SerialWrapper {
    inner: Serial<pac::USART6>,
}

// Safety: the firmware uses a single-threaded runtime; no concurrent access.
unsafe impl Send for SerialWrapper {}
unsafe impl Sync for SerialWrapper {}

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

impl embedded_io::ErrorType for SerialWrapper {
    type Error = SerialPortError;
}
impl embedded_io::Read for SerialWrapper {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        let mut read = 0;
        for b in buf.iter_mut() {
            match self.inner.read() {
                Ok(byte) => {
                    *b = byte;
                    read += 1;
                }
                Err(nb::Error::WouldBlock) => break, // no more data right now
                Err(nb::Error::Other(e)) => match e {
                    UartError::Overrun => {
                        defmt::warn!("UART6 overrun, data lost");
                        // Clear and keep going; treat as no more data.
                        break;
                    }
                    _ => {
                        defmt::error!("UART6 read err: {:?}", e);
                        return Err(SerialPortError::Other);
                    }
                },
            }
        }
        Ok(read)
    }
}
impl embedded_io::Write for SerialWrapper {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        let mut written = 0;
        for &b in buf {
            match self.inner.write(b) {
                Ok(()) => written += 1,
                Err(nb::Error::WouldBlock) => break,
                Err(nb::Error::Other(e)) => {
                    defmt::error!("UART6 write err: {:?}", e);
                    return Err(SerialPortError::Other);
                }
            }
        }
        Ok(written)
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

#[entry]
fn main() -> ! {
    // Minimal bring-up: core peripherals + clocks + heap.
    let mut cp = cortex_m::Peripherals::take().unwrap();
    cp.SCB.enable_fpu();
    cp.DCB.enable_trace();
    DWT::unlock();
    cp.DWT.enable_cycle_counter();

    let dp = pac::Peripherals::take().unwrap();

    // Clocks: mirror the known-good sdcard-test setup.
    let pwr = dp.PWR.constrain();
    let vos = pwr.freeze();
    let rcc = dp.RCC.constrain();
    let ccdr = rcc
        .sys_ck(400.MHz())
        .pll1_q_ck(100.MHz())
        .freeze(vos, &dp.SYSCFG);
    let sdmmc1_rec = ccdr.peripheral.SDMMC1;

    defmt::info!("Boot: core enabled");
    defmt::info!("Clocks configured");

    init_heap();
    defmt::info!("Heap initialized");

    // Smoke-test allocator.
    let mut buf = alloc::vec![0u8; 1024];
    for (i, b) in buf.iter_mut().enumerate() {
        *b = (i & 0xFF) as u8;
    }
    defmt::info!("Heap alloc test passed: len={}", buf.len());

    // UART6 for CRSF and SDMMC for logging.
    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);
    let gpiod = dp.GPIOD.split(ccdr.peripheral.GPIOD);
    let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);
    let tx = gpioc.pc6.into_alternate();
    let rx = gpioc.pc7.into_alternate();

    let config = Config::default().baudrate(420_000.bps());
    let serial = SerialWrapper {
        inner: dp
            .USART6
            .serial((tx, rx), config, ccdr.peripheral.USART6, &ccdr.clocks)
            .expect("uart6 init"),
    };
    defmt::info!("UART6 init done");

    // Register serial port for CRSF (slot 0).
    if let Err(e) = cu_embedded_registry::register(0, serial) {
        defmt::error!("registry uart6 failed: {:?}", e);
        panic!("registry failed");
    }
    defmt::info!("UART6 registered in registry");

    // Initialize SD card for logging
    defmt::info!("Starting SD logger init");
    let logger = match init_sd_logger(
        dp.SDMMC1,
        sdmmc1_rec,
        &ccdr.clocks,
        (
            gpioc.pc12.into_alternate::<12>().speed(Speed::VeryHigh),
            gpiod.pd2.into_alternate::<12>().speed(Speed::VeryHigh),
            gpioc.pc8.into_alternate::<12>().speed(Speed::VeryHigh),
            gpioc.pc9.into_alternate::<12>().speed(Speed::VeryHigh),
            gpioc.pc10.into_alternate::<12>().speed(Speed::VeryHigh),
            gpioc.pc11.into_alternate::<12>().speed(Speed::VeryHigh),
        ),
    ) {
        Ok(l) => l,
        Err(e) => {
            defmt::error!("SD logger init failed: {}", e);
            loop {
                asm::wfi();
            }
        }
    };
    defmt::info!("SD logger ready");

    // Register BDShot board (STM32H7, GPIOE PE14/13/11/9) before Copper runtime start.
    let bdshot_resources = Stm32H7BoardResources {
        gpioe,
        dwt: cp.DWT,
        sysclk_hz: ccdr.clocks.sys_ck().raw(),
    };
    let bdshot_board = match Stm32H7Board::new(bdshot_resources) {
        Ok(board) => board,
        Err(e) => {
            defmt::error!("BDShot board init failed: {:?}", e);
            loop {
                asm::wfi();
            }
        }
    };
    if let Err(e) = register_stm32h7_board(bdshot_board) {
        defmt::error!("BDShot board registration failed: {:?}", e);
        loop {
            asm::wfi();
        }
    }
    defmt::info!("BDShot board registered");

    // Spin up Copper runtime with CRSF -> mapper -> sink graph.
    info!("Building Copper runtime clock");
    let clock = build_clock();
    info!("Clock ready, constructing app");
    let writer = Arc::new(Mutex::new(logger));

    match <FlightControllerApp as CuApplication<LogStorage, Logger>>::new(clock, writer) {
        Ok(mut app) => {
            info!("App constructed, starting Copper FC (H743, CRSF on UART6)...");
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

fn init_sd_logger(
    sdmmc1: pac::SDMMC1,
    sdmmc1_rec: rcc::rec::Sdmmc1,
    clocks: &rcc::CoreClocks,
    pins: (
        impl sdmmc::PinClk<pac::SDMMC1>,
        impl sdmmc::PinCmd<pac::SDMMC1>,
        impl sdmmc::PinD0<pac::SDMMC1>,
        impl sdmmc::PinD1<pac::SDMMC1>,
        impl sdmmc::PinD2<pac::SDMMC1>,
        impl sdmmc::PinD3<pac::SDMMC1>,
    ),
) -> Result<Logger, CuError> {
    defmt::info!("Basic Init done, creating sdmmc...");
    let mut sdmmc: Sdmmc<_, SdCard> = sdmmc1.sdmmc(pins, sdmmc1_rec, clocks);

    defmt::info!("Init()... ");
    sdmmc
        .init(25.MHz())
        .map_err(|_| CuError::from("SDMMC init failed"))?;
    defmt::info!("Init passed ... ");
    sdmmc.card().map_err(|_| CuError::from("no sd card"))?;
    defmt::info!("Card detected!!");

    let bd = ForceSyncSend::new(sdmmc.sdmmc_block_device());

    let (start, count) = match find_copper_partition(&bd) {
        Ok(Some((s, c))) => (s, c),
        Ok(None) => {
            defmt::warn!("Copper partition not found on SD");
            return Err(CuError::from("copper partition missing"));
        }
        Err(_) => {
            defmt::warn!("SD read error during partition scan");
            return Err(CuError::from("sd read error during partition scan"));
        }
    };

    Logger::new(bd, start, count)
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

#[exception]
unsafe fn HardFault(ef: &ExceptionFrame) -> ! {
    defmt::error!(
        "HardFault: pc=0x{:x} lr=0x{:x} sp=0x{:x}",
        ef.pc(),
        ef.lr(),
        cortex_m::register::msp::read()
    );
    loop {
        asm::bkpt();
    }
}
