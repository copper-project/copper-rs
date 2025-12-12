#![no_std]
#![no_main]

extern crate alloc;

use alloc::sync::Arc;
use cortex_m::{asm, peripheral::DWT};
use cortex_m_rt::{entry, exception, ExceptionFrame};
use cu29::prelude::*;
use defmt::info;
use defmt_rtt as _;
use buddy_system_allocator::LockedHeap as StdHeap;
use panic_probe as _;
use spin::Mutex;
use stm32h7xx_hal::{
    gpio::{
        gpioc::{PC6, PC7},
        Alternate, Speed,
    },
    pac,
    prelude::*,
    rcc,
    sdmmc::{self, SdCard, Sdmmc, SdmmcBlockDevice},
    serial::Error as UartError,
    nb,
    serial::{config::Config, Serial},
};
use cu_sensor_payloads::ImuPayload;
use cu_sdlogger::{find_copper_partition, EMMCLogger, EMMCSectionStorage, ForceSyncSend};
use alloc::vec;

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

static GREEN_LED: Mutex<Option<stm32h7xx_hal::gpio::gpioe::PE6<stm32h7xx_hal::gpio::Output<stm32h7xx_hal::gpio::PushPull>>>> = Mutex::new(None);

type Uart6Pins = (PC6<Alternate<7>>, PC7<Alternate<7>>);

struct SerialWrapper {
    inner: Serial<pac::USART6>,
}

impl CuTask for DummyImu {
    type Input<'m> = ();
    type Output<'m> = CuMsg<ImuPayload>;

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self> {
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

    // Clocks: match the INAV setup (480 MHz SYSCLK, 240 MHz HCLK, 120 MHz PCLKs)
    // and feed SDMMC from a 200 MHz PLL2R kernel clock.
    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.freeze();
    let rcc = dp.RCC.constrain();
    // Clocks: 25MHz HSE -> 400 MHz SYS, HCLK 200, PCLKs 100, PLL2R 200 for SDMMC.
    let mut ccdr = rcc
        .use_hse(25.MHz())
        .sys_ck(400.MHz())
        .hclk(200.MHz())
        .pclk1(100.MHz())
        .pclk2(100.MHz())
        .pclk3(100.MHz())
        .pclk4(100.MHz())
        .pll2_r_ck(200.MHz())
        .freeze(pwrcfg, &dp.SYSCFG);
    ccdr
        .peripheral
        .kernel_sdmmc_clk_mux(rcc::rec::SdmmcClkSel::Pll2R);
    let sys_hz = ccdr.clocks.sys_ck().raw();
    let sdmmc1_rec = ccdr.peripheral.SDMMC1;

    defmt::info!("Boot: core enabled");
    defmt::info!("Clocks configured: sysclk={}Hz", sys_hz);

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

    // Green status LED (PE6).
    let mut green_led = gpioe.pe6.into_push_pull_output();
    let _ = green_led.set_low();
    *GREEN_LED.lock() = Some(green_led);

    // Initialize SD card for logging
    defmt::info!("Starting SD logger init");
    let logger = match init_sd_logger(
        dp.SDMMC1,
        sdmmc1_rec,
        &ccdr.clocks,
        (
            gpioc
                .pc12
                .into_alternate::<12>()
                .internal_pull_up(false)
                .speed(Speed::VeryHigh), // CLK - no pull-up
            gpiod
                .pd2
                .into_alternate::<12>()
                .internal_pull_up(false)
                .speed(Speed::VeryHigh), // CMD no PU (matches INAV IOCFG_SDMMC)
            gpioc
                .pc8
                .into_alternate::<12>()
                .internal_pull_up(false)
                .speed(Speed::VeryHigh), // D0
            gpioc
                .pc9
                .into_alternate::<12>()
                .internal_pull_up(false)
                .speed(Speed::VeryHigh), // D1
            gpioc
                .pc10
                .into_alternate::<12>()
                .internal_pull_up(false)
                .speed(Speed::VeryHigh), // D2
            gpioc
                .pc11
                .into_alternate::<12>()
                .internal_pull_up(false)
                .speed(Speed::VeryHigh), // D3
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
    loop { asm::wfi(); }
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
    // Make sure SDMMC1 is clocked and reset cleanly before HAL touches it.
    unsafe {
        let rcc = &*pac::RCC::ptr();
        // Select PLL2R as kernel clock explicitly (00: PLL1Q, 01: PLL2R).
        rcc.d1ccipr.modify(|_, w| w.sdmmcsel().bit(true));
        // Enable clock and pulse reset.
        rcc.ahb3enr.modify(|_, w| w.sdmmc1en().set_bit());
        rcc.ahb3rstr.modify(|_, w| w.sdmmc1rst().set_bit());
        rcc.ahb3rstr.modify(|_, w| w.sdmmc1rst().clear_bit());
    }

    defmt::info!("SDMMC: constructing peripheral (kernel clk=200MHz via PLL2R)");
    // Force 1-bit bus during bring-up to reduce sensitivity to extra lines.
    let mut sdmmc: Sdmmc<_, SdCard> = sdmmc1.sdmmc(pins, sdmmc1_rec, clocks);
    // Match INAV: sample on falling edge.
    sdmmc
        .inner_mut()
        .clkcr
        .modify(|_, w| w.negedge().set_bit());
    // Start at a safe 400 kHz identification clock; can be raised later.
    let bus_frequency = 400.kHz();

    defmt::info!("SDMMC: init (ensure SD card is properly inserted)");
    defmt::info!("SDMMC: checking hardware status...");
    
    // Check if SDMMC is properly enabled before attempting init
    unsafe {
        let rcc = &*pac::RCC::ptr();
        let ahb3enr = rcc.ahb3enr.read();
        let d1ccipr = rcc.d1ccipr.read();
        defmt::info!("SDMMC1 clock enabled: {}", ahb3enr.sdmmc1en().bit());
        defmt::info!("SDMMC1 kernel clock sel: {}", d1ccipr.sdmmcsel().bit());
    }
    
    // Add some delay before init to ensure hardware is stable
    defmt::info!("SDMMC: waiting for hardware stabilization...");
    cortex_m::asm::delay(50_000_000); // ~50ms delay at typical CPU speeds
    
    defmt::info!("SDMMC: attempting init with {}Hz bus freq", bus_frequency.raw());
    
    // Try init with timeout detection by checking if we can at least start
    let init_result = sdmmc.init(bus_frequency);
    
    match init_result {
        Ok(_) => {
            defmt::info!("SDMMC: initialization successful, scanning for Copper partition");
        }
        Err(e) => {
            defmt::error!("SDMMC init failed with error: {:?}", defmt::Debug2Format(&e));
            return Err(CuError::from("SDMMC init failed"));
        }
    }

    if let Err(e) = sdmmc.card() {
        defmt::warn!("No SD card detected: {:?}", e);
        return Err(CuError::from("no sd card"));
    }

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
        let start = HEAP_MEM.as_mut_ptr() as usize;
        let size = core::mem::size_of_val(&HEAP_MEM);
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
    loop {}
}
