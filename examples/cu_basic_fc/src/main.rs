#![no_std]
#![no_main]

extern crate alloc;

use alloc::sync::Arc;
use cortex_m::{asm, peripheral::DWT};
use cortex_m_rt::{entry, exception, ExceptionFrame};
use cu29::prelude::*;
use defmt::info;
use defmt_rtt as _;
use embedded_alloc::Heap as StdHeap;
use panic_probe as _;
use spin::Mutex;
use stm32h7xx_hal::{
    gpio::{
        gpioc::{PC6, PC7},
        Alternate,
    },
    pac,
    prelude::*,
    serial::Error as UartError,
    nb,
    serial::{config::Config, Serial},
};
use bincode::{error::EncodeError, Encode};
use cu29_unifiedlog::{SectionHandle, UnifiedLogStatus};
use cu29::prelude::UnifiedLogType;
use cu_sensor_payloads::ImuPayload;
use alloc::vec;

mod tasks;

#[global_allocator]
static ALLOC: StdHeap = StdHeap::empty();

#[copper_runtime(config = "copperconfig.ron")]
struct FlightControllerApp {}

// UART6 pins: PC6 (TX) PC7 (RX), AF7.
type SerialPort = SerialWrapper;
type SerialPortError = embedded_io::ErrorKind;

// Dummy IMU source placeholder.
pub struct DummyImu;
pub type RpMpu9250Source = DummyImu;
impl Freezable for DummyImu {}

// Minimal no-op storage/logger to satisfy Copper generics.
struct NoopStorage;
#[derive(Clone, Copy)]
struct NoopLogger;

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

impl SectionStorage for NoopStorage {
    fn initialize<E: Encode>(&mut self, _hdr: &E) -> Result<usize, EncodeError> {
        Ok(0)
    }
    fn post_update_header<E: Encode>(&mut self, _hdr: &E) -> Result<usize, EncodeError> {
        Ok(0)
    }
    fn append<E: Encode>(&mut self, _item: &E) -> Result<usize, EncodeError> {
        Ok(0)
    }
    fn flush(&mut self) -> Result<usize, CuError> {
        Ok(0)
    }
}

impl UnifiedLogWrite<NoopStorage> for NoopLogger {
    fn add_section(
        &mut self,
        _ty: UnifiedLogType,
        _start: usize,
    ) -> Result<SectionHandle<NoopStorage>, CuError> {
        let header = cu29_unifiedlog::SectionHeader {
            entry_type: UnifiedLogType::Empty,
            ..Default::default()
        };
        SectionHandle::create(header, NoopStorage)
            .map_err(|e| CuError::from(format!("noop section create failed: {:?}", e)))
    }
    fn flush_section(&mut self, _handle: &mut SectionHandle<NoopStorage>) {}
    fn status(&self) -> UnifiedLogStatus {
        UnifiedLogStatus {
            total_used_space: 0,
            total_allocated_space: 0,
        }
    }
}

// embedded-io traits for registry use.
impl embedded_io::ErrorType for NoopLogger {
    type Error = core::convert::Infallible;
}
impl embedded_io::Write for NoopLogger {
    fn write(&mut self, _buf: &[u8]) -> Result<usize, Self::Error> {
        Ok(0)
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}
impl embedded_io::Read for NoopLogger {
    fn read(&mut self, _buf: &mut [u8]) -> Result<usize, Self::Error> {
        Ok(0)
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

    // Clocks: default configuration (~400MHz sysclk).
    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.freeze();
    let ccdr = dp.RCC.constrain().freeze(pwrcfg, &dp.SYSCFG);
    let sys_hz = ccdr.clocks.sys_ck().raw();

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

    // 64-bit division sanity check to ensure runtime helpers/FPU setup are OK.
    let div_test = 1_000_000_001u64 / 3u64;
    defmt::info!("64-bit div sanity: {}", div_test);

    // UART6 for CRSF.
    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);
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

    // Spin up Copper runtime with CRSF -> mapper -> sink graph.
    info!("Building Copper runtime clock");
    let clock = build_clock();
    info!("Clock ready, constructing app");
    let writer = Arc::new(Mutex::new(NoopLogger));

    match <FlightControllerApp as CuApplication<NoopStorage, NoopLogger>>::new(clock, writer) {
        Ok(mut app) => {
            info!("App constructed, starting Copper FC (H743, CRSF on UART6)...");
            let _ = <FlightControllerApp as CuApplication<NoopStorage, NoopLogger>>::run(&mut app);
            info!("Copper runtime returned unexpectedly");
        }
        Err(e) => {
            defmt::error!("App init failed: {:?}", e);
        }
    }
    loop { asm::wfi(); }
}

fn build_clock() -> RobotClock {
    // Use a mock clock to avoid calibration/division at startup.
    let (clock, _mock) = RobotClock::mock();
    clock
}

fn init_heap() {
    extern "C" {
        static mut __ebss: u8;
        static mut _stack_start: u8;
    }

    let start = (&raw mut __ebss) as *mut u8 as usize;
    let stack_top = (&raw mut _stack_start) as *mut u8 as usize;
    // Leave space for stacks/ISRs to grow downward.
    let reserve = 64 * 1024; // 64 KB stack guard
    let end = stack_top.saturating_sub(reserve);
    let size = end.saturating_sub(start);

    unsafe { ALLOC.init(start, size) }
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
