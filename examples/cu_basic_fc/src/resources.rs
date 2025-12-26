use alloc::string::String;
use cortex_m::peripheral::DWT;
use cu_bdshot::{Stm32H7Board, Stm32H7BoardResources, register_stm32h7_board};
use cu_sdlogger::{EMMCLogger, EMMCSectionStorage, ForceSyncSend, find_copper_partition};
use cu29::resource::{ResourceBundle, ResourceDecl, ResourceKey, ResourceManager};
use cu29::{CuError, CuResult};
use spin::Mutex;
use stm32h7xx_hal::{
    gpio::Speed,
    nb, pac,
    prelude::*,
    rcc,
    sdmmc::{self, SdCard, Sdmmc, SdmmcBlockDevice},
    serial::{Error as UartError, Serial, config::Config},
};

pub type SerialPortError = embedded_io::ErrorKind;

pub struct SerialWrapper<U> {
    inner: Serial<U>,
    label: &'static str,
}

// Safety: the firmware uses a single-threaded runtime; no concurrent access.
unsafe impl<U> Send for SerialWrapper<U> {}
unsafe impl<U> Sync for SerialWrapper<U> {}

impl<U> SerialWrapper<U> {
    fn new(inner: Serial<U>, label: &'static str) -> Self {
        Self { inner, label }
    }
}

macro_rules! impl_serial_wrapper_io {
    ($usart:ty) => {
        impl embedded_io::ErrorType for SerialWrapper<$usart> {
            type Error = SerialPortError;
        }

        impl embedded_io::Read for SerialWrapper<$usart> {
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
                                defmt::warn!("{} overrun, data lost", self.label);
                                // Clear and keep going; treat as no more data.
                                break;
                            }
                            _ => {
                                defmt::error!("{} read err: {:?}", self.label, e);
                                return Err(SerialPortError::Other);
                            }
                        },
                    }
                }
                Ok(read)
            }
        }

        impl embedded_io::Write for SerialWrapper<$usart> {
            fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
                let mut written = 0;
                for &b in buf {
                    match self.inner.write(b) {
                        Ok(()) => written += 1,
                        Err(nb::Error::WouldBlock) => break,
                        Err(nb::Error::Other(e)) => {
                            defmt::error!("{} write err: {:?}", self.label, e);
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
    };
}

impl_serial_wrapper_io!(pac::USART6);
impl_serial_wrapper_io!(pac::USART2);

pub type Uart6Port = SerialWrapper<pac::USART6>;
pub type Uart2Port = SerialWrapper<pac::USART2>;
pub type SerialPort = Uart6Port;

pub type GreenLed =
    stm32h7xx_hal::gpio::gpioe::PE6<stm32h7xx_hal::gpio::Output<stm32h7xx_hal::gpio::PushPull>>;

type SdBlockDev = ForceSyncSend<SdmmcBlockDevice<Sdmmc<pac::SDMMC1, SdCard>>>;
pub type LogStorage = EMMCSectionStorage<SdBlockDev>;
pub type Logger = EMMCLogger<SdBlockDev>;

fn lookup(resources: &[ResourceDecl], bundle: &str, name: &str) -> CuResult<ResourceKey> {
    let mut path = String::new();
    path.push_str(bundle);
    path.push('.');
    path.push_str(name);
    resources
        .iter()
        .find(|decl| decl.path == path)
        .map(|decl| decl.key)
        .ok_or_else(|| CuError::from("Resource not declared"))
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
) -> CuResult<Logger> {
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

pub struct MicoAirH743;

impl ResourceBundle for MicoAirH743 {
    fn build(
        bundle_id: &str,
        _config: Option<&cu29::config::ComponentConfig>,
        resources: &[ResourceDecl],
        manager: &mut ResourceManager,
    ) -> CuResult<()> {
        let mut cp = cortex_m::Peripherals::take()
            .ok_or_else(|| CuError::from("cortex-m peripherals already taken"))?;
        cp.SCB.enable_fpu();
        cp.DCB.enable_trace();
        DWT::unlock();
        cp.DWT.enable_cycle_counter();

        let dp = pac::Peripherals::take()
            .ok_or_else(|| CuError::from("stm32 peripherals already taken"))?;

        // Clocks: mirror the known-good sdcard-test setup.
        let pwr = dp.PWR.constrain();
        let vos = pwr.freeze();
        let rcc = dp.RCC.constrain();
        let ccdr = rcc
            .sys_ck(400.MHz())
            .pll1_q_ck(100.MHz())
            .freeze(vos, &dp.SYSCFG);
        let sdmmc1_rec = ccdr.peripheral.SDMMC1;

        let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
        let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);
        let gpiod = dp.GPIOD.split(ccdr.peripheral.GPIOD);
        let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);

        let green_led = gpioe.pe6.into_push_pull_output();

        let uart6_config = Config::default().baudrate(420_000.bps());
        let uart6 = SerialWrapper::new(
            dp.USART6
                .serial(
                    (gpioc.pc6.into_alternate(), gpioc.pc7.into_alternate()),
                    uart6_config,
                    ccdr.peripheral.USART6,
                    &ccdr.clocks,
                )
                .map_err(|_| CuError::from("uart6 init failed"))?,
            "UART6",
        );

        let uart2_config = Config::default().baudrate(115_200.bps());
        let uart2 = SerialWrapper::new(
            dp.USART2
                .serial(
                    (gpioa.pa2.into_alternate(), gpioa.pa3.into_alternate()),
                    uart2_config,
                    ccdr.peripheral.USART2,
                    &ccdr.clocks,
                )
                .map_err(|_| CuError::from("uart2 init failed"))?,
            "UART2",
        );

        let logger = init_sd_logger(
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
        )?;

        let bdshot_resources = Stm32H7BoardResources {
            m1: gpioe.pe14,
            m2: gpioe.pe13,
            m3: gpioe.pe11,
            m4: gpioe.pe9,
            dwt: cp.DWT,
            sysclk_hz: ccdr.clocks.sys_ck().raw(),
        };
        let bdshot_board = Stm32H7Board::new(bdshot_resources)?;
        register_stm32h7_board(bdshot_board)?;

        let uart6_key = lookup(resources, bundle_id, "uart6")?.typed();
        let uart2_key = lookup(resources, bundle_id, "uart2")?.typed();
        let led_key = lookup(resources, bundle_id, "green_led")?.typed();
        let logger_key = lookup(resources, bundle_id, "logger")?.typed();

        manager.add_owned(uart6_key, uart6)?;
        manager.add_owned(uart2_key, uart2)?;
        manager.add_owned(led_key, Mutex::new(green_led))?;
        manager.add_owned(logger_key, logger)?;
        Ok(())
    }
}
