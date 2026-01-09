#![no_std]
#![doc = include_str!("../README.md")]

use cortex_m::peripheral::DWT;
use cu_bdshot::{Stm32H7Board, Stm32H7BoardResources, register_stm32h7_board};
use cu_sdlogger::{EMMCLogger, EMMCSectionStorage, ForceSyncSend, find_copper_partition};
use cu29::resource::{ResourceBundle, ResourceManager};
use cu29::{CuError, CuResult, bundle_resources};
use embedded_hal::adc::OneShot;
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::blocking::spi::Transfer;
use embedded_hal::digital::v2::OutputPin;
use spin::Mutex;
use stm32h7xx_hal::{
    adc,
    delay::Delay,
    gpio::{Analog, Output, PushPull, Speed},
    nb, pac,
    prelude::*,
    rcc::{self, rec::AdcClkSel},
    sdmmc::{self, SdCard, Sdmmc, SdmmcBlockDevice},
    serial::{Error as UartError, Serial, config::Config},
    spi::{Config as SpiConfig, Mode, Phase, Polarity, SpiExt},
};

// Throttle UART overrun warnings to once per second.
const UART_OVERRUN_LOG_PERIOD_SECS: u32 = 1;
const UART6_BAUD: u32 = 420_000;
const UART2_BAUD: u32 = 115_200;

pub type SerialPortError = embedded_io::ErrorKind;

/// UART wrapper implementing embedded-io traits with overrun throttling.
pub struct SerialWrapper<U> {
    inner: Serial<U>,
    label: &'static str,
    last_overrun_cycle: Option<u32>,
    overrun_period_cycles: u32,
}

// Safety: the firmware uses a single-threaded runtime; no concurrent access.
unsafe impl<U> Send for SerialWrapper<U> {}
unsafe impl<U> Sync for SerialWrapper<U> {}

impl<U> SerialWrapper<U> {
    fn new(inner: Serial<U>, label: &'static str, overrun_period_cycles: u32) -> Self {
        Self {
            inner,
            label,
            last_overrun_cycle: None,
            overrun_period_cycles,
        }
    }

    fn should_warn_overrun(&mut self) -> bool {
        if self.overrun_period_cycles == 0 {
            return true;
        }
        let now = DWT::cycle_count();
        match self.last_overrun_cycle {
            Some(prev) if now.wrapping_sub(prev) < self.overrun_period_cycles => false,
            _ => {
                self.last_overrun_cycle = Some(now);
                true
            }
        }
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
                                if self.should_warn_overrun() {
                                    defmt::warn!("{} overrun, data lost", self.label);
                                }
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
                    match nb::block!(self.inner.write(b)) {
                        Ok(()) => written += 1,
                        Err(e) => {
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

// Resource type aliases exposed by the bundle.
pub type Uart6Port = SerialWrapper<pac::USART6>;
pub type Uart2Port = SerialWrapper<pac::USART2>;

/// Wraps HAL types that are only used from the single-threaded runtime.
pub struct SingleThreaded<T>(T);

impl<T> SingleThreaded<T> {
    pub fn new(inner: T) -> Self {
        Self(inner)
    }
}

// Safety: the firmware uses a single-threaded runtime; no concurrent access.
unsafe impl<T> Send for SingleThreaded<T> {}
unsafe impl<T> Sync for SingleThreaded<T> {}

impl<T> Transfer<u8> for SingleThreaded<T>
where
    T: Transfer<u8>,
{
    type Error = T::Error;

    fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<&'w [u8], Self::Error> {
        self.0.transfer(words)
    }
}

impl<T> OutputPin for SingleThreaded<T>
where
    T: OutputPin,
{
    type Error = T::Error;

    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.0.set_low()
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.0.set_high()
    }
}

impl<T> DelayMs<u32> for SingleThreaded<T>
where
    T: DelayMs<u32>,
{
    fn delay_ms(&mut self, ms: u32) {
        self.0.delay_ms(ms);
    }
}

pub type GreenLed =
    stm32h7xx_hal::gpio::gpioe::PE6<stm32h7xx_hal::gpio::Output<stm32h7xx_hal::gpio::PushPull>>;

pub type Bmi088Spi =
    SingleThreaded<stm32h7xx_hal::spi::Spi<pac::SPI2, stm32h7xx_hal::spi::Enabled, u8>>;
pub type Bmi088AccCs = SingleThreaded<stm32h7xx_hal::gpio::gpiod::PD4<Output<PushPull>>>;
pub type Bmi088GyrCs = SingleThreaded<stm32h7xx_hal::gpio::gpiod::PD5<Output<PushPull>>>;
pub type Bmi088Delay = SingleThreaded<Delay>;
pub type BatterySensePin = stm32h7xx_hal::gpio::gpioc::PC0<Analog>;

pub struct BatteryAdc {
    adc: adc::Adc<pac::ADC1, adc::Enabled>,
    pin: BatterySensePin,
}

// Safety: the firmware uses a single-threaded runtime; no concurrent access.
unsafe impl Send for BatteryAdc {}
unsafe impl Sync for BatteryAdc {}

impl BatteryAdc {
    fn new(adc: adc::Adc<pac::ADC1, adc::Enabled>, pin: BatterySensePin) -> Self {
        Self { adc, pin }
    }

    pub fn read_raw_blocking(&mut self) -> u32 {
        nb::block!(self.adc.read(&mut self.pin)).unwrap_or(0)
    }

    pub fn slope(&self) -> u32 {
        self.adc.slope()
    }
}

type SdBlockDev = ForceSyncSend<SdmmcBlockDevice<Sdmmc<pac::SDMMC1, SdCard>>>;
pub type LogStorage = EMMCSectionStorage<SdBlockDev>;
pub type Logger = EMMCLogger<SdBlockDev>;

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

/// Resource bundle for the MicoAir H743 board.
pub struct MicoAirH743;

bundle_resources!(
    MicoAirH743: Uart6, Uart2, GreenLed, Logger, Bmi088Spi, Bmi088AccCs, Bmi088GyrCs, Bmi088Delay, BatteryAdc
);

impl ResourceBundle for MicoAirH743 {
    fn build(
        bundle: cu29::resource::BundleContext<Self>,
        _config: Option<&cu29::config::ComponentConfig>,
        manager: &mut ResourceManager,
    ) -> CuResult<()> {
        // Here we bring up the entire board and register the resources.

        let mut cp = cortex_m::Peripherals::take()
            .ok_or_else(|| CuError::from("cortex-m peripherals already taken"))?;
        cp.SCB.enable_fpu();
        cp.DCB.enable_trace();
        DWT::unlock();
        let mut dwt = cp.DWT;
        dwt.enable_cycle_counter();
        let syst = cp.SYST;

        let dp = pac::Peripherals::take()
            .ok_or_else(|| CuError::from("stm32 peripherals already taken"))?;

        // Clocks: mirror the known-good sdcard-test setup.
        let pwr = dp.PWR.constrain();
        let vos = pwr.freeze();
        let rcc = dp.RCC.constrain();
        let mut ccdr = rcc
            .sys_ck(400.MHz())
            .pll1_q_ck(100.MHz())
            .freeze(vos, &dp.SYSCFG);
        ccdr.peripheral.kernel_adc_clk_mux(AdcClkSel::Per);
        let sysclk_hz = ccdr.clocks.sys_ck().raw();
        let sdmmc1_rec = ccdr.peripheral.SDMMC1;
        let mut delay = Delay::new(syst, ccdr.clocks);

        let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
        let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);
        let gpiod = dp.GPIOD.split(ccdr.peripheral.GPIOD);
        let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);
        let battery_pin = gpioc.pc0.into_analog();

        let green_led = gpioe.pe6.into_push_pull_output();
        let mut adc1 = adc::Adc::adc1(
            dp.ADC1,
            4.MHz(),
            &mut delay,
            ccdr.peripheral.ADC12,
            &ccdr.clocks,
        )
        .enable();
        adc1.set_resolution(adc::Resolution::TwelveBit);
        adc1.set_sample_time(adc::AdcSampleTime::T_64);
        let battery_adc = BatteryAdc::new(adc1, battery_pin);

        let overrun_period_cycles = sysclk_hz.saturating_mul(UART_OVERRUN_LOG_PERIOD_SECS);
        let uart6_config = Config::default().baudrate(UART6_BAUD.bps());
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
            overrun_period_cycles,
        );

        let uart2_baud = _config
            .and_then(|cfg| cfg.get::<u32>("uart2_baud"))
            .unwrap_or(UART2_BAUD);
        let uart2_config = Config::default().baudrate(uart2_baud.bps());
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
            overrun_period_cycles,
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

        let sck = gpiod.pd3.into_alternate::<5>().speed(Speed::VeryHigh);
        let miso = gpioc.pc2.into_alternate::<5>().speed(Speed::VeryHigh);
        let mosi = gpioc.pc3.into_alternate::<5>().speed(Speed::VeryHigh);
        let mut bmi088_acc_cs = gpiod.pd4.into_push_pull_output();
        let mut bmi088_gyr_cs = gpiod.pd5.into_push_pull_output();
        bmi088_acc_cs.set_high();
        bmi088_gyr_cs.set_high();

        let spi_cfg = SpiConfig::new(Mode {
            polarity: Polarity::IdleHigh,
            phase: Phase::CaptureOnSecondTransition,
        });
        let bmi088_spi: stm32h7xx_hal::spi::Spi<pac::SPI2, stm32h7xx_hal::spi::Enabled, u8> =
            dp.SPI2.spi(
                (sck, miso, mosi),
                spi_cfg,
                10.MHz(),
                ccdr.peripheral.SPI2,
                &ccdr.clocks,
            );

        let bmi088_spi = SingleThreaded::new(bmi088_spi);
        let bmi088_acc_cs = SingleThreaded::new(bmi088_acc_cs);
        let bmi088_gyr_cs = SingleThreaded::new(bmi088_gyr_cs);
        let bmi088_delay = SingleThreaded::new(delay);

        let bdshot_resources = Stm32H7BoardResources {
            m1: gpioe.pe14,
            m2: gpioe.pe13,
            m3: gpioe.pe11,
            m4: gpioe.pe9,
            dwt,
            sysclk_hz,
        };
        let bdshot_board = Stm32H7Board::new(bdshot_resources)?;
        register_stm32h7_board(bdshot_board)?;

        let uart6_key = bundle.key(MicoAirH743Id::Uart6);
        let uart2_key = bundle.key(MicoAirH743Id::Uart2);
        let led_key = bundle.key(MicoAirH743Id::GreenLed);
        let logger_key = bundle.key(MicoAirH743Id::Logger);
        let bmi088_spi_key = bundle.key(MicoAirH743Id::Bmi088Spi);
        let bmi088_acc_cs_key = bundle.key(MicoAirH743Id::Bmi088AccCs);
        let bmi088_gyr_cs_key = bundle.key(MicoAirH743Id::Bmi088GyrCs);
        let bmi088_delay_key = bundle.key(MicoAirH743Id::Bmi088Delay);
        let battery_adc_key = bundle.key(MicoAirH743Id::BatteryAdc);

        manager.add_owned(uart6_key, uart6)?;
        manager.add_owned(uart2_key, uart2)?;
        manager.add_owned(led_key, Mutex::new(green_led))?;
        manager.add_owned(logger_key, logger)?;
        manager.add_owned(bmi088_spi_key, bmi088_spi)?;
        manager.add_owned(bmi088_acc_cs_key, bmi088_acc_cs)?;
        manager.add_owned(bmi088_gyr_cs_key, bmi088_gyr_cs)?;
        manager.add_owned(bmi088_delay_key, bmi088_delay)?;
        manager.add_owned(battery_adc_key, battery_adc)?;
        Ok(())
    }
}
