use cu29::prelude::*;
use defmt::info;
use embedded_hal::digital::ErrorType;
use embedded_hal::spi::ErrorType as SpiErrorType;
use embedded_hal::spi::MODE_0;
use hal::fugit::RateExtU32;
use rp235x_hal as hal;

use crate::embedded_hal::{
    DriverError, EmbeddedHalDriver, EmbeddedHalSettings, DEFAULT_GYRO_CAL_MS,
    DEFAULT_GYRO_SAMPLE_DELAY_MS,
};
use crate::{map_debug_error, ImuPayload, Mpu9250Device, Mpu9250Factory, Mpu9250Source, WhoAmI};

const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;
const DEFAULT_SPI_HZ: u32 = 1_000_000;

type Mosi = hal::gpio::Pin<hal::gpio::bank0::Gpio11, hal::gpio::FunctionSpi, hal::gpio::PullDown>;
type Miso = hal::gpio::Pin<hal::gpio::bank0::Gpio12, hal::gpio::FunctionSpi, hal::gpio::PullDown>;
type Sck = hal::gpio::Pin<hal::gpio::bank0::Gpio10, hal::gpio::FunctionSpi, hal::gpio::PullDown>;
type SpiPins = (Mosi, Miso, Sck);
type SpiBus = hal::Spi<hal::spi::Enabled, hal::pac::SPI1, SpiPins>;
type DelayTimer = hal::Timer<hal::timer::CopyableTimer0>;
type CsPin = hal::gpio::Pin<
    hal::gpio::bank0::Gpio13,
    hal::gpio::FunctionSio<hal::gpio::SioOutput>,
    hal::gpio::PullDown,
>;
type CsError = <CsPin as ErrorType>::Error;
type SpiErr = <SpiBus as SpiErrorType>::Error;
type MpuError = DriverError<SpiErr, CsError>;

/// RP235x HAL-backed MPU9250 driver wired to SPI1 (GPIO10-13).
pub struct RpMpu9250 {
    driver: EmbeddedHalDriver<SpiBus, CsPin>,
}

impl RpMpu9250 {
    fn setup_peripherals(xosc_hz: u32, spi_hz: u32) -> CuResult<(SpiBus, CsPin, DelayTimer)> {
        let mut p = hal::pac::Peripherals::take().ok_or("RP235x peripherals unavailable")?;
        let mut watchdog = hal::watchdog::Watchdog::new(p.WATCHDOG);
        let clocks = hal::clocks::init_clocks_and_plls(
            xosc_hz,
            p.XOSC,
            p.CLOCKS,
            p.PLL_SYS,
            p.PLL_USB,
            &mut p.RESETS,
            &mut watchdog,
        )
        .map_err(|err| map_debug_error("init_clocks_and_plls", err))?;

        let mut timer: DelayTimer = hal::Timer::new_timer0(p.TIMER0, &mut p.RESETS, &clocks);

        let sio = hal::sio::Sio::new(p.SIO);
        let pins = hal::gpio::Pins::new(p.IO_BANK0, p.PADS_BANK0, sio.gpio_bank0, &mut p.RESETS);

        let sck: Sck = pins.gpio10.into_function();
        let mosi: Mosi = pins.gpio11.into_function();
        let miso: Miso = pins.gpio12.into_function();
        let mut cs: CsPin = pins.gpio13.into_push_pull_output();
        let _ = cs.set_high();

        let spi: SpiBus = hal::Spi::<_, _, _, 8>::new(p.SPI1, (mosi, miso, sck)).init(
            &mut p.RESETS,
            clocks.peripheral_clock.freq(),
            spi_hz.Hz(),
            hal::spi::FrameFormat::MotorolaSpi(MODE_0),
        );

        Ok((spi, cs, timer))
    }

    fn init_mpu(
        spi: SpiBus,
        cs: CsPin,
        timer: DelayTimer,
        settings: EmbeddedHalSettings,
    ) -> CuResult<EmbeddedHalDriver<SpiBus, CsPin>> {
        EmbeddedHalDriver::new(spi, cs, timer, settings)
    }
}

impl Mpu9250Device for RpMpu9250 {
    type Error = MpuError;

    fn who_am_i(&mut self) -> Result<WhoAmI, Self::Error> {
        self.driver.who_am_i()
    }

    fn read_measure(&mut self) -> Result<ImuPayload, Self::Error> {
        self.driver.read_measure()
    }
}

impl Mpu9250Factory for RpMpu9250 {
    fn try_new(config: Option<&ComponentConfig>) -> CuResult<Self> {
        let xosc_hz: u32 = config
            .and_then(|cfg| cfg.get("xosc_hz"))
            .unwrap_or(XOSC_CRYSTAL_FREQ);
        let spi_hz: u32 = config
            .and_then(|cfg| cfg.get("spi_hz"))
            .unwrap_or(DEFAULT_SPI_HZ);
        let gyro_cal_ms: u32 = config
            .and_then(|cfg| cfg.get("gyro_cal_ms"))
            .unwrap_or(DEFAULT_GYRO_CAL_MS);
        let gyro_sample_delay_ms: u32 = config
            .and_then(|cfg| cfg.get("gyro_sample_delay_ms"))
            .unwrap_or(DEFAULT_GYRO_SAMPLE_DELAY_MS);

        let (spi, cs, timer) = Self::setup_peripherals(xosc_hz, spi_hz)?;
        let settings = EmbeddedHalSettings {
            gyro_cal_ms,
            gyro_sample_delay_ms,
        };
        let driver = Self::init_mpu(spi, cs, timer, settings)?;

        if gyro_cal_ms > 0 {
            let bias = crate::embedded_hal::gyro_bias();
            info!(
                "Gyro bias calibrated: [{:.4}, {:.4}, {:.4}] rad/s",
                bias[0], bias[1], bias[2]
            );
        }

        Ok(Self { driver })
    }
}

/// Convenient source alias using the RP235x HAL SPI1 backend.
pub type RpMpu9250Source = Mpu9250Source<RpMpu9250>;
