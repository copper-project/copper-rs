use cu29::prelude::*;
use cu_embedded_registry as reg;
use defmt::info;
use embedded_hal::digital::ErrorType;
use embedded_hal::spi::ErrorType as SpiErrorType;
use rp235x_hal as hal;

use crate::embedded_hal::{
    DriverError, EmbeddedHalDriver, EmbeddedHalSettings, DEFAULT_GYRO_CAL_MS,
    DEFAULT_GYRO_SAMPLE_DELAY_MS,
};
use crate::{ImuPayload, Mpu9250Device, Mpu9250Factory, Mpu9250Source, WhoAmI};

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
        let spi_slot: usize = config.and_then(|cfg| cfg.get("spi_slot")).unwrap_or(0);
        let cs_slot: usize = config.and_then(|cfg| cfg.get("cs_slot")).unwrap_or(0);
        let delay_slot: usize = config.and_then(|cfg| cfg.get("delay_slot")).unwrap_or(0);

        let settings = EmbeddedHalSettings::from_config(config);

        let spi: SpiBus = reg::take_spi(spi_slot)
            .ok_or_else(|| CuError::from("cu_embedded_registry: SPI slot not registered"))?;
        let cs: CsPin = reg::take_cs(cs_slot)
            .ok_or_else(|| CuError::from("cu_embedded_registry: CS slot not registered"))?;
        let timer: DelayTimer = reg::take_delay(delay_slot)
            .ok_or_else(|| CuError::from("cu_embedded_registry: delay slot not registered"))?;

        let driver = Self::init_mpu(spi, cs, timer, settings)?;

        if settings.gyro_cal_ms > 0 {
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
