use cu29::prelude::*;
use embedded_hal::digital::PinState;
use gpio_cdev::{Chip, LineRequestFlags};
use linux_embedded_hal::spidev::SpiModeFlags;
use linux_embedded_hal::spidev::SpidevOptions;
use linux_embedded_hal::{CdevPin, Delay, SpidevBus};

use crate::embedded_hal::{DriverError, EmbeddedHalDriver, EmbeddedHalSettings};
use crate::{map_debug_error, ImuPayload, Mpu9250Device, Mpu9250Factory, Mpu9250Source, WhoAmI};

const DEFAULT_SPI_PATH: &str = "/dev/spidev0.0";
const DEFAULT_SPI_HZ: u32 = 1_000_000;
const DEFAULT_GPIO_CHIP: &str = "/dev/gpiochip0";

type SpiBus = SpidevBus;
type CsPin = CdevPin;
type SpiErr = <SpiBus as embedded_hal::spi::ErrorType>::Error;
type CsErr = <CsPin as embedded_hal::digital::ErrorType>::Error;

/// Linux embedded-hal backend for the MPU9250 using spidev + gpio-cdev CS.
pub struct LinuxMpu9250 {
    driver: EmbeddedHalDriver<SpiBus, CsPin>,
}

impl LinuxMpu9250 {
    fn setup_spi(spi_path: &str, spi_hz: u32) -> CuResult<SpiBus> {
        let mut bus =
            SpidevBus::open(spi_path).map_err(|e| map_debug_error("open spidev bus", e))?;
        let opts = SpidevOptions::new()
            .bits_per_word(8)
            .max_speed_hz(spi_hz)
            .mode(SpiModeFlags::SPI_MODE_0 | SpiModeFlags::SPI_NO_CS)
            .build();
        bus.0
            .configure(&opts)
            .map_err(|e| map_debug_error("configure spidev", e))?;
        Ok(bus)
    }

    fn setup_cs(gpio_chip: &str, cs_line: u32) -> CuResult<CsPin> {
        let mut chip = Chip::new(gpio_chip).map_err(|e| map_debug_error("gpio chip", e))?;
        let handle = chip
            .get_line(cs_line)
            .and_then(|line| line.request(LineRequestFlags::OUTPUT, 1, "cu-mpu9250"))
            .map_err(|e| map_debug_error("gpio line", e))?;
        let pin = CdevPin::new(handle)
            .and_then(|pin| pin.into_output_pin(PinState::High))
            .map_err(|e| map_debug_error("gpio output pin", e))?;
        Ok(pin)
    }
}

impl Mpu9250Device for LinuxMpu9250 {
    type Error = DriverError<SpiErr, CsErr>;

    fn who_am_i(&mut self) -> Result<WhoAmI, Self::Error> {
        self.driver.who_am_i()
    }

    fn read_measure(&mut self) -> Result<ImuPayload, Self::Error> {
        self.driver.read_measure()
    }
}

impl Mpu9250Factory for LinuxMpu9250 {
    fn try_new(config: Option<&ComponentConfig>) -> CuResult<Self> {
        let cfg = config.ok_or("LinuxMpu9250 needs a config (spi_path, gpio_chip, cs_line)")?;

        let spi_path: String = cfg.get("spi_path").unwrap_or(DEFAULT_SPI_PATH.to_string());
        let gpio_chip: String = cfg
            .get("gpio_chip")
            .unwrap_or(DEFAULT_GPIO_CHIP.to_string());
        let spi_hz: u32 = cfg.get("spi_hz").unwrap_or(DEFAULT_SPI_HZ);
        let cs_line: u32 = cfg
            .get("cs_line")
            .ok_or("LinuxMpu9250 requires a gpio cdev cs_line")?;

        let settings = EmbeddedHalSettings::from_config(config);

        let spi = Self::setup_spi(&spi_path, spi_hz)?;
        let cs = Self::setup_cs(&gpio_chip, cs_line)?;
        let delay = Delay;

        let driver = EmbeddedHalDriver::new(spi, cs, delay, settings)?;
        Ok(Self { driver })
    }
}

/// Convenient source alias for Linux spidev + gpio-cdev users.
pub type LinuxMpu9250Source = Mpu9250Source<LinuxMpu9250>;
