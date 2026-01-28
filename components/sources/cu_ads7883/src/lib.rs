#[cfg(mock)]
mod mock {
    use std::io;

    pub struct Spidev;

    pub fn read_adc(_spi: &mut Spidev) -> io::Result<u16> {
        Ok(0)
    }
}

use bincode::{Decode, Encode};
use cu29::prelude::*;
use serde::{Deserialize, Serialize};

#[cfg(hardware)]
use spidev::{SpiModeFlags, Spidev, SpidevOptions, SpidevTransfer};

#[cfg(mock)]
use mock::Spidev;

pub struct ADS7883 {
    spi: Spidev,
    integrated_value: u64,
}

const INTEGRATION_FACTOR: u64 = 8;

/// This opens a SPI device at the given path.
/// The path is usually something like "/dev/spidev0.0" (the default)
/// The max_speed_hz is the maximum speed of the SPI bus in Hz. The ADS7883 can handle up to 48MHz.
/// i.e. 48_000_000 (the default)
#[cfg(hardware)]
fn open_spi(dev_device: Option<&str>, max_speed_hz: Option<u32>) -> std::io::Result<Spidev> {
    let dev_device = dev_device.unwrap_or("/dev/spidev0.0");
    let max_speed_hz = max_speed_hz.unwrap_or(48_000_000);
    let mut spi = Spidev::open(dev_device)?;
    let options = SpidevOptions::new()
        .bits_per_word(8)
        .max_speed_hz(max_speed_hz)
        .mode(SpiModeFlags::SPI_MODE_1)
        .build();
    spi.configure(&options)?;
    Ok(spi)
}

#[derive(Debug, Clone, Copy, Default, Encode, Decode, PartialEq, Serialize, Deserialize)]
pub struct ADCReadingPayload<T>
where
    T: Into<u128> + Copy, // Trick to say all unsigned integers.
{
    pub analog_value: T,
}

/// This is the type of message that the ADS7883 driver will send.
pub type ADSReadingPayload = ADCReadingPayload<u16>;

impl From<&ADCReadingPayload<u16>> for f32 {
    fn from(payload: &ADCReadingPayload<u16>) -> f32 {
        payload.analog_value as f32
    }
}

impl Freezable for ADS7883 {} // This device is stateless.

/// Reads one sample from the ADC.
/// The value is a 12-bit number. i.e. 0-4095
/// 0    -> 0v
/// 4095 -> VDD
#[inline]
#[cfg(hardware)]
fn read_adc(spi: &mut Spidev) -> std::io::Result<u16> {
    let mut rx_buf = [0u8; 2];

    let mut transfer = SpidevTransfer::read(&mut rx_buf);
    spi.transfer(&mut transfer)?;

    // There are 2 bits of padding in the 12-bit ADC value
    // from the datasheet:
    //
    // "The device outputs data while the
    // conversion is in progress. The data word contains two leading zeros, followed by 12-bit data in MSB first format
    // and padded by two lagging zeros."
    //
    let adc_value = (((rx_buf[0] as u16) << 8) | (rx_buf[1] as u16)) >> 2;

    Ok(adc_value)
}

#[cfg(mock)]
use mock::read_adc;

impl CuSrcTask for ADS7883 {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(ADSReadingPayload);

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        #[cfg(hardware)]
        let spi = {
            let (spi_dev, max_speed_hz) = read_spi_config(config)?;
            open_spi(spi_dev.as_deref(), max_speed_hz)
                .map_err(|e| CuError::new_with_cause("Could not open the ADS7883 SPI device", e))?
        };

        #[cfg(mock)]
        let spi = {
            let _ = config;
            Spidev {}
        };

        Ok(ADS7883 {
            spi,
            integrated_value: 0,
        })
    }
    fn start(&mut self, clock: &RobotClock) -> CuResult<()> {
        debug!("ADS7883 started at {}", clock.now());
        // initialize the integrated value.
        self.integrated_value = read_adc_value(&mut self.spi)? as u64;
        self.integrated_value *= INTEGRATION_FACTOR;
        Ok(())
    }
    fn process(&mut self, clock: &RobotClock, new_msg: &mut Self::Output<'_>) -> CuResult<()> {
        let bf = clock.now();
        let analog_value = read_adc_value(&mut self.spi)?;
        // hard to know exactly when the value was read.
        // Should be within a couple of microseconds with the ioctl opverhead.
        let af = clock.now();
        let tov = (af + bf) / 2u64;

        self.integrated_value = ((self.integrated_value + analog_value as u64)
            * INTEGRATION_FACTOR)
            / (INTEGRATION_FACTOR + 1);

        let result = (self.integrated_value / INTEGRATION_FACTOR) as u16;
        let output = ADSReadingPayload {
            analog_value: result,
        };
        new_msg.set_payload(output);
        new_msg.tov = Some(tov).into();
        new_msg.metadata.set_status(result);
        Ok(())
    }
}

#[cfg(test)]
pub mod test_support {
    use super::*;

    pub struct ADS78883TestSink;

    impl Freezable for ADS78883TestSink {}

    impl CuSinkTask for ADS78883TestSink {
        type Resources<'r> = ();
        type Input<'m> = input_msg!(ADSReadingPayload);

        fn new(
            _config: Option<&ComponentConfig>,
            _resources: Self::Resources<'_>,
        ) -> CuResult<Self> {
            Ok(Self {})
        }

        fn process(&mut self, _clock: &RobotClock, new_msg: &Self::Input<'_>) -> CuResult<()> {
            debug!("Received: {}", &new_msg.payload());
            Ok(())
        }
    }
}

#[cfg(hardware)]
fn read_spi_config(config: Option<&ComponentConfig>) -> CuResult<(Option<String>, Option<u32>)> {
    let config = match config {
        Some(config) => config,
        None => return Ok((None, None)),
    };

    let spi_dev = config.get::<String>("spi_dev")?;
    let max_speed_hz = config.get::<u32>("max_speed_hz")?;
    Ok((spi_dev, max_speed_hz))
}

fn read_adc_value(spi: &mut Spidev) -> CuResult<u16> {
    read_adc(spi)
        .map_err(|e| CuError::new_with_cause("Could not read the ADC value from the ADS7883", e))
}
