use bincode::{Decode, Encode};
use cu29::clock::RobotClock;
use cu29::config::ComponentConfig;
use cu29::cutask::{CuMsg, CuSrcTask, CuTaskLifecycle, Freezable};
use cu29::{output_msg, CuError, CuResult};
use cu29_log_derive::debug;
use serde::{Deserialize, Serialize};
use spidev::{SpiModeFlags, Spidev, SpidevOptions, SpidevTransfer};
use std::io;

pub struct ADS7883 {
    spi: Spidev,
    integrated_value: u64,
}

const INTEGRATION_FACTOR: u64 = 8;

/// This opens a SPI device at the given path.
/// The path is usually something like "/dev/spidev0.0" (the default)
/// The max_speed_hz is the maximum speed of the SPI bus in Hz. The ADS7883 can handle up to 48MHz.
/// i.e. 48_000_000 (the default)
fn open_spi(dev_device: Option<&str>, max_speed_hz: Option<u32>) -> io::Result<Spidev> {
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

// Some convience function.
impl From<ADSReadingPayload> for u16 {
    fn from(msg: ADSReadingPayload) -> Self {
        msg.analog_value
    }
}

impl CuTaskLifecycle for ADS7883 {
    fn new(config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        match config {
            Some(config) => {
                let maybe_string = config.get::<String>("spi_dev");
                let maybe_spidev = maybe_string.as_deref();
                let maybe_max_speed_hz = config.get("max_speed_hz");
                let spi = open_spi(maybe_spidev, maybe_max_speed_hz).map_err(|e| {
                    CuError::new_with_cause("Could not open the ADS7883 SPI device", e)
                })?;
                Ok(ADS7883 {
                    spi,
                    integrated_value: 0,
                })
            }
            None => {
                let spi = open_spi(None, None).map_err(|e| {
                    CuError::new_with_cause("Could not open the ADS7883 SPI device (Note: no config specified for the node so it took the default config)", e)
                })?;
                Ok(ADS7883 {
                    spi,
                    integrated_value: 0,
                })
            }
        }
    }
    fn start(&mut self, clock: &RobotClock) -> CuResult<()> {
        debug!("ADS7883 started at {}", clock.now());
        // initialize the integrated value.
        self.integrated_value = read_adc(&mut self.spi).map_err(|e| {
            CuError::new_with_cause("Could not read the ADC value from the ADS7883", e)
        })? as u64;
        self.integrated_value *= INTEGRATION_FACTOR;
        Ok(())
    }
}

impl Freezable for ADS7883 {} // This device is stateless.

/// Reads one sample from the ADC.
/// The value is a 12-bit number. i.e. 0-4095
/// 0    -> 0v
/// 4095 -> VDD
#[inline]
fn read_adc(spi: &mut Spidev) -> io::Result<u16> {
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
    let adc_value = ((rx_buf[0] as u16) << 8 | (rx_buf[1] as u16)) >> 2;

    Ok(adc_value)
}

impl<'cl> CuSrcTask<'cl> for ADS7883 {
    type Output = output_msg!('cl, ADSReadingPayload);

    fn process(&mut self, clock: &RobotClock, new_msg: Self::Output) -> CuResult<()> {
        let bf = clock.now();
        let analog_value = read_adc(&mut self.spi).map_err(|e| {
            CuError::new_with_cause("Could not read the ADC value from the ADS7883", e)
        })?;
        // hard to know exactly when the value was read.
        // Should be within a couple of microseconds with the ioctl opverhead.
        let output = ADSReadingPayload { analog_value };
        let af = clock.now();
        new_msg.metadata.tov = Some((af + bf) / 2u64).into();

        self.integrated_value = ((self.integrated_value + analog_value as u64)
            * INTEGRATION_FACTOR)
            / (INTEGRATION_FACTOR + 1);

        let output = ADSReadingPayload {
            analog_value: (self.integrated_value / INTEGRATION_FACTOR) as u16,
        };
        new_msg.set_payload(output);
        new_msg.metadata.tov = ((clock.now() + bf) / 2u64).into();
        Ok(())
    }
}

pub mod test_support {
    use super::*;
    use cu29::cutask::CuSinkTask;
    use cu29::input_msg;
    use cu29_log_derive::debug;

    pub struct ADS78883TestSink;

    impl Freezable for ADS78883TestSink {}

    impl CuTaskLifecycle for ADS78883TestSink {
        fn new(_config: Option<&ComponentConfig>) -> CuResult<Self> {
            Ok(Self {})
        }
    }

    impl<'cl> CuSinkTask<'cl> for ADS78883TestSink {
        type Input = input_msg!('cl, ADSReadingPayload);

        fn process(&mut self, _clock: &RobotClock, new_msg: Self::Input) -> CuResult<()> {
            debug!("Received: {}", &new_msg.payload());
            Ok(())
        }
    }
}
