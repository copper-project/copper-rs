use cu29::bundle_resources;
use cu29::prelude::*;
use cu29::resource::{ResourceBundle, ResourceManager};
use embedded_hal::digital;
use embedded_hal::spi;
use embedded_io::{ErrorType as EmbeddedErrorType, Read as EmbeddedRead, Write as EmbeddedWrite};
use std::string::String;
use std::sync::Arc;
use std::sync::Mutex;
use thiserror::Error;

use crate::serial::find_serial_device;

pub mod parts;
pub mod serial;

pub const SPIDRIVER_VID: u16 = 0x0403;
pub const SPIDRIVER_PID: u16 = 0x6015;

// FIXME: We should re-use Exclusive from cu_linux_resources!

/// Wrapper for resources that are logically exclusive/owned by a single
/// component but still need to satisfy `Sync` bounds at registration time.
///
/// This keeps synchronization adaptation at the bundle/resource boundary instead
/// of pushing wrappers into every bridge/task that consumes the resource.
pub struct Exclusive<T>(T);

impl<T> Exclusive<T> {
    pub const fn new(inner: T) -> Self {
        Self(inner)
    }

    pub fn into_inner(self) -> T {
        self.0
    }

    pub fn get_mut(&mut self) -> &mut T {
        &mut self.0
    }
}

// SAFETY: `Exclusive<T>` is only handed out by value via `take()` for owned
// resources. The wrapped `T` is not concurrently aliased through this wrapper.
unsafe impl<T: Send> Sync for Exclusive<T> {}

impl<T: std::io::Read> std::io::Read for Exclusive<T> {
    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        self.0.read(buf)
    }
}

impl<T: std::io::Write> std::io::Write for Exclusive<T> {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        self.0.write(buf)
    }

    fn flush(&mut self) -> std::io::Result<()> {
        self.0.flush()
    }
}

impl<T: EmbeddedErrorType> EmbeddedErrorType for Exclusive<T> {
    type Error = T::Error;
}

impl<T: EmbeddedRead> EmbeddedRead for Exclusive<T> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.0.read(buf)
    }
}

impl<T: EmbeddedWrite> EmbeddedWrite for Exclusive<T> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.0.write(buf)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.0.flush()
    }
}

impl<T> embedded_hal::i2c::ErrorType for Exclusive<T>
where
    T: embedded_hal::i2c::ErrorType,
{
    type Error = T::Error;
}

impl<T> embedded_hal::i2c::I2c for Exclusive<T>
where
    T: embedded_hal::i2c::I2c,
{
    fn read(&mut self, address: u8, read: &mut [u8]) -> Result<(), Self::Error> {
        self.0.read(address, read)
    }

    fn write(&mut self, address: u8, write: &[u8]) -> Result<(), Self::Error> {
        self.0.write(address, write)
    }

    fn write_read(
        &mut self,
        address: u8,
        write: &[u8],
        read: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.0.write_read(address, write, read)
    }

    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        self.0.transaction(address, operations)
    }
}

impl<T> embedded_hal::digital::ErrorType for Exclusive<T>
where
    T: embedded_hal::digital::ErrorType,
{
    type Error = T::Error;
}

pub type Spi = parts::SPI<SpiDriverShared<crate::serial::Tx, crate::serial::Rx>>;
pub type PinA = parts::PinA<SpiDriverShared<crate::serial::Tx, crate::serial::Rx>>;
pub type PinB = parts::PinB<SpiDriverShared<crate::serial::Tx, crate::serial::Rx>>;
pub type CS = parts::CS<SpiDriverShared<crate::serial::Tx, crate::serial::Rx>>;

pub struct SpiDriverResources;

bundle_resources!(
    SpiDriverResources:
    Spi,
    ChipSelect,
    PinA,
    PinB,
);

impl ResourceBundle for SpiDriverResources {
    fn build(
        bundle: cu29::resource::BundleContext<Self>,
        config: Option<&ComponentConfig>,
        manager: &mut ResourceManager,
    ) -> CuResult<()> {
        let serial_dev = match get_string(config, "dev")? {
            Some(x) => x,
            // if no device path is specified, scan by Vendor/Product ID
            None => match find_serial_device(SPIDRIVER_VID, SPIDRIVER_PID)? {
                Some(serialport::SerialPortInfo {
                    port_name,
                    port_type: serialport::SerialPortType::UsbPort(port_info),
                }) => {
                    info!(
                        "Using serial device: {} ({:x}:{:x}) (serial: {:?})",
                        &port_name, SPIDRIVER_VID, SPIDRIVER_PID, &port_info.serial_number
                    );
                    port_name
                }
                _ => {
                    return Err(CuError::from(format!(
                        "Could not find USB spidriver device with VID {:x} and PID {:x}",
                        SPIDRIVER_VID, SPIDRIVER_PID
                    )));
                }
            },
        };

        let serial_builder = linux_embedded_hal::serialport::new(serial_dev, 460_800)
            .data_bits(linux_embedded_hal::serialport::DataBits::Eight)
            .flow_control(linux_embedded_hal::serialport::FlowControl::None)
            .parity(linux_embedded_hal::serialport::Parity::None)
            .stop_bits(linux_embedded_hal::serialport::StopBits::One);

        let serial = linux_embedded_hal::Serial::open_from_builder(serial_builder)
            .map_err(|e| CuError::new_with_cause("Failed to open serial port", e))?;
        let serial = crate::serial::Serial::new(serial);
        let (tx, rx) = serial.split();

        let spi_driver = spidriver::SPIDriver::new(tx, rx);
        // owned SpiDriver device
        let cu_driver = SpiDriverShared {
            spi_driver: Arc::new(Mutex::new(spi_driver)),
        };

        let spi = crate::parts::SPI::new(cu_driver.clone());
        let cs = crate::parts::CS::new(cu_driver.clone());
        let pin_a = crate::parts::PinA::new(cu_driver.clone());
        let pin_b = crate::parts::PinB::new(cu_driver);

        manager.add_shared(
            bundle.key(SpiDriverResourcesId::Spi),
            std::sync::Arc::new(spi),
        )?;

        manager.add_shared(
            bundle.key(SpiDriverResourcesId::ChipSelect),
            std::sync::Arc::new(cs),
        )?;

        manager.add_shared(
            bundle.key(SpiDriverResourcesId::PinA),
            std::sync::Arc::new(pin_a),
        )?;

        manager.add_shared(
            bundle.key(SpiDriverResourcesId::PinB),
            std::sync::Arc::new(pin_b),
        )?;

        Ok(())
    }
}

fn get_string(config: Option<&ComponentConfig>, key: &str) -> CuResult<Option<String>> {
    match config {
        Some(cfg) => Ok(cfg.get::<String>(key)?.filter(|value| !value.is_empty())),
        None => Ok(None),
    }
}

#[derive(Clone)]
/// Wrapper around the [`spidriver::SPIDriver`] which allows it to be shared across multiple tasks while still satisfying Sync bounds.
pub struct SpiDriverShared<TX = crate::serial::Tx, RX = crate::serial::Rx>
where
    TX: embedded_io::Write,
    RX: embedded_io::Read,
    RX::Error: core::fmt::Debug,
    TX::Error: core::fmt::Debug,
{
    spi_driver: Arc<Mutex<spidriver::SPIDriver<TX, RX>>>,
}

#[derive(Debug, Error)]
#[error("SPI driver error: {inner:?}")]
pub struct Error<TX, RX> {
    inner: spidriver::Error<TX, RX>,
}

impl<TX, RX> From<spidriver::Error<TX, RX>> for Error<TX, RX> {
    fn from(error: spidriver::Error<TX, RX>) -> Self {
        Self { inner: error }
    }
}

impl<TX, RX> digital::ErrorType for Error<TX, RX>
where
    TX: core::fmt::Debug,
    RX: core::fmt::Debug,
{
    type Error = Error<TX, RX>;
}

impl<TX, RX> digital::Error for Error<TX, RX>
where
    TX: core::fmt::Debug,
    RX: core::fmt::Debug,
{
    fn kind(&self) -> digital::ErrorKind {
        digital::ErrorKind::Other
    }
}

impl<TX, RX> spi::ErrorType for Error<TX, RX>
where
    TX: core::fmt::Debug,
    RX: core::fmt::Debug,
{
    type Error = Error<TX, RX>;
}

impl<TX, RX> spi::Error for Error<TX, RX>
where
    TX: core::fmt::Debug,
    RX: core::fmt::Debug,
{
    fn kind(&self) -> spi::ErrorKind {
        spi::ErrorKind::Other
    }
}

impl<TX, RX> embedded_io::ErrorType for Error<TX, RX>
where
    TX: embedded_io::ErrorType + core::fmt::Debug,
    RX: embedded_io::ErrorType + core::fmt::Debug,
{
    type Error = Error<TX, RX>;
}

impl<TX, RX> embedded_io::Error for Error<TX, RX>
where
    TX: embedded_io::ErrorType + core::fmt::Debug,
    RX: embedded_io::ErrorType + core::fmt::Debug,
{
    fn kind(&self) -> embedded_io::ErrorKind {
        match self.inner {
            spidriver::Error::Protocol => {
                // Map the underlying Pin error to an embedded_io::ErrorKind
                // For simplicity, we treat all Pin errors as Other, but this could be refined
                embedded_io::ErrorKind::Other
            }
            spidriver::Error::Request => embedded_io::ErrorKind::Other,
            spidriver::Error::Write(ref _tx_err) => embedded_io::ErrorKind::Other,
            spidriver::Error::Read(ref _rx_err) => embedded_io::ErrorKind::Other,
        }
    }
}

/// Copied from [`spidriver_hal`]
impl<TX, RX> spidriver_hal::hal::Comms for SpiDriverShared<TX, RX>
where
    TX: embedded_io::Write,
    RX: embedded_io::Read,
{
    type Error = Error<TX::Error, RX::Error>;

    fn set_cs(&self, high: bool) -> Result<(), Self::Error> {
        let mut sd = self.spi_driver.lock().unwrap();
        // let sd = &mut *sd;
        if high {
            Ok(sd.unselect()?) // SPI is active low, so high means unselected
        } else {
            Ok(sd.select()?)
        }
    }

    fn set_a(&self, high: bool) -> Result<(), Self::Error> {
        let mut sd = self.spi_driver.lock().unwrap();
        Ok(sd.set_a(high)?)
    }

    fn set_b(&self, high: bool) -> Result<(), Self::Error> {
        let mut sd = self.spi_driver.lock().unwrap();
        Ok(sd.set_b(high)?)
    }

    fn write(&self, data: &[u8]) -> Result<(), Self::Error> {
        let mut sd = self.spi_driver.lock().unwrap();

        let mut remain = data;
        while !remain.is_empty() {
            let len: usize = if remain.len() > 64 { 64 } else { remain.len() };
            let (this, next) = remain.split_at(len);
            sd.write(this).map_err(|err| Error { inner: err })?;
            remain = next;
        }
        Ok(())
    }

    fn transfer_in_place<'w>(&self, data: &'w mut [u8]) -> Result<&'w [u8], Self::Error> {
        let mut sd = self.spi_driver.lock().unwrap();

        let mut remain = &mut data[..];
        while !remain.is_empty() {
            let len: usize = if remain.len() > 64 { 64 } else { remain.len() };
            let (this, next) = remain.split_at_mut(len);
            sd.transfer_in_place(this)
                .map_err(|err| Error { inner: err })?;
            remain = next;
        }
        Ok(data)
    }
}
