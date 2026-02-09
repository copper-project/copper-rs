#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(feature = "std")]
use cu29::prelude::*;

#[cfg(feature = "std")]
use cu29::resource::{ResourceBundle, ResourceManager};

#[cfg(feature = "std")]
use embedded_io::{Read as EmbeddedRead, Write as EmbeddedWrite};
#[cfg(feature = "embedded-io-07")]
use embedded_io_07 as embedded_io07;

#[cfg(feature = "std")]
use std::string::String;

#[cfg(feature = "std")]
pub const SERIAL_PATH_KEY: &str = "serial_path";
#[cfg(feature = "std")]
pub const SERIAL_DEVICE_KEY: &str = "device";
#[cfg(feature = "std")]
pub const SERIAL_BAUDRATE_KEY: &str = "baudrate";
#[cfg(feature = "std")]
pub const SERIAL_TIMEOUT_MS_KEY: &str = "timeout_ms";

#[cfg(feature = "std")]
pub const I2C_BUS_KEY: &str = "i2c_bus";
#[cfg(feature = "std")]
pub const I2C_DEVICE_KEY: &str = "bus";

#[cfg(feature = "std")]
pub const GPIO_OUT_PIN_KEY: &str = "gpio_out_pin";
#[cfg(feature = "std")]
pub const GPIO_IN_PIN_KEY: &str = "gpio_in_pin";
#[cfg(feature = "std")]
pub const GPIO_PIN_KEY: &str = "pin";

#[cfg(feature = "std")]
pub const DEFAULT_SERIAL_BAUDRATE: u32 = 115_200;
#[cfg(feature = "std")]
pub const DEFAULT_SERIAL_TIMEOUT_MS: u64 = 50;

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

#[cfg(feature = "std")]
impl<T: std::io::Read> std::io::Read for Exclusive<T> {
    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        self.0.read(buf)
    }
}

#[cfg(feature = "std")]
impl<T: std::io::Write> std::io::Write for Exclusive<T> {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        self.0.write(buf)
    }

    fn flush(&mut self) -> std::io::Result<()> {
        self.0.flush()
    }
}

#[cfg(feature = "embedded-io-07")]
impl<T: embedded_io07::ErrorType> embedded_io07::ErrorType for Exclusive<T> {
    type Error = T::Error;
}

#[cfg(feature = "embedded-io-07")]
impl<T: embedded_io07::Read> embedded_io07::Read for Exclusive<T> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.0.read(buf)
    }
}

#[cfg(feature = "embedded-io-07")]
impl<T: embedded_io07::Write> embedded_io07::Write for Exclusive<T> {
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

impl<T> embedded_hal::digital::OutputPin for Exclusive<T>
where
    T: embedded_hal::digital::OutputPin,
{
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.0.set_low()
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.0.set_high()
    }
}

impl<T> embedded_hal::digital::StatefulOutputPin for Exclusive<T>
where
    T: embedded_hal::digital::StatefulOutputPin,
{
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        self.0.is_set_high()
    }

    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        self.0.is_set_low()
    }
}

impl<T> embedded_hal::digital::InputPin for Exclusive<T>
where
    T: embedded_hal::digital::InputPin,
{
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        self.0.is_high()
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        self.0.is_low()
    }
}

#[cfg(feature = "std")]
pub struct LinuxSerialPort {
    inner: Exclusive<Box<dyn serialport::SerialPort>>,
}

#[cfg(feature = "std")]
impl LinuxSerialPort {
    pub fn new(inner: Box<dyn serialport::SerialPort>) -> Self {
        Self {
            inner: Exclusive::new(inner),
        }
    }

    pub fn open(path: &str, baudrate: u32, timeout_ms: u64) -> std::io::Result<Self> {
        let port = serialport::new(path, baudrate)
            .timeout(std::time::Duration::from_millis(timeout_ms))
            .open()?;
        Ok(Self::new(port))
    }
}

#[cfg(feature = "std")]
impl std::io::Read for LinuxSerialPort {
    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        self.inner.read(buf)
    }
}

#[cfg(feature = "std")]
impl std::io::Write for LinuxSerialPort {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        self.inner.write(buf)
    }

    fn flush(&mut self) -> std::io::Result<()> {
        self.inner.flush()
    }
}

#[cfg(feature = "std")]
impl embedded_io::ErrorType for LinuxSerialPort {
    type Error = std::io::Error;
}

#[cfg(feature = "std")]
impl EmbeddedRead for LinuxSerialPort {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        std::io::Read::read(self, buf)
    }
}

#[cfg(feature = "std")]
impl EmbeddedWrite for LinuxSerialPort {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        std::io::Write::write(self, buf)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        std::io::Write::flush(self)
    }
}

#[cfg(all(feature = "std", target_os = "linux"))]
pub type LinuxI2c = Exclusive<linux_embedded_hal::I2cdev>;
#[cfg(all(feature = "std", target_os = "linux"))]
pub type LinuxOutputPin = Exclusive<rppal::gpio::OutputPin>;
#[cfg(all(feature = "std", target_os = "linux"))]
pub type LinuxInputPin = Exclusive<rppal::gpio::InputPin>;

#[cfg(feature = "std")]
pub struct LinuxResources;

#[cfg(feature = "std")]
bundle_resources!(LinuxResources: Serial, I2c, GpioOut, GpioIn);

#[cfg(feature = "std")]
impl ResourceBundle for LinuxResources {
    fn build(
        bundle: cu29::resource::BundleContext<Self>,
        config: Option<&ComponentConfig>,
        manager: &mut ResourceManager,
    ) -> CuResult<()> {
        let cfg = config.ok_or_else(|| {
            CuError::from(format!(
                "Linux resource bundle `{}` requires configuration",
                bundle.bundle_id()
            ))
        })?;

        let serial_path = get_string(cfg, SERIAL_PATH_KEY)?.or(get_string(cfg, SERIAL_DEVICE_KEY)?);
        if let Some(path) = serial_path {
            let baudrate = cfg
                .get::<u32>(SERIAL_BAUDRATE_KEY)?
                .unwrap_or(DEFAULT_SERIAL_BAUDRATE);
            let timeout_ms = cfg
                .get::<u64>(SERIAL_TIMEOUT_MS_KEY)?
                .unwrap_or(DEFAULT_SERIAL_TIMEOUT_MS);
            let serial = LinuxSerialPort::open(&path, baudrate, timeout_ms).map_err(|err| {
                CuError::from(format!(
                    "Failed to open serial `{path}` at {baudrate} baud: {err}"
                ))
            })?;
            let serial_key = bundle.key(LinuxResourcesId::Serial);
            manager.add_owned(serial_key, serial)?;
        }

        let i2c_bus = get_string(cfg, I2C_BUS_KEY)?.or(get_string(cfg, I2C_DEVICE_KEY)?);
        if let Some(bus) = i2c_bus {
            #[cfg(target_os = "linux")]
            {
                let dev = linux_embedded_hal::I2cdev::new(&bus)
                    .map_err(|err| CuError::new_with_cause("Failed to open I2C bus", err))?;
                let i2c_key = bundle.key(LinuxResourcesId::I2c);
                manager.add_owned(i2c_key, Exclusive::new(dev))?;
            }
            #[cfg(not(target_os = "linux"))]
            {
                return Err(CuError::from(format!(
                    "I2C bus `{bus}` requested in `{}` but I2C is only supported on Linux",
                    bundle.bundle_id()
                )));
            }
        }

        let gpio_out_pin = cfg
            .get::<u8>(GPIO_OUT_PIN_KEY)?
            .or(cfg.get::<u8>(GPIO_PIN_KEY)?);
        if let Some(pin) = gpio_out_pin {
            #[cfg(target_os = "linux")]
            {
                let gpio = rppal::gpio::Gpio::new().map_err(|err| {
                    CuError::new_with_cause("Failed to initialize GPIO subsystem", err)
                })?;
                let output = gpio
                    .get(pin)
                    .map_err(|err| CuError::new_with_cause("Failed to get GPIO output pin", err))?
                    .into_output();
                let gpio_out_key = bundle.key(LinuxResourcesId::GpioOut);
                manager.add_owned(gpio_out_key, Exclusive::new(output))?;
            }
            #[cfg(not(target_os = "linux"))]
            {
                return Err(CuError::from(format!(
                    "GPIO output pin `{pin}` requested in `{}` but GPIO is only supported on Linux",
                    bundle.bundle_id()
                )));
            }
        }

        if let Some(pin) = cfg.get::<u8>(GPIO_IN_PIN_KEY)? {
            #[cfg(target_os = "linux")]
            {
                let gpio = rppal::gpio::Gpio::new().map_err(|err| {
                    CuError::new_with_cause("Failed to initialize GPIO subsystem", err)
                })?;
                let input = gpio
                    .get(pin)
                    .map_err(|err| CuError::new_with_cause("Failed to get GPIO input pin", err))?
                    .into_input();
                let gpio_in_key = bundle.key(LinuxResourcesId::GpioIn);
                manager.add_owned(gpio_in_key, Exclusive::new(input))?;
            }
            #[cfg(not(target_os = "linux"))]
            {
                return Err(CuError::from(format!(
                    "GPIO input pin `{pin}` requested in `{}` but GPIO is only supported on Linux",
                    bundle.bundle_id()
                )));
            }
        }

        Ok(())
    }
}

#[cfg(feature = "std")]
fn get_string(cfg: &ComponentConfig, key: &str) -> CuResult<Option<String>> {
    Ok(cfg.get::<String>(key)?.filter(|value| !value.is_empty()))
}

#[cfg(all(test, feature = "embedded-io-07"))]
mod tests {
    use super::Exclusive;

    struct MockIo {
        rx: [u8; 4],
        rx_len: usize,
        tx: [u8; 4],
        tx_len: usize,
    }

    impl MockIo {
        fn new(rx: &[u8]) -> Self {
            let mut buf = [0_u8; 4];
            buf[..rx.len()].copy_from_slice(rx);
            Self {
                rx: buf,
                rx_len: rx.len(),
                tx: [0; 4],
                tx_len: 0,
            }
        }
    }

    impl embedded_io_07::ErrorType for MockIo {
        type Error = core::convert::Infallible;
    }

    impl embedded_io_07::Read for MockIo {
        fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
            let n = core::cmp::min(buf.len(), self.rx_len);
            buf[..n].copy_from_slice(&self.rx[..n]);
            Ok(n)
        }
    }

    impl embedded_io_07::Write for MockIo {
        fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
            let n = core::cmp::min(buf.len(), self.tx.len());
            self.tx[..n].copy_from_slice(&buf[..n]);
            self.tx_len = n;
            Ok(n)
        }

        fn flush(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }
    }

    #[test]
    fn exclusive_forwards_embedded_io_07_traits() {
        let mut wrapped = Exclusive::new(MockIo::new(&[1, 2, 3]));

        let mut rx = [0_u8; 4];
        let read = embedded_io_07::Read::read(&mut wrapped, &mut rx).unwrap();
        assert_eq!(read, 3);
        assert_eq!(&rx[..3], &[1, 2, 3]);

        let written = embedded_io_07::Write::write(&mut wrapped, &[9, 8]).unwrap();
        assert_eq!(written, 2);
        embedded_io_07::Write::flush(&mut wrapped).unwrap();

        let inner = wrapped.into_inner();
        assert_eq!(&inner.tx[..inner.tx_len], &[9, 8]);
    }
}
