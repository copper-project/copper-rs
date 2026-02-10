#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(feature = "std")]
use cu29::prelude::*;

#[cfg(feature = "std")]
use cu29::bundle_resources;

#[cfg(feature = "std")]
use cu29::resource::{ResourceBundle, ResourceManager};

#[cfg(feature = "std")]
use embedded_io::{Read as EmbeddedRead, Write as EmbeddedWrite};
#[cfg(feature = "embedded-io-07")]
use embedded_io_07 as embedded_io07;

#[cfg(feature = "std")]
use std::string::String;

#[cfg(feature = "std")]
pub const SERIAL_ACM0_PATH_KEY: &str = "tty_acm0_path";
#[cfg(feature = "std")]
pub const SERIAL_ACM1_PATH_KEY: &str = "tty_acm1_path";
#[cfg(feature = "std")]
pub const SERIAL_ACM2_PATH_KEY: &str = "tty_acm2_path";
#[cfg(feature = "std")]
pub const SERIAL_USB0_PATH_KEY: &str = "tty_usb0_path";
#[cfg(feature = "std")]
pub const SERIAL_USB1_PATH_KEY: &str = "tty_usb1_path";
#[cfg(feature = "std")]
pub const SERIAL_USB2_PATH_KEY: &str = "tty_usb2_path";
#[cfg(feature = "std")]
pub const SERIAL_BAUDRATE_KEY: &str = "serial_baudrate";
#[cfg(feature = "std")]
pub const SERIAL_TIMEOUT_MS_KEY: &str = "serial_timeout_ms";

#[cfg(feature = "std")]
pub const I2C0_PATH_KEY: &str = "i2c0_path";
#[cfg(feature = "std")]
pub const I2C1_PATH_KEY: &str = "i2c1_path";
#[cfg(feature = "std")]
pub const I2C2_PATH_KEY: &str = "i2c2_path";

#[cfg(feature = "std")]
pub const GPIO_OUT0_PIN_KEY: &str = "gpio_out0_pin";
#[cfg(feature = "std")]
pub const GPIO_OUT1_PIN_KEY: &str = "gpio_out1_pin";
#[cfg(feature = "std")]
pub const GPIO_OUT2_PIN_KEY: &str = "gpio_out2_pin";
#[cfg(feature = "std")]
pub const GPIO_IN0_PIN_KEY: &str = "gpio_in0_pin";
#[cfg(feature = "std")]
pub const GPIO_IN1_PIN_KEY: &str = "gpio_in1_pin";
#[cfg(feature = "std")]
pub const GPIO_IN2_PIN_KEY: &str = "gpio_in2_pin";

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
bundle_resources!(
    LinuxResources:
        SerialAcm0,
        SerialAcm1,
        SerialAcm2,
        SerialUsb0,
        SerialUsb1,
        SerialUsb2,
        I2c0,
        I2c1,
        I2c2,
        GpioOut0,
        GpioOut1,
        GpioOut2,
        GpioIn0,
        GpioIn1,
        GpioIn2
);

#[cfg(feature = "std")]
pub const LINUX_RESOURCE_SLOT_NAMES: &[&str] = &[
    "serial_acm0",
    "serial_acm1",
    "serial_acm2",
    "serial_usb0",
    "serial_usb1",
    "serial_usb2",
    "i2c0",
    "i2c1",
    "i2c2",
    "gpio_out0",
    "gpio_out1",
    "gpio_out2",
    "gpio_in0",
    "gpio_in1",
    "gpio_in2",
];

#[cfg(feature = "std")]
struct SerialSlot {
    id: LinuxResourcesId,
    key: &'static str,
    default_path: &'static str,
}

#[cfg(feature = "std")]
const SERIAL_SLOTS: &[SerialSlot] = &[
    SerialSlot {
        id: LinuxResourcesId::SerialAcm0,
        key: SERIAL_ACM0_PATH_KEY,
        default_path: "/dev/ttyACM0",
    },
    SerialSlot {
        id: LinuxResourcesId::SerialAcm1,
        key: SERIAL_ACM1_PATH_KEY,
        default_path: "/dev/ttyACM1",
    },
    SerialSlot {
        id: LinuxResourcesId::SerialAcm2,
        key: SERIAL_ACM2_PATH_KEY,
        default_path: "/dev/ttyACM2",
    },
    SerialSlot {
        id: LinuxResourcesId::SerialUsb0,
        key: SERIAL_USB0_PATH_KEY,
        default_path: "/dev/ttyUSB0",
    },
    SerialSlot {
        id: LinuxResourcesId::SerialUsb1,
        key: SERIAL_USB1_PATH_KEY,
        default_path: "/dev/ttyUSB1",
    },
    SerialSlot {
        id: LinuxResourcesId::SerialUsb2,
        key: SERIAL_USB2_PATH_KEY,
        default_path: "/dev/ttyUSB2",
    },
];

#[cfg(feature = "std")]
struct I2cSlot {
    id: LinuxResourcesId,
    key: &'static str,
    default_path: &'static str,
}

#[cfg(feature = "std")]
const I2C_SLOTS: &[I2cSlot] = &[
    I2cSlot {
        id: LinuxResourcesId::I2c0,
        key: I2C0_PATH_KEY,
        default_path: "/dev/i2c-0",
    },
    I2cSlot {
        id: LinuxResourcesId::I2c1,
        key: I2C1_PATH_KEY,
        default_path: "/dev/i2c-1",
    },
    I2cSlot {
        id: LinuxResourcesId::I2c2,
        key: I2C2_PATH_KEY,
        default_path: "/dev/i2c-2",
    },
];

#[cfg(feature = "std")]
struct GpioSlot {
    id: LinuxResourcesId,
    key: &'static str,
}

#[cfg(feature = "std")]
const GPIO_OUT_SLOTS: &[GpioSlot] = &[
    GpioSlot {
        id: LinuxResourcesId::GpioOut0,
        key: GPIO_OUT0_PIN_KEY,
    },
    GpioSlot {
        id: LinuxResourcesId::GpioOut1,
        key: GPIO_OUT1_PIN_KEY,
    },
    GpioSlot {
        id: LinuxResourcesId::GpioOut2,
        key: GPIO_OUT2_PIN_KEY,
    },
];

#[cfg(feature = "std")]
const GPIO_IN_SLOTS: &[GpioSlot] = &[
    GpioSlot {
        id: LinuxResourcesId::GpioIn0,
        key: GPIO_IN0_PIN_KEY,
    },
    GpioSlot {
        id: LinuxResourcesId::GpioIn1,
        key: GPIO_IN1_PIN_KEY,
    },
    GpioSlot {
        id: LinuxResourcesId::GpioIn2,
        key: GPIO_IN2_PIN_KEY,
    },
];

#[cfg(feature = "std")]
impl ResourceBundle for LinuxResources {
    fn build(
        bundle: cu29::resource::BundleContext<Self>,
        config: Option<&ComponentConfig>,
        manager: &mut ResourceManager,
    ) -> CuResult<()> {
        let baudrate = get_u32(config, SERIAL_BAUDRATE_KEY)?.unwrap_or(DEFAULT_SERIAL_BAUDRATE);
        let timeout_ms =
            get_u64(config, SERIAL_TIMEOUT_MS_KEY)?.unwrap_or(DEFAULT_SERIAL_TIMEOUT_MS);

        for slot in SERIAL_SLOTS {
            let path =
                get_string(config, slot.key)?.unwrap_or_else(|| String::from(slot.default_path));
            match LinuxSerialPort::open(&path, baudrate, timeout_ms) {
                Ok(serial) => {
                    manager.add_owned(bundle.key(slot.id), serial)?;
                }
                Err(err) => {
                    eprintln!(
                        "LinuxResources: skipping serial slot {} ({}): {}",
                        slot_name(slot.id),
                        path,
                        err
                    );
                }
            }
        }

        #[cfg(target_os = "linux")]
        for slot in I2C_SLOTS {
            let path =
                get_string(config, slot.key)?.unwrap_or_else(|| String::from(slot.default_path));
            match linux_embedded_hal::I2cdev::new(&path) {
                Ok(dev) => {
                    manager.add_owned(bundle.key(slot.id), Exclusive::new(dev))?;
                }
                Err(err) => {
                    eprintln!(
                        "LinuxResources: skipping i2c slot {} ({}): {}",
                        slot_name(slot.id),
                        path,
                        err
                    );
                }
            }
        }

        #[cfg(target_os = "linux")]
        {
            let gpio = rppal::gpio::Gpio::new().map_err(|err| {
                CuError::new_with_cause("Failed to initialize GPIO subsystem", err)
            })?;

            for slot in GPIO_OUT_SLOTS {
                if let Some(pin) = get_u8(config, slot.key)? {
                    match gpio.get(pin) {
                        Ok(pin) => {
                            manager.add_owned(
                                bundle.key(slot.id),
                                Exclusive::new(pin.into_output()),
                            )?;
                        }
                        Err(err) => {
                            eprintln!(
                                "LinuxResources: skipping gpio output slot {} (pin {}): {}",
                                slot_name(slot.id),
                                pin,
                                err
                            );
                        }
                    }
                }
            }

            for slot in GPIO_IN_SLOTS {
                if let Some(pin) = get_u8(config, slot.key)? {
                    match gpio.get(pin) {
                        Ok(pin) => {
                            manager
                                .add_owned(bundle.key(slot.id), Exclusive::new(pin.into_input()))?;
                        }
                        Err(err) => {
                            eprintln!(
                                "LinuxResources: skipping gpio input slot {} (pin {}): {}",
                                slot_name(slot.id),
                                pin,
                                err
                            );
                        }
                    }
                }
            }
        }

        #[cfg(not(target_os = "linux"))]
        {
            for slot in GPIO_OUT_SLOTS {
                if let Some(pin) = get_u8(config, slot.key)? {
                    eprintln!(
                        "LinuxResources: requested gpio output slot {} on pin {} but GPIO is only supported on Linux",
                        slot_name(slot.id),
                        pin
                    );
                }
            }
            for slot in GPIO_IN_SLOTS {
                if let Some(pin) = get_u8(config, slot.key)? {
                    eprintln!(
                        "LinuxResources: requested gpio input slot {} on pin {} but GPIO is only supported on Linux",
                        slot_name(slot.id),
                        pin
                    );
                }
            }
        }

        Ok(())
    }
}

#[cfg(feature = "std")]
fn slot_name(id: LinuxResourcesId) -> &'static str {
    LINUX_RESOURCE_SLOT_NAMES[id as usize]
}

#[cfg(feature = "std")]
fn get_string(config: Option<&ComponentConfig>, key: &str) -> CuResult<Option<String>> {
    match config {
        Some(cfg) => Ok(cfg.get::<String>(key)?.filter(|value| !value.is_empty())),
        None => Ok(None),
    }
}

#[cfg(feature = "std")]
fn get_u8(config: Option<&ComponentConfig>, key: &str) -> CuResult<Option<u8>> {
    match config {
        Some(cfg) => Ok(cfg.get::<u8>(key)?),
        None => Ok(None),
    }
}

#[cfg(feature = "std")]
fn get_u32(config: Option<&ComponentConfig>, key: &str) -> CuResult<Option<u32>> {
    match config {
        Some(cfg) => Ok(cfg.get::<u32>(key)?),
        None => Ok(None),
    }
}

#[cfg(feature = "std")]
fn get_u64(config: Option<&ComponentConfig>, key: &str) -> CuResult<Option<u64>> {
    match config {
        Some(cfg) => Ok(cfg.get::<u64>(key)?),
        None => Ok(None),
    }
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
