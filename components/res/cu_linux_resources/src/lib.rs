use cu29::bundle_resources;
use cu29::prelude::*;
use cu29::resource::{ResourceBundle, ResourceManager};
use embedded_io::{Read as EmbeddedRead, Write as EmbeddedWrite};
#[cfg(feature = "embedded-io-07")]
use embedded_io_07 as embedded_io07;
use serialport::{Parity as SerialParity, StopBits as SerialStopBits};
use std::string::String;

pub const SERIAL0_DEV_KEY: &str = "serial0_dev";
pub const SERIAL0_BAUDRATE_KEY: &str = "serial0_baudrate";
pub const SERIAL0_PARITY_KEY: &str = "serial0_parity";
pub const SERIAL0_STOPBITS_KEY: &str = "serial0_stopbits";
pub const SERIAL0_TIMEOUT_MS_KEY: &str = "serial0_timeout_ms";

pub const SERIAL1_DEV_KEY: &str = "serial1_dev";
pub const SERIAL1_BAUDRATE_KEY: &str = "serial1_baudrate";
pub const SERIAL1_PARITY_KEY: &str = "serial1_parity";
pub const SERIAL1_STOPBITS_KEY: &str = "serial1_stopbits";
pub const SERIAL1_TIMEOUT_MS_KEY: &str = "serial1_timeout_ms";

pub const SERIAL2_DEV_KEY: &str = "serial2_dev";
pub const SERIAL2_BAUDRATE_KEY: &str = "serial2_baudrate";
pub const SERIAL2_PARITY_KEY: &str = "serial2_parity";
pub const SERIAL2_STOPBITS_KEY: &str = "serial2_stopbits";
pub const SERIAL2_TIMEOUT_MS_KEY: &str = "serial2_timeout_ms";

pub const SERIAL3_DEV_KEY: &str = "serial3_dev";
pub const SERIAL3_BAUDRATE_KEY: &str = "serial3_baudrate";
pub const SERIAL3_PARITY_KEY: &str = "serial3_parity";
pub const SERIAL3_STOPBITS_KEY: &str = "serial3_stopbits";
pub const SERIAL3_TIMEOUT_MS_KEY: &str = "serial3_timeout_ms";

pub const SERIAL4_DEV_KEY: &str = "serial4_dev";
pub const SERIAL4_BAUDRATE_KEY: &str = "serial4_baudrate";
pub const SERIAL4_PARITY_KEY: &str = "serial4_parity";
pub const SERIAL4_STOPBITS_KEY: &str = "serial4_stopbits";
pub const SERIAL4_TIMEOUT_MS_KEY: &str = "serial4_timeout_ms";

pub const SERIAL5_DEV_KEY: &str = "serial5_dev";
pub const SERIAL5_BAUDRATE_KEY: &str = "serial5_baudrate";
pub const SERIAL5_PARITY_KEY: &str = "serial5_parity";
pub const SERIAL5_STOPBITS_KEY: &str = "serial5_stopbits";
pub const SERIAL5_TIMEOUT_MS_KEY: &str = "serial5_timeout_ms";

pub const I2C0_DEV_KEY: &str = "i2c0_dev";
pub const I2C1_DEV_KEY: &str = "i2c1_dev";
pub const I2C2_DEV_KEY: &str = "i2c2_dev";

pub const GPIO_OUT0_PIN_KEY: &str = "gpio_out0_pin";
pub const GPIO_OUT1_PIN_KEY: &str = "gpio_out1_pin";
pub const GPIO_OUT2_PIN_KEY: &str = "gpio_out2_pin";
pub const GPIO_IN0_PIN_KEY: &str = "gpio_in0_pin";
pub const GPIO_IN1_PIN_KEY: &str = "gpio_in1_pin";
pub const GPIO_IN2_PIN_KEY: &str = "gpio_in2_pin";

pub const DEFAULT_SERIAL_BAUDRATE: u32 = 115_200;
pub const DEFAULT_SERIAL_TIMEOUT_MS: u64 = 50;
pub const DEFAULT_SERIAL_PARITY: SerialParity = SerialParity::None;
pub const DEFAULT_SERIAL_STOPBITS: SerialStopBits = SerialStopBits::One;

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

pub struct LinuxSerialPort {
    inner: Exclusive<Box<dyn serialport::SerialPort>>,
}

impl LinuxSerialPort {
    pub fn new(inner: Box<dyn serialport::SerialPort>) -> Self {
        Self {
            inner: Exclusive::new(inner),
        }
    }

    pub fn open(dev: &str, baudrate: u32, timeout_ms: u64) -> std::io::Result<Self> {
        let config = SerialSlotConfig {
            dev: dev.to_string(),
            baudrate,
            parity: DEFAULT_SERIAL_PARITY,
            stop_bits: DEFAULT_SERIAL_STOPBITS,
            timeout_ms,
        };
        Self::open_with_config(&config)
    }

    pub fn open_with_config(config: &SerialSlotConfig) -> std::io::Result<Self> {
        let port = serialport::new(config.dev.as_str(), config.baudrate)
            .parity(config.parity)
            .stop_bits(config.stop_bits)
            .timeout(std::time::Duration::from_millis(config.timeout_ms))
            .open()?;
        Ok(Self::new(port))
    }
}

impl std::io::Read for LinuxSerialPort {
    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        self.inner.read(buf)
    }
}

impl std::io::Write for LinuxSerialPort {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        self.inner.write(buf)
    }

    fn flush(&mut self) -> std::io::Result<()> {
        self.inner.flush()
    }
}

impl embedded_io::ErrorType for LinuxSerialPort {
    type Error = std::io::Error;
}

impl EmbeddedRead for LinuxSerialPort {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        std::io::Read::read(self, buf)
    }
}

impl EmbeddedWrite for LinuxSerialPort {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        std::io::Write::write(self, buf)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        std::io::Write::flush(self)
    }
}

#[cfg(target_os = "linux")]
pub type LinuxI2c = Exclusive<linux_embedded_hal::I2cdev>;
#[cfg(target_os = "linux")]
pub type LinuxOutputPin = Exclusive<rppal::gpio::OutputPin>;
#[cfg(target_os = "linux")]
pub type LinuxInputPin = Exclusive<rppal::gpio::InputPin>;

pub struct LinuxResources;

bundle_resources!(
    LinuxResources:
        Serial0,
        Serial1,
        Serial2,
        Serial3,
        Serial4,
        Serial5,
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

const LINUX_RESOURCE_SLOT_NAMES: &[&str] = &[
    "serial0",
    "serial1",
    "serial2",
    "serial3",
    "serial4",
    "serial5",
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

struct SerialSlot {
    id: LinuxResourcesId,
    dev_key: &'static str,
    baudrate_key: &'static str,
    parity_key: &'static str,
    stopbits_key: &'static str,
    timeout_ms_key: &'static str,
}

#[derive(Clone, Debug)]
pub struct SerialSlotConfig {
    pub dev: String,
    pub baudrate: u32,
    pub parity: SerialParity,
    pub stop_bits: SerialStopBits,
    pub timeout_ms: u64,
}

const SERIAL_SLOTS: &[SerialSlot] = &[
    SerialSlot {
        id: LinuxResourcesId::Serial0,
        dev_key: SERIAL0_DEV_KEY,
        baudrate_key: SERIAL0_BAUDRATE_KEY,
        parity_key: SERIAL0_PARITY_KEY,
        stopbits_key: SERIAL0_STOPBITS_KEY,
        timeout_ms_key: SERIAL0_TIMEOUT_MS_KEY,
    },
    SerialSlot {
        id: LinuxResourcesId::Serial1,
        dev_key: SERIAL1_DEV_KEY,
        baudrate_key: SERIAL1_BAUDRATE_KEY,
        parity_key: SERIAL1_PARITY_KEY,
        stopbits_key: SERIAL1_STOPBITS_KEY,
        timeout_ms_key: SERIAL1_TIMEOUT_MS_KEY,
    },
    SerialSlot {
        id: LinuxResourcesId::Serial2,
        dev_key: SERIAL2_DEV_KEY,
        baudrate_key: SERIAL2_BAUDRATE_KEY,
        parity_key: SERIAL2_PARITY_KEY,
        stopbits_key: SERIAL2_STOPBITS_KEY,
        timeout_ms_key: SERIAL2_TIMEOUT_MS_KEY,
    },
    SerialSlot {
        id: LinuxResourcesId::Serial3,
        dev_key: SERIAL3_DEV_KEY,
        baudrate_key: SERIAL3_BAUDRATE_KEY,
        parity_key: SERIAL3_PARITY_KEY,
        stopbits_key: SERIAL3_STOPBITS_KEY,
        timeout_ms_key: SERIAL3_TIMEOUT_MS_KEY,
    },
    SerialSlot {
        id: LinuxResourcesId::Serial4,
        dev_key: SERIAL4_DEV_KEY,
        baudrate_key: SERIAL4_BAUDRATE_KEY,
        parity_key: SERIAL4_PARITY_KEY,
        stopbits_key: SERIAL4_STOPBITS_KEY,
        timeout_ms_key: SERIAL4_TIMEOUT_MS_KEY,
    },
    SerialSlot {
        id: LinuxResourcesId::Serial5,
        dev_key: SERIAL5_DEV_KEY,
        baudrate_key: SERIAL5_BAUDRATE_KEY,
        parity_key: SERIAL5_PARITY_KEY,
        stopbits_key: SERIAL5_STOPBITS_KEY,
        timeout_ms_key: SERIAL5_TIMEOUT_MS_KEY,
    },
];

#[cfg(target_os = "linux")]
struct I2cSlot {
    id: LinuxResourcesId,
    dev_key: &'static str,
}

#[cfg(target_os = "linux")]
const I2C_SLOTS: &[I2cSlot] = &[
    I2cSlot {
        id: LinuxResourcesId::I2c0,
        dev_key: I2C0_DEV_KEY,
    },
    I2cSlot {
        id: LinuxResourcesId::I2c1,
        dev_key: I2C1_DEV_KEY,
    },
    I2cSlot {
        id: LinuxResourcesId::I2c2,
        dev_key: I2C2_DEV_KEY,
    },
];

struct GpioSlot {
    id: LinuxResourcesId,
    key: &'static str,
}

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

impl ResourceBundle for LinuxResources {
    fn build(
        bundle: cu29::resource::BundleContext<Self>,
        config: Option<&ComponentConfig>,
        manager: &mut ResourceManager,
    ) -> CuResult<()> {
        for slot in SERIAL_SLOTS {
            let Some(serial_config) = read_serial_slot_config(config, slot)? else {
                continue; // Skip slots without explicit config
            };
            match LinuxSerialPort::open_with_config(&serial_config) {
                Ok(serial) => {
                    manager.add_owned(bundle.key(slot.id), serial)?;
                }
                Err(err) => {
                    warning!(
                        "LinuxResources: skipping serial slot {} (dev {}): {}",
                        slot_name(slot.id),
                        serial_config.dev,
                        err.to_string()
                    );
                }
            }
        }

        #[cfg(target_os = "linux")]
        for slot in I2C_SLOTS {
            let Some(dev) = get_string(config, slot.dev_key)? else {
                continue; // Skip slots without explicit config
            };
            match linux_embedded_hal::I2cdev::new(&dev) {
                Ok(i2c) => {
                    manager.add_owned(bundle.key(slot.id), Exclusive::new(i2c))?;
                }
                Err(err) => {
                    warning!(
                        "LinuxResources: skipping i2c slot {} (dev {}): {}",
                        slot_name(slot.id),
                        dev,
                        err.to_string()
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
                            warning!(
                                "LinuxResources: skipping gpio output slot {} (pin {}): {}",
                                slot_name(slot.id),
                                pin,
                                err.to_string()
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
                            warning!(
                                "LinuxResources: skipping gpio input slot {} (pin {}): {}",
                                slot_name(slot.id),
                                pin,
                                err.to_string()
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
                    warning!(
                        "LinuxResources: requested gpio output slot {} on pin {} but GPIO is only supported on Linux",
                        slot_name(slot.id),
                        pin
                    );
                }
            }
            for slot in GPIO_IN_SLOTS {
                if let Some(pin) = get_u8(config, slot.key)? {
                    warning!(
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

fn read_serial_slot_config(
    config: Option<&ComponentConfig>,
    slot: &SerialSlot,
) -> CuResult<Option<SerialSlotConfig>> {
    let Some(dev) = get_string(config, slot.dev_key)? else {
        return Ok(None); // No device configured for this slot
    };
    let baudrate = get_u32(config, slot.baudrate_key)?.unwrap_or(DEFAULT_SERIAL_BAUDRATE);
    let parity = get_serial_parity(config, slot.parity_key)?.unwrap_or(DEFAULT_SERIAL_PARITY);
    let stop_bits =
        get_serial_stop_bits(config, slot.stopbits_key)?.unwrap_or(DEFAULT_SERIAL_STOPBITS);
    let timeout_ms = get_u64(config, slot.timeout_ms_key)?.unwrap_or(DEFAULT_SERIAL_TIMEOUT_MS);

    Ok(Some(SerialSlotConfig {
        dev,
        baudrate,
        parity,
        stop_bits,
        timeout_ms,
    }))
}

fn get_serial_parity(
    config: Option<&ComponentConfig>,
    key: &str,
) -> CuResult<Option<SerialParity>> {
    let Some(raw) = get_string(config, key)? else {
        return Ok(None);
    };
    Ok(Some(parse_serial_parity_value(raw.as_str())?))
}

fn get_serial_stop_bits(
    config: Option<&ComponentConfig>,
    key: &str,
) -> CuResult<Option<SerialStopBits>> {
    let Some(raw) = get_u8(config, key)? else {
        return Ok(None);
    };
    Ok(Some(parse_serial_stop_bits_value(raw)?))
}

fn parse_serial_parity_value(raw: &str) -> CuResult<SerialParity> {
    let normalized = raw.trim().to_ascii_lowercase();
    match normalized.as_str() {
        "none" => Ok(SerialParity::None),
        "odd" => Ok(SerialParity::Odd),
        "even" => Ok(SerialParity::Even),
        _ => Err(CuError::from(format!(
            "Invalid parity '{raw}'. Expected one of: none, odd, even"
        ))),
    }
}

fn parse_serial_stop_bits_value(raw: u8) -> CuResult<SerialStopBits> {
    match raw {
        1 => Ok(SerialStopBits::One),
        2 => Ok(SerialStopBits::Two),
        _ => Err(CuError::from(format!(
            "Invalid stopbits value '{raw}'. Expected 1 or 2"
        ))),
    }
}

fn slot_name(id: LinuxResourcesId) -> &'static str {
    LINUX_RESOURCE_SLOT_NAMES[id as usize]
}

fn get_string(config: Option<&ComponentConfig>, key: &str) -> CuResult<Option<String>> {
    match config {
        Some(cfg) => Ok(cfg.get::<String>(key)?.filter(|value| !value.is_empty())),
        None => Ok(None),
    }
}

fn get_u8(config: Option<&ComponentConfig>, key: &str) -> CuResult<Option<u8>> {
    match config {
        Some(cfg) => Ok(cfg.get::<u8>(key)?),
        None => Ok(None),
    }
}

fn get_u32(config: Option<&ComponentConfig>, key: &str) -> CuResult<Option<u32>> {
    match config {
        Some(cfg) => Ok(cfg.get::<u32>(key)?),
        None => Ok(None),
    }
}

fn get_u64(config: Option<&ComponentConfig>, key: &str) -> CuResult<Option<u64>> {
    match config {
        Some(cfg) => Ok(cfg.get::<u64>(key)?),
        None => Ok(None),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_serial_parity_value_accepts_expected_inputs() {
        assert!(matches!(
            parse_serial_parity_value("none").unwrap(),
            SerialParity::None
        ));
        assert!(matches!(
            parse_serial_parity_value("Odd").unwrap(),
            SerialParity::Odd
        ));
        assert!(matches!(
            parse_serial_parity_value("EVEN").unwrap(),
            SerialParity::Even
        ));
    }

    #[test]
    fn parse_serial_parity_value_rejects_invalid_input() {
        assert!(parse_serial_parity_value("mark").is_err());
    }

    #[test]
    fn parse_serial_stop_bits_value_accepts_expected_inputs() {
        assert!(matches!(
            parse_serial_stop_bits_value(1).unwrap(),
            SerialStopBits::One
        ));
        assert!(matches!(
            parse_serial_stop_bits_value(2).unwrap(),
            SerialStopBits::Two
        ));
    }

    #[test]
    fn parse_serial_stop_bits_value_rejects_invalid_input() {
        assert!(parse_serial_stop_bits_value(0).is_err());
        assert!(parse_serial_stop_bits_value(3).is_err());
    }

    #[cfg(feature = "embedded-io-07")]
    struct MockIo {
        rx: [u8; 4],
        rx_len: usize,
        tx: [u8; 4],
        tx_len: usize,
    }

    #[cfg(feature = "embedded-io-07")]
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

    #[cfg(feature = "embedded-io-07")]
    impl embedded_io_07::ErrorType for MockIo {
        type Error = core::convert::Infallible;
    }

    #[cfg(feature = "embedded-io-07")]
    impl embedded_io_07::Read for MockIo {
        fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
            let n = core::cmp::min(buf.len(), self.rx_len);
            buf[..n].copy_from_slice(&self.rx[..n]);
            Ok(n)
        }
    }

    #[cfg(feature = "embedded-io-07")]
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

    #[cfg(feature = "embedded-io-07")]
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
