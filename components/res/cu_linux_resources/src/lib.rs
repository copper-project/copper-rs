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

pub const GPIO0_NAME: &str = "gpio0";
pub const GPIO1_NAME: &str = "gpio1";
pub const GPIO2_NAME: &str = "gpio2";
pub const GPIO3_NAME: &str = "gpio3";
pub const GPIO4_NAME: &str = "gpio4";
pub const GPIO5_NAME: &str = "gpio5";

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
pub type LinuxOutputPin = Exclusive<rppal::gpio::IoPin>;
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
        Gpio0,
        Gpio1,
        Gpio2,
        Gpio3,
        Gpio4,
        Gpio5
);

const LINUX_RESOURCE_SLOT_NAMES: &[&str] = &[
    "serial0", "serial1", "serial2", "serial3", "serial4", "serial5", "i2c0", "i2c1", "i2c2",
    GPIO0_NAME, GPIO1_NAME, GPIO2_NAME, GPIO3_NAME, GPIO4_NAME, GPIO5_NAME,
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

#[cfg_attr(not(any(target_os = "linux", test)), allow(dead_code))]
struct GpioSlot {
    id: LinuxResourcesId,
    name: &'static str,
    pin_key: &'static str,
    direction_key: &'static str,
    bias_key: &'static str,
    initial_level_key: &'static str,
}

#[cfg(any(target_os = "linux", test))]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
enum GpioDirection {
    Input,
    Output,
}

#[cfg(any(target_os = "linux", test))]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
enum GpioBias {
    Off,
    PullDown,
    PullUp,
}

#[cfg(any(target_os = "linux", test))]
impl GpioBias {
    #[cfg(target_os = "linux")]
    const fn into_rppal(self) -> rppal::gpio::Bias {
        match self {
            GpioBias::Off => rppal::gpio::Bias::Off,
            GpioBias::PullDown => rppal::gpio::Bias::PullDown,
            GpioBias::PullUp => rppal::gpio::Bias::PullUp,
        }
    }
}

#[cfg(any(target_os = "linux", test))]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
enum GpioInitialLevel {
    Low,
    High,
}

#[cfg(any(target_os = "linux", test))]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
struct GpioSlotConfig {
    pin: u8,
    direction: GpioDirection,
    bias: GpioBias,
    initial_level: Option<GpioInitialLevel>,
}

const GPIO_SLOTS: &[GpioSlot] = &[
    GpioSlot {
        id: LinuxResourcesId::Gpio0,
        name: GPIO0_NAME,
        pin_key: "gpio0_pin",
        direction_key: "gpio0_direction",
        bias_key: "gpio0_bias",
        initial_level_key: "gpio0_initial_level",
    },
    GpioSlot {
        id: LinuxResourcesId::Gpio1,
        name: GPIO1_NAME,
        pin_key: "gpio1_pin",
        direction_key: "gpio1_direction",
        bias_key: "gpio1_bias",
        initial_level_key: "gpio1_initial_level",
    },
    GpioSlot {
        id: LinuxResourcesId::Gpio2,
        name: GPIO2_NAME,
        pin_key: "gpio2_pin",
        direction_key: "gpio2_direction",
        bias_key: "gpio2_bias",
        initial_level_key: "gpio2_initial_level",
    },
    GpioSlot {
        id: LinuxResourcesId::Gpio3,
        name: GPIO3_NAME,
        pin_key: "gpio3_pin",
        direction_key: "gpio3_direction",
        bias_key: "gpio3_bias",
        initial_level_key: "gpio3_initial_level",
    },
    GpioSlot {
        id: LinuxResourcesId::Gpio4,
        name: GPIO4_NAME,
        pin_key: "gpio4_pin",
        direction_key: "gpio4_direction",
        bias_key: "gpio4_bias",
        initial_level_key: "gpio4_initial_level",
    },
    GpioSlot {
        id: LinuxResourcesId::Gpio5,
        name: GPIO5_NAME,
        pin_key: "gpio5_pin",
        direction_key: "gpio5_direction",
        bias_key: "gpio5_bias",
        initial_level_key: "gpio5_initial_level",
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
            let mut configured_gpio_slots: std::vec::Vec<(
                LinuxResourcesId,
                &'static str,
                GpioSlotConfig,
            )> = std::vec::Vec::new();

            for slot in GPIO_SLOTS {
                let Some(slot_config) = read_gpio_slot_config(config, slot)? else {
                    continue;
                };
                configured_gpio_slots.push((slot.id, slot.name, slot_config));
            }

            if !configured_gpio_slots.is_empty() {
                let gpio = rppal::gpio::Gpio::new().map_err(|err| {
                    CuError::new_with_cause("Failed to initialize GPIO subsystem", err)
                })?;

                for (slot_id, slot_name, slot_config) in configured_gpio_slots {
                    let pin = match gpio.get(slot_config.pin) {
                        Ok(pin) => pin,
                        Err(err) => {
                            warning!(
                                "LinuxResources: skipping gpio slot {} (pin {}): {}",
                                slot_name,
                                slot_config.pin,
                                err.to_string()
                            );
                            continue;
                        }
                    };
                    match slot_config.direction {
                        GpioDirection::Input => {
                            let mut pin = pin.into_input();
                            pin.set_bias(slot_config.bias.into_rppal());
                            manager.add_owned(bundle.key(slot_id), Exclusive::new(pin))?;
                        }
                        GpioDirection::Output => {
                            let mut pin = pin.into_io(rppal::gpio::Mode::Output);
                            pin.set_bias(slot_config.bias.into_rppal());
                            if let Some(initial_level) = slot_config.initial_level {
                                match initial_level {
                                    GpioInitialLevel::Low => pin.set_low(),
                                    GpioInitialLevel::High => pin.set_high(),
                                }
                            }
                            manager.add_owned(bundle.key(slot_id), Exclusive::new(pin))?;
                        }
                    }
                }
            }
        }

        #[cfg(not(target_os = "linux"))]
        {
            for slot in GPIO_SLOTS {
                if let Some(pin) = get_u8(config, slot.pin_key)? {
                    warning!(
                        "LinuxResources: requested gpio slot {} on pin {} but GPIO is only supported on Linux",
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

#[cfg(any(target_os = "linux", test))]
fn read_gpio_slot_config(
    config: Option<&ComponentConfig>,
    slot: &GpioSlot,
) -> CuResult<Option<GpioSlotConfig>> {
    let pin = get_u8(config, slot.pin_key)?;
    let direction = get_string(config, slot.direction_key)?;
    let bias = get_string(config, slot.bias_key)?;
    let initial_level = get_string(config, slot.initial_level_key)?;

    let Some(pin) = pin else {
        if direction.is_some() || bias.is_some() || initial_level.is_some() {
            return Err(CuError::from(format!(
                "Config key '{}' is required when configuring {}",
                slot.pin_key, slot.name
            )));
        }
        return Ok(None);
    };

    let direction_raw = direction.ok_or_else(|| {
        CuError::from(format!(
            "Config key '{}' is required when '{}' is set",
            slot.direction_key, slot.pin_key
        ))
    })?;
    let direction = parse_gpio_direction_value(direction_raw.as_str())?;

    let bias = match bias {
        Some(raw) => parse_gpio_bias_value(raw.as_str())?,
        None => GpioBias::Off,
    };

    let initial_level = match initial_level {
        Some(raw) => Some(parse_gpio_initial_level_value(raw.as_str())?),
        None => None,
    };

    if matches!(direction, GpioDirection::Input) && initial_level.is_some() {
        return Err(CuError::from(format!(
            "Config key '{}' is only valid when '{}' is 'output'",
            slot.initial_level_key, slot.direction_key
        )));
    }

    Ok(Some(GpioSlotConfig {
        pin,
        direction,
        bias,
        initial_level,
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

#[cfg(any(target_os = "linux", test))]
fn parse_gpio_direction_value(raw: &str) -> CuResult<GpioDirection> {
    let normalized = raw.trim().to_ascii_lowercase();
    match normalized.as_str() {
        "input" => Ok(GpioDirection::Input),
        "output" => Ok(GpioDirection::Output),
        _ => Err(CuError::from(format!(
            "Invalid GPIO direction '{raw}'. Expected one of: input, output"
        ))),
    }
}

#[cfg(any(target_os = "linux", test))]
fn parse_gpio_bias_value(raw: &str) -> CuResult<GpioBias> {
    let normalized = raw.trim().to_ascii_lowercase();
    match normalized.as_str() {
        "off" | "none" => Ok(GpioBias::Off),
        "pull_down" | "pulldown" => Ok(GpioBias::PullDown),
        "pull_up" | "pullup" => Ok(GpioBias::PullUp),
        _ => Err(CuError::from(format!(
            "Invalid GPIO bias '{raw}'. Expected one of: off, pull_up, pull_down"
        ))),
    }
}

#[cfg(any(target_os = "linux", test))]
fn parse_gpio_initial_level_value(raw: &str) -> CuResult<GpioInitialLevel> {
    let normalized = raw.trim().to_ascii_lowercase();
    match normalized.as_str() {
        "low" => Ok(GpioInitialLevel::Low),
        "high" => Ok(GpioInitialLevel::High),
        _ => Err(CuError::from(format!(
            "Invalid GPIO initial level '{raw}'. Expected one of: low, high"
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

    #[test]
    fn parse_gpio_direction_value_accepts_expected_inputs() {
        assert!(matches!(
            parse_gpio_direction_value("input").unwrap(),
            GpioDirection::Input
        ));
        assert!(matches!(
            parse_gpio_direction_value("Output").unwrap(),
            GpioDirection::Output
        ));
    }

    #[test]
    fn parse_gpio_direction_value_rejects_invalid_input() {
        assert!(parse_gpio_direction_value("io").is_err());
    }

    #[test]
    fn parse_gpio_bias_value_accepts_expected_inputs() {
        assert!(matches!(
            parse_gpio_bias_value("off").unwrap(),
            GpioBias::Off
        ));
        assert!(matches!(
            parse_gpio_bias_value("pull_down").unwrap(),
            GpioBias::PullDown
        ));
        assert!(matches!(
            parse_gpio_bias_value("PullUp").unwrap(),
            GpioBias::PullUp
        ));
    }

    #[test]
    fn parse_gpio_bias_value_rejects_invalid_input() {
        assert!(parse_gpio_bias_value("hold").is_err());
    }

    #[test]
    fn parse_gpio_initial_level_value_accepts_expected_inputs() {
        assert!(matches!(
            parse_gpio_initial_level_value("low").unwrap(),
            GpioInitialLevel::Low
        ));
        assert!(matches!(
            parse_gpio_initial_level_value("HIGH").unwrap(),
            GpioInitialLevel::High
        ));
    }

    #[test]
    fn parse_gpio_initial_level_value_rejects_invalid_input() {
        assert!(parse_gpio_initial_level_value("toggle").is_err());
    }

    #[test]
    fn read_gpio_slot_config_requires_pin_when_aux_keys_are_present() {
        let mut cfg = ComponentConfig::new();
        cfg.set("gpio0_direction", "input".to_string());
        assert!(read_gpio_slot_config(Some(&cfg), &GPIO_SLOTS[0]).is_err());
    }

    #[test]
    fn read_gpio_slot_config_requires_direction_when_pin_is_set() {
        let mut cfg = ComponentConfig::new();
        cfg.set("gpio0_pin", 23_u8);
        assert!(read_gpio_slot_config(Some(&cfg), &GPIO_SLOTS[0]).is_err());
    }

    #[test]
    fn read_gpio_slot_config_defaults_bias_and_preserves_level_for_output() {
        let mut cfg = ComponentConfig::new();
        cfg.set("gpio0_pin", 23_u8);
        cfg.set("gpio0_direction", "output".to_string());

        let slot_config = read_gpio_slot_config(Some(&cfg), &GPIO_SLOTS[0])
            .unwrap()
            .expect("slot should be configured");
        assert_eq!(
            slot_config,
            GpioSlotConfig {
                pin: 23,
                direction: GpioDirection::Output,
                bias: GpioBias::Off,
                initial_level: None,
            }
        );
    }

    #[test]
    fn read_gpio_slot_config_parses_explicit_initial_level_for_output() {
        let mut cfg = ComponentConfig::new();
        cfg.set("gpio0_pin", 23_u8);
        cfg.set("gpio0_direction", "output".to_string());
        cfg.set("gpio0_initial_level", "high".to_string());

        let slot_config = read_gpio_slot_config(Some(&cfg), &GPIO_SLOTS[0])
            .unwrap()
            .expect("slot should be configured");
        assert_eq!(
            slot_config,
            GpioSlotConfig {
                pin: 23,
                direction: GpioDirection::Output,
                bias: GpioBias::Off,
                initial_level: Some(GpioInitialLevel::High),
            }
        );
    }

    #[test]
    fn read_gpio_slot_config_rejects_initial_level_for_input() {
        let mut cfg = ComponentConfig::new();
        cfg.set("gpio0_pin", 23_u8);
        cfg.set("gpio0_direction", "input".to_string());
        cfg.set("gpio0_initial_level", "high".to_string());
        assert!(read_gpio_slot_config(Some(&cfg), &GPIO_SLOTS[0]).is_err());
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
