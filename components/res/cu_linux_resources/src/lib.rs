use cu29::bundle_resources;
use cu29::prelude::*;
use cu29::resource::{ResourceBundle, ResourceManager};
use embedded_io::{ErrorType as EmbeddedErrorType, Read as EmbeddedRead, Write as EmbeddedWrite};
use serialport::{Parity as SerialParity, StopBits as SerialStopBits};
use std::string::String;

#[cfg(all(unix, feature = "serial-rts"))]
mod serial_rts;
#[cfg(all(unix, feature = "serial-rts"))]
pub use serial_rts::{LinuxSerialRtsPin, SerialRtsError};

pub const SERIAL0_DEV_KEY: &str = "serial0_dev";
pub const SERIAL0_BAUDRATE_KEY: &str = "serial0_baudrate";
pub const SERIAL0_PARITY_KEY: &str = "serial0_parity";
pub const SERIAL0_STOPBITS_KEY: &str = "serial0_stopbits";
pub const SERIAL0_TIMEOUT_MS_KEY: &str = "serial0_timeout_ms";
pub const SERIAL0_NAME: &str = "serial0";

pub const SERIAL1_DEV_KEY: &str = "serial1_dev";
pub const SERIAL1_BAUDRATE_KEY: &str = "serial1_baudrate";
pub const SERIAL1_PARITY_KEY: &str = "serial1_parity";
pub const SERIAL1_STOPBITS_KEY: &str = "serial1_stopbits";
pub const SERIAL1_TIMEOUT_MS_KEY: &str = "serial1_timeout_ms";
pub const SERIAL1_NAME: &str = "serial1";

pub const SERIAL2_DEV_KEY: &str = "serial2_dev";
pub const SERIAL2_BAUDRATE_KEY: &str = "serial2_baudrate";
pub const SERIAL2_PARITY_KEY: &str = "serial2_parity";
pub const SERIAL2_STOPBITS_KEY: &str = "serial2_stopbits";
pub const SERIAL2_TIMEOUT_MS_KEY: &str = "serial2_timeout_ms";
pub const SERIAL2_NAME: &str = "serial2";

pub const SERIAL3_DEV_KEY: &str = "serial3_dev";
pub const SERIAL3_BAUDRATE_KEY: &str = "serial3_baudrate";
pub const SERIAL3_PARITY_KEY: &str = "serial3_parity";
pub const SERIAL3_STOPBITS_KEY: &str = "serial3_stopbits";
pub const SERIAL3_TIMEOUT_MS_KEY: &str = "serial3_timeout_ms";
pub const SERIAL3_NAME: &str = "serial3";

pub const SERIAL4_DEV_KEY: &str = "serial4_dev";
pub const SERIAL4_BAUDRATE_KEY: &str = "serial4_baudrate";
pub const SERIAL4_PARITY_KEY: &str = "serial4_parity";
pub const SERIAL4_STOPBITS_KEY: &str = "serial4_stopbits";
pub const SERIAL4_TIMEOUT_MS_KEY: &str = "serial4_timeout_ms";
pub const SERIAL4_NAME: &str = "serial4";

pub const SERIAL5_DEV_KEY: &str = "serial5_dev";
pub const SERIAL5_BAUDRATE_KEY: &str = "serial5_baudrate";
pub const SERIAL5_PARITY_KEY: &str = "serial5_parity";
pub const SERIAL5_STOPBITS_KEY: &str = "serial5_stopbits";
pub const SERIAL5_TIMEOUT_MS_KEY: &str = "serial5_timeout_ms";
pub const SERIAL5_NAME: &str = "serial5";

pub const I2C0_DEV_KEY: &str = "i2c0_dev";
pub const I2C1_DEV_KEY: &str = "i2c1_dev";
pub const I2C2_DEV_KEY: &str = "i2c2_dev";
pub const I2C0_NAME: &str = "i2c0";
pub const I2C1_NAME: &str = "i2c1";
pub const I2C2_NAME: &str = "i2c2";

pub const GPIO0_NAME: &str = "gpio0";
pub const GPIO1_NAME: &str = "gpio1";
pub const GPIO2_NAME: &str = "gpio2";
pub const GPIO3_NAME: &str = "gpio3";
pub const GPIO4_NAME: &str = "gpio4";
pub const GPIO5_NAME: &str = "gpio5";
pub const GPIO0_PIN_KEY: &str = "gpio0_pin";
pub const GPIO1_PIN_KEY: &str = "gpio1_pin";
pub const GPIO2_PIN_KEY: &str = "gpio2_pin";
pub const GPIO3_PIN_KEY: &str = "gpio3_pin";
pub const GPIO4_PIN_KEY: &str = "gpio4_pin";
pub const GPIO5_PIN_KEY: &str = "gpio5_pin";
pub const GPIO0_DIRECTION_KEY: &str = "gpio0_direction";
pub const GPIO1_DIRECTION_KEY: &str = "gpio1_direction";
pub const GPIO2_DIRECTION_KEY: &str = "gpio2_direction";
pub const GPIO3_DIRECTION_KEY: &str = "gpio3_direction";
pub const GPIO4_DIRECTION_KEY: &str = "gpio4_direction";
pub const GPIO5_DIRECTION_KEY: &str = "gpio5_direction";
pub const GPIO0_BIAS_KEY: &str = "gpio0_bias";
pub const GPIO1_BIAS_KEY: &str = "gpio1_bias";
pub const GPIO2_BIAS_KEY: &str = "gpio2_bias";
pub const GPIO3_BIAS_KEY: &str = "gpio3_bias";
pub const GPIO4_BIAS_KEY: &str = "gpio4_bias";
pub const GPIO5_BIAS_KEY: &str = "gpio5_bias";
pub const GPIO0_INITIAL_LEVEL_KEY: &str = "gpio0_initial_level";
pub const GPIO1_INITIAL_LEVEL_KEY: &str = "gpio1_initial_level";
pub const GPIO2_INITIAL_LEVEL_KEY: &str = "gpio2_initial_level";
pub const GPIO3_INITIAL_LEVEL_KEY: &str = "gpio3_initial_level";
pub const GPIO4_INITIAL_LEVEL_KEY: &str = "gpio4_initial_level";
pub const GPIO5_INITIAL_LEVEL_KEY: &str = "gpio5_initial_level";

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
    /// Open a UART and an independently owned, active-low RTS# output.
    #[cfg(all(unix, feature = "serial-rts"))]
    pub fn open_with_rts(config: &SerialSlotConfig) -> std::io::Result<(Self, LinuxSerialRtsPin)> {
        let (port, rts) = serial_rts::open(config)?;
        Ok((Self::new(Box::new(port)), rts))
    }

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

/// Native nonblocking UART for resource stacks. Configure a serial slot with
/// `serialN_nonblocking: true` to export this type.
#[cfg(unix)]
#[derive(Debug)]
pub struct LinuxNonblockingSerialPort(std::fs::File);

#[cfg(unix)]
impl LinuxNonblockingSerialPort {
    /// Open a nonblocking UART and an independently owned, active-low RTS# output.
    #[cfg(feature = "serial-rts")]
    pub fn open_with_rts(config: &SerialSlotConfig) -> std::io::Result<(Self, LinuxSerialRtsPin)> {
        let (port, rts) = serial_rts::open(config)?;
        Ok((Self::from_native(port)?, rts))
    }

    pub fn open_with_config(config: &SerialSlotConfig) -> std::io::Result<Self> {
        let port = serialport::new(&config.dev, config.baudrate)
            .parity(config.parity)
            .stop_bits(config.stop_bits)
            .open_native()?;
        Self::from_native(port)
    }

    /// Take ownership of an already configured native UART.
    pub fn from_native(port: serialport::TTYPort) -> std::io::Result<Self> {
        use std::os::fd::{AsRawFd, FromRawFd, IntoRawFd};
        let fd = port.as_raw_fd();
        // SAFETY: the live TTYPort owns fd; fcntl changes only its status flags.
        let flags = unsafe { libc::fcntl(fd, libc::F_GETFL) };
        if flags < 0 || unsafe { libc::fcntl(fd, libc::F_SETFL, flags | libc::O_NONBLOCK) } < 0 {
            return Err(std::io::Error::last_os_error());
        }
        // SAFETY: ownership transfers once from TTYPort into File.
        Ok(Self(unsafe {
            std::fs::File::from_raw_fd(port.into_raw_fd())
        }))
    }

    fn result(result: std::io::Result<usize>) -> std::io::Result<usize> {
        match result {
            Err(error)
                if matches!(
                    error.kind(),
                    std::io::ErrorKind::WouldBlock | std::io::ErrorKind::Interrupted
                ) =>
            {
                Ok(0)
            }
            other => other,
        }
    }
}
#[cfg(unix)]
impl embedded_io::ErrorType for LinuxNonblockingSerialPort {
    type Error = std::io::Error;
}
#[cfg(unix)]
impl cu_serial::SerialIo for LinuxNonblockingSerialPort {
    fn try_read(&mut self, bytes: &mut [u8]) -> std::io::Result<usize> {
        let result = std::io::Read::read(&mut self.0, bytes);
        if !bytes.is_empty() && matches!(result, Ok(0)) {
            return Err(std::io::Error::from(std::io::ErrorKind::UnexpectedEof));
        }
        Self::result(result)
    }
    fn try_write(&mut self, bytes: &[u8]) -> std::io::Result<usize> {
        Self::result(std::io::Write::write(&mut self.0, bytes))
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
        Serial0 = "serial0",
        Serial1 = "serial1",
        Serial2 = "serial2",
        Serial3 = "serial3",
        Serial4 = "serial4",
        Serial5 = "serial5",
        I2c0 = "i2c0",
        I2c1 = "i2c1",
        I2c2 = "i2c2",
        Gpio0 = "gpio0",
        Gpio1 = "gpio1",
        Gpio2 = "gpio2",
        Gpio3 = "gpio3",
        Gpio4 = "gpio4",
        Gpio5 = "gpio5",
        Serial0Rts = "serial0_rts",
        Serial1Rts = "serial1_rts",
        Serial2Rts = "serial2_rts",
        Serial3Rts = "serial3_rts",
        Serial4Rts = "serial4_rts",
        Serial5Rts = "serial5_rts"
);

const LINUX_RESOURCE_SLOT_NAMES: &[&str] = &[
    SERIAL0_NAME,
    SERIAL1_NAME,
    SERIAL2_NAME,
    SERIAL3_NAME,
    SERIAL4_NAME,
    SERIAL5_NAME,
    I2C0_NAME,
    I2C1_NAME,
    I2C2_NAME,
    GPIO0_NAME,
    GPIO1_NAME,
    GPIO2_NAME,
    GPIO3_NAME,
    GPIO4_NAME,
    GPIO5_NAME,
    "serial0_rts",
    "serial1_rts",
    "serial2_rts",
    "serial3_rts",
    "serial4_rts",
    "serial5_rts",
];

struct SerialSlot {
    id: LinuxResourcesId,
    rts_id: LinuxResourcesId,
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
        rts_id: LinuxResourcesId::Serial0Rts,
        dev_key: SERIAL0_DEV_KEY,
        baudrate_key: SERIAL0_BAUDRATE_KEY,
        parity_key: SERIAL0_PARITY_KEY,
        stopbits_key: SERIAL0_STOPBITS_KEY,
        timeout_ms_key: SERIAL0_TIMEOUT_MS_KEY,
    },
    SerialSlot {
        id: LinuxResourcesId::Serial1,
        rts_id: LinuxResourcesId::Serial1Rts,
        dev_key: SERIAL1_DEV_KEY,
        baudrate_key: SERIAL1_BAUDRATE_KEY,
        parity_key: SERIAL1_PARITY_KEY,
        stopbits_key: SERIAL1_STOPBITS_KEY,
        timeout_ms_key: SERIAL1_TIMEOUT_MS_KEY,
    },
    SerialSlot {
        id: LinuxResourcesId::Serial2,
        rts_id: LinuxResourcesId::Serial2Rts,
        dev_key: SERIAL2_DEV_KEY,
        baudrate_key: SERIAL2_BAUDRATE_KEY,
        parity_key: SERIAL2_PARITY_KEY,
        stopbits_key: SERIAL2_STOPBITS_KEY,
        timeout_ms_key: SERIAL2_TIMEOUT_MS_KEY,
    },
    SerialSlot {
        id: LinuxResourcesId::Serial3,
        rts_id: LinuxResourcesId::Serial3Rts,
        dev_key: SERIAL3_DEV_KEY,
        baudrate_key: SERIAL3_BAUDRATE_KEY,
        parity_key: SERIAL3_PARITY_KEY,
        stopbits_key: SERIAL3_STOPBITS_KEY,
        timeout_ms_key: SERIAL3_TIMEOUT_MS_KEY,
    },
    SerialSlot {
        id: LinuxResourcesId::Serial4,
        rts_id: LinuxResourcesId::Serial4Rts,
        dev_key: SERIAL4_DEV_KEY,
        baudrate_key: SERIAL4_BAUDRATE_KEY,
        parity_key: SERIAL4_PARITY_KEY,
        stopbits_key: SERIAL4_STOPBITS_KEY,
        timeout_ms_key: SERIAL4_TIMEOUT_MS_KEY,
    },
    SerialSlot {
        id: LinuxResourcesId::Serial5,
        rts_id: LinuxResourcesId::Serial5Rts,
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

macro_rules! gpio_slot {
    ($id:ident, $name:ident, $pin_key:ident, $direction_key:ident, $bias_key:ident, $initial_level_key:ident) => {
        GpioSlot {
            id: LinuxResourcesId::$id,
            name: $name,
            pin_key: $pin_key,
            direction_key: $direction_key,
            bias_key: $bias_key,
            initial_level_key: $initial_level_key,
        }
    };
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
    gpio_slot!(
        Gpio0,
        GPIO0_NAME,
        GPIO0_PIN_KEY,
        GPIO0_DIRECTION_KEY,
        GPIO0_BIAS_KEY,
        GPIO0_INITIAL_LEVEL_KEY
    ),
    gpio_slot!(
        Gpio1,
        GPIO1_NAME,
        GPIO1_PIN_KEY,
        GPIO1_DIRECTION_KEY,
        GPIO1_BIAS_KEY,
        GPIO1_INITIAL_LEVEL_KEY
    ),
    gpio_slot!(
        Gpio2,
        GPIO2_NAME,
        GPIO2_PIN_KEY,
        GPIO2_DIRECTION_KEY,
        GPIO2_BIAS_KEY,
        GPIO2_INITIAL_LEVEL_KEY
    ),
    gpio_slot!(
        Gpio3,
        GPIO3_NAME,
        GPIO3_PIN_KEY,
        GPIO3_DIRECTION_KEY,
        GPIO3_BIAS_KEY,
        GPIO3_INITIAL_LEVEL_KEY
    ),
    gpio_slot!(
        Gpio4,
        GPIO4_NAME,
        GPIO4_PIN_KEY,
        GPIO4_DIRECTION_KEY,
        GPIO4_BIAS_KEY,
        GPIO4_INITIAL_LEVEL_KEY
    ),
    gpio_slot!(
        Gpio5,
        GPIO5_NAME,
        GPIO5_PIN_KEY,
        GPIO5_DIRECTION_KEY,
        GPIO5_BIAS_KEY,
        GPIO5_INITIAL_LEVEL_KEY
    ),
];

impl ResourceBundle for LinuxResources {
    fn build(
        bundle: cu29::resource::BundleContext<Self>,
        config: Option<&ComponentConfig>,
        manager: &mut ResourceManager,
    ) -> CuResult<()> {
        for slot in SERIAL_SLOTS {
            let rts = read_serial_rts_config(config, slot)?;
            let Some(serial_config) = read_serial_slot_config(config, slot)? else {
                continue; // Skip slots without explicit config
            };
            let nonblocking_key = format!("{}_nonblocking", slot_name(slot.id));
            let nonblocking = config
                .and_then(|cfg| cfg.get::<bool>(&nonblocking_key).transpose())
                .transpose()?
                .unwrap_or(false);
            if rts {
                #[cfg(all(unix, feature = "serial-rts"))]
                {
                    if nonblocking {
                        let (serial, pin) = LinuxNonblockingSerialPort::open_with_rts(
                            &serial_config,
                        )
                        .map_err(|err| CuError::new_with_cause("Open serial RTS resource", err))?;
                        manager.add_owned(bundle.key(slot.id), serial)?;
                        manager.add_owned(bundle.key(slot.rts_id), pin)?;
                    } else {
                        let (serial, pin) = LinuxSerialPort::open_with_rts(&serial_config)
                            .map_err(|err| {
                                CuError::new_with_cause("Open serial RTS resource", err)
                            })?;
                        manager.add_owned(bundle.key(slot.id), serial)?;
                        manager.add_owned(bundle.key(slot.rts_id), pin)?;
                    }
                    continue;
                }
                #[cfg(not(all(unix, feature = "serial-rts")))]
                return Err(CuError::from(
                    "Serial RTS requires Unix and the cu-linux-resources/serial-rts feature",
                ));
            }
            if nonblocking {
                #[cfg(unix)]
                {
                    let serial = LinuxNonblockingSerialPort::open_with_config(&serial_config)
                        .map_err(|err| {
                            CuError::new_with_cause("Open nonblocking serial resource", err)
                        })?;
                    manager.add_owned(bundle.key(slot.id), serial)?;
                    continue;
                }
                #[cfg(not(unix))]
                return Err(CuError::from("Nonblocking serial resources require Unix"));
            }
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

fn read_serial_rts_config(config: Option<&ComponentConfig>, slot: &SerialSlot) -> CuResult<bool> {
    let key = slot_name(slot.rts_id);
    let enabled = config
        .and_then(|cfg| cfg.get::<bool>(key).transpose())
        .transpose()?
        .unwrap_or(false);
    if enabled && get_string(config, slot.dev_key)?.is_none() {
        return Err(CuError::from(format!("{key} requires {}", slot.dev_key)));
    }
    Ok(enabled)
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
    use cu29::resource::{NamedResourceBundleDecl, resource_index_by_name};

    #[test]
    fn named_resource_bundle_decl_matches_public_linux_slot_names() {
        let declared = <LinuxResources as NamedResourceBundleDecl>::NAMES;
        assert_eq!(declared, LINUX_RESOURCE_SLOT_NAMES);

        for (idx, name) in declared.iter().enumerate() {
            assert_eq!(resource_index_by_name::<LinuxResources>(name), idx);
        }
    }

    fn rts_config(value: &str) -> ComponentConfig {
        let ron = format!(
            r#"(resources: [(id: "board", provider: "cu_linux_resources::LinuxResources",
                config: {{ "serial0_rts": {value} }})], tasks: [], cnx: [])"#,
        );
        cu29::config::CuConfig::deserialize_ron(&ron)
            .unwrap()
            .resources
            .remove(0)
            .config
            .unwrap()
    }

    #[test]
    fn serial_rts_is_opt_in_and_requires_a_device() {
        let slot = &SERIAL_SLOTS[0];
        assert!(!read_serial_rts_config(None, slot).unwrap());
        let mut config = rts_config("false");
        assert!(!read_serial_rts_config(Some(&config), slot).unwrap());
        config = rts_config("true");
        assert!(read_serial_rts_config(Some(&config), slot).is_err());
        config.set("serial0_dev", "/dev/ttyUSB0".to_string());
        assert!(read_serial_rts_config(Some(&config), slot).unwrap());
        assert!(!read_serial_rts_config(Some(&config), &SERIAL_SLOTS[1]).unwrap());
        config.set("serial0_rts", "invalid".to_string());
        assert!(read_serial_rts_config(Some(&config), slot).is_err());
    }

    #[cfg(not(feature = "serial-rts"))]
    #[test]
    fn serial_rts_request_without_feature_fails_before_opening_the_device() {
        let mut config = rts_config("true");
        config.set("serial0_dev", "/not/a/serial/device".to_string());
        let mut manager = ResourceManager::new(&[LINUX_RESOURCE_SLOT_NAMES.len()]);
        let error = LinuxResources::build(
            cu29::resource::BundleContext::new(cu29::resource::BundleIndex::new(0), "board"),
            Some(&config),
            &mut manager,
        )
        .unwrap_err();
        assert!(error.to_string().contains("serial-rts feature"));
    }

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
        cfg.set(GPIO0_DIRECTION_KEY, "input".to_string());
        assert!(read_gpio_slot_config(Some(&cfg), &GPIO_SLOTS[0]).is_err());
    }

    #[test]
    fn read_gpio_slot_config_requires_direction_when_pin_is_set() {
        let mut cfg = ComponentConfig::new();
        cfg.set(GPIO0_PIN_KEY, 23_u8);
        assert!(read_gpio_slot_config(Some(&cfg), &GPIO_SLOTS[0]).is_err());
    }

    #[test]
    fn read_gpio_slot_config_defaults_bias_and_preserves_level_for_output() {
        let mut cfg = ComponentConfig::new();
        cfg.set(GPIO0_PIN_KEY, 23_u8);
        cfg.set(GPIO0_DIRECTION_KEY, "output".to_string());

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
        cfg.set(GPIO0_PIN_KEY, 23_u8);
        cfg.set(GPIO0_DIRECTION_KEY, "output".to_string());
        cfg.set(GPIO0_INITIAL_LEVEL_KEY, "high".to_string());

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
        cfg.set(GPIO0_PIN_KEY, 23_u8);
        cfg.set(GPIO0_DIRECTION_KEY, "input".to_string());
        cfg.set(GPIO0_INITIAL_LEVEL_KEY, "high".to_string());
        assert!(read_gpio_slot_config(Some(&cfg), &GPIO_SLOTS[0]).is_err());
    }

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

    impl embedded_io::ErrorType for MockIo {
        type Error = core::convert::Infallible;
    }

    impl embedded_io::Read for MockIo {
        fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
            let n = core::cmp::min(buf.len(), self.rx_len);
            buf[..n].copy_from_slice(&self.rx[..n]);
            Ok(n)
        }
    }

    impl embedded_io::Write for MockIo {
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
    fn exclusive_forwards_embedded_io_traits() {
        let mut wrapped = Exclusive::new(MockIo::new(&[1, 2, 3]));

        let mut rx = [0_u8; 4];
        let read = embedded_io::Read::read(&mut wrapped, &mut rx).unwrap();
        assert_eq!(read, 3);
        assert_eq!(&rx[..3], &[1, 2, 3]);

        let written = embedded_io::Write::write(&mut wrapped, &[9, 8]).unwrap();
        assert_eq!(written, 2);
        embedded_io::Write::flush(&mut wrapped).unwrap();

        let inner = wrapped.into_inner();
        assert_eq!(&inner.tx[..inner.tx_len], &[9, 8]);
    }
}

#[cfg(all(test, unix))]
mod nonblocking_tests {
    use super::*;
    use cu_serial::SerialIo;
    #[test]
    fn native_uart_reports_backpressure_and_preserves_binary_bytes() {
        let (a, b) = serialport::TTYPort::pair().unwrap();
        let mut tx = LinuxNonblockingSerialPort::from_native(a).unwrap();
        let mut rx = LinuxNonblockingSerialPort::from_native(b).unwrap();
        let mut buf = [0; 16];
        assert_eq!(rx.try_read(&mut buf).unwrap(), 0);
        assert_eq!(tx.try_write(&[0, 0x7e, 0x7d, 255]).unwrap(), 4);
        let mut received = 0;
        for _ in 0..1000 {
            received += rx.try_read(&mut buf[received..4]).unwrap();
            if received == 4 {
                break;
            }
            std::thread::yield_now();
        }
        assert_eq!(&buf[..received], &[0, 0x7e, 0x7d, 255]);
        let data = [42; 65536];
        let written = tx.try_write(&data).unwrap();
        assert!(written > 0 && written < data.len());
        let mut blocked = false;
        for _ in 0..128 {
            if tx.try_write(&data).unwrap() == 0 {
                blocked = true;
                break;
            }
        }
        assert!(blocked);
    }
}
