use cu29::bundle_resources;
use cu29::prelude::*;
use cu29::resource::{ResourceBundle, ResourceManager};
use embedded_hal::i2c::Error;
use std::string::String;

use i2cdriver::NormalMode;

pub const I2CDRIVER_VID: u16 = 1027;
pub const I2CDRIVER_PID: u16 = 24597;

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

pub type I2cDriver = Exclusive<i2cdriver::I2CDriver<NormalMode>>;

pub struct I2cDriverResources;

bundle_resources!(
    I2cDriverResources:
    I2C,
);

impl ResourceBundle for I2cDriverResources {
    fn build(
        bundle: cu29::resource::BundleContext<Self>,
        config: Option<&ComponentConfig>,
        manager: &mut ResourceManager,
    ) -> CuResult<()> {
        let dev_name = get_string(config, "dev")?;

        // if no device path is specified, scan by Vendor/Product ID
        let dev_name = match dev_name {
            Some(x) => x,
            None => match find_serial_device(I2CDRIVER_VID, I2CDRIVER_PID)? {
                Some(serialport::SerialPortInfo {
                    port_name,
                    port_type: serialport::SerialPortType::UsbPort(port_info),
                }) => {
                    info!(
                        "Using serial device: {} (serial: {:?})",
                        &port_name,
                        port_info.serial_number.clone()
                    );
                    port_name
                }
                _ => {
                    return Err(CuError::from(format!(
                        "Could not find USB i2cdriver device with VID {:x} and PID {:x}",
                        I2CDRIVER_VID, I2CDRIVER_PID
                    )));
                }
            },
        };

        let i2c_driver = i2cdriver::I2CDriver::open(&dev_name).map_err(|e| match e {
            i2cdriver::Error::Open { source, path } => {
                return CuError::new_with_cause(
                    &format!("Failed to open I2C driver serial connection to `{}`", path),
                    source,
                );
            }
            other => CuError::new_with_cause(
                &format!("Failure I2C driver serial connection: {:?}", other.kind()),
                other,
            ),
        })?;
        let i2c_exclusive = Exclusive(i2c_driver);

        manager.add_owned(bundle.key(I2cDriverResourcesId::I2C), i2c_exclusive)?;

        Ok(())
    }
}

fn get_string(config: Option<&ComponentConfig>, key: &str) -> CuResult<Option<String>> {
    match config {
        Some(cfg) => Ok(cfg.get::<String>(key)?.filter(|value| !value.is_empty())),
        None => Ok(None),
    }
}

/// Finds the first serial port matching the given VID and PID.
fn find_serial_device(vid: u16, pid: u16) -> CuResult<Option<serialport::SerialPortInfo>> {
    let available_ports = serialport::available_ports().map_err(|cause| {
        CuError::new_with_cause("Failed to enumerate available serial ports", cause)
    })?;

    let device_port_info = available_ports.into_iter()
        .find(|port| matches!(&port.port_type, serialport::SerialPortType::UsbPort(usb_port) if usb_port.pid == pid && usb_port.vid == vid));

    Ok(device_port_info)
}

#[cfg(test)]
mod tests {

    #[cfg(test)]
    mod tests {
        use embedded_hal::i2c;

        use crate::I2cDriver;

        fn assert_i2c_error_type<P: i2c::ErrorType>() {}
        fn assert_is_i2c<P: i2c::I2c>() {}

        #[test]
        fn test_is_type_i2c() {
            assert_i2c_error_type::<I2cDriver>();
            assert_is_i2c::<I2cDriver>();
        }
    }
}
