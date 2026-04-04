use std::sync::{Arc, Mutex};

use cu29::{CuError, CuResult};

use embedded_hal_nb::{
    nb::block,
    serial::{Read as _, Write as _},
};
use linux_embedded_hal::SerialError;
use serialport::{SerialPortInfo, SerialPortType};

use thiserror::Error;

#[derive(Debug, Error)]
pub enum Error {
    #[error("IO error")]
    Io(#[from] std::io::Error),
    #[error("Serial error")]
    Linux(#[from] SerialError),
}

impl embedded_io::Error for Error {
    fn kind(&self) -> embedded_io::ErrorKind {
        embedded_io::ErrorKind::Other
    }
}

#[derive(Clone)]
pub struct Tx(Arc<Mutex<linux_embedded_hal::Serial>>);

#[derive(Clone)]
pub struct Rx(Arc<Mutex<linux_embedded_hal::Serial>>);

pub struct Serial(Arc<Mutex<linux_embedded_hal::Serial>>);

impl Serial {
    pub fn new(port: linux_embedded_hal::Serial) -> Self {
        Serial(Arc::new(Mutex::new(port)))
    }

    pub fn split(self) -> (Tx, Rx) {
        (Tx(Arc::clone(&self.0)), Rx(Arc::clone(&self.0)))
    }
}

impl embedded_io::ErrorType for Rx {
    type Error = Error;
}
impl embedded_io::Read for Rx {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        // we should return 0 on buf.len() = 0
        if buf.is_empty() {
            return Ok(0);
        }

        let mut written = 0_usize;
        for byte in buf.iter_mut() {
            match block!((*self.0).lock().unwrap().read()) {
                Ok(read_byte) => {
                    *byte = read_byte;
                    written = written.wrapping_add(1);
                }
                Err(e) => return Err(Error::Linux(e)),
            }
        }
        Ok(written)
    }
}

impl embedded_io::ErrorType for Tx {
    type Error = Error;
}
impl embedded_io::Write for Tx {
    fn write(&mut self, bytes: &[u8]) -> Result<usize, Self::Error> {
        let mut written = 0_usize;
        for byte in bytes {
            match block!((*self.0).lock().unwrap().write(*byte)) {
                Ok(_) => {
                    written = written.wrapping_add(1);
                }
                // todo: bad write
                Err(e) => return Err(Error::Linux(e)),
            }
        }
        Ok(written)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        match block!((*self.0).lock().unwrap().flush()) {
            Ok(_) => Ok(()),
            Err(e) => Err(Error::Linux(e)),
        }
    }
}

/// Finds the first serial port matching the given VID and PID.
pub fn find_serial_device(vid: u16, pid: u16) -> CuResult<Option<SerialPortInfo>> {
    let available_ports = serialport::available_ports().map_err(|cause| {
        CuError::new_with_cause("Failed to enumerate available serial ports", cause)
    })?;

    let device_port_info = available_ports
        .into_iter()
        .find(|port| matches!(&port.port_type, SerialPortType::UsbPort(usb_port) if usb_port.pid == pid && usb_port.vid == vid));

    Ok(device_port_info)
}
