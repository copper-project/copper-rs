use crate::SerialSlotConfig;
use embedded_hal::digital::{ErrorType, OutputPin};
use std::os::fd::{AsRawFd, BorrowedFd, OwnedFd};

/// Owned RTS# output for a USB-to-TTL UART with active-low RTS signaling.
///
/// `set_low` asserts RTS; `set_high` deasserts it. The pin starts high.
/// Output changes use modem-control ioctls.
/// Its duplicated descriptor keeps the device open independently of the data handle.
/// Requires the `serial-rts` feature, Unix, and compatible TTL logic levels.
#[derive(Debug)]
pub struct LinuxSerialRtsPin(OwnedFd);

#[derive(Debug)]
pub struct SerialRtsError(std::io::Error);

impl core::fmt::Display for SerialRtsError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        self.0.fmt(f)
    }
}

impl std::error::Error for SerialRtsError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        Some(&self.0)
    }
}

impl embedded_hal::digital::Error for SerialRtsError {
    fn kind(&self) -> embedded_hal::digital::ErrorKind {
        embedded_hal::digital::ErrorKind::Other
    }
}

impl ErrorType for LinuxSerialRtsPin {
    type Error = SerialRtsError;
}

impl OutputPin for LinuxSerialRtsPin {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.set_asserted(true).map_err(SerialRtsError)
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.set_asserted(false).map_err(SerialRtsError)
    }
}

impl LinuxSerialRtsPin {
    fn set_asserted(&mut self, asserted: bool) -> std::io::Result<()> {
        let mask: libc::c_int = libc::TIOCM_RTS;
        let request = if asserted {
            libc::TIOCMBIS
        } else {
            libc::TIOCMBIC
        };
        // SAFETY: the owned fd remains live, and these ioctls read a c_int mask.
        // Changing only RTS avoids overwriting concurrent changes to other lines.
        if unsafe { libc::ioctl(self.0.as_raw_fd(), request, &mask) } < 0 {
            return Err(std::io::Error::last_os_error());
        }
        Ok(())
    }
}

pub(crate) fn open(
    config: &SerialSlotConfig,
) -> std::io::Result<(serialport::TTYPort, LinuxSerialRtsPin)> {
    let port = serialport::new(&config.dev, config.baudrate)
        .parity(config.parity)
        .stop_bits(config.stop_bits)
        .timeout(std::time::Duration::from_millis(config.timeout_ms))
        .flow_control(serialport::FlowControl::None)
        .open_native()?;
    // SAFETY: port owns the descriptor throughout this borrow. Duplication creates
    // a separate owned fd without sharing a Rust UART object or adding an I/O lock.
    let fd = unsafe { BorrowedFd::borrow_raw(port.as_raw_fd()) }.try_clone_to_owned()?;
    let mut pin = LinuxSerialRtsPin(fd);
    // Probe support and leave SET inactive before the radio's startup sequence.
    pin.set_asserted(false)?;
    Ok((port, pin))
}

#[cfg(all(test, target_os = "linux"))]
mod tests {
    use super::*;
    use serialport::SerialPort;

    #[test]
    fn unsupported_modem_control_fails_open_instead_of_silently_exporting_a_pin() {
        let (_master, slave) = serialport::TTYPort::pair().unwrap();
        let config = SerialSlotConfig {
            dev: slave.name().unwrap(),
            baudrate: 9600,
            parity: serialport::Parity::None,
            stop_bits: serialport::StopBits::One,
            timeout_ms: 50,
        };
        drop(slave);
        let err = open(&config).unwrap_err();
        assert_eq!(err.raw_os_error(), Some(libc::ENOTTY));
    }
}
