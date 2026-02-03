#![no_std]

#[cfg(all(feature = "embedded-io-06", feature = "embedded-io-07"))]
compile_error!("Enable only one of `embedded-io-06` or `embedded-io-07` for cu-serial-util.");

#[cfg(not(any(feature = "embedded-io-06", feature = "embedded-io-07")))]
compile_error!("Enable one of `embedded-io-06` or `embedded-io-07` for cu-serial-util.");

#[cfg(feature = "embedded-io-06")]
use embedded_io_06 as embedded_io;
#[cfg(feature = "embedded-io-07")]
use embedded_io_07 as embedded_io;

use embedded_io::{ErrorType, Read, Write};
use spin::Mutex;

pub struct LockedSerial<T>(pub Mutex<T>);

impl<T> LockedSerial<T> {
    pub fn new(inner: T) -> Self {
        Self(Mutex::new(inner))
    }
}

impl<T: ErrorType> ErrorType for LockedSerial<T> {
    type Error = T::Error;
}

impl<T: Read> Read for LockedSerial<T> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        let mut guard = self.0.lock();
        guard.read(buf)
    }
}

impl<T: Write> Write for LockedSerial<T> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        let mut guard = self.0.lock();
        guard.write(buf)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        let mut guard = self.0.lock();
        guard.flush()
    }
}
