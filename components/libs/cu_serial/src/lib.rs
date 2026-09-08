#![cfg_attr(not(feature = "std"), no_std)]

//! Bounded, nonblocking byte I/O. UART writes can accept a prefix; callers own
//! the remaining bytes. Zero means no progress, never EOF. Disconnection is an error.
use embedded_io::{ErrorType, Read, ReadReady, Write, WriteReady};

pub trait SerialIo: ErrorType {
    /// Read once without waiting, locking, allocating or retrying.
    fn try_read(&mut self, bytes: &mut [u8]) -> Result<usize, Self::Error>;
    /// Write once without waiting, locking, allocating or retrying.
    /// `Ok(n)` accepts exactly the first `n <= bytes.len()` bytes; `Err` accepts none.
    fn try_write(&mut self, bytes: &[u8]) -> Result<usize, Self::Error>;
}

/// Adapt a HAL implementing the embedded-io readiness guarantees.
#[derive(Debug)]
pub struct ReadySerial<S>(pub S);

impl<S: ErrorType> ErrorType for ReadySerial<S> {
    type Error = S::Error;
}

impl<S: Read + ReadReady + Write + WriteReady> SerialIo for ReadySerial<S> {
    fn try_read(&mut self, bytes: &mut [u8]) -> Result<usize, Self::Error> {
        if bytes.is_empty() || !self.0.read_ready()? {
            return Ok(0);
        }
        self.0.read(bytes)
    }
    fn try_write(&mut self, bytes: &[u8]) -> Result<usize, Self::Error> {
        if bytes.is_empty() || !self.0.write_ready()? {
            return Ok(0);
        }
        self.0.write(bytes)
    }
}
