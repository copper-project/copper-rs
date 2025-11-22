#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(not(feature = "std"))]
extern crate alloc;

#[cfg(not(feature = "std"))]
use alloc::{boxed::Box, format};
use core::any::{Any, TypeId};
use cu29::prelude::*;
use embedded_io::{Read, Write};
use spin::Mutex;

type SerialEntry = Box<dyn Any + Send>;

struct SerialSlot {
    type_id: TypeId,
    entry: SerialEntry,
}

pub const MAX_SERIAL_SLOTS: usize = 8;

struct SerialSlots {
    entries: [Option<SerialSlot>; MAX_SERIAL_SLOTS],
}

impl SerialSlots {
    const fn new() -> Self {
        Self {
            entries: [None, None, None, None, None, None, None, None],
        }
    }
}

static SERIAL_SLOTS: Mutex<SerialSlots> = Mutex::new(SerialSlots::new());

/// Register a serial port at the specified slot index.
///
/// # Arguments
///
/// * `slot` - The slot index (0 to MAX_SERIAL_SLOTS-1)
/// * `serial` - The serial port implementing embedded-io Read/Write traits
///
/// # Returns
///
/// Returns `CuResult<()>` indicating success or failure
pub fn register<S, E>(slot: usize, serial: S) -> CuResult<()>
where
    S: Write<Error = E> + Read<Error = E> + Send + 'static,
{
    if slot >= MAX_SERIAL_SLOTS {
        return Err(CuError::from(format!(
            "Serial registry slot {slot} out of range (max {MAX_SERIAL_SLOTS})"
        )));
    }
    let mut slots = SERIAL_SLOTS.lock();
    slots.entries[slot] = Some(SerialSlot {
        type_id: TypeId::of::<S>(),
        entry: Box::new(serial),
    });
    Ok(())
}

/// Take (consume) a serial port from the specified slot index.
///
/// # Arguments
///
/// * `slot` - The slot index (0 to MAX_SERIAL_SLOTS-1)
///
/// # Returns
///
/// Returns `Some(S)` if the slot contains a serial port of type S, `None` otherwise.
/// Note: This consumes the serial port from the slot, leaving it empty.
pub fn take<S, E>(slot: usize) -> Option<S>
where
    S: Write<Error = E> + Read<Error = E> + Send + 'static,
{
    if slot >= MAX_SERIAL_SLOTS {
        return None;
    }
    let mut slots = SERIAL_SLOTS.lock();
    let record = slots.entries[slot].take()?;
    if record.type_id != TypeId::of::<S>() {
        slots.entries[slot] = Some(record);
        return None;
    }
    record.entry.downcast::<S>().map(|boxed| *boxed).ok()
}

/// Get the slot status for debugging purposes.
///
/// # Returns
///
/// Returns an array indicating which slots are occupied.
pub fn slot_status() -> [bool; MAX_SERIAL_SLOTS] {
    let slots = SERIAL_SLOTS.lock();
    let mut status = [false; MAX_SERIAL_SLOTS];
    for (i, slot) in slots.entries.iter().enumerate() {
        status[i] = slot.is_some();
    }
    status
}

/// Clear all slots. Useful for testing.
#[cfg(test)]
pub fn clear_all() {
    let mut slots = SERIAL_SLOTS.lock();
    for slot in &mut slots.entries {
        *slot = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_io::{ErrorKind, ErrorType};

    #[derive(Debug)]
    struct TestError;

    impl embedded_io::Error for TestError {
        fn kind(&self) -> ErrorKind {
            ErrorKind::Other
        }
    }

    struct MockSerial;

    impl ErrorType for MockSerial {
        type Error = TestError;
    }

    impl embedded_io::Write for MockSerial {
        fn write(&mut self, _buf: &[u8]) -> Result<usize, Self::Error> {
            Ok(0)
        }

        fn flush(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }
    }

    impl embedded_io::Read for MockSerial {
        fn read(&mut self, _buf: &mut [u8]) -> Result<usize, Self::Error> {
            Ok(0)
        }
    }

    #[test]
    fn test_register_and_take() {
        clear_all();

        let serial = MockSerial;
        assert!(register(0, serial).is_ok());

        let status = slot_status();
        assert!(status[0]);
        assert!(!status[1]);

        let recovered: Option<MockSerial> = take(0);
        assert!(recovered.is_some());

        let status = slot_status();
        assert!(!status[0]);
    }

    #[test]
    fn test_slot_bounds() {
        clear_all();

        let serial = MockSerial;
        assert!(register(MAX_SERIAL_SLOTS, serial).is_err());

        let recovered: Option<MockSerial> = take(MAX_SERIAL_SLOTS);
        assert!(recovered.is_none());
    }
}
