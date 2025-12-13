#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(not(feature = "std"))]
extern crate alloc;

#[cfg(not(feature = "std"))]
use alloc::{boxed::Box, format};
use core::any::{Any, TypeId};
use cu29::prelude::*;
use embedded_hal as eh1;
use embedded_io::{Read, Write};
use spin::Mutex;

type Entry = Box<dyn Any + Send>;

struct TypedSlot {
    type_id: TypeId,
    entry: Entry,
}

macro_rules! define_registry {
    ($name:ident) => {
        struct $name {
            entries: [Option<TypedSlot>; 8],
        }

        impl $name {
            const fn new() -> Self {
                Self {
                    entries: [None, None, None, None, None, None, None, None],
                }
            }
        }
    };
}

define_registry!(SerialSlots);
define_registry!(SpiSlots);
define_registry!(CsSlots);
define_registry!(DelaySlots);

static SERIAL_SLOTS: Mutex<SerialSlots> = Mutex::new(SerialSlots::new());
static SPI_SLOTS: Mutex<SpiSlots> = Mutex::new(SpiSlots::new());
static CS_SLOTS: Mutex<CsSlots> = Mutex::new(CsSlots::new());
static DELAY_SLOTS: Mutex<DelaySlots> = Mutex::new(DelaySlots::new());

pub const MAX_SERIAL_SLOTS: usize = 8;
pub const MAX_SPI_SLOTS: usize = 8;
pub const MAX_CS_SLOTS: usize = 8;
pub const MAX_DELAY_SLOTS: usize = 8;

fn register_inner<T>(
    slots: &mut [Option<TypedSlot>],
    max: usize,
    slot: usize,
    value: T,
) -> CuResult<()>
where
    T: Any + Send + 'static,
{
    if slot >= max {
        return Err(CuError::from(format!(
            "Registry slot {slot} out of range (max {max})"
        )));
    }
    slots[slot] = Some(TypedSlot {
        type_id: TypeId::of::<T>(),
        entry: Box::new(value),
    });
    Ok(())
}

fn take_inner<T>(slots: &mut [Option<TypedSlot>], max: usize, slot: usize) -> Option<T>
where
    T: Any + Send + 'static,
{
    if slot >= max {
        return None;
    }
    let record = slots[slot].take()?;
    if record.type_id != TypeId::of::<T>() {
        slots[slot] = Some(record);
        return None;
    }
    record.entry.downcast::<T>().map(|boxed| *boxed).ok()
}

fn status_inner(slots: &[Option<TypedSlot>], max: usize) -> [bool; 8] {
    let mut status = [false; 8];
    let slice = &slots[..max.min(8)];
    for (i, slot) in slice.iter().enumerate() {
        status[i] = slot.is_some();
    }
    status
}

/// Register a serial port at the specified slot index.
///
/// Returns `CuResult<()>` indicating success or failure
pub fn register<S, E>(slot: usize, serial: S) -> CuResult<()>
where
    S: Write<Error = E> + Read<Error = E> + Send + 'static,
{
    let mut slots = SERIAL_SLOTS.lock();
    register_inner(&mut slots.entries, MAX_SERIAL_SLOTS, slot, serial)
}

/// Take (consume) a serial port from the specified slot index.
///
/// Returns `Some(S)` if the slot contains a serial port of type S, `None` otherwise.
/// Note: This consumes the serial port from the slot, leaving it empty.
pub fn take<S, E>(slot: usize) -> Option<S>
where
    S: Write<Error = E> + Read<Error = E> + Send + 'static,
{
    let mut slots = SERIAL_SLOTS.lock();
    take_inner(&mut slots.entries, MAX_SERIAL_SLOTS, slot)
}

/// Get the serial slot status for debugging purposes.
pub fn slot_status() -> [bool; 8] {
    let slots = SERIAL_SLOTS.lock();
    status_inner(&slots.entries, MAX_SERIAL_SLOTS)
}

/// Register an SPI bus that implements embedded-hal 1.0 `SpiBus`.
pub fn register_spi<S>(slot: usize, spi: S) -> CuResult<()>
where
    S: eh1::spi::SpiBus<u8> + Send + 'static,
    S::Error: Send + 'static,
{
    let mut slots = SPI_SLOTS.lock();
    register_inner(&mut slots.entries, MAX_SPI_SLOTS, slot, spi)
}

/// Consume an SPI bus from the specified slot index.
pub fn take_spi<S>(slot: usize) -> Option<S>
where
    S: eh1::spi::SpiBus<u8> + Send + 'static,
    S::Error: Send + 'static,
{
    let mut slots = SPI_SLOTS.lock();
    take_inner(&mut slots.entries, MAX_SPI_SLOTS, slot)
}

/// Register a chip-select output pin implementing embedded-hal 1.0 `OutputPin`.
pub fn register_cs<P>(slot: usize, pin: P) -> CuResult<()>
where
    P: eh1::digital::OutputPin + Send + 'static,
    P::Error: Send + 'static,
{
    let mut slots = CS_SLOTS.lock();
    register_inner(&mut slots.entries, MAX_CS_SLOTS, slot, pin)
}

/// Consume a chip-select output pin from the specified slot index.
pub fn take_cs<P>(slot: usize) -> Option<P>
where
    P: eh1::digital::OutputPin + Send + 'static,
    P::Error: Send + 'static,
{
    let mut slots = CS_SLOTS.lock();
    take_inner(&mut slots.entries, MAX_CS_SLOTS, slot)
}

/// Register a delay provider implementing embedded-hal 1.0 `DelayNs`.
pub fn register_delay<D>(slot: usize, delay: D) -> CuResult<()>
where
    D: eh1::delay::DelayNs + Send + 'static,
{
    let mut slots = DELAY_SLOTS.lock();
    register_inner(&mut slots.entries, MAX_DELAY_SLOTS, slot, delay)
}

/// Consume a delay provider from the specified slot index.
pub fn take_delay<D>(slot: usize) -> Option<D>
where
    D: eh1::delay::DelayNs + Send + 'static,
{
    let mut slots = DELAY_SLOTS.lock();
    take_inner(&mut slots.entries, MAX_DELAY_SLOTS, slot)
}

/// Get the SPI slot status for debugging purposes.
pub fn spi_slot_status() -> [bool; 8] {
    let slots = SPI_SLOTS.lock();
    status_inner(&slots.entries, MAX_SPI_SLOTS)
}

/// Get the chip-select slot status for debugging purposes.
pub fn cs_slot_status() -> [bool; 8] {
    let slots = CS_SLOTS.lock();
    status_inner(&slots.entries, MAX_CS_SLOTS)
}

/// Get the delay slot status for debugging purposes.
pub fn delay_slot_status() -> [bool; 8] {
    let slots = DELAY_SLOTS.lock();
    status_inner(&slots.entries, MAX_DELAY_SLOTS)
}

/// Clear all slots. Useful for testing.
#[cfg(test)]
pub fn clear_all() {
    {
        let mut slots = SERIAL_SLOTS.lock();
        for slot in &mut slots.entries {
            *slot = None;
        }
    }
    {
        let mut slots = SPI_SLOTS.lock();
        for slot in &mut slots.entries {
            *slot = None;
        }
    }
    {
        let mut slots = CS_SLOTS.lock();
        for slot in &mut slots.entries {
            *slot = None;
        }
    }
    {
        let mut slots = DELAY_SLOTS.lock();
        for slot in &mut slots.entries {
            *slot = None;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_io::{ErrorKind, ErrorType};

    #[derive(Debug, Clone, Copy)]
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

    #[derive(Debug, Clone, Copy)]
    struct MockHalError;

    impl eh1::spi::Error for MockHalError {
        fn kind(&self) -> eh1::spi::ErrorKind {
            eh1::spi::ErrorKind::Other
        }
    }

    impl eh1::digital::Error for MockHalError {
        fn kind(&self) -> eh1::digital::ErrorKind {
            eh1::digital::ErrorKind::Other
        }
    }

    struct MockSpi;

    impl eh1::spi::ErrorType for MockSpi {
        type Error = MockHalError;
    }

    impl eh1::spi::SpiBus<u8> for MockSpi {
        fn read(&mut self, _words: &mut [u8]) -> Result<(), Self::Error> {
            Ok(())
        }

        fn write(&mut self, _words: &[u8]) -> Result<(), Self::Error> {
            Ok(())
        }

        fn transfer(&mut self, _read: &mut [u8], _write: &[u8]) -> Result<(), Self::Error> {
            Ok(())
        }

        fn transfer_in_place(&mut self, _words: &mut [u8]) -> Result<(), Self::Error> {
            Ok(())
        }

        fn flush(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }
    }

    struct OtherSpi;

    impl eh1::spi::ErrorType for OtherSpi {
        type Error = MockHalError;
    }

    impl eh1::spi::SpiBus<u8> for OtherSpi {
        fn read(&mut self, _words: &mut [u8]) -> Result<(), Self::Error> {
            Ok(())
        }

        fn write(&mut self, _words: &[u8]) -> Result<(), Self::Error> {
            Ok(())
        }

        fn transfer(&mut self, _read: &mut [u8], _write: &[u8]) -> Result<(), Self::Error> {
            Ok(())
        }

        fn transfer_in_place(&mut self, _words: &mut [u8]) -> Result<(), Self::Error> {
            Ok(())
        }

        fn flush(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }
    }

    struct MockCs;

    impl eh1::digital::ErrorType for MockCs {
        type Error = MockHalError;
    }

    impl eh1::digital::OutputPin for MockCs {
        fn set_low(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }

        fn set_high(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }
    }

    struct OtherCs;

    impl eh1::digital::ErrorType for OtherCs {
        type Error = MockHalError;
    }

    impl eh1::digital::OutputPin for OtherCs {
        fn set_low(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }

        fn set_high(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }
    }

    struct MockDelay;

    impl eh1::delay::DelayNs for MockDelay {
        fn delay_ns(&mut self, _ns: u32) {}
    }

    struct OtherDelay;

    impl eh1::delay::DelayNs for OtherDelay {
        fn delay_ns(&mut self, _ns: u32) {}
    }

    #[test]
    fn test_register_and_take_serial() {
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
    fn test_register_and_take_spi() {
        clear_all();
        assert!(register_spi(0, MockSpi).is_ok());
        assert!(spi_slot_status()[0]);
        assert!(take_spi::<MockSpi>(0).is_some());
        assert!(!spi_slot_status()[0]);

        // Type mismatch should preserve the slot
        assert!(register_spi(1, MockSpi).is_ok());
        assert!(take_spi::<OtherSpi>(1).is_none());
        assert!(spi_slot_status()[1]);
    }

    #[test]
    fn test_register_and_take_cs() {
        clear_all();
        assert!(register_cs(0, MockCs).is_ok());
        assert!(cs_slot_status()[0]);
        assert!(take_cs::<MockCs>(0).is_some());
        assert!(!cs_slot_status()[0]);

        assert!(register_cs(1, MockCs).is_ok());
        assert!(take_cs::<OtherCs>(1).is_none());
        assert!(cs_slot_status()[1]);
    }

    #[test]
    fn test_register_and_take_delay() {
        clear_all();
        assert!(register_delay(0, MockDelay).is_ok());
        assert!(delay_slot_status()[0]);
        assert!(take_delay::<MockDelay>(0).is_some());
        assert!(!delay_slot_status()[0]);

        assert!(register_delay(1, MockDelay).is_ok());
        assert!(take_delay::<OtherDelay>(1).is_none());
        assert!(delay_slot_status()[1]);
    }

    #[test]
    fn test_slot_bounds() {
        clear_all();
        let serial = MockSerial;
        assert!(register(MAX_SERIAL_SLOTS, serial).is_err());

        let recovered: Option<MockSerial> = take(MAX_SERIAL_SLOTS);
        assert!(recovered.is_none());

        assert!(register_spi(MAX_SPI_SLOTS, MockSpi).is_err());
        assert!(take_spi::<MockSpi>(MAX_SPI_SLOTS).is_none());

        assert!(register_cs(MAX_CS_SLOTS, MockCs).is_err());
        assert!(take_cs::<MockCs>(MAX_CS_SLOTS).is_none());

        assert!(register_delay(MAX_DELAY_SLOTS, MockDelay).is_err());
        assert!(take_delay::<MockDelay>(MAX_DELAY_SLOTS).is_none());
    }
}
