use embedded_hal::{
    digital,
    spi::{self, Operation},
};
use spidriver_hal::hal::Comms;

/// `SPI` implements some of the SPI-related traits from `embedded-hal` in terms
/// of an SPIDriver device.
pub struct SPI<SD>(SD);

impl<SD> Clone for SPI<SD>
where
    SD: Clone,
{
    fn clone(&self) -> Self {
        Self(self.0.clone())
    }
}

impl<SD> SPI<SD>
where
    SD: Comms,
{
    pub(crate) fn new(sd: SD) -> Self {
        Self(sd)
    }
}

impl<SD, E> spi::ErrorType for SPI<SD>
where
    SD: Comms<Error = E>,
    E: spi::Error,
{
    type Error = E;
}

impl<SD, E> spi::SpiDevice for SPI<SD>
where
    SD: Comms<Error = E>,
    E: spi::Error,
{
    /// Implements blocking SPI `Transfer` by passing the given data to the
    /// `SPIDriver` in chunks of up to 64 bytes each.
    ///
    /// Because of the chunking behaviour, larger messages may have inconsistent
    /// timing at the chunk boundaries, which may affect devices with particularly
    /// sensitive clock timing constraints.
    fn transaction(
        &mut self,
        operations: &mut [spi::Operation<'_, u8>],
    ) -> Result<(), Self::Error> {
        for op in operations {
            match op {
                Operation::TransferInPlace(words) => self.0.transfer_in_place(words).map(drop)?,
                _ => {
                    // do nothing for now
                }
            }
        }
        Ok(())
    }
}

impl<SD, E> spi::SpiBus for SPI<SD>
where
    SD: Comms<Error = E>,
    E: spi::Error,
{
    fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        // Send dummy bytes while reading
        words.fill(0);
        self.0.transfer_in_place(words).map(|_| ())
    }

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        self.0.write(words)
    }

    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        // SPI requires equal-length buffers
        let len = read.len().min(write.len());

        // Copy write data into read buffer so we can perform in-place transfer
        read[..len].copy_from_slice(&write[..len]);

        self.0.transfer_in_place(&mut read[..len]).map(|_| ())
    }

    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        self.0.transfer_in_place(words).map(|_| ())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        // Backend appears synchronous, so nothing to flush
        Ok(())
    }
}
/// `CS` implements some of the digital IO traits from `embedded-hal` in
/// terms of an SPIDriver device's Chip Select pin.
pub struct CS<SD: Comms>(SD);

impl<SD> Clone for CS<SD>
where
    SD: Comms + Clone,
{
    fn clone(&self) -> Self {
        Self(self.0.clone())
    }
}

impl<SD> CS<SD>
where
    SD: Comms,
{
    pub(crate) fn new(sd: SD) -> Self {
        Self(sd)
    }
}

impl<SD, E> digital::ErrorType for CS<SD>
where
    SD: Comms<Error = E>,
    E: digital::Error,
{
    type Error = E;
}

impl<SD, E> digital::OutputPin for CS<SD>
where
    SD: Comms<Error = E>,
    E: digital::Error,
{
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.0.set_cs(false)
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.0.set_cs(true)
    }
}

/// `PinA` implements some of the digital IO traits from `embedded-hal` in
/// terms of an `SPIDriver` device's auxiliary output pin A.
pub struct PinA<SD: Comms>(SD);

impl<SD> Clone for PinA<SD>
where
    SD: Comms + Clone,
{
    fn clone(&self) -> Self {
        Self(self.0.clone())
    }
}

impl<SD> PinA<SD>
where
    SD: Comms,
{
    pub(crate) fn new(sd: SD) -> Self {
        Self(sd)
    }
}

impl<SD, E> digital::ErrorType for PinA<SD>
where
    SD: Comms<Error = E>,
    E: digital::Error,
{
    type Error = E;
}

impl<SD, E> digital::OutputPin for PinA<SD>
where
    SD: Comms<Error = E>,
    E: digital::Error,
{
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.0.set_a(false)
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.0.set_a(true)
    }
}

/// `PinB` implements some of the digital IO traits from `embedded-hal` in
/// terms of an `SPIDriver` device's auxiliary output pin B.
pub struct PinB<SD: Comms>(SD);

impl<SD> Clone for PinB<SD>
where
    SD: Comms + Clone,
{
    fn clone(&self) -> Self {
        Self(self.0.clone())
    }
}

impl<SD> PinB<SD>
where
    SD: Comms,
{
    pub(crate) fn new(sd: SD) -> Self {
        Self(sd)
    }
}

impl<SD, E> digital::ErrorType for PinB<SD>
where
    SD: Comms<Error = E>,
    E: digital::Error,
{
    type Error = E;
}

impl<SD, E> digital::OutputPin for PinB<SD>
where
    SD: Comms<Error = E>,
    E: digital::Error,
{
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.0.set_b(false)
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.0.set_b(true)
    }
}

#[cfg(test)]
mod tests {
    use embedded_hal::{digital, spi};

    use crate::SpiDriverShared;

    use super::*;

    fn assert_digital_error_type<P: digital::ErrorType>() {}
    fn assert_is_output_pin<P: digital::OutputPin>() {}

    fn assert_spi_error_type<P: spi::ErrorType>() {}
    fn assert_is_spi<P: spi::SpiBus>() {}

    #[test]
    fn test_is_type_digital_pin() {
        assert_digital_error_type::<CS<SpiDriverShared>>();
        assert_is_output_pin::<CS<SpiDriverShared>>();

        assert_digital_error_type::<PinA<SpiDriverShared>>();
        assert_is_output_pin::<PinA<SpiDriverShared>>();

        assert_digital_error_type::<PinB<SpiDriverShared>>();
        assert_is_output_pin::<PinB<SpiDriverShared>>();
    }

    #[test]
    fn test_is_type_spi() {
        assert_spi_error_type::<SPI<SpiDriverShared>>();
        assert_is_spi::<SPI<SpiDriverShared>>();
    }
}
