use std::io;

pub struct Spidev {
    // nothing
}

pub fn read_adc(_spi: &mut Spidev) -> io::Result<u16> {
    Ok(0)
}
