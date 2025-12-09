use defmt::Format;

#[derive(Debug, Clone, Copy, Format)]
pub struct SdCid {
    raw: [u32; 4],
}

impl SdCid {
    pub fn new(raw: [u32; 4]) -> Self {
        Self { raw }
    }
}

#[derive(Debug, Clone, Copy, Format)]
pub struct SdCsd {
    raw: [u32; 4],
}

impl SdCsd {
    pub fn new(raw: [u32; 4]) -> Self {
        Self { raw }
    }
}

#[derive(Debug, Clone, Copy, Format)]
pub struct SdCic {
    pub supports_1p2v: bool,
    pub supports_pcie: bool,
}

#[derive(Debug, Clone, Copy, Format)]
pub struct SdOcr {
    pub ocr: u32,
}

impl SdOcr {
    pub fn is_busy(&self) -> bool {
        // Busy until the power-up status bit (bit 31) is asserted.
        self.ocr & (1 << 31) == 0
    }

    pub fn get_voltage_window(&self) -> u32 {
        // Voltage window bits 23:0 as defined by the SD OCR register layout.
        self.ocr & 0x00FF_FFFF
    }
}
