//! RFC 8681 FEC Scheme-Specific Information and payload identifiers.

use core::fmt;

use crate::{DensityThreshold, EncodingSymbolId};

/// An error in RFC 8681 wire metadata.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum WireError {
    /// A byte slice does not have the required fixed wire length.
    InvalidLength {
        /// Required byte count.
        expected: usize,
        /// Supplied byte count.
        actual: usize,
    },
    /// A repair window must contain at least one source symbol.
    EmptyRepairWindow,
    /// The 12-bit NSS field cannot represent the requested window size.
    RepairWindowTooLarge,
}

impl fmt::Display for WireError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidLength { expected, actual } => {
                write!(formatter, "expected {expected} bytes, received {actual}")
            }
            Self::EmptyRepairWindow => formatter.write_str("repair window must not be empty"),
            Self::RepairWindowTooLarge => {
                formatter.write_str("repair window exceeds the 12-bit NSS field")
            }
        }
    }
}

impl core::error::Error for WireError {}

/// The three-octet RFC 8681 FEC Scheme-Specific Information value.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct FecSchemeSpecificInfo {
    symbol_size: u16,
    window_size_ratio: u8,
}

impl TryFrom<&[u8]> for FecSchemeSpecificInfo {
    type Error = WireError;

    fn try_from(bytes: &[u8]) -> Result<Self, Self::Error> {
        let bytes: [u8; 3] = bytes.try_into().map_err(|_| WireError::InvalidLength {
            expected: 3,
            actual: bytes.len(),
        })?;
        Ok(Self::from_bytes(bytes))
    }
}

impl From<FecSchemeSpecificInfo> for [u8; 3] {
    fn from(value: FecSchemeSpecificInfo) -> Self {
        value.to_bytes()
    }
}

impl From<[u8; 3]> for FecSchemeSpecificInfo {
    fn from(bytes: [u8; 3]) -> Self {
        Self::from_bytes(bytes)
    }
}

impl FecSchemeSpecificInfo {
    /// Creates scheme-specific information from the symbol size and WSR value.
    pub const fn new(symbol_size: u16, window_size_ratio: u8) -> Self {
        Self {
            symbol_size,
            window_size_ratio,
        }
    }

    /// Returns the encoding symbol size in bytes.
    pub const fn symbol_size(self) -> u16 {
        self.symbol_size
    }

    /// Returns the window size ratio, where zero means unspecified.
    pub const fn window_size_ratio(self) -> u8 {
        self.window_size_ratio
    }

    /// Encodes the value in network byte order.
    pub const fn to_bytes(self) -> [u8; 3] {
        let size = self.symbol_size.to_be_bytes();
        [size[0], size[1], self.window_size_ratio]
    }

    /// Decodes a value in network byte order.
    pub const fn from_bytes(bytes: [u8; 3]) -> Self {
        Self::new(u16::from_be_bytes([bytes[0], bytes[1]]), bytes[2])
    }
}

/// The four-octet Explicit Source FEC Payload ID from RFC 8681.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SourcePayloadId {
    esi: EncodingSymbolId,
}

impl SourcePayloadId {
    /// Creates an identifier for an encoding symbol.
    pub const fn new(esi: EncodingSymbolId) -> Self {
        Self { esi }
    }

    /// Returns the Encoding Symbol ID (ESI).
    pub const fn esi(self) -> EncodingSymbolId {
        self.esi
    }

    /// Encodes the identifier in network byte order.
    pub const fn to_bytes(self) -> [u8; 4] {
        self.esi.get().to_be_bytes()
    }

    /// Decodes an identifier in network byte order.
    pub const fn from_bytes(bytes: [u8; 4]) -> Self {
        Self::new(EncodingSymbolId::new(u32::from_be_bytes(bytes)))
    }
}

impl TryFrom<&[u8]> for SourcePayloadId {
    type Error = WireError;

    fn try_from(bytes: &[u8]) -> Result<Self, Self::Error> {
        let bytes: [u8; 4] = bytes.try_into().map_err(|_| WireError::InvalidLength {
            expected: 4,
            actual: bytes.len(),
        })?;
        Ok(Self::from_bytes(bytes))
    }
}

impl From<SourcePayloadId> for [u8; 4] {
    fn from(value: SourcePayloadId) -> Self {
        value.to_bytes()
    }
}

impl From<[u8; 4]> for SourcePayloadId {
    fn from(bytes: [u8; 4]) -> Self {
        Self::from_bytes(bytes)
    }
}

/// The eight-octet Repair FEC Payload ID from RFC 8681.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RepairPayloadId {
    repair_key: u16,
    density: DensityThreshold,
    source_symbols: u16,
    first_source_esi: EncodingSymbolId,
}

impl RepairPayloadId {
    /// Creates a repair payload identifier.
    pub const fn new(
        repair_key: u16,
        density: DensityThreshold,
        source_symbols: u16,
        first_source_esi: EncodingSymbolId,
    ) -> Result<Self, WireError> {
        if source_symbols == 0 {
            return Err(WireError::EmptyRepairWindow);
        }
        if source_symbols > 0x0fff {
            return Err(WireError::RepairWindowTooLarge);
        }
        Ok(Self {
            repair_key,
            density,
            source_symbols,
            first_source_esi,
        })
    }

    /// Returns the coefficient-generator seed.
    pub const fn repair_key(self) -> u16 {
        self.repair_key
    }

    /// Returns the coding-coefficient density threshold.
    pub const fn density(self) -> DensityThreshold {
        self.density
    }

    /// Returns the number of source symbols in the encoding window.
    pub const fn source_symbols(self) -> u16 {
        self.source_symbols
    }

    /// Returns the ESI of the first source symbol in the encoding window.
    pub const fn first_source_esi(self) -> EncodingSymbolId {
        self.first_source_esi
    }

    /// Advances the repair key for another symbol packed in the same packet.
    pub const fn for_symbol_offset(self, offset: u16) -> Self {
        Self {
            repair_key: self.repair_key.wrapping_add(offset),
            ..self
        }
    }

    /// Encodes the identifier in network byte order.
    pub const fn to_bytes(self) -> [u8; 8] {
        let key = self.repair_key.to_be_bytes();
        let density_and_count =
            (((self.density.get() as u16) << 12) | self.source_symbols).to_be_bytes();
        let first = self.first_source_esi.get().to_be_bytes();
        [
            key[0],
            key[1],
            density_and_count[0],
            density_and_count[1],
            first[0],
            first[1],
            first[2],
            first[3],
        ]
    }

    /// Decodes an identifier in network byte order.
    pub fn from_bytes(bytes: [u8; 8]) -> Result<Self, WireError> {
        let repair_key = u16::from_be_bytes([bytes[0], bytes[1]]);
        let density_and_count = u16::from_be_bytes([bytes[2], bytes[3]]);
        let density = DensityThreshold::new((density_and_count >> 12) as u8)
            .expect("a four-bit density is always valid");
        let source_symbols = density_and_count & 0x0fff;
        let first_source_esi =
            EncodingSymbolId::new(u32::from_be_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]));
        Self::new(repair_key, density, source_symbols, first_source_esi)
    }
}

impl TryFrom<&[u8]> for RepairPayloadId {
    type Error = WireError;

    fn try_from(bytes: &[u8]) -> Result<Self, Self::Error> {
        let bytes: [u8; 8] = bytes.try_into().map_err(|_| WireError::InvalidLength {
            expected: 8,
            actual: bytes.len(),
        })?;
        Self::from_bytes(bytes)
    }
}

impl TryFrom<[u8; 8]> for RepairPayloadId {
    type Error = WireError;

    fn try_from(bytes: [u8; 8]) -> Result<Self, Self::Error> {
        Self::from_bytes(bytes)
    }
}

impl From<RepairPayloadId> for [u8; 8] {
    fn from(value: RepairPayloadId) -> Self {
        value.to_bytes()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Field;

    #[test]
    fn wire_values_are_big_endian_and_round_trip() {
        let info = FecSchemeSpecificInfo::new(0x0578, 191);
        assert_eq!(info.to_bytes(), [0x05, 0x78, 191]);
        assert_eq!(FecSchemeSpecificInfo::from_bytes(info.to_bytes()), info);

        let source = SourcePayloadId::new(EncodingSymbolId::new(0x1234_5678));
        assert_eq!(source.to_bytes(), [0x12, 0x34, 0x56, 0x78]);
        assert_eq!(SourcePayloadId::from_bytes(source.to_bytes()), source);

        let repair = RepairPayloadId::new(
            0x1234,
            DensityThreshold::new(10).unwrap(),
            0x0bcd,
            EncodingSymbolId::new(0x89ab_cdef),
        )
        .unwrap();
        assert_eq!(
            repair.to_bytes(),
            [0x12, 0x34, 0xab, 0xcd, 0x89, 0xab, 0xcd, 0xef]
        );
        assert_eq!(RepairPayloadId::from_bytes(repair.to_bytes()), Ok(repair));
        assert_eq!(repair.for_symbol_offset(u16::MAX).repair_key(), 0x1233);
        assert_eq!(Field::Gf2.fec_encoding_id(), 9);
        assert_eq!(Field::Gf256.fec_encoding_id(), 10);

        assert_eq!(
            FecSchemeSpecificInfo::try_from(info.to_bytes().as_slice()),
            Ok(info)
        );
        assert_eq!(
            SourcePayloadId::try_from(source.to_bytes().as_slice()),
            Ok(source)
        );
        assert_eq!(
            RepairPayloadId::try_from(repair.to_bytes().as_slice()),
            Ok(repair)
        );
    }

    #[test]
    fn rejects_unrepresentable_windows() {
        assert_eq!(
            RepairPayloadId::new(0, DensityThreshold::FULL, 0, EncodingSymbolId::new(0)),
            Err(WireError::EmptyRepairWindow)
        );
        assert_eq!(
            RepairPayloadId::new(0, DensityThreshold::FULL, 4096, EncodingSymbolId::new(0),),
            Err(WireError::RepairWindowTooLarge)
        );
        assert_eq!(
            RepairPayloadId::from_bytes([0; 8]),
            Err(WireError::EmptyRepairWindow)
        );
        assert_eq!(
            SourcePayloadId::try_from([0_u8; 3].as_slice()),
            Err(WireError::InvalidLength {
                expected: 4,
                actual: 3,
            })
        );
    }
}
