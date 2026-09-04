use crate::{Error, RecordKind, Result};
use alloc::vec::Vec;
use crc::{CRC_32_ISCSI, Crc};

const PACKET_MAGIC: [u8; 4] = *b"CULS";
pub const WIRE_VERSION: u8 = 1;
pub const PACKET_HEADER_LEN: usize = 72;
const CRC_OFFSET: usize = 68;
const CRC32C: Crc<u32> = Crc::<u32>::new(&CRC_32_ISCSI);

/// Independently scheduled transport lanes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Lane {
    Control = 0,
    ReplayCritical = 1,
    LargeObject = 2,
    StructuredLog = 3,
    Visualization = 4,
}

impl TryFrom<u8> for Lane {
    type Error = Error;

    fn try_from(value: u8) -> Result<Self> {
        match value {
            0 => Ok(Self::Control),
            1 => Ok(Self::ReplayCritical),
            2 => Ok(Self::LargeObject),
            3 => Ok(Self::StructuredLog),
            4 => Ok(Self::Visualization),
            _ => Err(Error::UnknownLane(value)),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum FecScheme {
    RlcGf2 = 0,
    RlcGf256 = 1,
    RaptorQ = 2,
}

impl TryFrom<u8> for FecScheme {
    type Error = Error;

    fn try_from(value: u8) -> Result<Self> {
        match value {
            0 => Ok(Self::RlcGf2),
            1 => Ok(Self::RlcGf256),
            2 => Ok(Self::RaptorQ),
            _ => Err(Error::UnknownFecScheme(value)),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum FecSymbolKind {
    Source = 0,
    Repair = 1,
}

impl TryFrom<u8> for FecSymbolKind {
    type Error = Error;

    fn try_from(value: u8) -> Result<Self> {
        match value {
            0 => Ok(Self::Source),
            1 => Ok(Self::Repair),
            _ => Err(Error::UnknownSymbolKind(value)),
        }
    }
}

/// Fixed-endian metadata repeated on every independently reorderable symbol.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct WireHeader {
    pub lane: Lane,
    pub record_kind: RecordKind,
    pub fec_scheme: FecScheme,
    pub symbol_kind: FecSymbolKind,
    pub session_id: [u8; 16],
    pub sender_id: u32,
    pub packet_sequence: u64,
    pub object_id: u64,
    /// Scheme-specific fixed-width FEC metadata.
    pub fec_metadata: [u8; 12],
    /// Fragment count for a systematic symbol; zero for a repair symbol.
    pub fragment_count: u32,
}

/// One validated wire symbol and its FEC payload.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct WirePacket {
    pub header: WireHeader,
    pub payload: Vec<u8>,
}

impl WirePacket {
    pub fn encode(&self) -> Result<Vec<u8>> {
        let payload_len = u16::try_from(self.payload.len())
            .map_err(|_| Error::InvalidConfig("wire payload exceeds u16"))?;
        let mut bytes = Vec::with_capacity(PACKET_HEADER_LEN + self.payload.len());
        bytes.extend_from_slice(&PACKET_MAGIC);
        bytes.push(WIRE_VERSION);
        bytes.push(PACKET_HEADER_LEN as u8);
        bytes.push(self.header.lane as u8);
        bytes.push(self.header.record_kind as u8);
        bytes.push(self.header.fec_scheme as u8);
        bytes.push(self.header.symbol_kind as u8);
        bytes.extend_from_slice(&0_u16.to_be_bytes());
        bytes.extend_from_slice(&self.header.session_id);
        bytes.extend_from_slice(&self.header.sender_id.to_be_bytes());
        bytes.extend_from_slice(&self.header.packet_sequence.to_be_bytes());
        bytes.extend_from_slice(&self.header.object_id.to_be_bytes());
        bytes.extend_from_slice(&self.header.fec_metadata);
        bytes.extend_from_slice(&self.header.fragment_count.to_be_bytes());
        bytes.extend_from_slice(&payload_len.to_be_bytes());
        bytes.extend_from_slice(&0_u16.to_be_bytes());
        bytes.extend_from_slice(&0_u32.to_be_bytes());
        bytes.extend_from_slice(&self.payload);
        let checksum = CRC32C.checksum(&bytes);
        bytes[CRC_OFFSET..PACKET_HEADER_LEN].copy_from_slice(&checksum.to_be_bytes());
        Ok(bytes)
    }

    pub fn decode(bytes: &[u8]) -> Result<Self> {
        if bytes.len() < PACKET_HEADER_LEN {
            return Err(Error::TruncatedPacket);
        }
        if bytes[..4] != PACKET_MAGIC {
            return Err(Error::InvalidMagic);
        }
        let version = bytes[4];
        if version != WIRE_VERSION {
            return Err(Error::UnsupportedVersion(version));
        }
        if bytes[5] as usize != PACKET_HEADER_LEN {
            return Err(Error::InvalidHeaderLength(bytes[5]));
        }
        let payload_len = u16::from_be_bytes(bytes[64..66].try_into().unwrap()) as usize;
        if bytes.len() != PACKET_HEADER_LEN + payload_len {
            return Err(Error::PayloadLengthMismatch);
        }

        let expected_checksum =
            u32::from_be_bytes(bytes[CRC_OFFSET..PACKET_HEADER_LEN].try_into().unwrap());
        let mut digest = CRC32C.digest();
        digest.update(&bytes[..CRC_OFFSET]);
        digest.update(&[0_u8; 4]);
        digest.update(&bytes[PACKET_HEADER_LEN..]);
        if digest.finalize() != expected_checksum {
            return Err(Error::CrcMismatch);
        }

        let mut session_id = [0_u8; 16];
        session_id.copy_from_slice(&bytes[12..28]);
        let mut fec_metadata = [0_u8; 12];
        fec_metadata.copy_from_slice(&bytes[48..60]);

        Ok(Self {
            header: WireHeader {
                lane: Lane::try_from(bytes[6])?,
                record_kind: RecordKind::try_from(bytes[7])?,
                fec_scheme: FecScheme::try_from(bytes[8])?,
                symbol_kind: FecSymbolKind::try_from(bytes[9])?,
                session_id,
                sender_id: u32::from_be_bytes(bytes[28..32].try_into().unwrap()),
                packet_sequence: u64::from_be_bytes(bytes[32..40].try_into().unwrap()),
                object_id: u64::from_be_bytes(bytes[40..48].try_into().unwrap()),
                fec_metadata,
                fragment_count: u32::from_be_bytes(bytes[60..64].try_into().unwrap()),
            },
            payload: bytes[PACKET_HEADER_LEN..].to_vec(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec;

    fn fixture() -> WirePacket {
        WirePacket {
            header: WireHeader {
                lane: Lane::LargeObject,
                record_kind: RecordKind::KeyFrame,
                fec_scheme: FecScheme::RaptorQ,
                symbol_kind: FecSymbolKind::Source,
                session_id: [0x11; 16],
                sender_id: 0x2233_4455,
                packet_sequence: 0x6677_8899_aabb_ccdd,
                object_id: 0x0102_0304_0506_0708,
                fec_metadata: [0, 0, 0, 0, 3, 0, 0, 8, 1, 0, 1, 1],
                fragment_count: 1,
            },
            payload: vec![0, 0, 0, 0, 1, 2, 3, 0, 0, 0, 0, 0],
        }
    }

    #[test]
    fn fixed_header_has_a_stable_golden_prefix() {
        let encoded = fixture().encode().unwrap();
        assert_eq!(
            &encoded[..12],
            &[b'C', b'U', b'L', b'S', 1, 72, 2, 2, 2, 0, 0, 0]
        );
        assert_eq!(WirePacket::decode(&encoded).unwrap(), fixture());
    }

    #[test]
    fn corruption_is_rejected_before_fec() {
        let mut encoded = fixture().encode().unwrap();
        *encoded.last_mut().unwrap() ^= 0x40;
        assert_eq!(WirePacket::decode(&encoded), Err(Error::CrcMismatch));
    }
}
