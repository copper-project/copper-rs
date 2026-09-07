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
    /// Scheme-specific 32-bit identifier: RLC fragment count or RaptorQ payload ID.
    pub fragment_count: u32,
}

/// Owned packet for callers that retain or edit payloads.
/// Receiver parsing uses [`WirePacketRef`] to borrow the datagram instead.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct WirePacket {
    pub header: WireHeader,
    pub payload: Vec<u8>,
}

impl WirePacket {
    pub fn encode(&self) -> Result<Vec<u8>> {
        let mut bytes = alloc::vec![0; PACKET_HEADER_LEN + self.payload.len()];
        let encoded = encode_packet_into(self.header, &self.payload, &mut bytes)?;
        debug_assert_eq!(encoded, bytes.len());
        Ok(bytes)
    }

    pub fn decode(bytes: &[u8]) -> Result<Self> {
        let packet = WirePacketRef::decode(bytes)?;
        Ok(Self {
            header: packet.header,
            payload: packet.payload.to_vec(),
        })
    }
}

/// Validated packet view borrowing the transport's receive buffer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct WirePacketRef<'a> {
    pub header: WireHeader,
    pub payload: &'a [u8],
}

impl<'a> WirePacketRef<'a> {
    pub fn decode(bytes: &'a [u8]) -> Result<Self> {
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
            payload: &bytes[PACKET_HEADER_LEN..],
        })
    }
}

/// Encodes one packet into caller-owned storage and returns its exact length.
pub fn encode_packet_into(header: WireHeader, payload: &[u8], output: &mut [u8]) -> Result<usize> {
    let payload_len = u16::try_from(payload.len())
        .map_err(|_| Error::InvalidConfig("wire payload exceeds u16"))?;
    let needed = PACKET_HEADER_LEN
        .checked_add(payload.len())
        .ok_or(Error::InvalidConfig("packet length overflow"))?;
    if output.len() < needed {
        return Err(Error::BufferTooSmall {
            needed,
            available: output.len(),
        });
    }

    let bytes = &mut output[..needed];
    bytes.fill(0);
    bytes[..4].copy_from_slice(&PACKET_MAGIC);
    bytes[4] = WIRE_VERSION;
    bytes[5] = PACKET_HEADER_LEN as u8;
    bytes[6] = header.lane as u8;
    bytes[7] = header.record_kind as u8;
    bytes[8] = header.fec_scheme as u8;
    bytes[9] = header.symbol_kind as u8;
    bytes[12..28].copy_from_slice(&header.session_id);
    bytes[28..32].copy_from_slice(&header.sender_id.to_be_bytes());
    bytes[32..40].copy_from_slice(&header.packet_sequence.to_be_bytes());
    bytes[40..48].copy_from_slice(&header.object_id.to_be_bytes());
    bytes[48..60].copy_from_slice(&header.fec_metadata);
    bytes[60..64].copy_from_slice(&header.fragment_count.to_be_bytes());
    bytes[64..66].copy_from_slice(&payload_len.to_be_bytes());
    bytes[PACKET_HEADER_LEN..].copy_from_slice(payload);
    let checksum = CRC32C.checksum(bytes);
    bytes[CRC_OFFSET..PACKET_HEADER_LEN].copy_from_slice(&checksum.to_be_bytes());
    Ok(needed)
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

    #[test]
    fn caller_owned_encoding_matches_allocating_encoding() {
        let packet = fixture();
        let expected = packet.encode().unwrap();
        let mut storage = [0xaa; 128];
        let encoded = encode_packet_into(packet.header, &packet.payload, &mut storage).unwrap();

        assert_eq!(&storage[..encoded], expected);
        assert!(storage[encoded..].iter().all(|byte| *byte == 0xaa));
    }
}
