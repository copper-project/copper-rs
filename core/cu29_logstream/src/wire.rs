//! Packed transport framing. No version or schema is sent in packet headers;
//! sender and receiver must be built for the matching application.

use crate::{Error, RecordKind, Result};
use alloc::vec::Vec;
use crc::{CRC_32_ISCSI, Crc};

const PACKET_MAGIC: [u8; 4] = *b"CULS";
/// Maximum packet header length, used to bound storage and the shared FEC symbol size.
/// RaptorQ uses 56 bytes; RLC uses 36 for source packets and 40 for repair packets.
pub const PACKET_HEADER_LEN: usize = 56;
const COMMON_HEADER_LEN: usize = 28;
const RLC_SOURCE_HEADER_LEN: usize = 36;
const RLC_REPAIR_HEADER_LEN: usize = 40;

const fn packet_header_len(scheme: FecScheme, kind: FecSymbolKind) -> usize {
    match (scheme, kind) {
        (FecScheme::RaptorQ, _) => PACKET_HEADER_LEN,
        (_, FecSymbolKind::Source) => RLC_SOURCE_HEADER_LEN,
        (_, FecSymbolKind::Repair) => RLC_REPAIR_HEADER_LEN,
    }
}
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
    /// RaptorQ object identity. Not transmitted for RLC; decoded as zero.
    pub object_id: u64,
    /// RaptorQ OTI (12 bytes), RLC source ID (first 4), or RLC repair ID (first 8).
    /// Unused RLC bytes are not transmitted and decode as zero.
    pub fec_metadata: [u8; 12],
    /// RaptorQ payload ID. Not transmitted for RLC; decoded as zero.
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
        let mut bytes = alloc::vec![0; packet_header_len(self.header.fec_scheme, self.header.symbol_kind) + self.payload.len()];
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
    /// Decodes exactly one complete packet supplied by the carrier.
    /// The payload occupies the remaining packet extent after the header.
    pub fn decode(bytes: &'a [u8]) -> Result<Self> {
        if bytes.len() < COMMON_HEADER_LEN {
            return Err(Error::TruncatedPacket);
        }
        if bytes[..4] != PACKET_MAGIC {
            return Err(Error::InvalidMagic);
        }
        let fec_scheme = FecScheme::try_from(bytes[6])?;
        let symbol_kind = FecSymbolKind::try_from(bytes[7])?;
        let header_len = packet_header_len(fec_scheme, symbol_kind);
        if bytes.len() < header_len {
            return Err(Error::TruncatedPacket);
        }
        let crc_offset = header_len - 4;
        let expected_checksum =
            u32::from_be_bytes(bytes[crc_offset..header_len].try_into().unwrap());
        let mut digest = CRC32C.digest();
        digest.update(&bytes[..crc_offset]);
        digest.update(&[0_u8; 4]);
        digest.update(&bytes[header_len..]);
        if digest.finalize() != expected_checksum {
            return Err(Error::CrcMismatch);
        }

        let mut session_id = [0_u8; 16];
        session_id.copy_from_slice(&bytes[8..24]);
        let mut fec_metadata = [0_u8; 12];
        let (object_id, fragment_count) = match fec_scheme {
            FecScheme::RaptorQ => {
                fec_metadata.copy_from_slice(&bytes[36..48]);
                (
                    u64::from_be_bytes(bytes[28..36].try_into().unwrap()),
                    u32::from_be_bytes(bytes[48..52].try_into().unwrap()),
                )
            }
            FecScheme::RlcGf2 | FecScheme::RlcGf256 => {
                let metadata_len = crc_offset - COMMON_HEADER_LEN;
                fec_metadata[..metadata_len].copy_from_slice(&bytes[COMMON_HEADER_LEN..crc_offset]);
                (0, 0)
            }
        };

        Ok(Self {
            header: WireHeader {
                lane: Lane::try_from(bytes[4])?,
                record_kind: RecordKind::try_from(bytes[5])?,
                fec_scheme,
                symbol_kind,
                session_id,
                sender_id: u32::from_be_bytes(bytes[24..28].try_into().unwrap()),
                object_id,
                fec_metadata,
                fragment_count,
            },
            payload: &bytes[header_len..],
        })
    }
}

/// Encodes one packet into caller-owned storage and returns its exact length.
pub fn encode_packet_into(header: WireHeader, payload: &[u8], output: &mut [u8]) -> Result<usize> {
    let header_len = packet_header_len(header.fec_scheme, header.symbol_kind);
    let crc_offset = header_len - 4;
    let needed = header_len
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
    bytes[4] = header.lane as u8;
    bytes[5] = header.record_kind as u8;
    bytes[6] = header.fec_scheme as u8;
    bytes[7] = header.symbol_kind as u8;
    bytes[8..24].copy_from_slice(&header.session_id);
    bytes[24..28].copy_from_slice(&header.sender_id.to_be_bytes());
    match header.fec_scheme {
        FecScheme::RaptorQ => {
            bytes[28..36].copy_from_slice(&header.object_id.to_be_bytes());
            bytes[36..48].copy_from_slice(&header.fec_metadata);
            bytes[48..52].copy_from_slice(&header.fragment_count.to_be_bytes());
        }
        FecScheme::RlcGf2 | FecScheme::RlcGf256 => {
            bytes[COMMON_HEADER_LEN..crc_offset]
                .copy_from_slice(&header.fec_metadata[..crc_offset - COMMON_HEADER_LEN]);
        }
    }
    bytes[header_len..].copy_from_slice(payload);
    let checksum = CRC32C.checksum(bytes);
    bytes[crc_offset..header_len].copy_from_slice(&checksum.to_be_bytes());
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
        assert_eq!(&encoded[..8], &[b'C', b'U', b'L', b'S', 2, 2, 2, 0]);
        assert_eq!(encoded.len(), 56 + fixture().payload.len());
        assert_eq!(&encoded[8..24], &[0x11; 16]);
        assert_eq!(&encoded[24..28], &0x2233_4455_u32.to_be_bytes());
        assert_eq!(&encoded[28..36], &0x0102_0304_0506_0708_u64.to_be_bytes());
        assert_eq!(&encoded[36..48], &fixture().header.fec_metadata);
        assert_eq!(&encoded[48..52], &1_u32.to_be_bytes());
        assert_eq!(&encoded[56..], fixture().payload);
        assert_eq!(WirePacket::decode(&encoded).unwrap(), fixture());
    }

    fn rlc_fixture(scheme: FecScheme, kind: FecSymbolKind) -> WirePacket {
        let mut packet = fixture();
        packet.header.lane = Lane::ReplayCritical;
        packet.header.record_kind = RecordKind::CopperList;
        packet.header.fec_scheme = scheme;
        packet.header.symbol_kind = kind;
        packet.header.object_id = 0;
        packet.header.fragment_count = 0;
        packet.header.fec_metadata = [0; 12];
        let id_len = match kind {
            FecSymbolKind::Source => 4,
            FecSymbolKind::Repair => 8,
        };
        packet.header.fec_metadata[..id_len].copy_from_slice(&[1, 2, 3, 4, 5, 6, 7, 8][..id_len]);
        packet
    }

    #[test]
    fn rlc_headers_carry_only_the_active_fec_id() {
        for scheme in [FecScheme::RlcGf2, FecScheme::RlcGf256] {
            for (kind, header_len, id_len) in [
                (FecSymbolKind::Source, 36, 4),
                (FecSymbolKind::Repair, 40, 8),
            ] {
                let packet = rlc_fixture(scheme, kind);
                let encoded = packet.encode().unwrap();
                assert_eq!(encoded.len(), header_len + packet.payload.len());
                assert_eq!(
                    &encoded[..8],
                    &[
                        b'C',
                        b'U',
                        b'L',
                        b'S',
                        1,
                        RecordKind::CopperList as u8,
                        scheme as u8,
                        kind as u8
                    ]
                );
                assert_eq!(&encoded[8..24], &[0x11; 16]);
                assert_eq!(&encoded[24..28], &0x2233_4455_u32.to_be_bytes());
                assert_eq!(
                    &encoded[28..28 + id_len],
                    &packet.header.fec_metadata[..id_len]
                );
                assert_eq!(header_len, 28 + id_len + 4);
                assert_eq!(&encoded[header_len..], packet.payload);
                assert_eq!(WirePacket::decode(&encoded).unwrap(), packet);

                let mut exact_storage = vec![0; encoded.len()];
                assert_eq!(
                    encode_packet_into(packet.header, &packet.payload, &mut exact_storage).unwrap(),
                    encoded.len()
                );
                assert_eq!(exact_storage, encoded);
                let available = encoded.len() - 1;
                assert_eq!(
                    encode_packet_into(
                        packet.header,
                        &packet.payload,
                        &mut exact_storage[..available]
                    ),
                    Err(Error::BufferTooSmall {
                        needed: encoded.len(),
                        available
                    })
                );
            }
        }
    }

    #[test]
    fn all_header_layouts_reject_truncation_trailing_bytes_and_corruption() {
        for scheme in [FecScheme::RlcGf2, FecScheme::RlcGf256, FecScheme::RaptorQ] {
            for kind in [FecSymbolKind::Source, FecSymbolKind::Repair] {
                let mut packet = if scheme == FecScheme::RaptorQ {
                    fixture()
                } else {
                    rlc_fixture(scheme, kind)
                };
                packet.header.symbol_kind = kind;
                for payload in [vec![], vec![0x5a; 64]] {
                    packet.payload = payload;
                    let encoded = packet.encode().unwrap();
                    assert_eq!(WirePacket::decode(&encoded).unwrap(), packet);
                    for end in 0..encoded.len() {
                        assert!(WirePacketRef::decode(&encoded[..end]).is_err());
                    }
                    let mut damaged = encoded.clone();
                    damaged.push(0);
                    assert_eq!(WirePacketRef::decode(&damaged), Err(Error::CrcMismatch));
                    damaged.pop();
                    for offset in 0..damaged.len() {
                        damaged[offset] ^= 1;
                        assert!(
                            WirePacketRef::decode(&damaged).is_err(),
                            "accepted corruption at {offset} in {scheme:?} {kind:?}"
                        );
                        damaged[offset] ^= 1;
                    }
                }
            }
        }
    }

    #[test]
    fn packet_extent_defines_payload_length_for_every_layout() {
        for scheme in [FecScheme::RlcGf2, FecScheme::RlcGf256, FecScheme::RaptorQ] {
            for kind in [FecSymbolKind::Source, FecSymbolKind::Repair] {
                let mut packet = if scheme == FecScheme::RaptorQ {
                    fixture()
                } else {
                    rlc_fixture(scheme, kind)
                };
                packet.header.symbol_kind = kind;
                let header_len = packet_header_len(scheme, kind);
                for payload_len in [0, 1, crate::DEFAULT_MAX_SYMBOL_SIZE, u16::MAX as usize + 1] {
                    packet.payload = vec![0x5a; payload_len];
                    let needed = header_len + payload_len;
                    let mut storage = vec![0xaa; needed + 1];
                    assert_eq!(
                        encode_packet_into(packet.header, &packet.payload, &mut storage).unwrap(),
                        needed
                    );
                    assert_eq!(storage[needed], 0xaa);
                    let decoded = WirePacketRef::decode(&storage[..needed]).unwrap();
                    assert_eq!(decoded.header, packet.header);
                    assert_eq!(decoded.payload, packet.payload);
                    assert_eq!(decoded.payload.as_ptr(), storage[header_len..].as_ptr());
                    assert_eq!(WirePacketRef::decode(&storage), Err(Error::CrcMismatch));
                    assert_eq!(
                        encode_packet_into(
                            packet.header,
                            &packet.payload,
                            &mut storage[..needed - 1]
                        ),
                        Err(Error::BufferTooSmall {
                            needed,
                            available: needed - 1,
                        })
                    );
                }
            }
        }
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
