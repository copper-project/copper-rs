//! Record encapsulation without content versions. Payloads require the producing
//! application's matching decoder; this envelope does not select a content schema.

use crate::{Error, Result};
use alloc::vec::Vec;

const RECORD_MAGIC: [u8; 4] = *b"CUSR";
pub const RECORD_HEADER_LEN: usize = 45;
const RECORD_DIGEST_OFFSET: usize = 13;

/// Semantic record families carried by the log stream.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum RecordKind {
    Manifest = 0,
    CopperList = 1,
    KeyFrame = 2,
    StructuredLog = 3,
    Lifecycle = 4,
    Gap = 5,
    RecoveryPoint = 6,
}

impl TryFrom<u8> for RecordKind {
    type Error = Error;

    fn try_from(value: u8) -> Result<Self> {
        match value {
            0 => Ok(Self::Manifest),
            1 => Ok(Self::CopperList),
            2 => Ok(Self::KeyFrame),
            3 => Ok(Self::StructuredLog),
            4 => Ok(Self::Lifecycle),
            5 => Ok(Self::Gap),
            6 => Ok(Self::RecoveryPoint),
            _ => Err(Error::UnknownRecordKind(value)),
        }
    }
}

/// A verified semantic record borrowing its recovered payload.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DecodedRecord<'a> {
    pub kind: RecordKind,
    pub object_id: u64,
    /// Digest binding the record kind, identity, length, and payload.
    pub digest: [u8; 32],
    pub payload: &'a [u8],
}

/// Frame and digest one semantic payload before FEC is applied.
pub fn encode_record(kind: RecordKind, object_id: u64, payload: &[u8]) -> Result<Vec<u8>> {
    let payload_len = u64::try_from(payload.len())
        .map_err(|_| Error::InvalidConfig("record payload length exceeds u64"))?;
    let capacity = RECORD_HEADER_LEN
        .checked_add(payload.len())
        .ok_or(Error::InvalidConfig("record length overflow"))?;
    let mut record = Vec::with_capacity(capacity);
    record.extend_from_slice(&RECORD_MAGIC);
    record.push(kind as u8);
    record.extend_from_slice(&object_id.to_be_bytes());
    record.extend_from_slice(&[0_u8; 32]);
    record.extend_from_slice(payload);

    let digest = record_digest(kind, object_id, payload_len, payload);
    record[RECORD_DIGEST_OFFSET..RECORD_HEADER_LEN].copy_from_slice(digest.as_bytes());
    Ok(record)
}

/// Writes a semantic-record header for a payload already stored immediately
/// after `header` in the caller's record buffer.
pub(crate) fn encode_record_header(
    kind: RecordKind,
    object_id: u64,
    payload: &[u8],
    header: &mut [u8],
) -> Result<()> {
    if header.len() < RECORD_HEADER_LEN {
        return Err(Error::BufferTooSmall {
            needed: RECORD_HEADER_LEN,
            available: header.len(),
        });
    }
    let payload_len = u64::try_from(payload.len())
        .map_err(|_| Error::InvalidConfig("record payload length exceeds u64"))?;
    let header = &mut header[..RECORD_HEADER_LEN];
    header.fill(0);
    header[..4].copy_from_slice(&RECORD_MAGIC);
    header[4] = kind as u8;
    header[5..13].copy_from_slice(&object_id.to_be_bytes());
    let digest = record_digest(kind, object_id, payload_len, payload);
    header[RECORD_DIGEST_OFFSET..RECORD_HEADER_LEN].copy_from_slice(digest.as_bytes());
    Ok(())
}

/// Validates and decodes one complete semantic record envelope.
///
/// The payload occupies the remainder of the reassembled record. Its derived
/// length remains part of the digest input, binding the exact payload extent.
pub fn decode_record(record: &[u8]) -> Result<DecodedRecord<'_>> {
    if record.len() < RECORD_HEADER_LEN {
        return Err(Error::TruncatedRecord);
    }
    if record[..4] != RECORD_MAGIC {
        return Err(Error::InvalidMagic);
    }
    let kind = RecordKind::try_from(record[4])?;
    let object_id = u64::from_be_bytes(record[5..13].try_into().unwrap());
    let payload = &record[RECORD_HEADER_LEN..];
    let payload_len = u64::try_from(payload.len())
        .map_err(|_| Error::InvalidConfig("record payload length exceeds u64"))?;
    let expected_digest = record_digest(kind, object_id, payload_len, payload);
    if record[RECORD_DIGEST_OFFSET..RECORD_HEADER_LEN] != expected_digest.as_bytes()[..] {
        return Err(Error::RecordDigestMismatch);
    }
    Ok(DecodedRecord {
        kind,
        object_id,
        digest: *expected_digest.as_bytes(),
        payload,
    })
}

fn record_digest(
    kind: RecordKind,
    object_id: u64,
    payload_len: u64,
    payload: &[u8],
) -> blake3::Hash {
    let mut hasher = blake3::Hasher::new();
    hasher.update(&[kind as u8]);
    hasher.update(&object_id.to_be_bytes());
    hasher.update(&payload_len.to_be_bytes());
    hasher.update(payload);
    hasher.finalize()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn packed_record_header_contains_only_framing_fields() {
        let record = encode_record(RecordKind::CopperList, 42, &[42]).unwrap();
        assert_eq!(record.len(), 46);
        assert_eq!(&record[..5], b"CUSR\x01");
        assert_eq!(&record[5..13], &42_u64.to_be_bytes());
        assert_eq!(record[45], 42);
        assert_eq!(decode_record(&record).unwrap().payload, &[42]);
        let mut header = [0xaa; RECORD_HEADER_LEN];
        encode_record_header(RecordKind::CopperList, 42, &[42], &mut header).unwrap();
        assert_eq!(header, record[..RECORD_HEADER_LEN]);
    }

    #[test]
    fn record_extent_defines_payload_length_without_changing_digest_binding() {
        for kind in [
            RecordKind::Manifest,
            RecordKind::CopperList,
            RecordKind::KeyFrame,
            RecordKind::StructuredLog,
            RecordKind::Lifecycle,
            RecordKind::Gap,
            RecordKind::RecoveryPoint,
        ] {
            for object_id in [0, 42, u64::MAX] {
                for payload_len in [0, 1, 95, 96, 4096] {
                    let payload = alloc::vec![0xa5; payload_len];
                    let encoded = encode_record(kind, object_id, &payload).unwrap();
                    assert_eq!(encoded.len(), 45 + payload_len);
                    let decoded = decode_record(&encoded).unwrap();
                    assert_eq!(decoded.kind, kind);
                    assert_eq!(decoded.object_id, object_id);
                    assert_eq!(decoded.payload, payload);
                    assert_eq!(decoded.payload.as_ptr(), encoded[45..].as_ptr());

                    // Keep the original digest preimage, including the omitted length.
                    let mut preimage = alloc::vec![kind as u8];
                    preimage.extend_from_slice(&object_id.to_be_bytes());
                    preimage.extend_from_slice(&(payload_len as u64).to_be_bytes());
                    preimage.extend_from_slice(&payload);
                    let digest = blake3::hash(&preimage);
                    assert_eq!(decoded.digest, *digest.as_bytes());
                    assert_eq!(&encoded[13..45], digest.as_bytes());

                    let mut header = [0xaa; RECORD_HEADER_LEN + 1];
                    encode_record_header(kind, object_id, &payload, &mut header).unwrap();
                    assert_eq!(&header[..RECORD_HEADER_LEN], &encoded[..45]);
                    assert_eq!(header[RECORD_HEADER_LEN], 0xaa);
                }
            }
        }
    }

    #[test]
    fn record_rejects_truncation_appended_bytes_and_corruption() {
        for payload in [b"".as_slice(), b"state"] {
            let encoded = encode_record(RecordKind::KeyFrame, 42, payload).unwrap();
            for end in 0..encoded.len() {
                let expected = if end < RECORD_HEADER_LEN {
                    Error::TruncatedRecord
                } else {
                    Error::RecordDigestMismatch
                };
                assert_eq!(decode_record(&encoded[..end]), Err(expected));
            }
            for extra in [b"\0".as_slice(), b"state", encoded.as_slice()] {
                let mut extended = encoded.clone();
                extended.extend_from_slice(extra);
                assert_eq!(decode_record(&extended), Err(Error::RecordDigestMismatch));
            }
            for offset in 0..encoded.len() {
                let mut corrupted = encoded.clone();
                corrupted[offset] ^= 1; // Also changes KeyFrame to a valid StructuredLog kind.
                let expected = if offset < 4 {
                    Error::InvalidMagic
                } else {
                    Error::RecordDigestMismatch
                };
                assert_eq!(decode_record(&corrupted), Err(expected));
            }
            let mut invalid_kind = encoded;
            invalid_kind[4] = u8::MAX;
            assert_eq!(
                decode_record(&invalid_kind),
                Err(Error::UnknownRecordKind(u8::MAX))
            );
        }
    }

    #[test]
    fn record_header_rejects_short_buffers_without_writing() {
        for len in 0..RECORD_HEADER_LEN {
            let mut header = [0xaa; RECORD_HEADER_LEN];
            assert_eq!(
                encode_record_header(RecordKind::CopperList, 42, b"state", &mut header[..len]),
                Err(Error::BufferTooSmall {
                    needed: RECORD_HEADER_LEN,
                    available: len,
                })
            );
            assert_eq!(header, [0xaa; RECORD_HEADER_LEN]);
        }
    }
}
