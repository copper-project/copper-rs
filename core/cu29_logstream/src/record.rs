use crate::{Error, Result};
use alloc::vec::Vec;

const RECORD_MAGIC: [u8; 4] = *b"CUSR";
const RECORD_VERSION: u8 = 1;
pub const RECORD_HEADER_LEN: usize = 56;
const RECORD_DIGEST_OFFSET: usize = 24;

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
    /// Digest binding the record version, kind, identity, length, and payload.
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
    record.push(RECORD_VERSION);
    record.push(kind as u8);
    record.extend_from_slice(&0_u16.to_be_bytes());
    record.extend_from_slice(&object_id.to_be_bytes());
    record.extend_from_slice(&payload_len.to_be_bytes());
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
    header[4] = RECORD_VERSION;
    header[5] = kind as u8;
    header[8..16].copy_from_slice(&object_id.to_be_bytes());
    header[16..24].copy_from_slice(&payload_len.to_be_bytes());
    let digest = record_digest(kind, object_id, payload_len, payload);
    header[RECORD_DIGEST_OFFSET..RECORD_HEADER_LEN].copy_from_slice(digest.as_bytes());
    Ok(())
}

/// Validates and decodes one complete semantic record envelope.
pub fn decode_record(record: &[u8]) -> Result<DecodedRecord<'_>> {
    if record.len() < RECORD_HEADER_LEN {
        return Err(Error::TruncatedRecord);
    }
    if record[..4] != RECORD_MAGIC {
        return Err(Error::InvalidMagic);
    }
    let version = record[4];
    if version != RECORD_VERSION {
        return Err(Error::UnsupportedVersion(version));
    }
    let kind = RecordKind::try_from(record[5])?;
    let object_id = u64::from_be_bytes(record[8..16].try_into().unwrap());
    let payload_len = u64::from_be_bytes(record[16..24].try_into().unwrap());
    let payload_len = usize::try_from(payload_len).map_err(|_| Error::RecordLengthMismatch)?;
    let expected_len = RECORD_HEADER_LEN
        .checked_add(payload_len)
        .ok_or(Error::RecordLengthMismatch)?;
    if record.len() != expected_len {
        return Err(Error::RecordLengthMismatch);
    }
    let payload = &record[RECORD_HEADER_LEN..];
    let expected_digest = record_digest(kind, object_id, payload_len as u64, payload);
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
    hasher.update(&[RECORD_VERSION, kind as u8]);
    hasher.update(&object_id.to_be_bytes());
    hasher.update(&payload_len.to_be_bytes());
    hasher.update(payload);
    hasher.finalize()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn record_digest_binds_identity_and_payload() {
        let encoded = encode_record(RecordKind::KeyFrame, 42, b"state").unwrap();
        let decoded = decode_record(&encoded).unwrap();
        assert_eq!(decoded.kind, RecordKind::KeyFrame);
        assert_eq!(decoded.object_id, 42);
        assert_eq!(decoded.payload, b"state");

        let mut corrupted = encoded;
        *corrupted.last_mut().unwrap() ^= 0x80;
        assert_eq!(decode_record(&corrupted), Err(Error::RecordDigestMismatch));
    }
}
