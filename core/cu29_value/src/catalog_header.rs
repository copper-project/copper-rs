//! Bootstrap metadata shared by embedded catalog writers and offline readers.

use bincode::Decode;
use bincode::Encode;
use bincode::error::DecodeError;
use serde::{Deserialize, Serialize};

pub(crate) const MAGIC: &[u8; 8] = b"CUVDCAT\0";
pub(crate) const HEADER_LEN: usize = 10;
/// Framing marker for catalog entries that may continue in later sections.
#[doc(hidden)]
pub const CATALOG_CHUNK_MAGIC: &[u8; 8] = b"CUVDCHNK";
pub(crate) const STREAM_VERSION: u16 = 4;
/// Maximum uncompressed catalog size accepted by the readers (16 MiB).
pub const VALUE_DECODE_CATALOG_MAX_BYTES: usize = 16 * 1024 * 1024;

/// The recorded CopperList envelope layout.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Encode, Decode, Serialize, Deserialize)]
pub enum ValueDecodeCatalogLayout {
    /// Shared presence/capture planes and delta-coded common metadata.
    Compact,
    /// Each message is encoded with its own metadata envelope.
    Flat,
}

/// Bootstrap information obtained without allocation or decompression.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ValueDecodeCatalogHeader {
    /// V1/V2 contain legacy string metadata. V3 uses typed metadata with Brotli;
    /// V4 uses typed metadata with streaming Heatshrink and a checksum.
    pub version: u16,
}

impl ValueDecodeCatalogHeader {
    /// Check the magic, supported version, and encoded size limit.
    pub fn read(bytes: &[u8]) -> Result<Self, DecodeError> {
        let header = bytes
            .get(..HEADER_LEN)
            .ok_or(DecodeError::Other("truncated ValueDecodeCatalog header"))?;
        let version = u16::from_le_bytes([header[8], header[9]]);
        if &header[..8] != MAGIC || !matches!(version, 1 | 2 | 3 | STREAM_VERSION) {
            return Err(DecodeError::Other("unsupported ValueDecodeCatalog format"));
        }
        // Heatshrink literals can expand by one bit per byte.
        if bytes.len()
            > HEADER_LEN + VALUE_DECODE_CATALOG_MAX_BYTES + VALUE_DECODE_CATALOG_MAX_BYTES / 8 + 8
        {
            return Err(DecodeError::Other("ValueDecodeCatalog exceeds 16 MiB"));
        }
        Ok(Self { version })
    }
}
