//! Offline readers for the shared payload catalog saved at application startup.
//!
//! Catalogs share one description graph across every compiled mission.

use crate::decode::ValueDecodeDescription;
use bincode::Decode;
use bincode::Encode;
use bincode::error::DecodeError;
use serde::{Deserialize, Serialize};

use crate::catalog_format::VERSION;
pub use crate::catalog_format::{VALUE_DECODE_CATALOG_MAX_BYTES, ValueDecodeCatalogLayout};

/// A recorded output slot in native CopperList encoding order.
#[derive(Clone, Debug, Encode, Decode, Serialize, Deserialize)]
pub struct ValueDecodeCatalogSlot {
    /// Task or bridge/channel identity from the generated output map.
    pub task_id: String,
    /// Configured message type.
    pub msg_type: String,
    /// Root binding into the shared description, or none for an uncaptured slot.
    pub binding: Option<usize>,
}

/// A versioned payload description catalog. This API is experimental.
///
/// The format uses bincode's standard configuration (little-endian, variable integers)
/// for both the catalog body and the recorded native payloads. Common CopperList
/// metadata is identified by `layout`. The format fixes the Compact and Flat envelope
/// rules as well as payload encoding; changing either requires a new version.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ValueDecodeCatalog {
    /// Bincode catalog format version, compressed with the description body.
    pub version: u16,
    /// CopperList envelope layout.
    pub layout: ValueDecodeCatalogLayout,
    /// Shared wire operations, schema bindings, names and storage units.
    pub description: ValueDecodeDescription,
    /// Ordered slot maps for every compiled mission, sharing the description graph.
    pub missions: Vec<ValueDecodeCatalogMission>,
}

/// Output slots for one numeric mission index in application metadata.
#[derive(Clone, Debug, Encode, Decode, Serialize, Deserialize)]
pub struct ValueDecodeCatalogMission {
    pub slots: Vec<ValueDecodeCatalogSlot>,
}

impl Encode for ValueDecodeCatalog {
    fn encode<E: bincode::enc::Encoder>(
        &self,
        encoder: &mut E,
    ) -> Result<(), bincode::error::EncodeError> {
        self.version.encode(encoder)?;
        self.layout.encode(encoder)?;
        self.description.encode(encoder)?;
        match self.version {
            VERSION => self.missions.encode(encoder),
            1 => {
                self.missions.len().encode(encoder)?;
                for (index, mission) in self.missions.iter().enumerate() {
                    (index as u32, mission).encode(encoder)?;
                }
                Ok(())
            }
            _ => Err(bincode::error::EncodeError::Other(
                "Unsupported catalog version",
            )),
        }
    }
}

impl<Context> Decode<Context> for ValueDecodeCatalog {
    fn decode<D: bincode::de::Decoder<Context = Context>>(
        decoder: &mut D,
    ) -> Result<Self, DecodeError> {
        let version = u16::decode(decoder)?;
        if !matches!(version, 1 | VERSION) {
            return Err(DecodeError::Other("Unsupported catalog version"));
        }
        let layout = Decode::decode(decoder)?;
        let description = Decode::decode(decoder)?;
        let missions = if version == 1 {
            Vec::<(u32, ValueDecodeCatalogMission)>::decode(decoder)?
                .into_iter()
                .enumerate()
                .map(|(position, (index, mission))| {
                    if position != index as usize {
                        return Err(DecodeError::Other("Invalid catalog mission index"));
                    }
                    Ok(mission)
                })
                .collect::<Result<_, _>>()?
        } else {
            Decode::decode(decoder)?
        };
        Ok(Self {
            version,
            layout,
            description,
            missions,
        })
    }
}
bincode::impl_borrow_decode!(ValueDecodeCatalog);

impl ValueDecodeCatalog {
    /// Decompress and read a catalog offline, enforcing the fixed size limit.
    pub fn from_blob(bytes: &[u8]) -> Result<Self, DecodeError> {
        Self::from_stream_blob(bytes)
    }

    fn from_stream_blob(bytes: &[u8]) -> Result<Self, DecodeError> {
        use heatshrink::{Poll, SinkError};
        if bytes.len() > VALUE_DECODE_CATALOG_MAX_BYTES * 9 / 8 + 8 {
            return Err(DecodeError::Other("Catalog exceeds offline size limit"));
        }
        let end = bytes
            .len()
            .checked_sub(8)
            .filter(|end| *end > 0)
            .ok_or(DecodeError::Other("truncated streaming catalog footer"))?;
        let footer = &bytes[end..];
        let raw_len = u32::from_le_bytes(
            footer[..4]
                .try_into()
                .map_err(|_| DecodeError::Other("invalid catalog length"))?,
        ) as usize;
        let checksum = u32::from_le_bytes(
            footer[4..]
                .try_into()
                .map_err(|_| DecodeError::Other("invalid catalog checksum"))?,
        );
        if raw_len > VALUE_DECODE_CATALOG_MAX_BYTES {
            return Err(DecodeError::Other("ValueDecodeCatalog exceeds 16 MiB"));
        }
        let mut raw = Vec::with_capacity(raw_len);
        let mut decoder = heatshrink::decoder::HeatshrinkDecoder::<10, 5, 32, 1024>::new();
        let compressed = &bytes[..end];
        let mut offset = 0;
        let mut buffer = [0; 256];
        loop {
            let before = offset;
            if offset < compressed.len() {
                match decoder.sink(&compressed[offset..]) {
                    Ok(consumed) => offset += consumed,
                    Err(SinkError::Full) => {}
                    Err(SinkError::Misuse) => {
                        return Err(DecodeError::Other("invalid compressed catalog"));
                    }
                }
            }
            let result = decoder
                .poll(&mut buffer)
                .map_err(|_| DecodeError::Other("invalid compressed catalog"))?;
            let count = result.bytes_written();
            if raw.len() + count > raw_len {
                return Err(DecodeError::Other("streaming catalog length mismatch"));
            }
            raw.extend_from_slice(&buffer[..count]);
            if matches!(result, Poll::Empty(_)) && offset == compressed.len() {
                break;
            }
            if offset == before && count == 0 {
                return Err(DecodeError::Other("invalid compressed catalog"));
            }
        }
        if raw.len() != raw_len || !crate::catalog_stream::crc32(u32::MAX, &raw) != checksum {
            return Err(DecodeError::Other(
                "streaming catalog checksum or length mismatch",
            ));
        }
        let (catalog, consumed) = bincode::decode_from_slice::<Self, _>(
            &raw,
            bincode::config::standard().with_limit::<VALUE_DECODE_CATALOG_MAX_BYTES>(),
        )?;
        if consumed != raw.len() {
            return Err(DecodeError::Other("trailing ValueDecodeCatalog body bytes"));
        }
        catalog.description.validate()?;
        for mission in &catalog.missions {
            for slot in &mission.slots {
                if slot
                    .binding
                    .is_some_and(|binding| binding >= catalog.description.bindings.len())
                {
                    return Err(DecodeError::Other(
                        "invalid ValueDecodeCatalog slot binding",
                    ));
                }
            }
        }
        Ok(catalog)
    }
}
