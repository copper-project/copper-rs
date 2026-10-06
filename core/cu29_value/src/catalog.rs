//! Versioned, compressed payload descriptions prepared on the build host.
//!
//! Catalogs share one description graph across all output slots. The bootstrap
//! header can be checked without decompressing or constructing native types.

use crate::decode::ValueDecodeDescription;
use bevy_reflect::GetTypeRegistration;
use bevy_reflect::TypeRegistry;
use bincode::Decode;
use bincode::Encode;
use bincode::ValueDecode;
use bincode::error::DecodeError;
#[cfg(feature = "decode-catalog-build")]
use bincode::error::EncodeError;
use bincode::value_decode::ValueDecodeRef;
use serde::{Deserialize, Serialize};

use crate::catalog_header::{HEADER_LEN, MAGIC};
pub use crate::catalog_header::{
    VALUE_DECODE_CATALOG_MAX_BYTES, ValueDecodeCatalogHeader, ValueDecodeCatalogLayout,
};

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
#[derive(Clone, Debug, Encode, Decode, Serialize, Deserialize)]
pub struct ValueDecodeCatalog {
    /// Mission selected by runtime generation.
    pub mission: String,
    /// Canonical effective Copper RON configuration.
    pub config_ron: String,
    /// CopperList envelope layout.
    pub layout: ValueDecodeCatalogLayout,
    /// Shared wire operations, schema bindings, names and storage units.
    pub description: ValueDecodeDescription,
    /// Payloads in recorded order, including uncaptured positions.
    pub slots: Vec<ValueDecodeCatalogSlot>,
}

impl ValueDecodeCatalog {
    /// Decompress and read a catalog offline, enforcing the fixed size limit.
    pub fn from_blob(bytes: &[u8]) -> Result<Self, DecodeError> {
        if bytes.starts_with(crate::catalog_header::CATALOG_CHUNK_MAGIC) {
            return Self::from_framed_blob(bytes);
        }
        ValueDecodeCatalogHeader::read(bytes)?;
        Self::from_stream_blob(bytes)
    }

    fn from_framed_blob(mut bytes: &[u8]) -> Result<Self, DecodeError> {
        let mut blob = Vec::new();
        let mut expected = 0u32;
        while !bytes.is_empty() {
            let header = bytes
                .get(..14)
                .ok_or(DecodeError::Other("truncated catalog chunk"))?;
            if &header[..8] != crate::catalog_header::CATALOG_CHUNK_MAGIC {
                return Err(DecodeError::Other("invalid catalog continuation"));
            }
            let sequence = u32::from_le_bytes(
                header[8..12]
                    .try_into()
                    .map_err(|_| DecodeError::Other("invalid chunk sequence"))?,
            );
            if sequence != expected {
                return Err(DecodeError::Other(
                    "duplicate, missing or out-of-order catalog chunk",
                ));
            }
            let len = usize::from(u16::from_le_bytes([header[12], header[13]]));
            if len == 0 || len > 128 {
                return Err(DecodeError::Other("invalid catalog chunk length"));
            }
            let data = bytes
                .get(14..14 + len)
                .ok_or(DecodeError::Other("truncated catalog chunk data"))?;
            if blob.len() + len
                > HEADER_LEN
                    + VALUE_DECODE_CATALOG_MAX_BYTES
                    + VALUE_DECODE_CATALOG_MAX_BYTES / 8
                    + 8
            {
                return Err(DecodeError::Other("ValueDecodeCatalog exceeds 16 MiB"));
            }
            blob.extend_from_slice(data);
            bytes = &bytes[14 + len..];
            expected += 1;
        }
        if !blob.starts_with(MAGIC) {
            return Err(DecodeError::Other("invalid catalog bootstrap"));
        }
        Self::from_blob(&blob)
    }

    fn from_stream_blob(bytes: &[u8]) -> Result<Self, DecodeError> {
        use heatshrink::{Poll, SinkError};
        let end = bytes
            .len()
            .checked_sub(8)
            .filter(|end| *end >= HEADER_LEN)
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
        let compressed = &bytes[HEADER_LEN..end];
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
        for slot in &catalog.slots {
            if slot
                .binding
                .is_some_and(|binding| binding >= catalog.description.bindings.len())
            {
                return Err(DecodeError::Other(
                    "invalid ValueDecodeCatalog slot binding",
                ));
            }
        }
        Ok(catalog)
    }

    /// Serialize and compress on the build host using the catalog's Heatshrink format.
    #[cfg(feature = "decode-catalog-build")]
    pub fn to_blob(&self) -> Result<Vec<u8>, EncodeError> {
        struct BlobWriter<'a>(&'a mut Vec<u8>);
        impl bincode::enc::write::Writer for BlobWriter<'_> {
            fn write(&mut self, bytes: &[u8]) -> Result<(), EncodeError> {
                self.0.extend_from_slice(bytes);
                Ok(())
            }
        }
        let mut blob = Vec::new();
        let mut writer = crate::catalog_stream::CompressedWriter::new(BlobWriter(&mut blob))?;
        bincode::encode_into_writer(self, &mut writer, bincode::config::standard())?;
        writer.finish()?;
        Ok(blob)
    }
}

/// Builds one shared graph from typed payload registrations on the host.
#[derive(Default)]
pub struct ValueDecodeCatalogBuilder {
    registry: TypeRegistry,
    roots: Vec<ValueDecodeRef>,
    slots: Vec<ValueDecodeCatalogSlot>,
}

impl ValueDecodeCatalogBuilder {
    /// Register reflected dependencies of an opaque payload's custom encoding.
    pub fn register<T: GetTypeRegistration>(&mut self) {
        self.registry.register::<T>();
    }

    /// Add a captured native payload in generated CopperList order.
    pub fn add<T: ValueDecode + GetTypeRegistration>(&mut self, task_id: &str, msg_type: &str) {
        self.registry.register::<T>();
        self.slots.push(ValueDecodeCatalogSlot {
            task_id: task_id.into(),
            msg_type: msg_type.into(),
            binding: Some(self.roots.len()),
        });
        self.roots.push(ValueDecodeRef::of::<T>());
    }

    /// Retain an uncaptured position without imposing description bounds on its type.
    pub fn add_uncaptured(&mut self, task_id: &str, msg_type: &str) {
        self.slots.push(ValueDecodeCatalogSlot {
            task_id: task_id.into(),
            msg_type: msg_type.into(),
            binding: None,
        });
    }

    /// Resolve reachable descriptions, deduplicating types and wire operations.
    pub fn finish(
        mut self,
        config_ron: &str,
        mission: &str,
        layout: ValueDecodeCatalogLayout,
    ) -> Result<ValueDecodeCatalog, DecodeError> {
        let (description, roots) =
            ValueDecodeDescription::from_registry_roots(&self.registry, &self.roots)?;
        for slot in &mut self.slots {
            if let Some(root) = slot.binding {
                slot.binding = Some(roots[root]);
            }
        }
        Ok(ValueDecodeCatalog {
            mission: mission.into(),
            config_ron: config_ron.into(),
            layout,
            description,
            slots: self.slots,
        })
    }
}

#[cfg(all(test, feature = "decode-catalog-build"))]
mod tests {
    use super::*;
    use crate::Value;
    use crate::decode::ValueDecodeLimits;

    fn catalog() -> ValueDecodeCatalog {
        let mut builder = ValueDecodeCatalogBuilder::default();
        builder.add::<u32>("first", "u32");
        builder.add::<u32>("second", "u32");
        builder.add_uncaptured("hidden", "Opaque");
        builder
            .finish("()", "default", ValueDecodeCatalogLayout::Compact)
            .unwrap()
    }

    #[test]
    fn test_catalog_shares_types_and_retains_uncaptured_slots() {
        let catalog = catalog();
        let bytes = catalog.to_blob().unwrap();
        let loaded = ValueDecodeCatalog::from_blob(&bytes).unwrap();
        assert_eq!(loaded.description.bindings.len(), 1);
        assert_eq!(loaded.description.schemas.len(), 1);
        assert_eq!(loaded.description.operations.len(), 1);
        assert_eq!(loaded.slots[0].binding, loaded.slots[1].binding);
        assert_eq!(loaded.slots[2].binding, None);
        let payload = bincode::encode_to_vec(300_u32, bincode::config::standard()).unwrap();
        assert_eq!(
            loaded
                .description
                .decode(
                    &payload,
                    bincode::config::standard(),
                    ValueDecodeLimits::default()
                )
                .unwrap(),
            (Value::U32(300), payload.len())
        );
        assert_eq!(bytes, catalog.to_blob().unwrap());
    }

    #[test]
    fn test_catalog_rejects_corruption_versions_and_lengths() {
        let bytes = catalog().to_blob().unwrap();
        assert_eq!(ValueDecodeCatalogHeader::read(&bytes).unwrap().version, 1);
        for version in [0u16, 2, 3, 4, u16::MAX] {
            let mut unsupported = bytes.clone();
            unsupported[8..HEADER_LEN].copy_from_slice(&version.to_le_bytes());
            assert!(ValueDecodeCatalog::from_blob(&unsupported).is_err());
        }
        for length in [0, 8, HEADER_LEN - 1, bytes.len() - 1] {
            assert!(ValueDecodeCatalog::from_blob(&bytes[..length]).is_err());
        }
        for offset in [0, 8, HEADER_LEN] {
            let mut corrupt = bytes.clone();
            corrupt[offset] ^= 0xff;
            assert!(
                ValueDecodeCatalog::from_blob(&corrupt).is_err(),
                "offset {offset}"
            );
        }
        let mut extra = bytes;
        extra.push(0);
        assert!(ValueDecodeCatalog::from_blob(&extra).is_err());
    }

    #[test]
    fn test_catalog_checks_slot_roots() {
        let mut catalog = catalog();
        catalog.slots[0].binding = Some(usize::MAX);
        assert!(ValueDecodeCatalog::from_blob(&catalog.to_blob().unwrap()).is_err());
    }

    #[test]
    fn test_registered_opaque_dependencies_decode_without_adding_slots() {
        #[derive(Clone, bincode::Encode, bevy_reflect::Reflect, serde::Serialize)]
        struct Dependency {
            channel: u16,
        }
        #[derive(Clone, bincode::Encode, bevy_reflect::Reflect, serde::Serialize)]
        #[reflect(opaque)]
        struct Batch {
            requests: Vec<Dependency>,
        }

        let mut missing = ValueDecodeCatalogBuilder::default();
        missing.add::<Batch>("source", "Batch");
        assert!(
            missing
                .finish("()", "default", ValueDecodeCatalogLayout::Compact)
                .is_err()
        );

        let mut builder = ValueDecodeCatalogBuilder::default();
        builder.register::<Dependency>();
        builder.add::<Batch>("source", "Batch");
        builder.add_uncaptured("hidden", "Opaque");
        let catalog = builder
            .finish("()", "default", ValueDecodeCatalogLayout::Compact)
            .unwrap();
        let mut catalog = ValueDecodeCatalog::from_blob(&catalog.to_blob().unwrap()).unwrap();
        assert_eq!(catalog.slots.len(), 2);
        assert_eq!(catalog.slots[0].task_id, "source");
        assert_eq!(catalog.slots[1].task_id, "hidden");
        assert_eq!(catalog.slots[1].binding, None);

        let sample = Batch {
            requests: vec![Dependency { channel: 300 }],
        };
        let config = bincode::config::standard();
        let bytes = bincode::encode_to_vec(&sample, config).unwrap();
        catalog.description.root = catalog.slots[0].binding.unwrap();
        assert_eq!(
            catalog
                .description
                .decode(&bytes, config, ValueDecodeLimits::default())
                .unwrap(),
            (crate::to_value(&sample).unwrap(), bytes.len())
        );
    }
}
