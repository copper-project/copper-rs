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
use std::io::Read;
#[cfg(feature = "decode-catalog-build")]
use std::io::Write;

const MAGIC: &[u8; 8] = b"CUVDCAT\0";
const VERSION: u16 = 1;
const HEADER_LEN: usize = 10;
/// Maximum uncompressed catalog size accepted by the V1 reader (16 MiB).
pub const VALUE_DECODE_CATALOG_MAX_BYTES: usize = 16 * 1024 * 1024;

/// The generated CopperList layout whose payload slots this catalog describes.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Encode, Decode)]
pub enum ValueDecodeCatalogLayout {
    /// Shared presence/capture planes and delta-coded common metadata.
    Compact,
    /// Each message is encoded with its own metadata envelope.
    Flat,
}

/// A recorded output slot in native CopperList encoding order.
#[derive(Clone, Debug, Encode, Decode)]
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
/// V1 uses bincode's standard configuration (little-endian, variable integers)
/// for both the catalog body and the recorded native payloads. Common CopperList
/// metadata is identified by `layout`; its standalone interpretation is a later
/// extension of the catalog format.
#[derive(Clone, Debug, Encode, Decode)]
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

/// Bootstrap information obtained without decompression.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ValueDecodeCatalogHeader {
    /// Catalog wire version; fixes the bincode configuration and Brotli compression.
    pub version: u16,
}

impl ValueDecodeCatalogHeader {
    /// Check the magic and version without allocating or decompressing the body.
    pub fn read(bytes: &[u8]) -> Result<Self, DecodeError> {
        let header = bytes
            .get(..HEADER_LEN)
            .ok_or(DecodeError::Other("truncated ValueDecodeCatalog header"))?;
        if &header[..8] != MAGIC || header[8..10] != VERSION.to_le_bytes() {
            return Err(DecodeError::Other("unsupported ValueDecodeCatalog format"));
        }
        if bytes.len() > HEADER_LEN + VALUE_DECODE_CATALOG_MAX_BYTES {
            return Err(DecodeError::Other("ValueDecodeCatalog exceeds 16 MiB"));
        }
        Ok(Self { version: VERSION })
    }
}

impl ValueDecodeCatalog {
    /// Decompress and read a catalog offline, enforcing the fixed size limit.
    pub fn from_blob(bytes: &[u8]) -> Result<Self, DecodeError> {
        ValueDecodeCatalogHeader::read(bytes)?;
        let mut raw = Vec::new();
        let mut reader = brotli::Decompressor::new(&bytes[HEADER_LEN..], 4096);
        reader
            .by_ref()
            .take(VALUE_DECODE_CATALOG_MAX_BYTES as u64 + 1)
            .read_to_end(&mut raw)
            .map_err(|_| DecodeError::Other("invalid compressed ValueDecodeCatalog"))?;
        if raw.len() > VALUE_DECODE_CATALOG_MAX_BYTES {
            return Err(DecodeError::Other("ValueDecodeCatalog exceeds 16 MiB"));
        }
        let mut end = [0; 1];
        if reader
            .read(&mut end)
            .map_err(|_| DecodeError::Other("trailing compressed ValueDecodeCatalog bytes"))?
            != 0
            || !reader.get_ref().is_empty()
        {
            return Err(DecodeError::Other(
                "trailing compressed ValueDecodeCatalog bytes",
            ));
        }
        let (catalog, used): (Self, _) = bincode::decode_from_slice(
            &raw,
            bincode::config::standard().with_limit::<VALUE_DECODE_CATALOG_MAX_BYTES>(),
        )?;
        if used != raw.len() {
            return Err(DecodeError::Other("trailing ValueDecodeCatalog body bytes"));
        }
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

    /// Serialize and compress on the build host, using Brotli's maximum quality 11.
    #[cfg(feature = "decode-catalog-build")]
    pub fn to_blob(&self) -> Result<Vec<u8>, EncodeError> {
        let raw = bincode::encode_to_vec(self, bincode::config::standard())?;
        if raw.len() > VALUE_DECODE_CATALOG_MAX_BYTES {
            return Err(EncodeError::Other("ValueDecodeCatalog exceeds 16 MiB"));
        }
        let mut compressed = Vec::new();
        {
            let mut writer = brotli::CompressorWriter::new(&mut compressed, 4096, 11, 24);
            writer
                .write_all(&raw)
                .map_err(|inner| EncodeError::Io { inner, index: 0 })?;
        }
        let mut blob = Vec::with_capacity(HEADER_LEN + compressed.len());
        blob.extend_from_slice(MAGIC);
        blob.extend_from_slice(&VERSION.to_le_bytes());
        blob.extend_from_slice(&compressed);
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
    fn test_catalog_v1_fixture_decodes_without_producer_types() {
        let bytes = include_bytes!("../tests/fixtures/catalog_v1.bin");
        let catalog = ValueDecodeCatalog::from_blob(bytes).unwrap();
        assert_eq!(ValueDecodeCatalogHeader::read(bytes).unwrap().version, 1);
        assert!(
            catalog
                .description
                .schemas
                .iter()
                .any(|schema| schema.type_path == "cu_self_describing_payloads::WheelSample")
        );
        let mut description = catalog.description;
        description.root = description
            .bindings
            .iter()
            .position(|binding| description.schemas[binding.schema].type_path == "u32")
            .unwrap();
        assert_eq!(
            description
                .decode(
                    &[42],
                    bincode::config::standard(),
                    ValueDecodeLimits::default()
                )
                .unwrap(),
            (Value::U32(42), 1)
        );
    }

    #[test]
    fn test_catalog_checks_slot_roots() {
        let mut catalog = catalog();
        catalog.slots[0].binding = Some(usize::MAX);
        assert!(ValueDecodeCatalog::from_blob(&catalog.to_blob().unwrap()).is_err());
    }
}
