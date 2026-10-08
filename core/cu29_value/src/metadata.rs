//! Framed logical metadata in portable value descriptions.
//!
//! Each entry is a codec-encoded u32 kind followed by a length-prefixed byte body.
//! Quantity kind 1 contains two little-endian u32 IDs: quantity and storage unit.
//! Storage alternatives 1 and 2 mean coherent storage and nanoseconds respectively.
//! Future IDs are retained verbatim; recognized malformed entries are rejected.

use alloc::vec::Vec;
use bincode::Decode;
use bincode::Encode;
use bincode::ValueDecode;
use bincode::ValueDecodeSpec;
use bincode::de::Decoder;
use bincode::de::read::Reader;
use bincode::enc::Encoder;
use bincode::error::DecodeError;
use bincode::error::EncodeError;
use cu29_value_types::Quantity;
use cu29_value_types::QuantityMetadata;
use cu29_value_types::TimeStorageUnit;
use cu29_value_types::ValueMetadata;
use serde::Deserialize;
use serde::Serialize;

/// Reader-side metadata, preserving future Copper kinds and storage alternatives.
/// Use [`Self::known`] to obtain metadata understood by this reader.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(try_from = "MetadataWire", into = "MetadataWire")]
pub struct ValueDecodeMetadata {
    value: MetadataValue,
}

#[derive(Clone, Debug, PartialEq, Eq)]
enum MetadataValue {
    Known(ValueMetadata),
    Unknown(MetadataWire),
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
struct MetadataWire {
    kind: u32,
    bytes: Vec<u8>,
}

impl From<ValueMetadata> for ValueDecodeMetadata {
    fn from(value: ValueMetadata) -> Self {
        Self {
            value: MetadataValue::Known(value),
        }
    }
}

impl ValueDecodeMetadata {
    /// Metadata understood by this Copper version.
    pub const fn known(&self) -> Option<ValueMetadata> {
        match self.value {
            MetadataValue::Known(value) => Some(value),
            MetadataValue::Unknown(_) => None,
        }
    }

    /// Permanent kind ID, including future kinds unknown to this reader.
    pub const fn kind_id(&self) -> u32 {
        match &self.value {
            MetadataValue::Known(value) => value.kind_id(),
            MetadataValue::Unknown(wire) => wire.kind,
        }
    }

    /// Raw body of an entry this reader could not interpret.
    pub fn unknown_bytes(&self) -> Option<&[u8]> {
        match &self.value {
            MetadataValue::Unknown(wire) => Some(&wire.bytes),
            MetadataValue::Known(_) => None,
        }
    }
}

fn quantity_bytes(quantity: QuantityMetadata) -> [u8; 8] {
    let mut bytes = [0; 8];
    bytes[..4].copy_from_slice(&quantity.quantity().id().to_le_bytes());
    bytes[4..].copy_from_slice(&quantity.storage_unit().id().to_le_bytes());
    bytes
}

impl From<ValueDecodeMetadata> for MetadataWire {
    fn from(entry: ValueDecodeMetadata) -> Self {
        match entry.value {
            MetadataValue::Known(ValueMetadata::Quantity(quantity)) => Self {
                kind: 1,
                bytes: quantity_bytes(quantity).to_vec(),
            },
            MetadataValue::Unknown(wire) => wire,
        }
    }
}

impl TryFrom<MetadataWire> for ValueDecodeMetadata {
    type Error = DecodeError;

    fn try_from(wire: MetadataWire) -> Result<Self, Self::Error> {
        if wire.kind != 1 {
            return Ok(Self {
                value: MetadataValue::Unknown(wire),
            });
        }
        let bytes: [u8; 8] = wire.bytes.as_slice().try_into().map_err(|_| {
            DecodeError::Other("Quantity metadata body must contain exactly eight bytes")
        })?;
        let quantity_id = u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);
        let unit_id = u32::from_le_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]);
        let Some(quantity) = Quantity::from_id(quantity_id) else {
            return Ok(Self {
                value: MetadataValue::Unknown(wire),
            });
        };
        let quantity = match unit_id {
            1 => QuantityMetadata::coherent(quantity),
            2 if quantity == Quantity::Time => QuantityMetadata::time(TimeStorageUnit::Nanosecond),
            2 => {
                return Err(DecodeError::Other(
                    "Nanosecond storage requires a time quantity",
                ));
            }
            _ => {
                return Ok(Self {
                    value: MetadataValue::Unknown(wire),
                });
            }
        };
        Ok(ValueMetadata::Quantity(quantity).into())
    }
}

impl Encode for ValueDecodeMetadata {
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        match &self.value {
            MetadataValue::Known(value) => value.encode(encoder),
            MetadataValue::Unknown(wire) => {
                wire.kind.encode(encoder)?;
                wire.bytes.encode(encoder)
            }
        }
    }
}

impl<Context> Decode<Context> for ValueDecodeMetadata {
    fn decode<D: Decoder<Context = Context>>(decoder: &mut D) -> Result<Self, DecodeError> {
        let kind = u32::decode(decoder)?;
        let length = u64::decode(decoder)?;
        let length = usize::try_from(length).map_err(|_| DecodeError::OutsideUsizeRange(length))?;
        if kind == 1 && length != 8 {
            return Err(DecodeError::Other(
                "Quantity metadata body must contain exactly eight bytes",
            ));
        }
        decoder.claim_container_read::<u8>(length)?;
        // Reserve only bytes successfully read. A corrupt future frame length
        // must not cause a large allocation before its truncated body is detected.
        let mut bytes = Vec::new();
        let mut chunk = [0; 256];
        while bytes.len() < length {
            let count = (length - bytes.len()).min(chunk.len());
            decoder.reader().read(&mut chunk[..count])?;
            bytes.extend_from_slice(&chunk[..count]);
        }
        MetadataWire { kind, bytes }.try_into()
    }
}
bincode::impl_borrow_decode!(ValueDecodeMetadata);

impl ValueDecode for ValueDecodeMetadata {
    const DECODE: &'static ValueDecodeSpec = <(u32, Vec<u8>) as ValueDecode>::DECODE;
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec;
    use bincode::config::Config;

    fn entry_bytes(kind: u32, bytes: Vec<u8>, config: impl Config) -> Vec<u8> {
        bincode::encode_to_vec((kind, bytes), config).unwrap()
    }

    fn check_frames(config: impl Config) {
        let known = ValueDecodeMetadata::from(ValueMetadata::Quantity(QuantityMetadata::time(
            TimeStorageUnit::Nanosecond,
        )));
        let future = entry_bytes(77, vec![0, 1, 2, 3, 4], config);
        let mut bytes = future.clone();
        bytes.extend(bincode::encode_to_vec(&known, config).unwrap());
        let (unknown, used): (ValueDecodeMetadata, _) =
            bincode::decode_from_slice(&bytes, config).unwrap();
        assert_eq!(used, future.len());
        assert_eq!(unknown.kind_id(), 77);
        assert_eq!(unknown.unknown_bytes(), Some([0, 1, 2, 3, 4].as_slice()));
        assert_eq!(bincode::encode_to_vec(&unknown, config).unwrap(), future);
        let (decoded, consumed): (ValueDecodeMetadata, _) =
            bincode::decode_from_slice(&bytes[used..], config).unwrap();
        assert_eq!(decoded, known);
        assert_eq!(consumed + used, bytes.len());
        for body in [
            [u32::MAX.to_le_bytes(), 1u32.to_le_bytes()].concat(),
            [Quantity::Time.id().to_le_bytes(), u32::MAX.to_le_bytes()].concat(),
        ] {
            let wire = entry_bytes(1, body, config);
            let (entry, used): (ValueDecodeMetadata, _) =
                bincode::decode_from_slice(&wire, config).unwrap();
            assert_eq!(used, wire.len());
            assert!(entry.known().is_none());
            assert_eq!(bincode::encode_to_vec(entry, config).unwrap(), wire);
        }
        assert_eq!(
            bincode::encode_to_vec(&known, config).unwrap(),
            entry_bytes(
                1,
                [Quantity::Time.id().to_le_bytes(), 2u32.to_le_bytes()].concat(),
                config
            )
        );
    }

    #[test]
    fn test_metadata_frames_under_every_codec_configuration() {
        let config = bincode::config::standard();
        check_frames(config);
        check_frames(config.with_big_endian());
        check_frames(config.with_fixed_int_encoding());
        check_frames(config.with_big_endian().with_fixed_int_encoding());
    }

    #[test]
    fn test_malformed_known_metadata_and_truncated_frames() {
        let config = bincode::config::standard();
        for body in [
            vec![],
            vec![0; 7],
            vec![0; 9],
            [Quantity::Length.id().to_le_bytes(), 2u32.to_le_bytes()].concat(),
        ] {
            let bytes = entry_bytes(1, body, config);
            assert!(bincode::decode_from_slice::<ValueDecodeMetadata, _>(&bytes, config).is_err());
        }
        for kind in [1, 77] {
            let oversized = bincode::encode_to_vec((kind, u64::MAX), config).unwrap();
            assert!(
                bincode::decode_from_slice::<ValueDecodeMetadata, _>(&oversized, config).is_err()
            );
            let bytes = entry_bytes(kind, vec![0; 8], config);
            for len in 0..bytes.len() {
                assert!(
                    bincode::decode_from_slice::<ValueDecodeMetadata, _>(&bytes[..len], config)
                        .is_err()
                );
            }
        }
    }

    #[test]
    fn test_large_future_body_is_preserved() {
        let config = bincode::config::standard();
        let bytes = entry_bytes(77, vec![42; 1025], config);
        let (entry, used): (ValueDecodeMetadata, _) =
            bincode::decode_from_slice(&bytes, config).unwrap();
        assert_eq!(used, bytes.len());
        assert_eq!(entry.unknown_bytes(), Some(vec![42; 1025].as_slice()));
        assert_eq!(bincode::encode_to_vec(entry, config).unwrap(), bytes);
    }

    #[test]
    fn test_metadata_decoding_honors_byte_limits() {
        let bytes = entry_bytes(77, vec![0; 128], bincode::config::standard());
        assert!(
            bincode::decode_from_slice::<ValueDecodeMetadata, _>(
                &bytes,
                bincode::config::standard().with_limit::<32>(),
            )
            .is_err()
        );
    }

    #[test]
    fn test_every_quantity_and_storage_alternative_round_trips() {
        let config = bincode::config::standard();
        for quantity in Quantity::ALL {
            let entry = ValueDecodeMetadata::from(ValueMetadata::Quantity(
                QuantityMetadata::coherent(*quantity),
            ));
            let bytes = bincode::encode_to_vec(&entry, config).unwrap();
            let (decoded, used): (ValueDecodeMetadata, _) =
                bincode::decode_from_slice(&bytes, config).unwrap();
            assert_eq!(decoded, entry);
            assert_eq!(used, bytes.len());
        }
        let entry = ValueDecodeMetadata::from(ValueMetadata::Quantity(QuantityMetadata::time(
            TimeStorageUnit::Nanosecond,
        )));
        let bytes = bincode::encode_to_vec(&entry, config).unwrap();
        let (decoded, _): (ValueDecodeMetadata, _) =
            bincode::decode_from_slice(&bytes, config).unwrap();
        assert_eq!(decoded, entry);
    }
    #[test]
    fn test_serde_preserves_recognized_and_future_metadata() {
        let config = bincode::config::standard();
        let known = ValueDecodeMetadata::from(ValueMetadata::Quantity(QuantityMetadata::coherent(
            Quantity::Velocity,
        )));
        let wire = entry_bytes(99, vec![3, 1, 4], config);
        let (unknown, _): (ValueDecodeMetadata, _) =
            bincode::decode_from_slice(&wire, config).unwrap();
        for entry in [known, unknown] {
            let tree = crate::to_value(&entry).unwrap();
            let decoded: ValueDecodeMetadata = tree.deserialize_into().unwrap();
            assert_eq!(decoded, entry);
        }
    }
}
