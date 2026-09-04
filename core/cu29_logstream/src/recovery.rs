//! Semantic keyframe and anchor records used to restart exact reconstruction.

use crate::{DecodedRecord, Error, RecordKind, Result, decode_record, encode_record};
use alloc::{string::ToString, vec::Vec};
use bincode::{Decode, Encode};
use cu29_runtime::curuntime::KeyFrame;

/// A verified restart point for one sender.
///
/// The keyframe restores component state immediately before `copperlist_id`.
/// The manifest supplies the continuous-lane FEC and schema configuration.
#[derive(Clone, Debug, PartialEq, Eq, Encode, Decode)]
pub struct Anchor {
    pub manifest_object_id: u64,
    pub manifest_record_digest: [u8; 32],
    pub copperlist_id: u64,
    pub keyframe_object_id: u64,
    pub keyframe_record_digest: [u8; 32],
}

impl Anchor {
    /// Checks that a recovered keyframe is exactly the record referenced here.
    pub fn references_keyframe(&self, record: DecodedRecord<'_>) -> bool {
        record.kind == RecordKind::KeyFrame
            && record.object_id == self.keyframe_object_id
            && record.digest == self.keyframe_record_digest
    }

    /// Checks that a recovered manifest is exactly the record referenced here.
    pub fn references_manifest(&self, record: DecodedRecord<'_>) -> bool {
        record.kind == RecordKind::Manifest
            && record.object_id == self.manifest_object_id
            && record.digest == self.manifest_record_digest
    }
}

/// Encodes a runtime keyframe as its canonical bincode semantic payload.
pub fn encode_keyframe(keyframe: &KeyFrame) -> Result<Vec<u8>> {
    bincode::encode_to_vec(keyframe, bincode::config::standard())
        .map_err(|error| Error::Codec(error.to_string()))
}

/// Decodes a complete keyframe payload and rejects trailing bytes.
pub fn decode_keyframe(payload: &[u8]) -> Result<KeyFrame> {
    decode_exact(payload, "keyframe")
}

/// Encodes an anchor as its canonical bincode semantic payload.
pub fn encode_anchor(anchor: &Anchor) -> Result<Vec<u8>> {
    bincode::encode_to_vec(anchor, bincode::config::standard())
        .map_err(|error| Error::Codec(error.to_string()))
}

/// Decodes a complete anchor payload and rejects trailing bytes.
pub fn decode_anchor(payload: &[u8]) -> Result<Anchor> {
    decode_exact(payload, "anchor")
}

/// Creates the separately transportable keyframe and anchor records.
///
/// Both use the CopperList boundary as their family-local object identifier.
pub fn encode_keyframe_and_anchor(
    keyframe: &KeyFrame,
    manifest_object_id: u64,
    manifest_record_digest: [u8; 32],
) -> Result<(Vec<u8>, Vec<u8>)> {
    let keyframe_payload = encode_keyframe(keyframe)?;
    let keyframe_record =
        encode_record(RecordKind::KeyFrame, keyframe.culistid, &keyframe_payload)?;
    let keyframe_decoded = decode_record(&keyframe_record)?;
    let anchor = Anchor {
        manifest_object_id,
        manifest_record_digest,
        copperlist_id: keyframe.culistid,
        keyframe_object_id: keyframe.culistid,
        keyframe_record_digest: keyframe_decoded.digest,
    };
    let anchor_payload = encode_anchor(&anchor)?;
    let anchor_record = encode_record(RecordKind::Anchor, keyframe.culistid, &anchor_payload)?;
    Ok((keyframe_record, anchor_record))
}

fn decode_exact<T: Decode<()>>(payload: &[u8], name: &'static str) -> Result<T> {
    let (value, consumed) = bincode::decode_from_slice(payload, bincode::config::standard())
        .map_err(|error| Error::Codec(error.to_string()))?;
    if consumed != payload.len() {
        return Err(Error::Codec(alloc::format!("{name} has trailing bytes")));
    }
    Ok(value)
}
