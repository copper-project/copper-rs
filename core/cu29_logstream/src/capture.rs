//! Generated selective streaming codecs. Encoding and hashing run on output workers.

use crate::{ApplicationSchema, Error, Result};
use alloc::{string::ToString, vec::Vec};
use bincode::{
    Decode, Encode,
    enc::{Encoder, EncoderImpl, write::Writer},
    error::EncodeError,
};
use cu29_runtime::copperlist::CopperList;
use cu29_traits::CopperListTuple;

/// One proof per output slot, including intentionally absent messages.
#[derive(Debug, Clone, PartialEq, Eq, Encode, Decode)]
pub struct CaptureProof {
    pub version: u16,
    pub original_presence: Vec<bool>,
    pub reconstructed_digests: Vec<Option<[u8; 32]>>,
}

/// Generated from the application's RON. Users do not implement slot dispatch.
pub trait CaptureDataSet: CopperListTuple {
    const RECONSTRUCTION: &'static [Option<u32>];
    fn original_presence(&self) -> Vec<bool>;
    fn stream_schema() -> ApplicationSchema;
    fn encode_capture<E: Encoder>(&self, encoder: &mut E) -> core::result::Result<(), EncodeError>;
    fn capture_proof(&self) -> Result<CaptureProof>;
    fn restore_sender_metadata(&mut self, captured: &Self);
}

pub struct CaptureView<'a, P>(pub &'a P);
impl<P: CaptureDataSet> Encode for CaptureView<'_, P> {
    fn encode<E: Encoder>(&self, encoder: &mut E) -> core::result::Result<(), EncodeError> {
        self.0.encode_capture(encoder)
    }
}

struct DigestWriter(blake3::Hasher);
impl Writer for DigestWriter {
    fn write(&mut self, bytes: &[u8]) -> core::result::Result<(), EncodeError> {
        self.0.update(bytes);
        Ok(())
    }
}

/// Hash a canonical payload without allocating an encoded payload buffer.
pub fn payload_digest(value: &impl Encode) -> Result<[u8; 32]> {
    let mut writer = DigestWriter(blake3::Hasher::new());
    value
        .encode(&mut EncoderImpl::new(
            &mut writer,
            bincode::config::standard(),
        ))
        .map_err(|e| Error::Codec(e.to_string()))?;
    Ok(*writer.0.finalize().as_bytes())
}

/// A captured native list and its explicit omission proof. Never a verified twin frame.
pub struct CapturedList<P: CopperListTuple> {
    pub copperlist: CopperList<P>,
    pub proof: CaptureProof,
}

pub fn encode_capture_record_into<P: CaptureDataSet>(
    list: &CopperList<P>,
    output: &mut [u8],
) -> Result<usize> {
    let proof = list.msgs.capture_proof()?;
    let header_len = crate::record::RECORD_HEADER_LEN;
    if output.len() < header_len {
        return Err(Error::BufferTooSmall {
            needed: header_len,
            available: output.len(),
        });
    }
    let (header, payload) = output.split_at_mut(header_len);
    let len = bincode::encode_into_slice(
        (&proof, list.id, list.get_state(), CaptureView(&list.msgs)),
        payload,
        bincode::config::standard(),
    )
    .map_err(|e| Error::Codec(e.to_string()))?;
    crate::record::encode_record_header(
        crate::RecordKind::CopperList,
        list.id,
        &payload[..len],
        header,
    )?;
    Ok(header_len + len)
}

/// Decode once; return the original native byte slice for capture archival.
pub fn decode_capture<P: CaptureDataSet>(payload: &[u8]) -> Result<(CapturedList<P>, &[u8])> {
    let (proof, offset): (CaptureProof, usize) =
        bincode::decode_from_slice(payload, bincode::config::standard().with_limit::<1048576>())
            .map_err(|e| Error::Codec(e.to_string()))?;
    let output_count = P::get_output_specs().len();
    if proof.version != 1
        || proof.original_presence.len() != output_count
        || proof.reconstructed_digests.len() != output_count
    {
        return Err(Error::InvalidConfig("invalid capture proof layout"));
    }
    let native = &payload[offset..];
    let copperlist = crate::decode_copperlist::<P>(native)?;
    let captured_presence = copperlist.msgs.original_presence();
    for (i, present) in captured_presence.iter().enumerate() {
        let reconstruct = P::RECONSTRUCTION.get(i).is_some_and(Option::is_some);
        if *present != (proof.original_presence[i] && !reconstruct)
            || (proof.reconstructed_digests[i].is_some() && !reconstruct)
        {
            return Err(Error::InvalidConfig(
                "capture payload/presence policy mismatch",
            ));
        }
    }
    Ok((CapturedList { copperlist, proof }, native))
}

impl<P: CaptureDataSet> CapturedList<P> {
    pub fn verify(&self) -> Result<()> {
        let actual = self.copperlist.msgs.capture_proof()?;
        if actual.original_presence != self.proof.original_presence
            || self
                .proof
                .reconstructed_digests
                .iter()
                .zip(&actual.reconstructed_digests)
                .any(|(expected, actual)| expected.is_some() && expected != actual)
        {
            return Err(Error::InvalidConfig(
                "deterministic reconstruction diverged",
            ));
        }
        Ok(())
    }
}

/// Optional developer checks. The production build does not encode or hash a reconstructed payload.
pub fn reconstruction_digest(value: &impl Encode) -> Result<Option<[u8; 32]>> {
    #[cfg(feature = "verify-reconstruction")]
    {
        payload_digest(value).map(Some)
    }
    #[cfg(not(feature = "verify-reconstruction"))]
    {
        let _ = value;
        Ok(None)
    }
}
