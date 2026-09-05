//! Generated selective native codecs. Optional verification runs on output workers.

use crate::{ApplicationSchema, Error, Result};
use alloc::string::ToString;
use bincode::{Encode, enc::Encoder, error::EncodeError};
use cu29_runtime::copperlist::CopperList;
use cu29_traits::CopperListTuple;

// Gate generated methods using this crate's feature, not a downstream app's
// feature names or the proc-macro host's independently resolved features.
#[doc(hidden)]
#[cfg(feature = "verify-reconstruction")]
#[macro_export]
macro_rules! __with_reconstruction_verification {
    ($($item:item)*) => { $($item)* };
}
#[doc(hidden)]
#[cfg(not(feature = "verify-reconstruction"))]
#[macro_export]
macro_rules! __with_reconstruction_verification {
    ($($item:item)*) => {};
}

/// Generated from RON. Slot dispatch and storage are fixed at compile time.
pub trait CaptureDataSet: CopperListTuple {
    const RECONSTRUCTION: &'static [Option<u32>];
    fn stream_schema() -> ApplicationSchema;
    fn encode_capture<E: Encoder>(&self, encoder: &mut E) -> core::result::Result<(), EncodeError>;
    fn validate_capture(&self) -> Result<()>;
    fn restore_sender_metadata(&mut self, captured: &Self);
    #[cfg(feature = "verify-reconstruction")]
    fn encode_reconstruction<E: Encoder>(
        &self,
        encoder: &mut E,
    ) -> core::result::Result<(), EncodeError>;
    #[cfg(feature = "verify-reconstruction")]
    fn reconstruction_digest(&self) -> Result<[u8; 32]> {
        payload_digest(&ReconstructionView(self))
    }
}

pub struct CaptureView<'a, P>(pub &'a P);
impl<P: CaptureDataSet> Encode for CaptureView<'_, P> {
    fn encode<E: Encoder>(&self, encoder: &mut E) -> core::result::Result<(), EncodeError> {
        self.0.encode_capture(encoder)
    }
}

#[cfg(feature = "verify-reconstruction")]
struct ReconstructionView<'a, P>(&'a P);
#[cfg(feature = "verify-reconstruction")]
impl<P: CaptureDataSet> Encode for ReconstructionView<'_, P> {
    fn encode<E: Encoder>(&self, encoder: &mut E) -> core::result::Result<(), EncodeError> {
        self.0.encode_reconstruction(encoder)
    }
}

#[cfg(feature = "verify-reconstruction")]
struct DigestWriter(blake3::Hasher);
#[cfg(feature = "verify-reconstruction")]
impl bincode::enc::write::Writer for DigestWriter {
    fn write(&mut self, bytes: &[u8]) -> core::result::Result<(), EncodeError> {
        self.0.update(bytes);
        Ok(())
    }
}

/// Debug-only digest of the reconstructed outputs, including payload presence.
/// Hashes directly from borrowed payloads without an intermediate allocation.
#[cfg(feature = "verify-reconstruction")]
pub fn payload_digest(value: &impl Encode) -> Result<[u8; 32]> {
    let mut writer = DigestWriter(blake3::Hasher::new());
    value
        .encode(&mut bincode::enc::EncoderImpl::new(
            &mut writer,
            bincode::config::standard(),
        ))
        .map_err(|e| Error::Codec(e.to_string()))?;
    Ok(*writer.0.finalize().as_bytes())
}

/// A native capture. Production stores only the CopperList itself.
pub struct CapturedList<P: CopperListTuple> {
    pub copperlist: CopperList<P>,
    #[cfg(feature = "verify-reconstruction")]
    pub digest: Option<[u8; 32]>,
}

impl<P: CopperListTuple> CapturedList<P> {
    /// Wrap an ordinary capture archive entry. Archives contain no debug digests.
    pub fn new(copperlist: CopperList<P>) -> Self {
        Self {
            copperlist,
            #[cfg(feature = "verify-reconstruction")]
            digest: None,
        }
    }
}

pub fn encode_capture_record_into<P: CaptureDataSet>(
    list: &CopperList<P>,
    output: &mut [u8],
) -> Result<usize> {
    let header_len = crate::record::RECORD_HEADER_LEN;
    if output.len() < header_len {
        return Err(Error::BufferTooSmall {
            needed: header_len,
            available: output.len(),
        });
    }
    let (header, payload) = output.split_at_mut(header_len);
    let len = bincode::encode_into_slice(
        (list.id, list.get_state(), CaptureView(&list.msgs)),
        payload,
        bincode::config::standard(),
    )
    .map_err(|e| Error::Codec(e.to_string()))?;
    // A developer build appends one fixed-size digest. Production is exactly
    // the existing native CopperList format, including its presence planes.
    #[cfg(feature = "verify-reconstruction")]
    let len = {
        let available = payload.len();
        let trailer = payload
            .get_mut(len..len + 32)
            .ok_or(Error::BufferTooSmall {
                needed: header_len + len + 32,
                available: header_len + available,
            })?;
        trailer.copy_from_slice(&list.msgs.reconstruction_digest()?);
        len + 32
    };
    crate::record::encode_record_header(
        crate::RecordKind::CopperList,
        list.id,
        &payload[..len],
        header,
    )?;
    Ok(header_len + len)
}

/// Decode once and borrow the native bytes for archival. Verification builds
/// accept production captures too; production rejects debug trailers explicitly.
pub fn decode_capture<P: CaptureDataSet>(payload: &[u8]) -> Result<(CapturedList<P>, &[u8])> {
    let (copperlist, consumed): (CopperList<P>, usize) =
        bincode::decode_from_slice(payload, bincode::config::standard())
            .map_err(|e| Error::Codec(e.to_string()))?;
    copperlist.msgs.validate_capture()?;
    #[cfg(feature = "verify-reconstruction")]
    let digest = match &payload[consumed..] {
        [] => None,
        bytes => Some(
            bytes
                .try_into()
                .map_err(|_| Error::InvalidConfig("invalid reconstruction digest trailer"))?,
        ),
    };
    #[cfg(not(feature = "verify-reconstruction"))]
    if consumed != payload.len() {
        return Err(Error::InvalidConfig(
            "capture has trailing bytes; debug captures require verify-reconstruction",
        ));
    }
    Ok((
        CapturedList {
            copperlist,
            #[cfg(feature = "verify-reconstruction")]
            digest,
        },
        &payload[..consumed],
    ))
}

#[cfg(feature = "verify-reconstruction")]
impl<P: CaptureDataSet> CapturedList<P> {
    pub fn verify(&self) -> Result<()> {
        if let Some(expected) = self.digest
            && self.copperlist.msgs.reconstruction_digest()? != expected
        {
            return Err(Error::InvalidConfig(
                "deterministic reconstruction diverged",
            ));
        }
        Ok(())
    }
}
