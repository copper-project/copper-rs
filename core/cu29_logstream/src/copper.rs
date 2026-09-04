use crate::record::{RECORD_HEADER_LEN, encode_record_header};
use crate::{Error, RecordKind, Result};
use alloc::{string::ToString, vec::Vec};
use cu29_runtime::copperlist::CopperList;
use cu29_traits::CopperListTuple;

/// Serializes the generated CopperList view selected by its application codec.
///
/// Generated codecs retain metadata for every slot while omitting payload bytes
/// excluded from the selected view, so the result may be a partial CopperList.
pub fn encode_copperlist<P: CopperListTuple>(copperlist: &CopperList<P>) -> Result<Vec<u8>> {
    bincode::encode_to_vec(copperlist, bincode::config::standard())
        .map_err(|error| Error::Codec(error.to_string()))
}

/// Serializes and frames one CopperList directly into reusable caller-owned storage.
pub fn encode_copperlist_record_into<P: CopperListTuple>(
    copperlist: &CopperList<P>,
    output: &mut [u8],
) -> Result<usize> {
    if output.len() < RECORD_HEADER_LEN {
        return Err(Error::BufferTooSmall {
            needed: RECORD_HEADER_LEN,
            available: output.len(),
        });
    }
    let (header, payload) = output.split_at_mut(RECORD_HEADER_LEN);
    let payload_len = bincode::encode_into_slice(copperlist, payload, bincode::config::standard())
        .map_err(|error| Error::Codec(error.to_string()))?;
    encode_record_header(
        RecordKind::CopperList,
        copperlist.id,
        &payload[..payload_len],
        header,
    )?;
    Ok(RECORD_HEADER_LEN + payload_len)
}

/// Decodes one application-typed full or partial CopperList.
pub fn decode_copperlist<P: CopperListTuple>(payload: &[u8]) -> Result<CopperList<P>> {
    let (copperlist, consumed) = bincode::decode_from_slice(payload, bincode::config::standard())
        .map_err(|error| Error::Codec(error.to_string()))?;
    if consumed != payload.len() {
        return Err(Error::Codec("CopperList has trailing bytes".into()));
    }
    Ok(copperlist)
}
