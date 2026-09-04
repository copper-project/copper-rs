use crate::{Error, Result};
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

/// Decodes one application-typed full or partial CopperList.
pub fn decode_copperlist<P: CopperListTuple>(payload: &[u8]) -> Result<CopperList<P>> {
    let (copperlist, consumed) = bincode::decode_from_slice(payload, bincode::config::standard())
        .map_err(|error| Error::Codec(error.to_string()))?;
    if consumed != payload.len() {
        return Err(Error::Codec("CopperList has trailing bytes".into()));
    }
    Ok(copperlist)
}
