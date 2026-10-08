//! Version, bounds and native envelope layout for startup catalogs.

use bincode::Decode;
use bincode::Encode;
use serde::{Deserialize, Serialize};

pub(crate) const VERSION: u16 = 1;
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
