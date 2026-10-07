//! Fixed-memory catalog encoding through the static metadata interface.

use bincode::{Encode, enc::Encoder, error::EncodeError};
use cu29_value::catalog_stream::{CatalogDescription, write_catalog};

/// Borrow a generated description and stream its compressed bytes to the logger.
#[doc(hidden)]
pub struct CompressedCatalog<'a>(pub &'a CatalogDescription);
impl Encode for CompressedCatalog<'_> {
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        write_catalog(encoder.writer(), self.0)
    }
}
