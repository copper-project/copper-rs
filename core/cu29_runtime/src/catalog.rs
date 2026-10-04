//! Startup recording of generated static catalog descriptions.

use alloc::string::ToString;
use alloc::sync::Arc;
use bincode::Encode;
use bincode::enc::Encoder;
use bincode::enc::write::Writer;
use bincode::error::EncodeError;
use cu29_traits::{CuError, CuResult, UnifiedLogType, WriteStream};
use cu29_unifiedlog::{LogStream, SectionStorage, UnifiedLogWrite};
use cu29_value::catalog_header::CATALOG_CHUNK_MAGIC;
use cu29_value::catalog_stream::{CatalogDescription, write_catalog};
#[cfg(not(feature = "std"))]
use spin::Mutex;
#[cfg(feature = "std")]
use std::sync::Mutex;

struct RawChunk<'a> {
    sequence: u32,
    bytes: &'a [u8],
}
impl Encode for RawChunk<'_> {
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        encoder.writer().write(CATALOG_CHUNK_MAGIC)?;
        encoder.writer().write(&self.sequence.to_le_bytes())?;
        encoder
            .writer()
            .write(&(self.bytes.len() as u16).to_le_bytes())?;
        encoder.writer().write(self.bytes)
    }
}

struct CatalogWriter<'a, S: SectionStorage, L: UnifiedLogWrite<S>> {
    stream: &'a mut LogStream<S, L>,
    sequence: u32,
}
impl<S: SectionStorage, L: UnifiedLogWrite<S>> Writer for CatalogWriter<'_, S, L> {
    fn write(&mut self, bytes: &[u8]) -> Result<(), EncodeError> {
        // Compression emits at most 128 bytes at a time. Header/footer writes are
        // smaller; every entry can be retried intact in the next log section.
        if !bytes.is_empty() {
            self.stream
                .log(&RawChunk {
                    sequence: self.sequence,
                    bytes,
                })
                .map_err(|_| EncodeError::Other("Could not write catalog chunk"))?;
            self.sequence += 1;
        }
        Ok(())
    }
}

/// Stream a generated catalog before resources and runtime streams are initialized.
pub fn record_value_decode_catalog<S: SectionStorage, L: UnifiedLogWrite<S>>(
    logger: Arc<Mutex<L>>,
    catalog: &CatalogDescription,
) -> CuResult<()> {
    let mut stream = LogStream::new(UnifiedLogType::ValueDecodeCatalog, logger, 4096)?;
    write_catalog(
        CatalogWriter {
            stream: &mut stream,
            sequence: 0,
        },
        catalog,
    )
    .map_err(|error| {
        CuError::from("Could not record payload catalog").add_cause(&error.to_string())
    })
}
