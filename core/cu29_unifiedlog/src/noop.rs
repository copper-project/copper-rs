use crate::{
    SECTION_HEADER_COMPACT_SIZE, SECTION_MAGIC, SectionHandle, SectionHeader, SectionStorage,
    UnifiedLogStatus, UnifiedLogWrite,
};
use bincode::Encode;
use bincode::config::standard;
use bincode::enc::EncoderImpl;
use bincode::enc::write::SizeWriter;
use bincode::error::EncodeError;
use cu29_traits::{CuResult, UnifiedLogType};

#[derive(Debug, Default)]
pub struct NoopSectionStorage {
    bytes_written: usize,
}

impl SectionStorage for NoopSectionStorage {
    fn initialize<E: Encode>(&mut self, header: &E) -> Result<usize, EncodeError> {
        let header_size = encoded_size(header)?;
        self.bytes_written = header_size;
        Ok(header_size)
    }

    fn post_update_header<E: Encode>(&mut self, header: &E) -> Result<usize, EncodeError> {
        encoded_size(header)
    }

    fn append<E: Encode>(&mut self, entry: &E) -> Result<usize, EncodeError> {
        let size = encoded_size(entry)?;
        self.bytes_written = self.bytes_written.saturating_add(size);
        Ok(size)
    }

    fn flush(&mut self) -> CuResult<usize> {
        Ok(self.bytes_written)
    }
}

#[derive(Debug, Default)]
pub struct NoopLogger {
    total_used_space: usize,
    total_allocated_space: usize,
}

impl NoopLogger {
    pub fn new() -> Self {
        Self::default()
    }
}

impl UnifiedLogWrite<NoopSectionStorage> for NoopLogger {
    fn add_section(
        &mut self,
        entry_type: UnifiedLogType,
        requested_section_size: usize,
    ) -> CuResult<SectionHandle<NoopSectionStorage>> {
        let allocated_size = requested_section_size.max(SECTION_HEADER_COMPACT_SIZE as usize);
        let section_header = SectionHeader {
            magic: SECTION_MAGIC,
            block_size: SECTION_HEADER_COMPACT_SIZE,
            entry_type,
            offset_to_next_section: allocated_size.min(u32::MAX as usize) as u32,
            used: 0,
            is_open: true,
        };

        self.total_allocated_space = self.total_allocated_space.saturating_add(allocated_size);
        SectionHandle::create(section_header, NoopSectionStorage::default())
    }

    fn flush_section(&mut self, section: &mut SectionHandle<NoopSectionStorage>) {
        section.mark_closed();
        let _ = section.post_update_header();
        if let Ok(used) = section.get_storage_mut().flush() {
            self.total_used_space = self.total_used_space.saturating_add(used);
        }
    }

    fn status(&self) -> UnifiedLogStatus {
        UnifiedLogStatus {
            total_used_space: self.total_used_space,
            total_allocated_space: self.total_allocated_space,
        }
    }
}

fn encoded_size<E: Encode>(value: &E) -> Result<usize, EncodeError> {
    let mut encoder = EncoderImpl::<_, _>::new(SizeWriter::default(), standard());
    value.encode(&mut encoder)?;
    Ok(encoder.into_writer().bytes_written)
}
