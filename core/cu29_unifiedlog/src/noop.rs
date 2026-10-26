use crate::{
    SECTION_HEADER_COMPACT_SIZE, SECTION_MAGIC, SectionHandle, SectionHeader, SectionStorage,
    UnifiedLogStatus, UnifiedLogWrite,
};
use bincode::Encode;
use bincode::config::standard;
use bincode::enc::EncoderImpl;
use bincode::enc::write::SizeWriter;
use bincode::error::EncodeError;
use cu29_traits::{
    CuResult, ObservedWriter, UnifiedLogType, abort_observed_encode, begin_observed_encode,
    finish_observed_encode,
};

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
        // Virtual sections must still fit LogStream's u32 used-byte counter.
        // Report full before advancing so the stream can rotate and retry.
        self.bytes_written = self
            .bytes_written
            .checked_add(size)
            .filter(|&bytes| bytes <= u32::MAX as usize)
            .ok_or(EncodeError::UnexpectedEnd)?;
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
            allocated: allocated_size as u64,
            next_section: 0,
            context: crate::SectionContext::default(),
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

    fn seal_metadata<C: Encode>(
        &mut self,
        _: &crate::ApplicationMetadata,
        _: Option<&C>,
    ) -> CuResult<()> {
        Ok(())
    }

    fn status(&self) -> UnifiedLogStatus {
        UnifiedLogStatus {
            total_used_space: self.total_used_space,
            total_allocated_space: self.total_allocated_space,
        }
    }
}

fn encoded_size<E: Encode>(value: &E) -> Result<usize, EncodeError> {
    begin_observed_encode();
    let result = (|| {
        let mut encoder =
            EncoderImpl::<_, _>::new(ObservedWriter::new(SizeWriter::default()), standard());
        value.encode(&mut encoder)?;
        Ok(encoder.into_writer().into_inner().bytes_written)
    })();
    match result {
        Ok(size) => {
            debug_assert_eq!(size, finish_observed_encode());
            Ok(size)
        }
        Err(err) => {
            abort_observed_encode();
            Err(err)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Arc, LogStream, Mutex};
    use cu29_traits::WriteStream;

    #[test]
    fn test_noop_section_rejects_overflow_without_advancing() {
        let mut storage = NoopSectionStorage {
            bytes_written: u32::MAX as usize - 1,
        };
        assert_eq!(storage.append(&0_u8).unwrap(), 1);
        assert!(matches!(
            storage.append(&0_u8),
            Err(EncodeError::UnexpectedEnd)
        ));
        assert_eq!(storage.bytes_written, u32::MAX as usize);
    }

    #[test]
    fn test_noop_stream_rotates_at_section_counter_limit() {
        let logger = Arc::new(Mutex::new(NoopLogger::new()));
        let mut stream = LogStream::new(UnifiedLogType::CopperList, logger.clone(), 4096).unwrap();
        // Seed the boundary instead of allocating or encoding four GiB of data.
        stream.current_section.header.used = u32::MAX - 2;
        stream.current_section.storage.bytes_written = u32::MAX as usize - 2;
        stream.log(&[0_u8; 2]).unwrap();
        assert_eq!(stream.current_section.header.used, u32::MAX);
        stream.log(&[0_u8; 2]).unwrap();
        assert_eq!(stream.current_section.header.used, 2);
        assert_eq!(WriteStream::<[u8; 2]>::last_log_bytes(&stream), Some(2));
        #[cfg(feature = "std")]
        let status = logger.lock().unwrap().status();
        #[cfg(not(feature = "std"))]
        let status = logger.lock().status();
        assert_eq!(status.total_used_space, u32::MAX as usize);
        assert_eq!(status.total_allocated_space, 8192);
    }
}
