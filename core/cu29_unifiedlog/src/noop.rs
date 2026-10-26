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
    capacity: usize,
}

impl SectionStorage for NoopSectionStorage {
    fn initialize<E: Encode>(&mut self, _: &E) -> Result<usize, EncodeError> {
        let header_size = SECTION_HEADER_COMPACT_SIZE as usize;
        self.bytes_written = header_size;
        Ok(header_size)
    }

    fn post_update_header<E: Encode>(&mut self, _: &E) -> Result<usize, EncodeError> {
        Ok(SECTION_HEADER_COMPACT_SIZE as usize)
    }

    fn append<E: Encode>(&mut self, entry: &E) -> Result<usize, EncodeError> {
        let size = encoded_size(entry)?;
        // Report full without advancing so the stream can rotate and retry.
        self.bytes_written = self
            .bytes_written
            .checked_add(size)
            .filter(|&bytes| bytes <= self.capacity)
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
        if allocated_size as u64 - SECTION_HEADER_COMPACT_SIZE as u64 > u32::MAX as u64 {
            return Err("Invalid section allocation size".into());
        }
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
        SectionHandle::create(
            section_header,
            NoopSectionStorage {
                bytes_written: 0,
                capacity: allocated_size,
            },
        )
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
    use alloc::string::ToString;

    use crate::{Arc, LogStream, Mutex};
    use cu29_traits::WriteStream;

    // no_std observed-encode accounting is process-wide, not thread-local.
    #[cfg(not(feature = "std"))]
    static ENCODE_LOCK: spin::Mutex<()> = spin::Mutex::new(());

    #[test]
    fn test_noop_section_respects_capacity() {
        #[cfg(not(feature = "std"))]
        let _guard = ENCODE_LOCK.lock();
        let mut logger = NoopLogger::new();
        let mut section = logger
            .add_section(
                UnifiedLogType::CopperList,
                SECTION_HEADER_COMPACT_SIZE as usize + 2,
            )
            .unwrap();
        assert_eq!(section.append([0_u8; 2]).unwrap(), 2);
        assert!(matches!(
            section.append(0_u8),
            Err(EncodeError::UnexpectedEnd)
        ));
        assert_eq!(
            section.get_storage_mut().flush().unwrap(),
            SECTION_HEADER_COMPACT_SIZE as usize + 2
        );
    }

    #[test]
    fn test_noop_stream_rotates_at_requested_capacity() {
        #[cfg(not(feature = "std"))]
        let _guard = ENCODE_LOCK.lock();
        let logger = Arc::new(Mutex::new(NoopLogger::new()));
        let capacity = SECTION_HEADER_COMPACT_SIZE as usize + 2;
        let mut stream =
            LogStream::new(UnifiedLogType::CopperList, logger.clone(), capacity).unwrap();
        stream.log(&[0_u8; 2]).unwrap();
        stream.log(&[0_u8; 2]).unwrap();
        assert_eq!(stream.current_section.header.used, 2);
        assert_eq!(WriteStream::<[u8; 2]>::last_log_bytes(&stream), Some(2));
        drop(stream);
        #[cfg(feature = "std")]
        let status = logger.lock().unwrap().status();
        #[cfg(not(feature = "std"))]
        let status = logger.lock().status();
        assert_eq!(status.total_used_space, 2 * capacity);
        assert_eq!(status.total_allocated_space, 2 * capacity);
    }

    #[test]
    fn test_noop_stream_rejects_entry_larger_than_empty_section() {
        #[cfg(not(feature = "std"))]
        let _guard = ENCODE_LOCK.lock();
        let logger = Arc::new(Mutex::new(NoopLogger::new()));
        let mut stream = LogStream::new(
            UnifiedLogType::CopperList,
            logger,
            SECTION_HEADER_COMPACT_SIZE as usize + 1,
        )
        .unwrap();
        assert!(
            stream
                .log(&[0_u8; 2])
                .unwrap_err()
                .to_string()
                .contains("Entry exceeds an empty section")
        );
        assert_eq!(stream.current_section.header.used, 0);
        stream.log(&0_u8).unwrap();
    }

    #[test]
    #[cfg(target_pointer_width = "64")]
    fn test_noop_section_rejects_unrepresentable_payload_capacity() {
        let mut logger = NoopLogger::new();
        let largest = SECTION_HEADER_COMPACT_SIZE as usize + u32::MAX as usize;
        assert!(
            logger
                .add_section(UnifiedLogType::CopperList, largest)
                .is_ok()
        );
        assert!(
            logger
                .add_section(UnifiedLogType::CopperList, largest + 1)
                .is_err()
        );
        assert_eq!(logger.status().total_allocated_space, largest);
    }
}
