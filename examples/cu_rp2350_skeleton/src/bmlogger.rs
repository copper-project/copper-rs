use bincode::config::standard;
use bincode::encode_into_slice;
use cu29::prelude::{
    SectionHandle, SectionHeader, UnifiedLogStatus, UnifiedLogWrite, SECTION_HEADER_COMPACT_SIZE,
    SECTION_MAGIC,
};
use cu29::UnifiedLogType;
use embedded_sdmmc::{Block, BlockDevice};

pub struct EMMCLogger<BD>
where
    BD: BlockDevice,
{
    bd: BD,
}

impl<BD> EMMCLogger<BD>
where
    BD: BlockDevice,
{
    /// Create a new logger.
    /// takes a block device like embedded_sdmmc::SdCard
    pub fn new(bd: BD) -> Self {
        EMMCLogger { bd }
    }
}

impl<BD> UnifiedLogWrite for EMMCLogger<BD>
where
    BD: BlockDevice + Send + Sync,
{
    fn add_section(
        &mut self,
        entry_type: UnifiedLogType,
        requested_section_size: usize,
    ) -> SectionHandle {
        let section_header = SectionHeader {
            magic: SECTION_MAGIC,
            block_size: SECTION_HEADER_COMPACT_SIZE,
            entry_type,
            offset_to_next_section: requested_section_size as u32,
            used: 0u32,
        };
        let mut section_header_blk = Block::new();

        encode_into_slice(&section_header, section_header_blk.as_mut(), standard())
            .expect("Failed to encode section header");

        SectionHandle::create(section_header, slice_static)
    }

    fn flush_section(&mut self, _section: &mut SectionHandle) {
        // no op for now
    }

    fn status(&self) -> UnifiedLogStatus {
        // no op for now
        UnifiedLogStatus {
            total_used_space: 0,
            total_allocated_space: 0,
        }
    }
}
