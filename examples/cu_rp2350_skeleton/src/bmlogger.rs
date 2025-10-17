use bincode::config::standard;
use bincode::{encode_into_slice, Encode};
use bincode::error::EncodeError;
use cu29::prelude::{SectionHandle, SectionHeader, SectionStorage, UnifiedLogStatus, UnifiedLogWrite, SECTION_HEADER_COMPACT_SIZE, SECTION_MAGIC};
use cu29::{CuResult, UnifiedLogType};
use embedded_sdmmc::{Block, BlockDevice};

pub struct EMMCSectionStorage<'a, BD>  where BD: BlockDevice {
    bd: &'a BD,
}

// The BlockDevice should be safe.
unsafe impl<BD>  Sync for EMMCSectionStorage<'_,  BD>  where BD: BlockDevice{}
unsafe impl<BD>  Send for EMMCSectionStorage<'_,  BD>  where BD: BlockDevice{}

impl<'a, BD> EMMCSectionStorage<'a,  BD> where BD: BlockDevice {
    fn new(bd: &'a BD) -> Self {
        EMMCSectionStorage {
            bd
        }
    }
}

impl<'a, BD> SectionStorage for EMMCSectionStorage<'a,  BD> where BD: BlockDevice {
    fn initialize<E: Encode>(&mut self, header: &E) -> Result<usize, EncodeError> {
        todo!()
    }

    fn post_update_header<E: Encode>(&mut self, header: &E) -> Result<usize, EncodeError> {
        todo!()
    }

    fn append<E: Encode>(&mut self, entry: &E) -> Result<usize, EncodeError> {
        todo!()
    }

    fn flush(&mut self) -> CuResult<usize> {
        todo!()
    }
}

pub struct EMMCLogger<BD>
where
    BD: BlockDevice,
{
    bd: BD,
}

unsafe impl<BD> Sync for EMMCLogger<BD> where BD: BlockDevice {}
unsafe impl<BD> Send for EMMCLogger<BD> where BD: BlockDevice {}

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

impl<'a, BD> UnifiedLogWrite<EMMCSectionStorage<'a, BD>> for EMMCLogger<BD>
where
    BD: BlockDevice + Send + Sync,
{
    fn add_section(
        &mut self,
        entry_type: UnifiedLogType,
        requested_section_size: usize,
    ) -> SectionHandle<EMMCSectionStorage<'a, BD>> {
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

        let section_storage = EMMCSectionStorage::new(&self.bd);

        SectionHandle::create(section_header, section_storage).unwrap()
    }

    fn flush_section(&mut self, _section: &mut SectionHandle<EMMCSectionStorage<'a, BD>>) {
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
