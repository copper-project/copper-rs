use crate::sdmmc::{Block, BlockCount, BlockDevice, BlockIdx};
use alloc::sync::Arc;
use bincode::Encode;
use cu29::prelude::*;
use cu29_unifiedlog::byte_log::{ByteLogger, ByteRegion, ByteSection, ByteStorage};
use spin::Mutex;

const BLK: usize = 512;

/// Synchronizes access to a shared block device, including concurrent section streams.
pub struct ForceSyncSend<T>(Mutex<T>);
impl<T> ForceSyncSend<T> {
    pub const fn new(inner: T) -> Self {
        Self(Mutex::new(inner))
    }
}
impl<B: BlockDevice> BlockDevice for ForceSyncSend<B> {
    type Error = B::Error;
    #[cfg(all(feature = "eh02", not(feature = "eh1")))]
    fn read(&self, blocks: &mut [Block], start: BlockIdx, reason: &str) -> Result<(), Self::Error> {
        self.0.lock().read(blocks, start, reason)
    }
    #[cfg(feature = "eh1")]
    fn read(&self, blocks: &mut [Block], start: BlockIdx) -> Result<(), Self::Error> {
        self.0.lock().read(blocks, start)
    }
    fn write(&self, blocks: &[Block], start: BlockIdx) -> Result<(), Self::Error> {
        self.0.lock().write(blocks, start)
    }
    fn num_blocks(&self) -> Result<BlockCount, Self::Error> {
        self.0.lock().num_blocks()
    }
}

/// A bounded partition byte range with one fixed read/modify/write block buffer.
pub struct SdRegion<BD: BlockDevice> {
    bd: Arc<ForceSyncSend<BD>>,
    start: u64,
    length: u64,
    pending: Option<u32>,
    buffer: Block,
    dirty: bool,
}
impl<BD: BlockDevice> SdRegion<BD> {
    fn read_block(&self, index: u32, block: &mut Block) -> CuResult<()> {
        #[cfg(all(feature = "eh02", not(feature = "eh1")))]
        let result = self.bd.read(
            core::slice::from_mut(block),
            BlockIdx(index),
            "Copper byte log",
        );
        #[cfg(feature = "eh1")]
        let result = self.bd.read(core::slice::from_mut(block), BlockIdx(index));
        result.map_err(|_| CuError::from("SD block read failed"))
    }
    fn flush_pending(&mut self) -> CuResult<()> {
        if self.dirty {
            self.bd
                .write(
                    core::slice::from_ref(&self.buffer),
                    BlockIdx(
                        self.pending
                            .ok_or_else(|| CuError::from("Missing SD block buffer"))?,
                    ),
                )
                .map_err(|_| CuError::from("SD block write failed"))?;
            self.dirty = false;
        }
        Ok(())
    }
    fn bounds(&self, offset: u64, size: usize) -> CuResult<u64> {
        if offset
            .checked_add(size as u64)
            .is_none_or(|end| end > self.length)
        {
            return Err(CuError::from("SD write exceeds section byte bounds"));
        }
        self.start
            .checked_add(offset)
            .ok_or_else(|| CuError::from("SD byte offset overflow"))
    }
}
impl<BD: BlockDevice + Send> ByteRegion for SdRegion<BD> {
    fn len(&self) -> u64 {
        self.length
    }
    fn read(&self, offset: u64, mut bytes: &mut [u8]) -> CuResult<()> {
        let mut cursor = self.bounds(offset, bytes.len())?;
        let mut block = Block::new();
        while !bytes.is_empty() {
            let index = u32::try_from(cursor / BLK as u64)
                .map_err(|_| CuError::from("SD block index overflow"))?;
            self.read_block(index, &mut block)?;
            let local = (cursor % BLK as u64) as usize;
            let count = bytes.len().min(BLK - local);
            bytes[..count].copy_from_slice(&block.as_ref()[local..local + count]);
            bytes = &mut bytes[count..];
            cursor += count as u64;
        }
        Ok(())
    }
    fn write(&mut self, offset: u64, mut bytes: &[u8]) -> CuResult<()> {
        let mut cursor = self.bounds(offset, bytes.len())?;
        while !bytes.is_empty() {
            let index = u32::try_from(cursor / BLK as u64)
                .map_err(|_| CuError::from("SD block index overflow"))?;
            if self.pending != Some(index) {
                self.flush_pending()?;
                let mut block = Block::new();
                self.read_block(index, &mut block)?;
                self.buffer = block;
                self.pending = Some(index);
            }
            let local = (cursor % BLK as u64) as usize;
            let count = bytes.len().min(BLK - local);
            self.buffer.as_mut()[local..local + count].copy_from_slice(&bytes[..count]);
            self.dirty = true;
            bytes = &bytes[count..];
            cursor += count as u64;
        }
        Ok(())
    }
    fn flush(&mut self) -> CuResult<()> {
        self.flush_pending()
    }
    fn abort(&mut self) {
        self.pending = None;
        self.dirty = false;
    }
}

pub struct SdStorage<BD: BlockDevice> {
    bd: Arc<ForceSyncSend<BD>>,
    start: u64,
    size: u64,
}
impl<BD: BlockDevice + Send> ByteStorage for SdStorage<BD> {
    type Region = SdRegion<BD>;
    fn len(&self) -> u64 {
        self.size
    }
    fn grow(&mut self, end: u64) -> CuResult<()> {
        if end > self.size {
            return Err(CuError::from("Log exceeds SD partition capacity"));
        }
        Ok(())
    }
    fn region(&self, offset: u64, length: u64) -> CuResult<Self::Region> {
        if offset.checked_add(length).is_none_or(|end| end > self.size) {
            return Err(CuError::from("Byte range exceeds SD partition"));
        }
        Ok(SdRegion {
            bd: self.bd.clone(),
            start: self.start + offset,
            length,
            pending: None,
            buffer: Block::new(),
            dirty: false,
        })
    }
}
pub type EMMCSectionStorage<BD> = ByteSection<SdRegion<BD>>;

/// SD/eMMC adapter using the same byte links and capacity policies as mmap.
pub struct EMMCLogger<BD: BlockDevice + Send>(ByteLogger<SdStorage<BD>>);
impl<BD: BlockDevice + Send> EMMCLogger<BD> {
    pub fn new(bd: BD, start: BlockIdx, size: BlockCount) -> CuResult<Self> {
        Self::with_policy(bd, start, size, CapacityPolicy::StopWhenFull)
    }
    pub fn with_policy(
        bd: BD,
        start: BlockIdx,
        size: BlockCount,
        policy: CapacityPolicy,
    ) -> CuResult<Self> {
        Self::validate_partition(&bd, start, size)?;
        let size = size.0 as u64 * BLK as u64;
        let storage = SdStorage {
            bd: Arc::new(ForceSyncSend::new(bd)),
            start: start.0 as u64 * BLK as u64,
            size,
        };
        ByteLogger::new(storage, BLK as u16, policy, size).map(Self)
    }
    pub fn append(
        bd: BD,
        start: BlockIdx,
        size: BlockCount,
        policy: CapacityPolicy,
    ) -> CuResult<Self> {
        Self::validate_partition(&bd, start, size)?;
        let size = size.0 as u64 * BLK as u64;
        let storage = SdStorage {
            bd: Arc::new(ForceSyncSend::new(bd)),
            start: start.0 as u64 * BLK as u64,
            size,
        };
        ByteLogger::append(storage, policy, size).map(Self)
    }
    fn validate_partition(bd: &BD, start: BlockIdx, size: BlockCount) -> CuResult<()> {
        let total = bd
            .num_blocks()
            .map_err(|_| CuError::from("Cannot determine SD capacity"))?
            .0;
        if start.0.checked_add(size.0).is_none_or(|end| end > total) {
            return Err(CuError::from("Partition exceeds SD device"));
        }
        Ok(())
    }
    pub fn close(&mut self) -> CuResult<()> {
        self.0.close()
    }
}
impl<BD: BlockDevice + Send> UnifiedLogWrite<EMMCSectionStorage<BD>> for EMMCLogger<BD> {
    fn add_section(
        &mut self,
        kind: UnifiedLogType,
        size: usize,
    ) -> CuResult<SectionHandle<EMMCSectionStorage<BD>>> {
        self.0.add_section(kind, size)
    }
    fn add_section_with_context(
        &mut self,
        kind: UnifiedLogType,
        size: usize,
        context: SectionContext,
    ) -> CuResult<SectionHandle<EMMCSectionStorage<BD>>> {
        self.0.add_section_with_context(kind, size, context)
    }
    fn seal_metadata<C: Encode>(
        &mut self,
        metadata: &ApplicationMetadata,
        catalog: Option<&C>,
    ) -> CuResult<()> {
        self.0.seal_metadata(metadata, catalog)
    }
    fn construction_context(
        &mut self,
        instance_id: u32,
        mission_index: u32,
    ) -> CuResult<SectionContext> {
        self.0.construction_context(instance_id, mission_index)
    }
    fn flush_section(&mut self, section: &mut SectionHandle<EMMCSectionStorage<BD>>) {
        self.0.flush_section(section)
    }
    fn try_flush_section(
        &mut self,
        section: &mut SectionHandle<EMMCSectionStorage<BD>>,
    ) -> CuResult<()> {
        self.0.try_flush_section(section)
    }
    fn status(&self) -> UnifiedLogStatus {
        self.0.status()
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use cu29_unifiedlog::{UnifiedLogRead, memmap::*};
    use std::sync::Mutex as HostMutex;
    use std::sync::atomic::{AtomicUsize, Ordering};
    use std::vec::Vec;

    #[derive(Clone)]
    struct Device {
        bytes: Arc<HostMutex<Vec<u8>>>,
        fail_after: Arc<AtomicUsize>,
    }
    impl Device {
        fn new() -> Self {
            Self {
                bytes: Arc::new(HostMutex::new(alloc::vec![0; 48 * BLK])),
                fail_after: Arc::new(AtomicUsize::new(usize::MAX)),
            }
        }
    }
    impl BlockDevice for Device {
        type Error = ();
        #[cfg(all(feature = "eh02", not(feature = "eh1")))]
        fn read(&self, blocks: &mut [Block], start: BlockIdx, _: &str) -> Result<(), ()> {
            self.read_blocks(blocks, start)
        }
        #[cfg(feature = "eh1")]
        fn read(&self, blocks: &mut [Block], start: BlockIdx) -> Result<(), ()> {
            self.read_blocks(blocks, start)
        }
        fn write(&self, blocks: &[Block], start: BlockIdx) -> Result<(), ()> {
            let left = self.fail_after.load(Ordering::Relaxed);
            if left == 0 {
                return Err(());
            }
            if left != usize::MAX {
                self.fail_after.fetch_sub(1, Ordering::Relaxed);
            }
            let mut bytes = self.bytes.lock().unwrap();
            for (index, block) in blocks.iter().enumerate() {
                let offset = (start.0 as usize + index) * BLK;
                bytes[offset..offset + BLK].copy_from_slice(block.as_ref());
            }
            Ok(())
        }
        fn num_blocks(&self) -> Result<BlockCount, ()> {
            Ok(BlockCount(48))
        }
    }
    impl Device {
        fn read_blocks(&self, blocks: &mut [Block], start: BlockIdx) -> Result<(), ()> {
            let bytes = self.bytes.lock().unwrap();
            for (index, block) in blocks.iter_mut().enumerate() {
                let offset = (start.0 as usize + index) * BLK;
                block.as_mut().copy_from_slice(&bytes[offset..offset + BLK]);
            }
            Ok(())
        }
    }
    fn metadata() -> ApplicationMetadata {
        ApplicationMetadata {
            app_type: "Robot".into(),
            app_name: "test".into(),
            app_version: "1".into(),
            git_commit: None,
            git_dirty: None,
            subsystem_id: None,
            subsystem_code: 0,
            effective_config_ron: "(tasks:[])".into(),
            missions: alloc::vec!["alpha".into(), "beta".into()],
            catalog_offset: 0,
        }
    }
    fn trace<S: SectionStorage, L: UnifiedLogWrite<S>>(logger: &mut L) {
        logger
            .seal_metadata(&metadata(), Some(&[1u8; 700]))
            .unwrap();
        let first = logger.construction_context(3, 0).unwrap();
        let second = logger.construction_context(5, 1).unwrap();
        for value in 0..100u32 {
            let context = if value.is_multiple_of(2) {
                first
            } else {
                second
            };
            let mut section = logger
                .add_section_with_context(UnifiedLogType::CopperList, 1536, context)
                .unwrap();
            section.append([value; 160]).unwrap();
            logger.flush_section(&mut section);
            if value.is_multiple_of(17) {
                let mut section = logger
                    .add_section_with_context(UnifiedLogType::RuntimeLifecycle, 1024, context)
                    .unwrap();
                section.append(value).unwrap();
                logger.flush_section(&mut section);
            }
        }
    }
    fn test_dir() -> tempfile::TempDir {
        let path =
            std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../target/sdlogger-tests");
        std::fs::create_dir_all(&path).unwrap();
        tempfile::TempDir::new_in(path).unwrap()
    }
    fn sections(path: &std::path::Path) -> Vec<(SectionContext, UnifiedLogType, Vec<u8>)> {
        let mut reader = MmapUnifiedLoggerRead::new(path).unwrap();
        let mut output = Vec::new();
        loop {
            let (header, data) = reader.raw_read_section().unwrap();
            if header.entry_type == UnifiedLogType::LastEntry {
                return output;
            }
            output.push((header.context, header.entry_type, data));
        }
    }
    #[test]
    fn mmap_and_sd_have_identical_retained_sections_after_repeated_wrap() {
        let device = Device::new();
        let dir = test_dir();
        {
            let mut logger = EMMCLogger::with_policy(
                device.clone(),
                BlockIdx(7),
                BlockCount(32),
                CapacityPolicy::OverwriteOldest,
            )
            .unwrap();
            trace(&mut logger);
        }
        let sd_path = dir.path().join("sd.copper");
        std::fs::write(
            dir.path().join("sd_0.copper"),
            &device.bytes.lock().unwrap()[7 * BLK..39 * BLK],
        )
        .unwrap();
        let mmap_path = dir.path().join("mmap.copper");
        {
            let MmapUnifiedLogger::Write(mut logger) = MmapUnifiedLoggerBuilder::new()
                .file_base_name(&mmap_path)
                .preallocated_size(4096)
                .rollover(32 * BLK)
                .write(true)
                .create(true)
                .build()
                .unwrap()
            else {
                unreachable!()
            };
            trace(&mut logger);
        }
        assert_eq!(sections(&sd_path), sections(&mmap_path));
        assert!(
            sections(&sd_path)
                .iter()
                .filter(|(_, kind, _)| *kind == UnifiedLogType::CopperList)
                .count()
                < 100
        );
    }
    #[test]
    fn failed_sd_write_restores_position_and_preserves_partial_block_prefix() {
        let device = Device::new();
        let mut logger = EMMCLogger::new(device.clone(), BlockIdx(7), BlockCount(32)).unwrap();
        let mut section = logger
            .add_section(UnifiedLogType::CopperList, 4096)
            .unwrap();
        section.append(11u32).unwrap();
        device.fail_after.store(1, Ordering::Relaxed);
        assert!(section.append([2u8; 1400]).is_err());
        device.fail_after.store(usize::MAX, Ordering::Relaxed);
        section.append(12u32).unwrap();
        logger.flush_section(&mut section);
        drop(section);
        drop(logger);
        let dir = test_dir();
        let path = dir.path().join("rollback.copper");
        std::fs::write(
            dir.path().join("rollback_0.copper"),
            &device.bytes.lock().unwrap()[7 * BLK..39 * BLK],
        )
        .unwrap();
        let data = sections(&path).remove(0).2;
        assert_eq!(
            bincode::decode_from_slice::<(u32, u32), _>(&data, bincode::config::standard())
                .unwrap(),
            ((11, 12), data.len())
        );
    }
    #[test]
    fn sd_append_compares_before_writing_and_reuses_metadata() {
        let device = Device::new();
        {
            let mut logger = EMMCLogger::new(device.clone(), BlockIdx(7), BlockCount(32)).unwrap();
            logger.seal_metadata(&metadata(), Some(&42u32)).unwrap();
        }
        let before = device.bytes.lock().unwrap().clone();
        {
            let mut logger = EMMCLogger::append(
                device.clone(),
                BlockIdx(7),
                BlockCount(32),
                CapacityPolicy::OverwriteOldest,
            )
            .unwrap();
            assert!(logger.seal_metadata(&metadata(), Some(&43u32)).is_err());
        }
        assert_eq!(*device.bytes.lock().unwrap(), before);
        {
            let mut logger = EMMCLogger::append(
                device.clone(),
                BlockIdx(7),
                BlockCount(32),
                CapacityPolicy::OverwriteOldest,
            )
            .unwrap();
            logger.seal_metadata(&metadata(), Some(&42u32)).unwrap();
            assert_eq!(logger.construction_context(8, 1).unwrap().run_id, 1);
        }
    }
}
