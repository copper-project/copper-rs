use crate::sdmmc::{Block, BlockCount, BlockDevice, BlockIdx};
use alloc::sync::Arc;
use bincode::Encode;
use core::num::NonZeroUsize;
use cu29::prelude::*;
use cu29_unifiedlog::byte_log::{ByteLogger, ByteRegion, ByteSection, ByteStorage};
use spin::Mutex;

const BLK: usize = 512;
const BATCH_BLOCKS: usize = 16;
const RECORDS_PER_COMMIT: NonZeroUsize = NonZeroUsize::new(32).unwrap();
const SECTION_HEADER_BYTES: u64 = cu29_unifiedlog::SECTION_HEADER_COMPACT_SIZE as u64;

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

/// A bounded partition byte range with a fixed buffer for contiguous sectors.
pub struct SdRegion<BD: BlockDevice> {
    bd: Arc<ForceSyncSend<BD>>,
    start: u64,
    length: u64,
    first_block: u32,
    buffered_blocks: usize,
    blocks: [Block; BATCH_BLOCKS],
    first_dirty: usize,
    batch_copperlists: bool,
    batching: bool,
}
impl<BD: BlockDevice> SdRegion<BD> {
    fn read_block(bd: &ForceSyncSend<BD>, index: u32, block: &mut Block) -> CuResult<()> {
        #[cfg(all(feature = "eh02", not(feature = "eh1")))]
        let result = bd.read(
            core::slice::from_mut(block),
            BlockIdx(index),
            "Copper byte log",
        );
        #[cfg(feature = "eh1")]
        let result = bd.read(core::slice::from_mut(block), BlockIdx(index));
        result.map_err(|_| CuError::from("SD block read failed"))
    }
    fn flush_pending(&mut self) -> CuResult<()> {
        if self.first_dirty < self.buffered_blocks {
            self.bd
                .write(
                    &self.blocks[self.first_dirty..self.buffered_blocks],
                    BlockIdx(self.first_block + self.first_dirty as u32),
                )
                .map_err(|_| CuError::from("SD block write failed"))?;
            self.first_dirty = self.buffered_blocks;
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
            Self::read_block(&self.bd, index, &mut block)?;
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
        if self.batching && offset == BLK as u64 && bytes.len() <= BLK {
            // The commit owns this sector. Publishing it leaves the payload tail in RAM.
            self.flush_pending()?;
            let index = u32::try_from(cursor / BLK as u64)
                .map_err(|_| CuError::from("SD block index overflow"))?;
            let mut commit = Block::new();
            commit.as_mut()[..bytes.len()].copy_from_slice(bytes);
            return self
                .bd
                .write(core::slice::from_ref(&commit), BlockIdx(index))
                .map_err(|_| CuError::from("SD commit write failed"));
        }
        while !bytes.is_empty() {
            let index = u32::try_from(cursor / BLK as u64)
                .map_err(|_| CuError::from("SD block index overflow"))?;
            let local = (cursor % BLK as u64) as usize;
            let count = bytes.len().min(BLK - local);
            let payload_start = self.start + SECTION_HEADER_BYTES;
            let in_payload = self.batching && cursor >= payload_start;
            let mut slot = index.wrapping_sub(self.first_block) as usize;
            if slot >= self.buffered_blocks {
                let extend = in_payload && slot == self.buffered_blocks && slot < BATCH_BLOCKS;
                if !extend {
                    self.flush_pending()?;
                    self.buffered_blocks = 0;
                    self.first_dirty = 0;
                    self.first_block = index;
                    slot = 0;
                }
                let block = &mut self.blocks[slot];
                if count == BLK || (in_payload && local == 0) {
                    *block = Block::new();
                } else {
                    Self::read_block(&self.bd, index, block)?;
                }
                self.buffered_blocks += 1;
            }
            self.blocks[slot].as_mut()[local..local + count].copy_from_slice(&bytes[..count]);
            self.first_dirty = self.first_dirty.min(slot);
            bytes = &bytes[count..];
            cursor += count as u64;
        }
        Ok(())
    }
    fn flush(&mut self) -> CuResult<()> {
        self.flush_pending()
    }
    fn begin_record_batching(&mut self) -> NonZeroUsize {
        self.batching = self.batch_copperlists;
        if self.batching {
            self.abort();
            RECORDS_PER_COMMIT
        } else {
            NonZeroUsize::MIN
        }
    }
    fn abort(&mut self) {
        self.buffered_blocks = 0;
        self.first_dirty = 0;
    }
    fn abort_record(&mut self, position: u64) {
        if !self.batching {
            self.abort();
            return;
        }
        let end_block = (self.start + position).div_ceil(BLK as u64);
        let retained_blocks = end_block.saturating_sub(u64::from(self.first_block)) as usize;
        self.buffered_blocks = self.buffered_blocks.min(retained_blocks);
        self.first_dirty = self.first_dirty.min(self.buffered_blocks);
    }
}

pub struct SdStorage<BD: BlockDevice> {
    bd: Arc<ForceSyncSend<BD>>,
    start: u64,
    size: u64,
    batch_copperlists: bool,
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
            first_block: 0,
            buffered_blocks: 0,
            blocks: core::array::from_fn(|_| Block::new()),
            first_dirty: 0,
            batch_copperlists: self.batch_copperlists,
            batching: false,
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
    /// Create a stop-when-full log that commits CopperLists in groups of 32.
    ///
    /// A reset can lose the current batch. Closing a section commits any remainder.
    pub fn new_batched(bd: BD, start: BlockIdx, size: BlockCount) -> CuResult<Self> {
        Self::create(bd, start, size, CapacityPolicy::StopWhenFull, true)
    }
    /// Start a batched run on blank media or append after the last allocated section.
    pub fn append_batched(bd: BD, start: BlockIdx, size: BlockCount) -> CuResult<Self> {
        Self::validate_partition(&bd, start, size)?;
        let capacity = size.0 as u64 * BLK as u64;
        let storage = SdStorage {
            bd: Arc::new(ForceSyncSend::new(bd)),
            start: start.0 as u64 * BLK as u64,
            size: capacity,
            batch_copperlists: true,
        };
        let mut header = [0u8; BLK];
        storage.region(0, BLK as u64)?.read(0, &mut header)?;
        if header == [0; BLK] || header == [0xff; BLK] {
            ByteLogger::new(storage, BLK as u16, CapacityPolicy::StopWhenFull, capacity).map(Self)
        } else {
            ByteLogger::append_after_reset(storage, capacity).map(Self)
        }
    }

    pub fn with_policy(
        bd: BD,
        start: BlockIdx,
        size: BlockCount,
        policy: CapacityPolicy,
    ) -> CuResult<Self> {
        Self::create(bd, start, size, policy, false)
    }
    fn create(
        bd: BD,
        start: BlockIdx,
        size: BlockCount,
        policy: CapacityPolicy,
        batch_copperlists: bool,
    ) -> CuResult<Self> {
        Self::validate_partition(&bd, start, size)?;
        let size = size.0 as u64 * BLK as u64;
        let storage = SdStorage {
            bd: Arc::new(ForceSyncSend::new(bd)),
            start: start.0 as u64 * BLK as u64,
            size,
            batch_copperlists,
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
            batch_copperlists: false,
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
        writes: Arc<HostMutex<Vec<usize>>>,
        reads: Arc<AtomicUsize>,
    }
    impl Device {
        fn new() -> Self {
            Self {
                bytes: Arc::new(HostMutex::new(alloc::vec![0; 48 * BLK])),
                fail_after: Arc::new(AtomicUsize::new(usize::MAX)),
                writes: Arc::new(HostMutex::new(Vec::new())),
                reads: Arc::new(AtomicUsize::new(0)),
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
            self.writes.lock().unwrap().push(blocks.len());
            Ok(())
        }
        fn num_blocks(&self) -> Result<BlockCount, ()> {
            Ok(BlockCount(48))
        }
    }
    impl Device {
        fn read_blocks(&self, blocks: &mut [Block], start: BlockIdx) -> Result<(), ()> {
            self.reads.fetch_add(blocks.len(), Ordering::Relaxed);
            let bytes = self.bytes.lock().unwrap();
            for (index, block) in blocks.iter_mut().enumerate() {
                let offset = (start.0 as usize + index) * BLK;
                block.as_mut().copy_from_slice(&bytes[offset..offset + BLK]);
            }
            Ok(())
        }
    }
    #[test]
    fn sd_batches_write_whole_sectors_without_reads_and_publish_after_payload() {
        let device = Device::new();
        let mut logger =
            EMMCLogger::new_batched(device.clone(), BlockIdx(7), BlockCount(32)).unwrap();
        let mut section = logger
            .add_section(UnifiedLogType::CopperList, 12_288)
            .unwrap();
        device.writes.lock().unwrap().clear();
        device.reads.store(0, Ordering::Relaxed);
        const RECORD_BYTES: usize = 250;
        let payload = [42u8; RECORD_BYTES];
        for _ in 1..RECORDS_PER_COMMIT.get() {
            section.append(payload).unwrap();
        }
        assert!(device.writes.lock().unwrap().is_empty());
        section.append(payload).unwrap();
        assert_eq!(*device.writes.lock().unwrap(), [BATCH_BLOCKS, 1]);
        assert_eq!(device.reads.load(Ordering::Relaxed), 0);

        section.append([43u8; RECORD_BYTES]).unwrap();
        assert_eq!(device.reads.load(Ordering::Relaxed), 0);
        assert_eq!(*device.writes.lock().unwrap(), [BATCH_BLOCKS, 1, 1]);
        let directory = test_dir();
        let path = directory.path().join("batched.copper");
        let snapshot = || {
            std::fs::write(
                directory.path().join("batched_0.copper"),
                &device.bytes.lock().unwrap()[7 * BLK..39 * BLK],
            )
            .unwrap();
            sections(&path).remove(0).2
        };
        let committed = snapshot();
        assert_eq!(committed, payload.repeat(RECORDS_PER_COMMIT.get()));
        logger.try_flush_section(&mut section).unwrap();
        logger.close().unwrap();
        let saved = snapshot();
        assert_eq!(saved.len(), committed.len() + RECORD_BYTES);
        assert_eq!(&saved[..committed.len()], committed);
        assert_eq!(&saved[committed.len()..], [43u8; RECORD_BYTES]);
    }

    #[test]
    fn failed_record_preserves_the_buffered_prefix() {
        struct FailingRecord;
        impl Encode for FailingRecord {
            fn encode<E: bincode::enc::Encoder>(
                &self,
                encoder: &mut E,
            ) -> Result<(), bincode::error::EncodeError> {
                [2u8; BATCH_BLOCKS * BLK].encode(encoder)?;
                Err(bincode::error::EncodeError::Other("test encoding failure"))
            }
        }
        let device = Device::new();
        let mut logger =
            EMMCLogger::new_batched(device.clone(), BlockIdx(7), BlockCount(32)).unwrap();
        let mut section = logger
            .add_section(UnifiedLogType::CopperList, 12_288)
            .unwrap();
        section.append(11u8).unwrap();
        assert!(section.append(FailingRecord).is_err());
        section.append(12u8).unwrap();
        logger.try_flush_section(&mut section).unwrap();
        logger.close().unwrap();
        let directory = test_dir();
        std::fs::write(
            directory.path().join("batched-rollback_0.copper"),
            &device.bytes.lock().unwrap()[7 * BLK..39 * BLK],
        )
        .unwrap();
        assert_eq!(
            sections(&directory.path().join("batched-rollback.copper"))[0].2,
            [11, 12]
        );
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
                .add_section_with_context(UnifiedLogType::CopperList, 2048, context)
                .unwrap();
            section.append([value; 160]).unwrap();
            logger.flush_section(&mut section);
            if value.is_multiple_of(17) {
                let mut section = logger
                    .add_section_with_context(UnifiedLogType::RuntimeLifecycle, 1536, context)
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
    fn sd_descriptor_linking_does_not_overwrite_an_active_encoders_commit() {
        struct Paused {
            entered: std::sync::mpsc::Sender<()>,
            resume: std::sync::mpsc::Receiver<()>,
        }
        impl bincode::Encode for Paused {
            fn encode<E: bincode::enc::Encoder>(
                &self,
                encoder: &mut E,
            ) -> Result<(), bincode::error::EncodeError> {
                bincode::Encode::encode(&12u32, encoder)?;
                self.entered.send(()).unwrap();
                self.resume
                    .recv_timeout(std::time::Duration::from_secs(5))
                    .unwrap();
                Ok(())
            }
        }
        let device = Device::new();
        let mut logger = EMMCLogger::new(device.clone(), BlockIdx(7), BlockCount(32)).unwrap();
        let mut first = logger
            .add_section(UnifiedLogType::CopperList, 2048)
            .unwrap();
        first.append(11u32).unwrap();
        let (entered, entering) = std::sync::mpsc::channel();
        let (resume, resuming) = std::sync::mpsc::channel();
        let worker = std::thread::spawn(move || {
            first
                .append(Paused {
                    entered,
                    resume: resuming,
                })
                .unwrap();
            first
        });
        entering
            .recv_timeout(std::time::Duration::from_secs(5))
            .unwrap();
        let mut next = logger
            .add_section(UnifiedLogType::CopperList, 2048)
            .unwrap();
        next.append(13u32).unwrap();
        resume.send(()).unwrap();
        let mut first = worker.join().unwrap();
        logger.try_flush_section(&mut first).unwrap();
        logger.try_flush_section(&mut next).unwrap();
        logger.close().unwrap();
        let dir = test_dir();
        let path = dir.path().join("concurrent.copper");
        std::fs::write(
            dir.path().join("concurrent_0.copper"),
            &device.bytes.lock().unwrap()[7 * BLK..39 * BLK],
        )
        .unwrap();
        let saved = sections(&path);
        assert_eq!(saved.len(), 2);
        assert_eq!(
            bincode::decode_from_slice::<(u32, u32), _>(&saved[0].2, bincode::config::standard())
                .unwrap()
                .0,
            (11, 12)
        );
        assert_eq!(
            bincode::decode_from_slice::<u32, _>(&saved[1].2, bincode::config::standard())
                .unwrap()
                .0,
            13
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
    fn sd_batched_replug_preserves_committed_records_across_runs() {
        let device = Device::new();
        let open =
            || EMMCLogger::append_batched(device.clone(), BlockIdx(7), BlockCount(32)).unwrap();
        let mut first = open();
        first.seal_metadata(&metadata(), Some(&42u32)).unwrap();
        let context = first.construction_context(0, 0).unwrap();
        let mut stream = first
            .add_section_with_context(UnifiedLogType::CopperList, 2048, context)
            .unwrap();
        for _ in 0..33 {
            stream.append(1u8).unwrap();
        }
        // Snapshot while the last record is still pending, as on loss of power.
        let image = device.bytes.lock().unwrap().clone();
        drop(stream);
        drop(first);
        *device.bytes.lock().unwrap() = image.clone();
        let mut second = open();
        let mut updated = metadata();
        updated.git_commit = Some("next revision".into());
        if cfg!(feature = "multi-build-log-append") {
            second.seal_metadata(&updated, Some(&43u32)).unwrap();
        } else {
            assert!(second.seal_metadata(&updated, Some(&43u32)).is_err());
            assert_eq!(*device.bytes.lock().unwrap(), image);
            second.seal_metadata(&metadata(), Some(&42u32)).unwrap();
        }
        let context = second.construction_context(0, 0).unwrap();
        assert_eq!(context.run_id, 2);
        let mut stream = second
            .add_section_with_context(UnifiedLogType::CopperList, 2048, context)
            .unwrap();
        for _ in 0..32 {
            stream.append(2u8).unwrap();
        }
        let dir = test_dir();
        std::fs::write(
            dir.path().join("replug_0.copper"),
            &device.bytes.lock().unwrap()[7 * BLK..39 * BLK],
        )
        .unwrap();
        let runs: Vec<_> = sections(&dir.path().join("replug.copper"))
            .into_iter()
            .filter(|(_, kind, _)| *kind == UnifiedLogType::CopperList)
            .map(|(ctx, _, bytes)| (ctx.run_id, bytes))
            .collect();
        assert_eq!(runs, [(1, alloc::vec![1; 32]), (2, alloc::vec![2; 32])]);
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
