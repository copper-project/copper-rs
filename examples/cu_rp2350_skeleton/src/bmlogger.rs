use alloc::sync::Arc;
use bincode::config::standard;
use bincode::enc::write::Writer as BincodeWriter;
use bincode::error::EncodeError;
use bincode::{Encode, encode_into_slice, encode_into_writer};
use core::cell::UnsafeCell;
use cu29::prelude::*;
use embedded_sdmmc::{Block, BlockCount, BlockDevice, BlockIdx};

// This is a minimal GPT parser to just find the copper partition.
// returns the offset in block and size in blocks of the partition.
// Cu29 GPT SD creator:
// - GPT
// - p1: 1 MiB FAT12, PARTLABEL=COPPERCFG
// - p2: rest, PARTLABEL=Cu29, TYPEGUID=29A2E0C9-0000-4C75-9229-000000000029
pub fn find_copper_partition<D: BlockDevice>(
    sd: &D,
) -> Result<Option<(BlockIdx, BlockCount)>, D::Error> {
    fn le32(x: &[u8]) -> u32 {
        u32::from_le_bytes([x[0], x[1], x[2], x[3]])
    }
    fn le64(x: &[u8]) -> u64 {
        u64::from_le_bytes([x[0], x[1], x[2], x[3], x[4], x[5], x[6], x[7]])
    }

    // Cu29 TYPE GUID = 29A2E0C9-0000-4C75-9229-000000000029
    // On-disk GPT storage = mixed endian layout:
    const CU29_TYPE_GUID_ONDISK: [u8; 16] = [
        0xC9, 0xE0, 0xA2, 0x29, // -
        0x00, 0x00, // -
        0x75, 0x4C, // -
        0x92, 0x29, // -
        0x00, 0x00, 0x00, 0x00, 0x00, 0x29,
    ];

    let mut hdr = [Block::new(); 1];
    sd.read(&mut hdr, BlockIdx(1))?;
    let h: &[u8] = &hdr[0].contents;
    if &h[0..8] != b"EFI PART" {
        error!("\"EFI PART\" ID not found");
        return Ok(None);
    }

    let ent_lba = le64(&h[72..80]);
    let num_ents = le32(&h[80..84]) as usize;
    let ent_sz = le32(&h[84..88]) as usize;
    if ent_sz == 0 {
        error!("ent_sz is 0");
        return Ok(None);
    }

    let mut remain = num_ents.checked_mul(ent_sz).unwrap_or(0);
    let mut lba = ent_lba as u32;
    let mut buf = [Block::new(); 1];

    while remain > 0 {
        sd.read(&mut buf, BlockIdx(lba))?;
        let b = &buf[0].contents;
        let usable = core::cmp::min(remain, Block::LEN);

        let mut off = 0usize;
        while off + ent_sz <= usable {
            let e = &b[off..off + ent_sz];

            debug!("checking entry {:?}", e);

            // Compare full 16-byte type GUID instead of e[0] == 0x29
            if e[0..16] == CU29_TYPE_GUID_ONDISK {
                let first = le64(&e[32..40]) as u32;
                let last = le64(&e[40..48]) as u32;
                if last >= first {
                    return Ok(Some((BlockIdx(first), BlockCount(last - first + 1))));
                }
            }

            off += ent_sz;
        }

        remain -= usable;
        lba = lba.saturating_add(1);
    }

    error!("just did not find Cu29 GPT entry");
    Ok(None)
}

#[repr(transparent)]
pub struct ForceSyncSend<T>(UnsafeCell<T>);
impl<T> ForceSyncSend<T> {
    pub const fn new(inner: T) -> Self {
        Self(UnsafeCell::new(inner))
    }
    #[inline]
    fn inner(&self) -> &T {
        unsafe { &*self.0.get() }
    }
    #[inline]
    #[allow(clippy::mut_from_ref)]
    fn inner_mut(&self) -> &mut T {
        unsafe { &mut *self.0.get() }
    }
}
unsafe impl<T> Send for ForceSyncSend<T> {}
unsafe impl<T> Sync for ForceSyncSend<T> {}

impl<B: BlockDevice> BlockDevice for ForceSyncSend<B> {
    type Error = B::Error;
    fn read(&self, blocks: &mut [Block], start: BlockIdx) -> Result<(), Self::Error> {
        self.inner_mut().read(blocks, start)
    }
    fn write(&self, blocks: &[Block], start: BlockIdx) -> Result<(), Self::Error> {
        self.inner_mut().write(blocks, start)
    }
    fn num_blocks(&self) -> Result<BlockCount, Self::Error> {
        self.inner().num_blocks()
    }
}

const BLK: usize = 512;

/// Implements a bincode Writer for some linear logging over a block device.
pub struct SdBlockWriter<BD: BlockDevice> {
    bd: Arc<ForceSyncSend<BD>>,
    current_blk: BlockIdx, // absolute block number for payload
    position_blk: usize,   // 0..512
    capacity_bytes: usize, // payload capacity for this section
    written: usize,        // payload bytes written so far
    buffer: Block,         // RMW buffer for current block
}

impl<BD: BlockDevice> SdBlockWriter<BD> {
    pub fn new(bd: Arc<ForceSyncSend<BD>>, start_block: BlockIdx, capacity_bytes: usize) -> Self {
        debug!(
            "new SdBlockWriter: start_block={}, capacity_bytes={}",
            start_block.0, capacity_bytes
        );
        Self {
            bd,
            current_blk: start_block,
            position_blk: 0,
            capacity_bytes,
            written: 0,
            buffer: Block::new(),
        }
    }

    #[allow(dead_code)]
    pub fn bytes_written(&self) -> usize {
        self.written
    }

    #[allow(dead_code)]
    pub fn capacity_left(&self) -> usize {
        self.capacity_bytes.saturating_sub(self.written)
    }

    #[inline]
    fn flush_full(&mut self) -> Result<(), EncodeError> {
        self.bd
            .write(core::slice::from_ref(&self.buffer), self.current_blk)
            .expect("write failed on full block");
        self.current_blk += BlockCount(1);
        self.position_blk = 0;
        self.buffer = Block::new();
        Ok(())
    }

    /// Force-flush the current tail block if partially filled.
    #[allow(dead_code)]
    pub fn flush_tail(&mut self) -> Result<(), EncodeError> {
        if self.position_blk != 0 {
            self.bd
                .write(core::slice::from_ref(&self.buffer), self.current_blk)
                .expect("write failed on flush");
            // Advance to the next block, start fresh at the boundary
            self.current_blk += BlockCount(1);
            self.position_blk = 0;
            self.buffer = Block::new();
        }
        Ok(())
    }
}

impl<BD: BlockDevice> BincodeWriter for SdBlockWriter<BD> {
    fn write(&mut self, mut bytes: &[u8]) -> Result<(), EncodeError> {
        if self
            .written
            .checked_add(bytes.len())
            .is_none_or(|w| w > self.capacity_bytes)
        {
            return Err(EncodeError::UnexpectedEnd);
        }

        if self.position_blk != 0 {
            let take = core::cmp::min(BLK - self.position_blk, bytes.len());
            self.buffer.as_mut()[self.position_blk..self.position_blk + take]
                .copy_from_slice(&bytes[..take]);
            self.position_blk += take;
            self.written += take;
            bytes = &bytes[take..];
            if self.position_blk == BLK {
                self.flush_full()?;
            }
        }

        while bytes.len() >= BLK {
            let mut blk = Block::new();
            blk.as_mut().copy_from_slice(&bytes[..BLK]);
            self.bd
                .write(core::slice::from_ref(&blk), self.current_blk)
                .expect("write failed");
            self.current_blk += BlockCount(1);
            self.written += BLK;
            bytes = &bytes[BLK..];
        }

        if !bytes.is_empty() {
            let n = bytes.len();
            self.buffer.as_mut()[self.position_blk..self.position_blk + n].copy_from_slice(bytes);
            self.position_blk += n;
            self.written += n;
            if self.position_blk == BLK {
                self.flush_full()?;
            }
        }

        Ok(())
    }
}

pub struct EMMCSectionStorage<BD: BlockDevice> {
    bd: Arc<ForceSyncSend<BD>>,
    start_block: BlockIdx,
    content_writer: SdBlockWriter<BD>,
}

impl<BD: BlockDevice> EMMCSectionStorage<BD> {
    // data_capacity is the space left in that section minus the header.
    fn new(bd: Arc<ForceSyncSend<BD>>, start_block: BlockIdx, data_capacity: usize) -> Self {
        let content_writer =
            SdBlockWriter::new(bd.clone(), start_block + BlockCount(1), data_capacity); // + 1 to skip the header
        Self {
            bd,
            start_block,
            content_writer,
        }
    }
}

impl<BD: BlockDevice> SectionStorage for EMMCSectionStorage<BD> {
    fn initialize<E: Encode>(&mut self, header: &E) -> Result<usize, EncodeError> {
        self.post_update_header(header)?;
        Ok(SECTION_HEADER_COMPACT_SIZE as usize)
    }

    fn post_update_header<E: Encode>(&mut self, header: &E) -> Result<usize, EncodeError> {
        // Re-encode header and write again to header blocks.
        let mut block = Block::new();
        let wrote = encode_into_slice(header, block.as_mut(), standard())?;
        self.bd
            .write(&[block], self.start_block)
            .map_err(|_| EncodeError::UnexpectedEnd)?;
        Ok(wrote)
    }

    fn append<E: Encode>(&mut self, entry: &E) -> Result<usize, EncodeError> {
        let bf = self.content_writer.written;
        encode_into_writer(entry, &mut self.content_writer, standard())
            .map_err(|_| EncodeError::UnexpectedEnd)?;
        Ok(self.content_writer.written - bf)
    }

    fn flush(&mut self) -> CuResult<usize> {
        let bf = self.content_writer.written;
        self.content_writer
            .flush_full()
            .map_err(|_| CuError::from("flush failed"))?;
        Ok(self.content_writer.written - bf)
    }
}

pub struct EMMCLogger<BD: BlockDevice> {
    bd: Arc<ForceSyncSend<BD>>,
    next_block: BlockIdx,
    last_block: BlockIdx,
}

impl<BD: BlockDevice> EMMCLogger<BD> {
    pub fn new(bd: BD, start: BlockIdx, size: BlockCount) -> CuResult<Self> {
        let main_header = MainHeader {
            magic: MAIN_MAGIC,
            first_section_offset: BLK as u16,
            page_size: BLK as u16,
        };
        let mut block: Block = Block::new();

        let _nb_bytes = encode_into_slice(&main_header, block.as_mut(), standard())
            .map_err(|_| CuError::from("Could not encode the main header"))?;

        debug!("Write the main header... ");
        bd.write(&[block], start)
            .map_err(|_| CuError::from("Could not write main header"))?;

        let next_block = start + BlockCount(1); // +1 to skip the main header
        let last_block = start + size;

        Ok(Self {
            bd: Arc::new(ForceSyncSend::new(bd)),
            next_block,
            last_block,
        })
    }

    // Allocate a section in this logger and return the start block index.
    fn alloc_section(&mut self, size: BlockCount) -> CuResult<BlockIdx> {
        debug!("allocating {} blocks", size.0);
        let start = self.next_block;
        self.next_block += size;
        if self.next_block > self.last_block {
            return Err(CuError::from("out of space"));
        }
        debug!("allocated started at {}", start.0);
        Ok(start)
    }
}

impl<BD> UnifiedLogWrite<EMMCSectionStorage<BD>> for EMMCLogger<BD>
where
    BD: BlockDevice + Send + Sync + 'static,
{
    fn add_section(
        &mut self,
        entry_type: UnifiedLogType,
        requested_section_size: usize,
    ) -> CuResult<SectionHandle<EMMCSectionStorage<BD>>> {
        let block_size = SECTION_HEADER_COMPACT_SIZE; // 512
        let section_header = SectionHeader {
            magic: SECTION_MAGIC,
            block_size,
            entry_type,
            offset_to_next_section: requested_section_size as u32,
            used: 0,
        };
        if block_size != 512 {
            panic!("EMMC: only 512 byte blocks supported");
        }

        debug!("new section size in bytes: {}", requested_section_size);

        let section_size_in_blks: u32 = (requested_section_size / block_size as usize) as u32 + 1; // always round up
        debug!("new section size in blocks: {}", section_size_in_blks);

        let start_block = self.alloc_section(BlockCount(section_size_in_blks))?;
        debug!("new section starts at {}", start_block.0);

        let storage = EMMCSectionStorage::new(
            Arc::clone(&self.bd),
            start_block,
            ((section_size_in_blks - 1) * block_size as u32) as usize,
        );

        // create handle (this will call storage.initialize(header))
        SectionHandle::create(section_header, storage)
    }

    fn flush_section(&mut self, section: &mut SectionHandle<EMMCSectionStorage<BD>>) {
        // and the end of the stream is ok.
        section
            .get_storage_mut()
            .flush()
            .expect("EMMC: flush failed");

        // and be sure the header is up-to-date
        section
            .post_update_header()
            .expect("EMMC: post update header failed");
    }

    fn status(&self) -> UnifiedLogStatus {
        UnifiedLogStatus {
            total_used_space: (self.next_block.0 as usize) * 512,
            total_allocated_space: (self.next_block.0 as usize) * 512,
        }
    }
}
