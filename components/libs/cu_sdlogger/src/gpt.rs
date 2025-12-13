use crate::sdmmc::{Block, BlockCount, BlockDevice, BlockIdx};

/// Scan the GPT on the SD/eMMC card and return the block range of the Cu29 partition.
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
    const CU29_TYPE_GUID_ONDISK: [u8; 16] = [
        0xC9, 0xE0, 0xA2, 0x29, 0x00, 0x00, 0x75, 0x4C, 0x92, 0x29, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x29,
    ];

    let mut hdr = [Block::new(); 1];
    read_blocks(sd, &mut hdr, BlockIdx(1), "gpt-lba1")?;
    let h: &[u8] = &hdr[0].contents;
    if &h[0..8] != b"EFI PART" {
        return Ok(None);
    }

    let ent_lba = le64(&h[72..80]);
    let num_ents = le32(&h[80..84]) as usize;
    let ent_sz = le32(&h[84..88]) as usize;
    if ent_sz == 0 {
        return Ok(None);
    }

    let mut remain = num_ents.checked_mul(ent_sz).unwrap_or(0);
    let mut lba = ent_lba as u32;
    let mut buf = [Block::new(); 1];

    while remain > 0 {
        read_blocks(sd, &mut buf, BlockIdx(lba), "gpt-entry")?;
        let b = &buf[0].contents;
        let usable = core::cmp::min(remain, Block::LEN);

        let mut off = 0usize;
        while off + ent_sz <= usable {
            let e = &b[off..off + ent_sz];
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

    Ok(None)
}

#[cfg(feature = "eh02")]
fn read_blocks<D: BlockDevice>(
    dev: &D,
    blocks: &mut [Block],
    start_block_idx: BlockIdx,
    reason: &str,
) -> Result<(), D::Error> {
    dev.read(blocks, start_block_idx, reason)
}

#[cfg(feature = "eh1")]
fn read_blocks<D: BlockDevice>(
    dev: &D,
    blocks: &mut [Block],
    start_block_idx: BlockIdx,
    _reason: &str,
) -> Result<(), D::Error> {
    dev.read(blocks, start_block_idx)
}
