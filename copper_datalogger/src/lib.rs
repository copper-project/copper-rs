use libc;

use bincode::encode_into_slice;
use bincode_derive::{Decode, Encode};

use bincode::config::Configuration;
use memmap2::MmapMut;
use std::fs::OpenOptions;

const MAIN_MAGIC: [u8; 4] = [0xB4, 0xA5, 0x50, 0xFF];

const SECTION_MAGIC: [u8; 2] = [0xFA, 0x57];

#[derive(Encode, Decode)]
struct MainHeader {
    magic: [u8; 4],
    first_section_offset: u16, // This is to align with a page at write time.
}

#[derive(Encode, Decode)]
enum EntryType {
    StructuredLogLine,
    CopperList,
}

#[derive(Encode, Decode)]
struct SectionHeader {
    magic: [u8; 2],
    entry_type: EntryType,
    section_size: u32, // offset of section_magic + section_size -> should be the index of the next section_magic
}

pub struct DataLogger {
    file: MmapMut,
    page_size: usize,
    current_position: usize,
    config: Configuration,
}

impl DataLogger {
    pub fn new(file_path: &str) -> std::io::Result<Self> {
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .open(file_path)?;
        let config = bincode::config::standard();
        let mut mmap = unsafe { MmapMut::map_mut(&file)? };
        let page_size = unsafe { libc::sysconf(libc::_SC_PAGESIZE) as usize };
        let mmap_start_addr = &mmap[0] as *const u8 as usize;
        let next_aligned_addr = (mmap_start_addr + page_size - 1) & !(page_size - 1);
        let main_header = MainHeader {
            magic: MAIN_MAGIC,
            first_section_offset: (next_aligned_addr - mmap_start_addr) as u16,
        };
        let nb_bytes = encode_into_slice(&main_header, &mut mmap[..], config)
            .expect("Failed to encode main header");

        Ok(Self {
            file: mmap,
            page_size,
            current_position: nb_bytes,
            config,
        })
    }

    pub fn add_section(&mut self, entry_type: EntryType, section_size: u32) {
        let section_header = SectionHeader {
            magic: SECTION_MAGIC,
            entry_type,
            section_size,
        };

        let nb_bytes = encode_into_slice(
            &section_header,
            &mut self.file[self.current_position..],
            self.config,
        )
        .expect("Failed to encode section header");
        self.current_position += nb_bytes;
    }

    #[inline]
    fn allocated_len(&self) -> usize {
        self.file.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {}
}
