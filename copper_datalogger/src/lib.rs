use libc;

use bincode::encode_into_slice;
use bincode::error::EncodeError;
use bincode::Encode;
use bincode_derive::Decode as dDecode;
use bincode_derive::Encode as dEncode;

use bincode::config::Configuration;
use memmap2::{MmapMut, RemapOptions};
use std::fs::{File, OpenOptions};
use std::io;
use std::path::Path;

const MAIN_MAGIC: [u8; 4] = [0xB4, 0xA5, 0x50, 0xFF];

const SECTION_MAGIC: [u8; 2] = [0xFA, 0x57];

#[derive(dEncode, dDecode)]
struct MainHeader {
    magic: [u8; 4],
    first_section_offset: u16, // This is to align with a page at write time.
}

#[derive(dEncode, dDecode, Copy, Clone)]
enum EntryType {
    StructuredLogLine,
    CopperList,
}

#[derive(dEncode, dDecode)]
struct SectionHeader {
    magic: [u8; 2],
    entry_type: EntryType,
    section_size: u32, // offset of section_magic + section_size -> should be the index of the next section_magic
}

pub struct DataLogger {
    file: File,
    mmap_buffer: MmapMut,
    page_size: usize,
    current_position: usize,
    config: Configuration,
}

trait Stream<'a> {
    fn log(&'a mut self, obj: &impl Encode);
}

struct MmapStream<'a> {
    entry_type: EntryType,
    parent_logger: &'a mut DataLogger,
    current_slice: &'a mut [u8],
    current_position: usize,
}

impl<'a> Stream<'a> for MmapStream<'a> {
    fn log(&'a mut self, obj: &impl Encode) {
        let result = encode_into_slice(
            obj,
            &mut self.current_slice[self.current_position..],
            self.parent_logger.config,
        );
        match result {
            Ok(nb_bytes) => {
                self.current_position += nb_bytes;
            }
            Err(e) => match e {
                EncodeError::UnexpectedEnd => {
                    self.parent_logger.add_section(self.entry_type, 1024);
                }
                _ => panic!("Unexpected error while encoding object: {:?}", e),
            },
        }
    }
}

const DEFAULT_LOGGER_SIZE: usize = 1024 * 1024 * 1024; // 1GB

impl DataLogger {
    pub fn new(file_path: &Path, preallocated_size: Option<usize>) -> std::io::Result<Self> {
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .open(file_path)?;
        let page_size = unsafe { libc::sysconf(libc::_SC_PAGESIZE) as usize };
        if let Some(size) = preallocated_size {
            file.set_len(size as u64)?;
        } else {
            file.set_len(DEFAULT_LOGGER_SIZE as u64)?;
        }
        let config = bincode::config::standard();
        let mut mmap = unsafe { MmapMut::map_mut(&file)? };
        let main_header = MainHeader {
            magic: MAIN_MAGIC,
            first_section_offset: page_size as u16,
        };
        let nb_bytes = encode_into_slice(&main_header, &mut mmap[..], config)
            .expect("Failed to encode main header");
        assert!(nb_bytes < page_size);

        Ok(Self {
            file,
            mmap_buffer: mmap,
            page_size,
            current_position: page_size,
            config,
        })
    }

    fn unsure_size(&mut self, size: usize) -> io::Result<()> {
        // Here it is important that the memory map resizes in place.
        // According to the documentation this is something unique to Linux to be able to do that.
        // TODO: support more platforms by pausing, flushing, remapping not in place.
        eprintln!("Resizing to {}", size);
        if size as usize > self.mmap_buffer.len() {
            let ropts = RemapOptions::default().may_move(false);
            self.file
                .set_len(size as u64)
                .expect("Failed to extend file");
            unsafe { self.mmap_buffer.remap(size, ropts) }?;
        }
        Ok(())
    }

    /// The returned slice is section_size or greater.
    pub fn add_section(&mut self, entry_type: EntryType, section_size: usize) -> &mut [u8] {
        // align current_position to the next page
        self.current_position =
            (self.current_position + self.page_size - 1) & !(self.page_size - 1);

        // We have the assumption here that the section header fits into a page.
        self.unsure_size(self.current_position + section_size + self.page_size)
            .expect("Failed to resize memory map");

        let section_header = SectionHeader {
            magic: SECTION_MAGIC,
            entry_type,
            section_size: section_size as u32,
        };

        let nb_bytes = encode_into_slice(
            &section_header,
            &mut self.mmap_buffer[self.current_position..],
            self.config,
        )
        .expect("Failed to encode section header");
        assert!(nb_bytes < self.page_size);
        self.current_position += nb_bytes;
        let end_of_section = self.current_position + section_size;
        let user_buffer = &mut self.mmap_buffer[self.current_position..end_of_section];
        self.current_position = end_of_section;
        user_buffer
    }

    #[inline]
    fn allocated_len(&self) -> usize {
        self.mmap_buffer.len()
    }

    #[inline]
    fn used(&self) -> usize {
        self.current_position
    }
}

impl Drop for DataLogger {
    fn drop(&mut self) {
        self.file
            .set_len(self.current_position as u64)
            .expect("Failed to trim datalogger file");
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::TempDir;

    #[test]
    fn test_truncation_and_sections_creations() {
        // create a randomized temporary file path using tempfile
        let tmp_dir = TempDir::new().expect("could not create a tmp dir");
        let file_path = tmp_dir.path().join("test.bin");
        let used = {
            let mut logger =
                DataLogger::new(&file_path, Some(100000)).expect("Failed to create logger");
            logger.add_section(EntryType::StructuredLogLine, 1024);
            logger.add_section(EntryType::CopperList, 2048);
            let used = logger.used();
            assert!(used < 3 * 4096); // ie. 3 headers, 1 page max per
                                      // logger drops
            used
        };

        let file = OpenOptions::new()
            .read(true)
            .open(file_path)
            .expect("Could not reopne en file");
        // Check if we have correctly truncated the file
        assert_eq!(file.metadata().unwrap().len(), used as u64);
    }
}
