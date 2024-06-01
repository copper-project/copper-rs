use libc;

use std::fs::{File, OpenOptions};
use std::io;
use std::io::Read;
use std::io::{BufReader, Seek};
use std::path::Path;
use std::slice::from_raw_parts_mut;
use std::sync::{Arc, Mutex};

use memmap2::{MmapMut, RemapOptions};

use bincode::config::standard;
use bincode::de::read::Reader;
use bincode::encode_into_slice;
use bincode::error::EncodeError;
use bincode::Encode;
use bincode::{decode_from_reader, decode_from_slice};
use bincode_derive::Decode as dDecode;
use bincode_derive::Encode as dEncode;

use copper::{CuError, CuResult, DataLogType, Stream};

const MAIN_MAGIC: [u8; 4] = [0xB4, 0xA5, 0x50, 0xFF];

const SECTION_MAGIC: [u8; 2] = [0xFA, 0x57];

#[derive(dEncode, dDecode)]
struct MainHeader {
    magic: [u8; 4],
    first_section_offset: u16, // This is to align with a page at write time.
}

#[derive(dEncode, dDecode)]
struct SectionHeader {
    magic: [u8; 2],
    entry_type: DataLogType,
    section_size: u32, // offset of section_magic + section_size -> should be the index of the next section_magic
}

struct MmapStream {
    entry_type: DataLogType,
    parent_logger: Arc<Mutex<DataLogger>>,
    current_slice: &'static mut [u8],
    current_position: usize,
    minimum_allocation_amount: usize,
}

impl MmapStream {
    fn new(
        entry_type: DataLogType,
        parent_logger: Arc<Mutex<DataLogger>>,
        current_slice: &'static mut [u8],
        minimum_allocation_amount: usize,
    ) -> Self {
        Self {
            entry_type,
            parent_logger,
            current_slice,
            current_position: 0,
            minimum_allocation_amount,
        }
    }
}

impl Stream for MmapStream {
    fn log(&mut self, obj: &impl Encode) -> CuResult<()> {
        let result = encode_into_slice(
            obj,
            &mut self.current_slice[self.current_position..],
            bincode::config::standard(),
        );
        match result {
            Ok(nb_bytes) => {
                self.current_position += nb_bytes;
                Ok(())
            }
            Err(e) => match e {
                EncodeError::UnexpectedEnd => {
                    let mut logger_guard = self.parent_logger.lock().unwrap();
                    // here compute with timing what should be the reasonable amount
                    let underlying_slice =
                        logger_guard.add_section(self.entry_type, self.minimum_allocation_amount);

                    // here we have the guarantee for exclusive access to that memory, the borrow checker cannot understand that ever.
                    self.current_slice = unsafe {
                        from_raw_parts_mut(underlying_slice.as_mut_ptr(), underlying_slice.len())
                    };
                    Ok(())
                }
                _ => {
                    let err =
                        <&str as Into<CuError>>::into("Unexpected error while encoding object.")
                            .add_cause(e.to_string().as_str());
                    Err(err)
                }
            },
        }
    }
}

impl Drop for MmapStream {
    fn drop(&mut self) {
        let mut logger_guard = self.parent_logger.lock().unwrap();
        logger_guard.unlock_section(self.current_slice);
    }
}

pub fn stream(
    logger: Arc<Mutex<DataLogger>>,
    entry_type: DataLogType,
    minimum_allocation_amount: usize,
) -> impl Stream {
    let clone = logger.clone();
    let mut logger = logger.lock().unwrap();
    let underlying_slice = logger.add_section(entry_type, minimum_allocation_amount);
    MmapStream::new(
        entry_type,
        clone,
        // here we have the guarantee for exclusive access to that memory, the borrow checker cannot understand that ever.
        unsafe { from_raw_parts_mut(underlying_slice.as_mut_ptr(), underlying_slice.len()) },
        minimum_allocation_amount,
    )
}

const DEFAULT_LOGGER_SIZE: usize = 1024 * 1024 * 1024; // 1GB

pub struct DataLogger {
    file: File,
    mmap_buffer: MmapMut,
    page_size: usize,
    current_global_position: usize,
    sections_in_flight: Vec<usize>,
    flushed_until: usize,
}

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
        let mut mmap = unsafe { MmapMut::map_mut(&file)? };
        let main_header = MainHeader {
            magic: MAIN_MAGIC,
            first_section_offset: page_size as u16,
        };
        let nb_bytes = encode_into_slice(&main_header, &mut mmap[..], standard())
            .expect("Failed to encode main header");
        assert!(nb_bytes < page_size);

        Ok(Self {
            file,
            mmap_buffer: mmap,
            page_size,
            current_global_position: page_size,
            sections_in_flight: Vec::with_capacity(16),
            flushed_until: 0,
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

    fn flush_until(&mut self, position: usize) {
        self.mmap_buffer
            .flush_async_range(self.flushed_until, position)
            .expect("Failed to flush memory map");
        self.flushed_until = position;
    }

    fn unlock_section(&mut self, section: &mut [u8]) {
        let base = self.mmap_buffer.as_mut_ptr() as usize;
        self.sections_in_flight
            .retain(|&x| x != section.as_mut_ptr() as usize - base);
        if self.sections_in_flight.is_empty() {
            self.flush_until(self.current_global_position);
            return;
        }
        if self.flushed_until < self.sections_in_flight[0] {
            self.flush_until(self.sections_in_flight[0]);
        }
    }

    /// The returned slice is section_size or greater.
    fn add_section(&mut self, entry_type: DataLogType, section_size: usize) -> &mut [u8] {
        // align current_position to the next page
        self.current_global_position =
            (self.current_global_position + self.page_size - 1) & !(self.page_size - 1);

        // We have the assumption here that the section header fits into a page.
        self.unsure_size(self.current_global_position + section_size + self.page_size)
            .expect("Failed to resize memory map");

        let section_header = SectionHeader {
            magic: SECTION_MAGIC,
            entry_type,
            section_size: section_size as u32,
        };

        let nb_bytes = encode_into_slice(
            &section_header,
            &mut self.mmap_buffer[self.current_global_position..],
            standard(),
        )
        .expect("Failed to encode section header");
        assert!(nb_bytes < self.page_size);

        self.current_global_position += nb_bytes;
        let end_of_section = self.current_global_position + section_size;
        let user_buffer = &mut self.mmap_buffer[self.current_global_position..end_of_section];

        // save the position to keep track for in flight sections
        self.sections_in_flight.push(self.current_global_position);
        self.current_global_position = end_of_section;

        user_buffer
    }

    #[inline]
    fn allocated_len(&self) -> usize {
        self.mmap_buffer.len()
    }

    #[inline]
    fn used(&self) -> usize {
        self.current_global_position
    }
}

impl Drop for DataLogger {
    fn drop(&mut self) {
        self.file
            .set_len(self.current_global_position as u64)
            .expect("Failed to trim datalogger file");
    }
}

// Section iterator returning the [u8] of the current section
pub struct SectionIterator {
    reader: BufReader<File>,
    datalogtype: Option<DataLogType>,
}

impl Iterator for SectionIterator {
    type Item = Vec<u8>;

    fn next(&mut self) -> Option<Self::Item> {
        let answer: Option<Vec<u8>> = loop {
            if let Ok(section_header) =
                decode_from_reader::<SectionHeader, _, _>(&mut self.reader, standard())
            {
                let mut section = vec![0; section_header.section_size as usize];
                let read = self.reader.read_exact(section.as_mut_slice());
                if read.is_err() {
                    break None;
                }
                if let Some(datalogtype) = self.datalogtype {
                    if section_header.entry_type == datalogtype {
                        break Some(section);
                    }
                }
            } else {
                break None;
            }
        };
        answer
    }
}

// make an iterator to read back a serialzed datalogger from a file
// optionally filter by type
pub fn read_datalogger(
    file_path: &Path,
    datalogtype: Option<DataLogType>,
) -> io::Result<impl Iterator<Item = Vec<u8>>> {
    let mut file = OpenOptions::new().read(true).open(file_path)?;
    let mut header = [0; 4096];
    let s = file.read(&mut header).unwrap();
    if s < 4096 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "Failed to read main header",
        ));
    }

    let main_header: MainHeader;
    let read: usize;
    (main_header, read) =
        decode_from_slice(&header[..], standard()).expect("Failed to decode main header");

    if main_header.magic != MAIN_MAGIC {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "Invalid magic number in main header",
        ));
    }

    Ok(SectionIterator {
        reader: BufReader::new(file),
        datalogtype,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;
    use tempfile::TempDir;
    fn make_a_logger(tmp_dir: &TempDir) -> (Arc<Mutex<DataLogger>>, PathBuf) {
        let file_path = tmp_dir.path().join("test.bin");
        (
            Arc::new(Mutex::new(
                DataLogger::new(&file_path, Some(100000)).expect("Failed to create logger"),
            )),
            file_path,
        )
    }

    #[test]
    fn test_truncation_and_sections_creations() {
        // create a randomized temporary file path using tempfile
        let tmp_dir = TempDir::new().expect("could not create a tmp dir");
        let file_path = tmp_dir.path().join("test.bin");
        let used = {
            let mut logger =
                DataLogger::new(&file_path, Some(100000)).expect("Failed to create logger");
            logger.add_section(DataLogType::StructuredLogLine, 1024);
            logger.add_section(DataLogType::CopperList, 2048);
            let used = logger.used();
            assert!(used < 3 * 4096); // ie. 3 headers, 1 page max per
                                      // logger drops
            used
        };

        let file = OpenOptions::new()
            .read(true)
            .open(file_path)
            .expect("Could not reopen the file");
        // Check if we have correctly truncated the file
        assert_eq!(file.metadata().unwrap().len(), used as u64);
    }

    #[test]
    fn test_one_section_self_cleaning() {
        let tmp_dir = TempDir::new().expect("could not create a tmp dir");
        let (logger, _) = make_a_logger(&tmp_dir);
        {
            let stream = stream(logger.clone(), DataLogType::StructuredLogLine, 1024);
            assert_eq!(logger.lock().unwrap().sections_in_flight.len(), 1);
        }
        let lg = logger.lock().unwrap();
        assert_eq!(lg.sections_in_flight.len(), 0);
        assert_eq!(lg.flushed_until, lg.current_global_position);
    }

    #[test]
    fn test_two_sections_self_cleaning_in_order() {
        let tmp_dir = TempDir::new().expect("could not create a tmp dir");
        let (logger, _) = make_a_logger(&tmp_dir);
        let s1 = stream(logger.clone(), DataLogType::StructuredLogLine, 1024);
        assert_eq!(logger.lock().unwrap().sections_in_flight.len(), 1);
        let s2 = stream(logger.clone(), DataLogType::StructuredLogLine, 1024);
        assert_eq!(logger.lock().unwrap().sections_in_flight.len(), 2);
        drop(s2);
        assert_eq!(logger.lock().unwrap().sections_in_flight.len(), 1);
        drop(s1);
        let lg = logger.lock().unwrap();
        assert_eq!(lg.sections_in_flight.len(), 0);
        assert_eq!(lg.flushed_until, lg.current_global_position);
    }

    #[test]
    fn test_two_sections_self_cleaning_out_of_order() {
        let tmp_dir = TempDir::new().expect("could not create a tmp dir");
        let (logger, _) = make_a_logger(&tmp_dir);
        let s1 = stream(logger.clone(), DataLogType::StructuredLogLine, 1024);
        assert_eq!(logger.lock().unwrap().sections_in_flight.len(), 1);
        let s2 = stream(logger.clone(), DataLogType::StructuredLogLine, 1024);
        assert_eq!(logger.lock().unwrap().sections_in_flight.len(), 2);
        drop(s1);
        assert_eq!(logger.lock().unwrap().sections_in_flight.len(), 1);
        drop(s2);
        let lg = logger.lock().unwrap();
        assert_eq!(lg.sections_in_flight.len(), 0);
        assert_eq!(lg.flushed_until, lg.current_global_position);
    }

    #[test]
    fn test_write_then_read_one_section() {
        let tmp_dir = TempDir::new().expect("could not create a tmp dir");
        let (logger, f) = make_a_logger(&tmp_dir);
        let p = f.as_path();
        println!("Path : {:?}", p);
        {
            let mut stream = stream(logger.clone(), DataLogType::StructuredLogLine, 1024);
            stream.log(&1u32);
            stream.log(&2u32);
            stream.log(&3u32);
        }
        let mut sections = read_datalogger(p, Some(DataLogType::StructuredLogLine)).unwrap();
        let section = sections.next().unwrap();
        let mut reader = BufReader::new(&section[..]);
        let v1: u32 = decode_from_reader(&mut reader, standard()).unwrap();
        let v2: u32 = decode_from_reader(&mut reader, standard()).unwrap();
        let v3: u32 = decode_from_reader(&mut reader, standard()).unwrap();
        assert_eq!(v1, 1);
        assert_eq!(v2, 2);
        assert_eq!(v3, 3);
    }
}
