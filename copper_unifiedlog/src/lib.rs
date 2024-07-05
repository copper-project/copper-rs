use libc;

use std::fs::{File, OpenOptions};
use std::io::BufReader;
use std::io::Read;
use std::path::{Path, PathBuf};
use std::slice::from_raw_parts_mut;
use std::sync::{Arc, Mutex};
use std::{io, mem};

use memmap2::{Mmap, MmapMut, RemapOptions};

use bincode::config::standard;
use bincode::encode_into_slice;
use bincode::error::EncodeError;
use bincode::Encode;
use bincode::{decode_from_reader, decode_from_slice};
use bincode_derive::Decode as dDecode;
use bincode_derive::Encode as dEncode;

use copper_traits::{CuError, CuResult, UnifiedLogType, WriteStream};

const MAIN_MAGIC: [u8; 4] = [0xB4, 0xA5, 0x50, 0xFF];

const SECTION_MAGIC: [u8; 2] = [0xFA, 0x57];

/// The main header of the datalogger.
#[derive(dEncode, dDecode, Debug)]
struct MainHeader {
    magic: [u8; 4],
    first_section_offset: u16, // This is to align with a page at write time.
    page_size: u16,
}

/// Each concurrent sublogger is tracked through a section header.
/// The entry type is used to identify the type of data in the section.
#[derive(dEncode, dDecode, Debug)]
struct SectionHeader {
    magic: [u8; 2],
    entry_type: UnifiedLogType,
    section_size: u32, // offset of section_magic + section_size + page rounding -> should be the index of the next section_magic
    filled_size: u32,  // how much of the section is filled.
    emagic: [u8; 2],
}

impl Default for SectionHeader {
    fn default() -> Self {
        Self {
            magic: SECTION_MAGIC,
            entry_type: UnifiedLogType::Empty,
            section_size: 0,
            filled_size: 0,
            emagic: [SECTION_MAGIC[1], SECTION_MAGIC[0]],
        }
    }
}

/// A wrapper around a memory mapped file to write to.
struct MmapStream {
    entry_type: UnifiedLogType,
    parent_logger: Arc<Mutex<UnifiedLoggerWrite>>,
    current_section: SectionHandle,
    current_position: usize,
    minimum_allocation_amount: usize,
}

impl MmapStream {
    fn new(
        entry_type: UnifiedLogType,
        parent_logger: Arc<Mutex<UnifiedLoggerWrite>>,
        minimum_allocation_amount: usize,
    ) -> Self {
        println!("Creating a new stream of type {:?}", entry_type);
        let section = parent_logger
            .lock()
            .unwrap()
            .add_section(entry_type, minimum_allocation_amount);
        println!("Returning Creating a new stream of type {:?}", entry_type);
        Self {
            entry_type,
            parent_logger: parent_logger,
            current_section: section,
            current_position: 0,
            minimum_allocation_amount,
        }
    }
}

impl<E: Encode> WriteStream<E> for MmapStream {
    fn log(&mut self, obj: &E) -> CuResult<()> {
        let dst = self.current_section.get_user_buffer();
        let result = encode_into_slice(obj, dst, standard());
        match result {
            Ok(nb_bytes) => {
                println!("Encoded object into slice {:x?}", &dst[0..nb_bytes]);
                self.current_position += nb_bytes;
                self.current_section.used += nb_bytes as u32;
                Ok(())
            }
            Err(e) => match e {
                EncodeError::UnexpectedEnd => {
                    println!("Unexpected end, creating a new section.");
                    let mut logger_guard = self.parent_logger.lock().unwrap();
                    self.current_section =
                        logger_guard.add_section(self.entry_type, self.minimum_allocation_amount);
                    let result = encode_into_slice(
                        obj,
                        self.current_section.get_user_buffer(),
                        standard(),
                    )
                    .expect(
                        "Failed to encode object in a newly minted section. Unrecoverable failure.",
                    ); // If we fail just after creating a section, there is not much we can do, we need to bail.
                    self.current_position += result;
                    self.current_section.used += result as u32;
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
        println!("Dropping stream of type {:?}", self.entry_type);
        let mut logger_guard = self.parent_logger.lock().unwrap();
        logger_guard.unlock_section(&mut self.current_section);
        println!(
            "End of Dropping stream should drop entry type {:?}",
            self.entry_type
        );
        mem::take(&mut self.current_section);
    }
}

/// Create a new stream to write to the unifiedlogger.
pub fn stream_write<E: Encode>(
    logger: Arc<Mutex<UnifiedLoggerWrite>>,
    entry_type: UnifiedLogType,
    minimum_allocation_amount: usize,
) -> impl WriteStream<E> {
    return MmapStream::new(entry_type, logger.clone(), minimum_allocation_amount);
}

const DEFAULT_LOGGER_SIZE: usize = 1024 * 1024 * 1024; // 1GB

/// Holder of the read or write side of the datalogger.
pub enum UnifiedLogger {
    Read(UnifiedLoggerRead),
    Write(UnifiedLoggerWrite),
}

/// Use this builder to create a new DataLogger.
pub struct UnifiedLoggerBuilder {
    file_path: Option<PathBuf>,
    preallocated_size: Option<usize>,
    write: bool,
    create: bool,
}

impl UnifiedLoggerBuilder {
    pub fn new() -> Self {
        Self {
            file_path: None,
            preallocated_size: None,
            write: false,
            create: false, // This is the safest default
        }
    }

    pub fn file_path(mut self, file_path: &Path) -> Self {
        self.file_path = Some(file_path.to_path_buf());
        self
    }

    pub fn preallocated_size(mut self, preallocated_size: usize) -> Self {
        self.preallocated_size = Some(preallocated_size);
        self
    }

    pub fn write(mut self, write: bool) -> Self {
        self.write = write;
        self
    }

    pub fn create(mut self, create: bool) -> Self {
        self.create = create;
        self
    }

    pub fn build(self) -> io::Result<UnifiedLogger> {
        if self.create && !self.write {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "Cannot create a read-only file",
            ));
        }
        let file_path = self
            .file_path
            .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidInput, "File path is required"))?;
        let mut options = OpenOptions::new();
        let mut options = options.read(true);
        if self.write {
            options = options.write(true);
            if self.create {
                options = options.create(true);
            } else {
                options = options.append(true);
            }
        }

        let file = options.open(file_path)?;

        let page_size = unsafe { libc::sysconf(libc::_SC_PAGESIZE) as usize };

        if self.write && self.create {
            if let Some(size) = self.preallocated_size {
                file.set_len(size as u64)?;
            } else {
                file.set_len(DEFAULT_LOGGER_SIZE as u64)?;
            }

            let mut mmap = unsafe { MmapMut::map_mut(&file) }?;
            let main_header = MainHeader {
                magic: MAIN_MAGIC,
                first_section_offset: page_size as u16,
                page_size: page_size as u16,
            };
            let nb_bytes = encode_into_slice(&main_header, &mut mmap[..], standard())
                .expect("Failed to encode main header");
            assert!(nb_bytes < page_size);
            Ok(UnifiedLogger::Write(UnifiedLoggerWrite {
                file,
                mmap_buffer: mmap,
                page_size,
                current_global_position: page_size,
                sections_in_flight: Vec::with_capacity(16),
                flushed_until: 0,
            }))
        } else {
            let mmap = unsafe { Mmap::map(&file) }?;
            let main_header: MainHeader;
            let _read: usize;
            (main_header, _read) =
                decode_from_slice(&mmap[..], standard()).expect("Failed to decode main header");
            if main_header.magic != MAIN_MAGIC {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidData,
                    "Invalid magic number in main header",
                ));
            }
            Ok(UnifiedLogger::Read(UnifiedLoggerRead {
                file,
                mmap_buffer: mmap,
                reading_position: main_header.first_section_offset as usize,
                page_size: main_header.page_size as usize,
            }))
        }
    }
}

/// A read side of the datalogger.
pub struct UnifiedLoggerRead {
    file: File,
    mmap_buffer: Mmap,
    reading_position: usize,
    page_size: usize,
}

/// A write side of the datalogger.
pub struct UnifiedLoggerWrite {
    file: File,
    mmap_buffer: MmapMut,
    page_size: usize,
    current_global_position: usize,
    sections_in_flight: Vec<usize>,
    flushed_until: usize,
}

pub struct SectionHandle {
    section_header: SectionHeader,
    buffer: &'static mut [u8], // This includes the encoded header for end of section patching.
    used: u32,                 // this is the size of the used part of the buffer.
}

// This is for a placeholder to unsure an orderly cleanup as we dodge the borrow checker.
impl Default for SectionHandle {
    fn default() -> Self {
        Self {
            section_header: SectionHeader::default(),
            buffer: &mut [],
            used: 0,
        }
    }
}

const MAX_HEADER_SIZE: usize = 128;

impl SectionHandle {
    // The buffer is considered static as it is a dedicated piece for the section.
    pub fn create(section_header: SectionHeader, buffer: &'static mut [u8]) -> Self {
        // here we assume with are passed a valid section.
        if buffer[0] != SECTION_MAGIC[0] || buffer[1] != SECTION_MAGIC[1] {
            panic!("Invalid section buffer, magic number not found");
        }

        if buffer.len() < MAX_HEADER_SIZE {
            panic!(
                "Invalid section buffer, too small: {}, it needs to be > {}",
                buffer.len(),
                MAX_HEADER_SIZE
            );
        }

        Self {
            section_header,
            buffer,
            used: 0,
        }
    }
    pub fn get_user_buffer(&mut self) -> &mut [u8] {
        &mut self.buffer[MAX_HEADER_SIZE + self.used as usize..]
    }
}

impl Drop for SectionHandle {
    fn drop(&mut self) {
        // no need to do anything if we never used the section.
        if self.used == 0 {
            println!(
                "Section {:?}: Dropping with no update, empty section",
                self.section_header.entry_type
            );
            return;
        }

        self.section_header.filled_size = self.used;
        println!(
            "Section {:?}: Dropping updating the header, we used {}",
            self.section_header.entry_type, self.used
        );

        let sz = encode_into_slice(&self.section_header, &mut self.buffer, standard())
            .expect("Failed to encode section header");

        // Fill with 0x43 the buffer between the end of the section header and MAX_HEADER_SIZE
        for i in sz..MAX_HEADER_SIZE {
            self.buffer[i] = 0x43;
        }

        println!(
            "Section buffer: {:x?} [...], header = {} bytes",
            &self.buffer[0usize..MAX_HEADER_SIZE + self.used as usize],
            sz
        );
    }
}

impl UnifiedLoggerWrite {
    fn unsure_size(&mut self, size: usize) -> io::Result<()> {
        // Here it is important that the memory map resizes in place.
        // According to the documentation this is something unique to Linux to be able to do that.
        // TODO: support more platforms by pausing, flushing, remapping not in place.
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

    fn flush(&mut self) {
        self.flush_until(self.current_global_position);
    }

    fn unlock_section(&mut self, section: &mut SectionHandle) {
        println!(
            "Section {:?}: Unlocking section, used {}",
            section.section_header.entry_type, section.used
        );
        let base = self.mmap_buffer.as_mut_ptr() as usize;
        let section_buffer_addr = section.buffer.as_mut_ptr() as usize;
        self.sections_in_flight
            .retain(|&x| x != section_buffer_addr as usize - base);
        if self.sections_in_flight.is_empty() {
            self.flush_until(self.current_global_position);
            return;
        }
        if self.flushed_until < self.sections_in_flight[0] {
            self.flush_until(self.sections_in_flight[0]);
        }
    }

    /// The returned slice is section_size or greater.
    fn add_section(&mut self, entry_type: UnifiedLogType, section_size: usize) -> SectionHandle {
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
            filled_size: 0u32,
            emagic: [SECTION_MAGIC[1], SECTION_MAGIC[0]],
        };

        println!(
            "Section {:?}: encoding into slice at {}",
            entry_type, self.current_global_position
        );
        let nb_bytes = encode_into_slice(
            &section_header,
            &mut self.mmap_buffer[self.current_global_position..],
            standard(),
        )
        .expect("Failed to encode section header");
        assert!(nb_bytes < self.page_size);
        println!(
            "Section {:?}: Section created with header using: {}",
            entry_type, nb_bytes
        );

        // save the position to keep track for in flight sections
        self.sections_in_flight.push(self.current_global_position);
        let end_of_section = self.current_global_position + section_size;
        let user_buffer = &mut self.mmap_buffer[self.current_global_position..end_of_section];

        // here we have the guarantee for exclusive access to that memory for the lifetime of the handle, the borrow checker cannot understand that ever.
        let handle_buffer =
            unsafe { from_raw_parts_mut(user_buffer.as_mut_ptr(), user_buffer.len()) };

        self.current_global_position = end_of_section;

        println!("Section {:?}: new with size {}", entry_type, section_size);
        SectionHandle::create(section_header, handle_buffer)
    }

    #[allow(dead_code)]
    #[inline]
    fn allocated_len(&self) -> usize {
        self.mmap_buffer.len()
    }

    #[allow(dead_code)]
    #[inline]
    fn used(&self) -> usize {
        self.current_global_position
    }
}

impl Drop for UnifiedLoggerWrite {
    fn drop(&mut self) {
        println!("Dropping UnifiedLoggerWrite logger");
        let section = self.add_section(UnifiedLogType::LastEntry, 4096);
        drop(section);
        self.flush();
        self.file
            .set_len(self.current_global_position as u64)
            .expect("Failed to trim datalogger file");
        println!("End of Dropping UnifiedLoggerWrite logger");
    }
}

impl UnifiedLoggerRead {
    pub fn read_next_section_type(
        &mut self,
        datalogtype: UnifiedLogType,
    ) -> CuResult<Option<Vec<u8>>> {
        // TODO: eventually implement a 0 copy of this too.
        loop {
            let header_result = self.read_section_header();
            if let Err(error) = header_result {
                return Err(CuError::new_with_cause(
                    "Could not read a sections header",
                    error,
                ));
            };
            let header = header_result.unwrap();

            // Reached the end of file
            if header.entry_type == UnifiedLogType::LastEntry {
                return Ok(None);
            }

            // Found a section of the requested type
            if header.entry_type == datalogtype {
                return Ok(Some(self.read_section_content(&header)?));
            }

            // Keep reading until we find the requested type
            self.reading_position += header.section_size as usize;

            if self.reading_position >= self.mmap_buffer.len() {
                return Err("Corrupted Log, past end of file".into());
            }
        }
    }

    /// Reads the section from the section header pos.
    pub fn read_section(&mut self) -> CuResult<Vec<u8>> {
        let read_result = self.read_section_header();
        if let Err(error) = read_result {
            return Err(CuError::new_with_cause(
                "Could not read a sections header",
                error,
            ));
        };

        self.read_section_content(&read_result.unwrap())
    }

    /// Reads the section content from the section header pos.
    fn read_section_content(&mut self, header: &SectionHeader) -> CuResult<Vec<u8>> {
        // TODO: we could optimize by asking the buffer to fill
        println!("Reading section of size {}", header.filled_size);

        let mut section = vec![0; header.filled_size as usize];
        let start_of_data = self.reading_position + MAX_HEADER_SIZE;
        section.copy_from_slice(
            &self.mmap_buffer[start_of_data..start_of_data + header.filled_size as usize],
        );

        println!("Read section: {:x?}", section);

        Ok(section)
    }

    fn read_section_header(&mut self) -> CuResult<SectionHeader> {
        let section_header: SectionHeader;
        let read: usize;
        (section_header, read) =
            decode_from_slice(&self.mmap_buffer[self.reading_position..], standard())
                .expect("Failed to decode section header");
        if section_header.magic != SECTION_MAGIC {
            return Err("Invalid magic number in section header".into());
        }
        println!("Read section header: {:?}", section_header);

        Ok(section_header)
    }
}

/// This a a convience wrapper around the UnifiedLoggerRead to implement the Read trait.
pub struct UnifiedLoggerIOReader {
    logger: UnifiedLoggerRead,
    log_type: UnifiedLogType,
    buffer: Vec<u8>,
    buffer_pos: usize,
}

impl UnifiedLoggerIOReader {
    pub fn new(logger: UnifiedLoggerRead, log_type: UnifiedLogType) -> Self {
        Self {
            logger,
            log_type,
            buffer: Vec::new(),
            buffer_pos: 0,
        }
    }

    fn fill_buffer(&mut self) -> io::Result<()> {
        println!("Filling buffer");
        match self.logger.read_next_section_type(self.log_type) {
            Ok(Some(section)) => {
                self.buffer = section;
                self.buffer_pos = 0;
            }
            Ok(None) => (), // No more sections of this type
            Err(e) => return Err(io::Error::new(io::ErrorKind::Other, e.to_string())),
        }
        println!("Filled buffer done");
        Ok(())
    }
}

impl Read for UnifiedLoggerIOReader {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        //println!("buf.len() {}", buf.len());
        //println!("self.buffer.len() {}", self.buffer.len());
        //println!("self.buffer_pos {}", self.buffer_pos);
        if self.buffer_pos >= self.buffer.len() {
            self.fill_buffer()?;
        }

        // If we still have no data after trying to fill the buffer, we're at EOF
        if self.buffer_pos >= self.buffer.len() {
            return Ok(0);
        }

        // Copy as much as we can from the buffer to `buf`
        let len = std::cmp::min(buf.len(), self.buffer.len() - self.buffer_pos);
        buf[..len].copy_from_slice(&self.buffer[self.buffer_pos..self.buffer_pos + len]);
        self.buffer_pos += len;
        //println!("Read {} bytes", len);
        Ok(len)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;
    use tempfile::TempDir;
    fn make_a_logger(tmp_dir: &TempDir) -> (Arc<Mutex<UnifiedLoggerWrite>>, PathBuf) {
        let file_path = tmp_dir.path().join("test.bin");
        let UnifiedLogger::Write(data_logger) = UnifiedLoggerBuilder::new()
            .write(true)
            .create(true)
            .file_path(&file_path)
            .preallocated_size(100000)
            .build()
            .expect("Failed to create logger")
        else {
            panic!("Failed to create logger")
        };

        (Arc::new(Mutex::new(data_logger)), file_path)
    }

    #[test]
    fn test_truncation_and_sections_creations() {
        let tmp_dir = TempDir::new().expect("could not create a tmp dir");
        let file_path = tmp_dir.path().join("test.bin");
        let _used = {
            let UnifiedLogger::Write(mut logger) = UnifiedLoggerBuilder::new()
                .write(true)
                .create(true)
                .file_path(&file_path)
                .preallocated_size(100000)
                .build()
                .expect("Failed to create logger")
            else {
                panic!("Failed to create logger")
            };
            logger.add_section(UnifiedLogType::StructuredLogLine, 1024);
            logger.add_section(UnifiedLogType::CopperList, 2048);
            let used = logger.used();
            assert!(used < 4 * 4096); // ie. 3 headers, 1 page max per
                                      // logger drops
            used
        };

        let _file = OpenOptions::new()
            .read(true)
            .open(file_path)
            .expect("Could not reopen the file");
        // Check if we have correctly truncated the file
        // TODO: recompute this math
        //assert_eq!(
        //    file.metadata().unwrap().len(),
        //    (used + size_of::<SectionHeader>()) as u64
        //);
    }

    #[test]
    fn test_one_section_self_cleaning() {
        let tmp_dir = TempDir::new().expect("could not create a tmp dir");
        let (logger, _) = make_a_logger(&tmp_dir);
        {
            let _stream =
                stream_write::<()>(logger.clone(), UnifiedLogType::StructuredLogLine, 1024);
            assert_eq!(logger.lock().unwrap().sections_in_flight.len(), 1);
        }
        assert_eq!(logger.lock().unwrap().sections_in_flight.len(), 0);
        let logger = logger.lock().unwrap();
        assert_eq!(logger.flushed_until, logger.current_global_position);
    }

    #[test]
    fn test_two_sections_self_cleaning_in_order() {
        let tmp_dir = TempDir::new().expect("could not create a tmp dir");
        let (logger, _) = make_a_logger(&tmp_dir);
        let s1 = stream_write::<()>(logger.clone(), UnifiedLogType::StructuredLogLine, 1024);
        assert_eq!(logger.lock().unwrap().sections_in_flight.len(), 1);
        let s2 = stream_write::<()>(logger.clone(), UnifiedLogType::StructuredLogLine, 1024);
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
        let s1 = stream_write::<()>(logger.clone(), UnifiedLogType::StructuredLogLine, 1024);
        assert_eq!(logger.lock().unwrap().sections_in_flight.len(), 1);
        let s2 = stream_write::<()>(logger.clone(), UnifiedLogType::StructuredLogLine, 1024);
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
            let mut stream = stream_write(logger.clone(), UnifiedLogType::StructuredLogLine, 1024);
            stream.log(&1u32).unwrap();
            stream.log(&2u32).unwrap();
            stream.log(&3u32).unwrap();
        }
        drop(logger);
        let UnifiedLogger::Read(mut dl) = UnifiedLoggerBuilder::new()
            .file_path(&f.to_path_buf())
            .build()
            .expect("Failed to build logger")
        else {
            panic!("Failed to build logger");
        };
        let section = dl
            .read_next_section_type(UnifiedLogType::StructuredLogLine)
            .expect("Failed to read section");
        assert!(section.is_some());
        let section = section.unwrap();

        let mut reader = BufReader::new(&section[..]);
        let v1: u32 = decode_from_reader(&mut reader, standard()).unwrap();
        let v2: u32 = decode_from_reader(&mut reader, standard()).unwrap();
        let v3: u32 = decode_from_reader(&mut reader, standard()).unwrap();
        assert_eq!(v1, 1);
        assert_eq!(v2, 2);
        assert_eq!(v3, 3);
    }

    /// Mimic a basic CopperList implementation.

    #[derive(Debug, dEncode, dDecode)]
    enum CopperListStateMock {
        Free,
        ProcessingTasks,
        BeingSerialized,
    }

    #[derive(dEncode, dDecode)]
    struct CopperList<P: Encode> {
        state: CopperListStateMock,
        payload: P, // This is generated from the runtime.
    }

    #[test]
    fn test_copperlist_list_like_logging() {
        let tmp_dir = TempDir::new().expect("could not create a tmp dir");
        let (logger, f) = make_a_logger(&tmp_dir);
        let p = f.as_path();
        println!("Path : {:?}", p);
        {
            let mut stream = stream_write(logger.clone(), UnifiedLogType::CopperList, 1024);
            let cl0 = CopperList {
                state: CopperListStateMock::Free,
                payload: (1u32, 2u32, 3u32),
            };
            let cl1 = CopperList {
                state: CopperListStateMock::ProcessingTasks,
                payload: (4u32, 5u32, 6u32),
            };
            stream.log(&cl0).unwrap();
            stream.log(&cl1).unwrap();
        }
        drop(logger);

        let UnifiedLogger::Read(mut dl) = UnifiedLoggerBuilder::new()
            .file_path(&f.to_path_buf())
            .build()
            .expect("Failed to build logger")
        else {
            panic!("Failed to build logger");
        };
        let section = dl
            .read_next_section_type(UnifiedLogType::CopperList)
            .expect("Failed to read section");
        assert!(section.is_some());
        let section = section.unwrap();

        let mut reader = BufReader::new(&section[..]);
        let cl0: CopperList<(u32, u32, u32)> = decode_from_reader(&mut reader, standard()).unwrap();
        let cl1: CopperList<(u32, u32, u32)> = decode_from_reader(&mut reader, standard()).unwrap();
        assert_eq!(cl0.payload.1, 2);
        assert_eq!(cl1.payload.2, 6);
    }
}
