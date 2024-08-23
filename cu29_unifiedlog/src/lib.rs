use libc;

use memmap2::{Mmap, MmapMut};
use std::fmt::{Debug, Formatter};
use std::fs::{File, OpenOptions};
use std::io::Read;
use std::path::{Path, PathBuf};
use std::slice::from_raw_parts_mut;
use std::sync::{Arc, Mutex};
use std::{io, mem};

use bincode::config::standard;
use bincode::decode_from_slice;
use bincode::encode_into_slice;
use bincode::error::EncodeError;
use bincode::{Decode, Encode};
use cu29_traits::{CuError, CuResult, UnifiedLogType, WriteStream};

const MAIN_MAGIC: [u8; 4] = [0xB4, 0xA5, 0x50, 0xFF];

const SECTION_MAGIC: [u8; 2] = [0xFA, 0x57];

/// The main file header of the datalogger.
#[derive(Encode, Decode, Debug)]
struct MainHeader {
    magic: [u8; 4],            // Magic number to identify the file.
    first_section_offset: u16, // This is to align with a page at write time.
    page_size: u16,
}

/// Each concurrent sublogger is tracked through a section header.
/// They form a linked list of sections.
/// The entry type is used to identify the type of data in the section.
#[derive(Encode, Decode, Debug)]
pub struct SectionHeader {
    magic: [u8; 2], // Magic number to identify the section.
    entry_type: UnifiedLogType,
    section_size: u32, // offset from the first byte of this header to the first byte of the next header (MAGIC to MAGIC).
    filled_size: u32,  // how much of the section is filled.
}

const MAX_HEADER_SIZE: usize = mem::size_of::<SectionHeader>() + 3usize; // 3 == additional worse case scenario for the 3 int variable encoding

impl Default for SectionHeader {
    fn default() -> Self {
        Self {
            magic: SECTION_MAGIC,
            entry_type: UnifiedLogType::Empty,
            section_size: 0,
            filled_size: 0,
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
        let section = parent_logger
            .lock()
            .unwrap()
            .add_section(entry_type, minimum_allocation_amount);
        Self {
            entry_type,
            parent_logger,
            current_section: section,
            current_position: 0,
            minimum_allocation_amount,
        }
    }
}

impl Debug for MmapStream {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(f, "MmapStream {{ entry_type: {:?}, current_position: {}, minimum_allocation_amount: {} }}", self.entry_type, self.current_position, self.minimum_allocation_amount)
    }
}

impl<E: Encode> WriteStream<E> for MmapStream {
    fn log(&mut self, obj: &E) -> CuResult<()> {
        let dst = self.current_section.get_user_buffer();
        let result = encode_into_slice(obj, dst, standard());
        match result {
            Ok(nb_bytes) => {
                self.current_position += nb_bytes;
                self.current_section.used += nb_bytes as u32;
                Ok(())
            }
            Err(e) => match e {
                EncodeError::UnexpectedEnd => {
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
        let mut logger_guard = self.parent_logger.lock().unwrap();
        logger_guard.unlock_section(&mut self.current_section);
        mem::take(&mut self.current_section);
    }
}

/// Create a new stream to write to the unifiedlogger.
pub fn stream_write<E: Encode>(
    logger: Arc<Mutex<UnifiedLoggerWrite>>,
    entry_type: UnifiedLogType,
    minimum_allocation_amount: usize,
) -> impl WriteStream<E> {
    MmapStream::new(entry_type, logger.clone(), minimum_allocation_amount)
}

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
        let page_size = unsafe { libc::sysconf(libc::_SC_PAGESIZE) as usize };

        if self.write && self.create {
            let ulw = UnifiedLoggerWrite::new(
                &self.file_path.unwrap(),
                self.preallocated_size.unwrap(),
                page_size,
            );

            Ok(UnifiedLogger::Write(ulw))
        } else {
            let file_path = self.file_path.ok_or_else(|| {
                io::Error::new(io::ErrorKind::InvalidInput, "File path is required")
            })?;
            let mut options = OpenOptions::new();
            let options = options.read(true);
            let file = options.open(file_path)?;
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
    #[allow(dead_code)]
    file: File,
    mmap_buffer: Mmap,
    reading_position: usize,
    #[allow(dead_code)]
    page_size: usize,
}

struct SlabEntry {
    file: File,
    mmap_buffer: MmapMut,
    current_global_position: usize,
    sections_in_flight: Vec<usize>,
    flushed_until: usize,
    page_size: usize,
}

impl Drop for SlabEntry {
    fn drop(&mut self) {
        self.close();
        if self.sections_in_flight.len() > 0 {
            self.sections_in_flight.clear(); // This is the best we can do.
            eprintln!("Sections in flight in a slab while it is being dropped.");
        }
    }
}

pub enum AllocatedSection {
    NoMoreSpace,
    Section(SectionHandle),
}

impl SlabEntry {
    fn new(file: File, page_size: usize) -> Self {
        let mmap_buffer = unsafe { MmapMut::map_mut(&file).expect("Failed to map file") };
        Self {
            file,
            mmap_buffer,
            current_global_position: 0,
            sections_in_flight: Vec::with_capacity(16),
            flushed_until: 0,
            page_size,
        }
    }

    fn flush_until(&mut self, position: usize) {
        self.mmap_buffer
            .flush_async_range(self.flushed_until, position)
            .expect("Failed to flush memory map");
        self.flushed_until = position;
    }

    fn unlock_section(&mut self, section: &mut SectionHandle) {
        let base = self.mmap_buffer.as_mut_ptr() as usize;
        let section_buffer_addr = section.buffer.as_mut_ptr() as usize;
        self.sections_in_flight
            .retain(|&x| x != section_buffer_addr - base);
        if self.sections_in_flight.is_empty() {
            self.flush_until(self.current_global_position);
            return;
        }
        if self.flushed_until < self.sections_in_flight[0] {
            self.flush_until(self.sections_in_flight[0]);
        }
    }

    #[inline]
    fn align_to_next_page(&self, ptr: usize) -> usize {
        (ptr + self.page_size - 1) & !(self.page_size - 1)
    }

    /// The returned slice is section_size or greater.
    fn add_section(
        &mut self,
        entry_type: UnifiedLogType,
        requested_section_size: usize,
    ) -> AllocatedSection {
        // align current_position to the next page
        self.current_global_position = self.align_to_next_page(self.current_global_position);
        let section_size = self.align_to_next_page(requested_section_size) as u32;

        // We need to have enough space to store the section in that slab
        if self.current_global_position + section_size as usize > self.mmap_buffer.len() {
            return AllocatedSection::NoMoreSpace;
        }

        let section_header = SectionHeader {
            magic: SECTION_MAGIC,
            entry_type,
            section_size,
            filled_size: 0u32,
        };

        let nb_bytes = encode_into_slice(
            &section_header,
            &mut self.mmap_buffer[self.current_global_position..],
            standard(),
        )
        .expect("Failed to encode section header");
        assert!(nb_bytes < self.page_size);

        // save the position to keep track for in flight sections
        self.sections_in_flight.push(self.current_global_position);
        let end_of_section = self.current_global_position + requested_section_size;
        let user_buffer = &mut self.mmap_buffer[self.current_global_position..end_of_section];

        // here we have the guarantee for exclusive access to that memory for the lifetime of the handle, the borrow checker cannot understand that ever.
        let handle_buffer =
            unsafe { from_raw_parts_mut(user_buffer.as_mut_ptr(), user_buffer.len()) };

        self.current_global_position = end_of_section;
        println!("Log Used {}MB", self.current_global_position / 1024 / 1024);

        AllocatedSection::Section(SectionHandle::create(section_header, handle_buffer))
    }

    fn close(&mut self) {
        self.flush_until(self.current_global_position);
        self.file
            .set_len(self.current_global_position as u64)
            .expect("Failed to trim datalogger file");
    }

    #[cfg(test)]
    fn used(&self) -> usize {
        self.current_global_position
    }
}

/// A SectionHandle is a handle to a section in the datalogger.
/// It allows to track the lifecycle of a section of the datalogger.
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
        if self.section_header.entry_type == UnifiedLogType::Empty || self.used == 0 {
            return;
        }

        self.section_header.filled_size = self.used;

        let _sz = encode_into_slice(&self.section_header, &mut self.buffer, standard())
            .expect("Failed to encode section header");
    }
}

/// A write side of the datalogger.
pub struct UnifiedLoggerWrite {
    /// the front slab is the current active slab for any new section.
    front_slab: SlabEntry,
    /// the back slab is the previous slab that is being flushed.
    back_slab: Option<SlabEntry>,
    /// base file path to create the backing files from.
    base_file_path: PathBuf,
    /// allocation size for the backing files.
    slab_size: usize,
    /// current suffix for the backing files.
    front_slab_suffix: usize,
}

fn build_file_path(file_path: &PathBuf, suffix: usize) -> PathBuf {
    let mut file_path = file_path.clone();
    let file_name = file_path.file_name().unwrap().to_str().unwrap();
    let mut file_name = file_name.split('.').collect::<Vec<&str>>();
    let extension = file_name.pop().unwrap();
    let file_name = file_name.join(".");
    let file_name = format!("{}_{}.{}", file_name, suffix, extension);
    file_path.set_file_name(file_name);
    file_path
}

fn make_slab_file(base_file_path: &PathBuf, slab_size: usize, slab_suffix: usize) -> File {
    let file_path = build_file_path(base_file_path, slab_suffix);
    let file = OpenOptions::new()
        .read(true)
        .write(true)
        .create(true)
        .open(&file_path)
        .expect("Failed to open file");
    file.set_len(slab_size as u64)
        .expect("Failed to set file length");
    file
}

impl UnifiedLoggerWrite {
    fn make_new_mmfile(&mut self) -> File {
        let file = make_slab_file(&self.base_file_path, self.slab_size, self.front_slab_suffix);
        self.front_slab_suffix += 1;
        file
    }

    fn new(base_file_path: &PathBuf, slab_size: usize, page_size: usize) -> Self {
        let file = make_slab_file(base_file_path, slab_size, 0);
        let mut front_slab = SlabEntry::new(file, page_size);

        // This is the first slab so add the main header.
        let main_header = MainHeader {
            magic: MAIN_MAGIC,
            first_section_offset: page_size as u16,
            page_size: page_size as u16,
        };
        let nb_bytes = encode_into_slice(&main_header, &mut front_slab.mmap_buffer[..], standard())
            .expect("Failed to encode main header");
        assert!(nb_bytes < page_size);
        front_slab.current_global_position = page_size; // align to the next page

        Self {
            front_slab,
            back_slab: None,
            base_file_path: base_file_path.clone(),
            slab_size,
            front_slab_suffix: 0,
        }
    }

    fn unlock_section(&mut self, section: &mut SectionHandle) {
        self.front_slab.unlock_section(section);
    }

    /// The returned slice is section_size or greater.
    fn add_section(
        &mut self,
        entry_type: UnifiedLogType,
        requested_section_size: usize,
    ) -> SectionHandle {
        let maybe_section = self
            .front_slab
            .add_section(entry_type, requested_section_size);

        match maybe_section {
            AllocatedSection::NoMoreSpace => {
                // move the front slab to the back slab.
                let new_slab = SlabEntry::new(self.make_new_mmfile(), self.front_slab.page_size);
                self.back_slab = Some(mem::replace(&mut self.front_slab, new_slab));
                match self
                    .front_slab
                    .add_section(entry_type, requested_section_size)
                {
                    AllocatedSection::NoMoreSpace => {
                        panic!("Failed to allocate a section in a new slab");
                    }
                    AllocatedSection::Section(section) => section,
                }
            }
            AllocatedSection::Section(section) => section,
        }
    }
}

impl Drop for UnifiedLoggerWrite {
    fn drop(&mut self) {
        let section = self.add_section(UnifiedLogType::LastEntry, 80); // TODO: determine that exactly
        drop(section);
        self.front_slab.close();
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
                let result = Some(self.read_section_content(&header)?);
                self.reading_position += header.section_size as usize;
                return Ok(result);
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

        let mut section = vec![0; header.filled_size as usize];
        let start_of_data = self.reading_position + MAX_HEADER_SIZE;
        section.copy_from_slice(
            &self.mmap_buffer[start_of_data..start_of_data + header.filled_size as usize],
        );

        Ok(section)
    }

    fn read_section_header(&mut self) -> CuResult<SectionHeader> {
        let section_header: SectionHeader;
        (section_header, _) =
            decode_from_slice(&self.mmap_buffer[self.reading_position..], standard())
                .expect("Failed to decode section header");
        if section_header.magic != SECTION_MAGIC {
            return Err("Invalid magic number in section header".into());
        }
        Ok(section_header)
    }
}

/// This a convience wrapper around the UnifiedLoggerRead to implement the Read trait.
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

    /// returns true if there is more data to read.
    fn fill_buffer(&mut self) -> io::Result<bool> {
        match self.logger.read_next_section_type(self.log_type) {
            Ok(Some(section)) => {
                self.buffer = section;
                self.buffer_pos = 0;
                Ok(true)
            }
            Ok(None) => Ok(false), // No more sections of this type
            Err(e) => return Err(io::Error::new(io::ErrorKind::Other, e.to_string())),
        }
    }
}

impl Read for UnifiedLoggerIOReader {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        if self.buffer_pos >= self.buffer.len() && !self.fill_buffer()? {
            // This means we hit the last section.
            return Ok(0);
        }

        // If we still have no data after trying to fill the buffer, we're at EOF
        if self.buffer_pos >= self.buffer.len() {
            return Ok(0);
        }

        // Copy as much as we can from the buffer to `buf`
        let len = std::cmp::min(buf.len(), self.buffer.len() - self.buffer_pos);
        buf[..len].copy_from_slice(&self.buffer[self.buffer_pos..self.buffer_pos + len]);
        self.buffer_pos += len;
        Ok(len)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bincode::decode_from_reader;
    use std::io::BufReader;
    use std::path::PathBuf;
    use tempfile::TempDir;
    fn make_a_logger(tmp_dir: &TempDir) -> (Arc<Mutex<UnifiedLoggerWrite>>, PathBuf, PathBuf) {
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

        (
            Arc::new(Mutex::new(data_logger)),
            file_path,
            tmp_dir.path().join("test_0.bin"),
        )
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
            let used = logger.front_slab.used();
            assert!(used < 4 * 4096); // ie. 3 headers, 1 page max per
                                      // logger drops
            used
        };

        let _file = OpenOptions::new()
            .read(true)
            .open(tmp_dir.path().join("test_0.bin"))
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
        let (logger, _, _) = make_a_logger(&tmp_dir);
        {
            let _stream =
                stream_write::<()>(logger.clone(), UnifiedLogType::StructuredLogLine, 1024);
            assert_eq!(
                logger.lock().unwrap().front_slab.sections_in_flight.len(),
                1
            );
        }
        assert_eq!(
            logger.lock().unwrap().front_slab.sections_in_flight.len(),
            0
        );
        let logger = logger.lock().unwrap();
        assert_eq!(
            logger.front_slab.flushed_until,
            logger.front_slab.current_global_position
        );
    }

    #[test]
    fn test_two_sections_self_cleaning_in_order() {
        let tmp_dir = TempDir::new().expect("could not create a tmp dir");
        let (logger, _, _) = make_a_logger(&tmp_dir);
        let s1 = stream_write::<()>(logger.clone(), UnifiedLogType::StructuredLogLine, 1024);
        assert_eq!(
            logger.lock().unwrap().front_slab.sections_in_flight.len(),
            1
        );
        let s2 = stream_write::<()>(logger.clone(), UnifiedLogType::StructuredLogLine, 1024);
        assert_eq!(
            logger.lock().unwrap().front_slab.sections_in_flight.len(),
            2
        );
        drop(s2);
        assert_eq!(
            logger.lock().unwrap().front_slab.sections_in_flight.len(),
            1
        );
        drop(s1);
        let lg = logger.lock().unwrap();
        assert_eq!(lg.front_slab.sections_in_flight.len(), 0);
        assert_eq!(
            lg.front_slab.flushed_until,
            lg.front_slab.current_global_position
        );
    }

    #[test]
    fn test_two_sections_self_cleaning_out_of_order() {
        let tmp_dir = TempDir::new().expect("could not create a tmp dir");
        let (logger, _, _) = make_a_logger(&tmp_dir);
        let s1 = stream_write::<()>(logger.clone(), UnifiedLogType::StructuredLogLine, 1024);
        assert_eq!(
            logger.lock().unwrap().front_slab.sections_in_flight.len(),
            1
        );
        let s2 = stream_write::<()>(logger.clone(), UnifiedLogType::StructuredLogLine, 1024);
        assert_eq!(
            logger.lock().unwrap().front_slab.sections_in_flight.len(),
            2
        );
        drop(s1);
        assert_eq!(
            logger.lock().unwrap().front_slab.sections_in_flight.len(),
            1
        );
        drop(s2);
        let lg = logger.lock().unwrap();
        assert_eq!(lg.front_slab.sections_in_flight.len(), 0);
        assert_eq!(
            lg.front_slab.flushed_until,
            lg.front_slab.current_global_position
        );
    }

    #[test]
    fn test_write_then_read_one_section() {
        let tmp_dir = TempDir::new().expect("could not create a tmp dir");
        let (logger, f, rb) = make_a_logger(&tmp_dir);
        {
            let mut stream = stream_write(logger.clone(), UnifiedLogType::StructuredLogLine, 1024);
            stream.log(&1u32).unwrap();
            stream.log(&2u32).unwrap();
            stream.log(&3u32).unwrap();
        }
        drop(logger);
        let UnifiedLogger::Read(mut dl) = UnifiedLoggerBuilder::new()
            .file_path(&rb)
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

    #[derive(Debug, Encode, Decode)]
    enum CopperListStateMock {
        Free,
        ProcessingTasks,
        BeingSerialized,
    }

    #[derive(Encode, Decode)]
    struct CopperList<P: bincode::enc::Encode> {
        state: CopperListStateMock,
        payload: P, // This is generated from the runtime.
    }

    #[test]
    fn test_copperlist_list_like_logging() {
        let tmp_dir = TempDir::new().expect("could not create a tmp dir");
        let (logger, f, rb) = make_a_logger(&tmp_dir);
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
            .file_path(&rb)
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
