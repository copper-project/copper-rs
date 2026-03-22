//! Linux `io_uring` write backend for the unified logger.
//!
//! This backend preserves the existing slab/section file format so logs remain
//! readable by the mmap-based reader and the higher-level export tools.

use crate::std_common::{create_base_alias_link, make_slab_file};
use crate::{
    AllocatedSection, MAIN_MAGIC, MainHeader, SECTION_HEADER_COMPACT_SIZE, SECTION_MAGIC,
    SectionHandle, SectionHeader, SectionStorage, UnifiedLogStatus, UnifiedLogWrite,
};

use AllocatedSection::Section;
use alloc::vec::Vec;
use bincode::config::standard;
use bincode::enc::EncoderImpl;
use bincode::enc::write::Writer;
use bincode::error::EncodeError;
use bincode::{Encode, encode_into_slice};
use core::mem::MaybeUninit;
use core::{mem, ptr, slice};
use cu29_traits::{
    CuError, CuResult, ObservedWriter, UnifiedLogType, abort_observed_encode,
    begin_observed_encode, finish_observed_encode,
};
use io_uring::{IoUring, opcode, types};
use std::fs::File;
use std::io;
use std::os::fd::AsRawFd;
use std::path::{Path, PathBuf};

pub struct IoUringSectionStorage {
    buffer: Vec<MaybeUninit<u8>>,
    offset: usize,
    block_size: usize,
    slab_suffix: usize,
    section_start: usize,
}

impl IoUringSectionStorage {
    fn new(
        mut buffer: Vec<MaybeUninit<u8>>,
        block_size: usize,
        slab_suffix: usize,
        section_start: usize,
    ) -> Self {
        // Only the header block must be initialized before the first header
        // encode. The rest of the section is filled by the encoder on demand.
        unsafe {
            ptr::write_bytes(buffer.as_mut_ptr(), 0, block_size);
        }
        Self {
            buffer,
            offset: 0,
            block_size,
            slab_suffix,
            section_start,
        }
    }

    fn initialized_prefix(&self) -> &[u8] {
        // SAFETY: Bytes in `0..self.offset` are initialized by construction:
        // the header block is zeroed up front and payload bytes are only
        // advanced after the encoder writes them.
        unsafe { slice::from_raw_parts(self.buffer.as_ptr() as *const u8, self.offset) }
    }
}

struct UninitSliceWriter<'storage> {
    slice: &'storage mut [MaybeUninit<u8>],
    original_length: usize,
}

impl<'storage> UninitSliceWriter<'storage> {
    fn new(bytes: &'storage mut [MaybeUninit<u8>]) -> Self {
        Self {
            original_length: bytes.len(),
            slice: bytes,
        }
    }

    fn bytes_written(&self) -> usize {
        self.original_length - self.slice.len()
    }
}

impl Writer for UninitSliceWriter<'_> {
    fn write(&mut self, bytes: &[u8]) -> Result<(), EncodeError> {
        if bytes.len() > self.slice.len() {
            return Err(EncodeError::UnexpectedEnd);
        }
        let (head, tail) = mem::take(&mut self.slice).split_at_mut(bytes.len());
        // SAFETY: `head` is valid writable storage for `bytes.len()` bytes and
        // becomes initialized by this copy.
        unsafe {
            ptr::copy_nonoverlapping(bytes.as_ptr(), head.as_mut_ptr() as *mut u8, bytes.len());
        }
        self.slice = tail;
        Ok(())
    }
}

impl SectionStorage for IoUringSectionStorage {
    fn initialize<E: Encode>(&mut self, header: &E) -> Result<usize, EncodeError> {
        self.post_update_header(header)?;
        self.offset = self.block_size;
        Ok(self.offset)
    }

    fn post_update_header<E: Encode>(&mut self, header: &E) -> Result<usize, EncodeError> {
        let mut encoder =
            EncoderImpl::new(UninitSliceWriter::new(&mut self.buffer[0..]), standard());
        header.encode(&mut encoder)?;
        Ok(encoder.into_writer().bytes_written())
    }

    fn append<E: Encode>(&mut self, entry: &E) -> Result<usize, EncodeError> {
        begin_observed_encode();
        let result = (|| {
            let mut encoder = EncoderImpl::new(
                ObservedWriter::new(UninitSliceWriter::new(&mut self.buffer[self.offset..])),
                standard(),
            );
            entry.encode(&mut encoder)?;
            Ok(encoder.into_writer().into_inner().bytes_written())
        })();
        let size = match result {
            Ok(size) => {
                debug_assert_eq!(size, finish_observed_encode());
                size
            }
            Err(err) => {
                abort_observed_encode();
                return Err(err);
            }
        };
        self.offset += size;
        Ok(size)
    }

    fn flush(&mut self) -> CuResult<usize> {
        Ok(self.offset)
    }
}

pub enum IoUringUnifiedLogger {
    Write(IoUringUnifiedLoggerWrite),
}

pub struct IoUringUnifiedLoggerBuilder {
    file_base_name: Option<PathBuf>,
    preallocated_size: Option<usize>,
    write: bool,
    create: bool,
    ring_entries: u32,
}

impl Default for IoUringUnifiedLoggerBuilder {
    fn default() -> Self {
        Self::new()
    }
}

impl IoUringUnifiedLoggerBuilder {
    pub fn new() -> Self {
        Self {
            file_base_name: None,
            preallocated_size: None,
            write: false,
            create: false,
            ring_entries: 32,
        }
    }

    pub fn file_base_name(mut self, file_path: &Path) -> Self {
        self.file_base_name = Some(file_path.to_path_buf());
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

    pub fn ring_entries(mut self, ring_entries: u32) -> Self {
        self.ring_entries = ring_entries;
        self
    }

    pub fn build(self) -> io::Result<IoUringUnifiedLogger> {
        if !self.write || !self.create {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "io_uring backend currently supports write/create mode only",
            ));
        }

        let file_path = self.file_base_name.ok_or_else(|| {
            io::Error::new(
                io::ErrorKind::InvalidInput,
                "File path is required for write mode",
            )
        })?;
        let preallocated_size = self.preallocated_size.ok_or_else(|| {
            io::Error::new(
                io::ErrorKind::InvalidInput,
                "Preallocated size is required for write mode",
            )
        })?;
        let page_size = page_size::get();
        let writer = IoUringUnifiedLoggerWrite::new(
            &file_path,
            preallocated_size,
            page_size,
            self.ring_entries,
        )?;
        Ok(IoUringUnifiedLogger::Write(writer))
    }
}

struct UringSlabEntry {
    file: File,
    current_global_position: usize,
    sections_offsets_in_flight: Vec<usize>,
    page_size: usize,
    temporary_end_marker: Option<usize>,
    slab_suffix: usize,
    slab_size: usize,
}

impl Drop for UringSlabEntry {
    fn drop(&mut self) {
        if let Err(error) = self.file.set_len(self.current_global_position as u64) {
            eprintln!("Failed to trim datalogger file: {}", error);
        }

        if !self.sections_offsets_in_flight.is_empty() {
            eprintln!("Error: slab dropped with unflushed sections");
        }
    }
}

impl UringSlabEntry {
    fn new(file: File, page_size: usize, slab_suffix: usize, slab_size: usize) -> Self {
        Self {
            file,
            current_global_position: 0,
            sections_offsets_in_flight: Vec::with_capacity(16),
            page_size,
            temporary_end_marker: None,
            slab_suffix,
            slab_size,
        }
    }

    fn align_to_next_page(&self, ptr: usize) -> usize {
        (ptr + self.page_size - 1) & !(self.page_size - 1)
    }

    fn clear_temporary_end_marker(&mut self) {
        if let Some(marker_start) = self.temporary_end_marker.take() {
            self.current_global_position = marker_start;
        }
    }

    fn add_section(
        &mut self,
        entry_type: UnifiedLogType,
        requested_section_size: usize,
    ) -> AllocatedSection<IoUringSectionStorage> {
        self.current_global_position = self.align_to_next_page(self.current_global_position);
        let section_size = self.align_to_next_page(requested_section_size) as u32;

        if self.current_global_position + section_size as usize > self.slab_size {
            return AllocatedSection::NoMoreSpace;
        }

        #[cfg(feature = "compact")]
        let block_size = SECTION_HEADER_COMPACT_SIZE;

        #[cfg(not(feature = "compact"))]
        let block_size = self.page_size as u16;

        let section_header = SectionHeader {
            magic: SECTION_MAGIC,
            block_size,
            entry_type,
            offset_to_next_section: section_size,
            used: 0,
            is_open: true,
        };

        self.sections_offsets_in_flight
            .push(self.current_global_position);
        let section_start = self.current_global_position;
        let mut buffer = Vec::with_capacity(requested_section_size);
        // SAFETY: The encoder initializes bytes before they are ever read back
        // and flushes only write the initialized prefix to disk.
        unsafe {
            buffer.set_len(requested_section_size);
        }
        let storage = IoUringSectionStorage::new(
            buffer,
            block_size as usize,
            self.slab_suffix,
            section_start,
        );
        self.current_global_position += requested_section_size;

        Section(SectionHandle::create(section_header, storage).expect("Failed to create section"))
    }

    fn write_end_marker(&mut self, ring: &mut IoUring, temporary: bool) -> CuResult<()> {
        let block_size = SECTION_HEADER_COMPACT_SIZE as usize;
        let marker_start = self.align_to_next_page(self.current_global_position);
        let marker_end = marker_start + block_size;
        if marker_end > self.slab_size {
            return Err("Not enough space to write end-of-log marker".into());
        }

        let header = SectionHeader {
            magic: SECTION_MAGIC,
            block_size: SECTION_HEADER_COMPACT_SIZE,
            entry_type: UnifiedLogType::LastEntry,
            offset_to_next_section: block_size as u32,
            used: 0,
            is_open: temporary,
        };

        let mut encoded = vec![0u8; block_size];
        encode_into_slice(&header, &mut encoded, standard())
            .map_err(|e| CuError::new_with_cause("Failed to encode end-of-log header", e))?;
        submit_write_all(ring, &self.file, marker_start as u64, &encoded)
            .map_err(|e| CuError::new_with_cause("Failed to write end-of-log marker", e))?;

        self.temporary_end_marker = Some(marker_start);
        self.current_global_position = marker_end;
        Ok(())
    }

    fn is_it_my_section(&self, section: &SectionHandle<IoUringSectionStorage>) -> bool {
        self.slab_suffix == section.get_storage().slab_suffix
    }

    fn flush_section(
        &mut self,
        ring: &mut IoUring,
        section: &mut SectionHandle<IoUringSectionStorage>,
    ) -> CuResult<()> {
        let storage = section.get_storage();
        submit_write_all(
            ring,
            &self.file,
            storage.section_start as u64,
            storage.initialized_prefix(),
        )
        .map_err(|e| CuError::new_with_cause("Failed to write section bytes", e))?;
        #[cfg(feature = "mmap-fsync")]
        submit_fsync(ring, &self.file)
            .map_err(|e| CuError::new_with_cause("Failed to fsync section bytes", e))?;
        self.sections_offsets_in_flight
            .retain(|&x| x != storage.section_start);
        Ok(())
    }
}

/// A write side of the datalogger using `io_uring` for transport.
pub struct IoUringUnifiedLoggerWrite {
    front_slab: UringSlabEntry,
    back_slabs: Vec<UringSlabEntry>,
    base_file_path: PathBuf,
    slab_size: usize,
    front_slab_suffix: usize,
    ring: IoUring,
}

fn submit_write_all(
    ring: &mut IoUring,
    file: &File,
    mut offset: u64,
    mut bytes: &[u8],
) -> io::Result<()> {
    while !bytes.is_empty() {
        let chunk_len = bytes.len().min(u32::MAX as usize);
        let entry = opcode::Write::new(
            types::Fd(file.as_raw_fd()),
            bytes.as_ptr(),
            chunk_len as u32,
        )
        .offset(offset)
        .build()
        .user_data(offset);

        {
            let mut sq = ring.submission();
            // SAFETY: The submission entry borrows the buffer pointer until the CQE is reaped
            // below, and the backing slice outlives the wait.
            unsafe {
                sq.push(&entry)
                    .map_err(|_| io::Error::other("io_uring submission queue is full"))?;
            }
        }

        ring.submit_and_wait(1)?;

        let cqe = {
            let mut cq = ring.completion();
            cq.next().ok_or_else(|| {
                io::Error::new(
                    io::ErrorKind::UnexpectedEof,
                    "io_uring submitted write without a completion",
                )
            })?
        };

        if cqe.result() < 0 {
            return Err(io::Error::from_raw_os_error(-cqe.result()));
        }
        if cqe.result() == 0 {
            return Err(io::Error::new(
                io::ErrorKind::WriteZero,
                "io_uring write completed with 0 bytes",
            ));
        }

        let written = cqe.result() as usize;
        offset += written as u64;
        bytes = &bytes[written..];
    }

    Ok(())
}

#[cfg(feature = "mmap-fsync")]
fn submit_fsync(ring: &mut IoUring, file: &File) -> io::Result<()> {
    let entry = opcode::Fsync::new(types::Fd(file.as_raw_fd()))
        .build()
        .user_data(u64::MAX);

    {
        let mut sq = ring.submission();
        // SAFETY: The SQE does not borrow external memory and is completed before return.
        unsafe {
            sq.push(&entry)
                .map_err(|_| io::Error::other("io_uring submission queue is full"))?;
        }
    }

    ring.submit_and_wait(1)?;

    let cqe = {
        let mut cq = ring.completion();
        cq.next().ok_or_else(|| {
            io::Error::new(
                io::ErrorKind::UnexpectedEof,
                "io_uring submitted fsync without a completion",
            )
        })?
    };

    if cqe.result() < 0 {
        return Err(io::Error::from_raw_os_error(-cqe.result()));
    }

    Ok(())
}

impl UnifiedLogWrite<IoUringSectionStorage> for IoUringUnifiedLoggerWrite {
    fn add_section(
        &mut self,
        entry_type: UnifiedLogType,
        requested_section_size: usize,
    ) -> CuResult<SectionHandle<IoUringSectionStorage>> {
        self.garbage_collect_backslabs();
        self.front_slab.clear_temporary_end_marker();
        let maybe_section = self
            .front_slab
            .add_section(entry_type, requested_section_size);

        match maybe_section {
            AllocatedSection::NoMoreSpace => {
                let new_slab = self.create_slab()?;
                self.back_slabs
                    .push(std::mem::replace(&mut self.front_slab, new_slab));
                match self
                    .front_slab
                    .add_section(entry_type, requested_section_size)
                {
                    AllocatedSection::NoMoreSpace => Err(CuError::from("out of space")),
                    Section(section) => {
                        self.place_end_marker(true)?;
                        Ok(section)
                    }
                }
            }
            Section(section) => {
                self.place_end_marker(true)?;
                Ok(section)
            }
        }
    }

    fn flush_section(&mut self, section: &mut SectionHandle<IoUringSectionStorage>) {
        section.mark_closed();
        section
            .post_update_header()
            .expect("Failed to update section header");
        for slab in self.back_slabs.iter_mut() {
            if slab.is_it_my_section(section) {
                slab.flush_section(&mut self.ring, section)
                    .expect("Failed to flush closed section");
                return;
            }
        }
        self.front_slab
            .flush_section(&mut self.ring, section)
            .expect("Failed to flush closed section");
    }

    fn status(&self) -> UnifiedLogStatus {
        UnifiedLogStatus {
            total_used_space: self.front_slab.current_global_position,
            total_allocated_space: self.slab_size * self.front_slab_suffix,
        }
    }
}

impl IoUringUnifiedLoggerWrite {
    fn new(
        base_file_path: &Path,
        slab_size: usize,
        page_size: usize,
        ring_entries: u32,
    ) -> io::Result<Self> {
        let file = make_slab_file(base_file_path, slab_size, 0)?;
        create_base_alias_link(base_file_path)?;
        let mut ring = IoUring::new(ring_entries)?;

        let main_header = MainHeader {
            magic: MAIN_MAGIC,
            first_section_offset: page_size as u16,
            page_size: page_size as u16,
        };
        let mut prolog = vec![0u8; page_size];
        let nb_bytes = encode_into_slice(&main_header, &mut prolog[..], standard())
            .map_err(|e| io::Error::other(format!("Failed to encode main header: {e}")))?;
        assert!(nb_bytes < page_size);
        submit_write_all(&mut ring, &file, 0, &prolog)?;

        let mut front_slab = UringSlabEntry::new(file, page_size, 0, slab_size);
        front_slab.current_global_position = page_size;

        Ok(Self {
            front_slab,
            back_slabs: Vec::new(),
            base_file_path: base_file_path.to_path_buf(),
            slab_size,
            front_slab_suffix: 0,
            ring,
        })
    }

    fn next_slab(&mut self) -> io::Result<File> {
        let next_suffix = self.front_slab_suffix + 1;
        let file = make_slab_file(&self.base_file_path, self.slab_size, next_suffix)?;
        self.front_slab_suffix = next_suffix;
        Ok(file)
    }

    fn create_slab(&mut self) -> CuResult<UringSlabEntry> {
        let file = self
            .next_slab()
            .map_err(|e| CuError::new_with_cause("Failed to create slab file", e))?;
        Ok(UringSlabEntry::new(
            file,
            self.front_slab.page_size,
            self.front_slab_suffix,
            self.slab_size,
        ))
    }

    fn garbage_collect_backslabs(&mut self) {
        self.back_slabs
            .retain_mut(|slab| !slab.sections_offsets_in_flight.is_empty());
    }

    fn place_end_marker(&mut self, temporary: bool) -> CuResult<()> {
        match self.front_slab.write_end_marker(&mut self.ring, temporary) {
            Ok(_) => Ok(()),
            Err(_) => {
                let new_slab = self.create_slab()?;
                self.back_slabs
                    .push(std::mem::replace(&mut self.front_slab, new_slab));
                self.front_slab.write_end_marker(&mut self.ring, temporary)
            }
        }
    }
}

impl Drop for IoUringUnifiedLoggerWrite {
    fn drop(&mut self) {
        #[cfg(debug_assertions)]
        eprintln!("Flushing the unified Logger ... ");

        self.front_slab.clear_temporary_end_marker();
        if let Err(e) = self.place_end_marker(false) {
            panic!("Failed to flush the unified logger: {}", e);
        }
        #[cfg(feature = "mmap-fsync")]
        if let Err(e) = submit_fsync(&mut self.ring, &self.front_slab.file) {
            panic!("Failed to fsync the unified logger: {}", e);
        }
        self.garbage_collect_backslabs();

        #[cfg(debug_assertions)]
        eprintln!("Unified Logger flushed.");
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::memmap::{MmapUnifiedLogger, MmapUnifiedLoggerBuilder, MmapUnifiedLoggerRead};
    use crate::std_common::build_slab_path;
    use crate::{UnifiedLogRead, stream_write};
    use bincode::de::read::SliceReader;
    use bincode::{Decode, Encode, decode_from_reader};
    use cu29_traits::WriteStream;
    use std::path::{Path, PathBuf};
    use std::sync::{Arc, Mutex};
    use tempfile::TempDir;

    const LARGE_SLAB: usize = 100 * 1024;
    const SMALL_SLAB: usize = 16 * 2 * 1024;

    fn io_uring_supported() -> bool {
        IoUring::new(8).is_ok()
    }

    fn make_a_logger(
        tmp_dir: &TempDir,
        slab_size: usize,
    ) -> (Arc<Mutex<IoUringUnifiedLoggerWrite>>, PathBuf) {
        let file_path = tmp_dir.path().join("test.bin");
        let logger = IoUringUnifiedLoggerBuilder::new()
            .write(true)
            .create(true)
            .file_base_name(&file_path)
            .preallocated_size(slab_size)
            .build()
            .expect("Failed to create logger");
        let IoUringUnifiedLogger::Write(data_logger) = logger;

        (Arc::new(Mutex::new(data_logger)), file_path)
    }

    fn mmap_reader(file_path: &Path) -> MmapUnifiedLoggerRead {
        let MmapUnifiedLogger::Read(reader) = MmapUnifiedLoggerBuilder::new()
            .file_base_name(file_path)
            .build()
            .expect("Failed to build mmap reader")
        else {
            panic!("Failed to build mmap reader");
        };
        reader
    }

    #[test]
    fn test_base_alias_exists_and_matches_first_slab() {
        if !io_uring_supported() {
            return;
        }

        let tmp_dir = TempDir::new().expect("could not create a tmp dir");
        let file_path = tmp_dir.path().join("test.bin");
        let _logger = IoUringUnifiedLoggerBuilder::new()
            .write(true)
            .create(true)
            .file_base_name(&file_path)
            .preallocated_size(LARGE_SLAB)
            .build()
            .expect("Failed to create logger");

        let first_slab = build_slab_path(&file_path, 0).expect("Failed to build first slab path");
        assert!(file_path.exists(), "base alias does not exist");
        assert!(first_slab.exists(), "first slab does not exist");

        let alias_bytes = std::fs::read(&file_path).expect("Failed to read base alias");
        let slab_bytes = std::fs::read(&first_slab).expect("Failed to read first slab");
        assert_eq!(alias_bytes, slab_bytes);
    }

    #[test]
    fn test_final_end_marker_is_not_temporary() {
        if !io_uring_supported() {
            return;
        }

        let tmp_dir = TempDir::new().expect("could not create a tmp dir");
        let (logger, file_path) = make_a_logger(&tmp_dir, LARGE_SLAB);
        {
            let mut stream = stream_write::<u32, IoUringSectionStorage>(
                logger.clone(),
                UnifiedLogType::CopperList,
                1024,
            )
            .unwrap();
            stream.log(&1u32).unwrap();
        }
        drop(logger);

        let mut reader = mmap_reader(&file_path);
        loop {
            let (header, _data) = reader
                .raw_read_section()
                .expect("Failed to read section while searching for EOF");
            if header.entry_type == UnifiedLogType::LastEntry {
                assert!(!header.is_open);
                break;
            }
        }
    }

    #[test]
    fn test_write_then_read_one_section() {
        if !io_uring_supported() {
            return;
        }

        let tmp_dir = TempDir::new().expect("could not create a tmp dir");
        let (logger, file_path) = make_a_logger(&tmp_dir, LARGE_SLAB);
        {
            let mut stream = stream_write::<u32, IoUringSectionStorage>(
                logger.clone(),
                UnifiedLogType::StructuredLogLine,
                1024,
            )
            .unwrap();
            stream.log(&1u32).unwrap();
            stream.log(&2u32).unwrap();
            stream.log(&3u32).unwrap();
        }
        drop(logger);

        let mut reader = mmap_reader(&file_path);
        let section = reader
            .read_next_section_type(UnifiedLogType::StructuredLogLine)
            .expect("Failed to read section")
            .expect("Missing structured-log section");

        let mut reader = SliceReader::new(&section[..]);
        let v1: u32 = decode_from_reader(&mut reader, standard()).unwrap();
        let v2: u32 = decode_from_reader(&mut reader, standard()).unwrap();
        let v3: u32 = decode_from_reader(&mut reader, standard()).unwrap();
        assert_eq!(v1, 1);
        assert_eq!(v2, 2);
        assert_eq!(v3, 3);
    }

    #[derive(Debug, Encode, Decode)]
    enum CopperListStateMock {
        Free,
        ProcessingTasks,
        BeingSerialized,
    }

    #[derive(Encode, Decode)]
    struct CopperList<P: bincode::enc::Encode> {
        state: CopperListStateMock,
        payload: P,
    }

    #[test]
    fn test_copperlist_list_like_logging() {
        if !io_uring_supported() {
            return;
        }

        let tmp_dir = TempDir::new().expect("could not create a tmp dir");
        let (logger, file_path) = make_a_logger(&tmp_dir, LARGE_SLAB);
        {
            let mut stream = stream_write::<CopperList<(u32, u32, u32)>, IoUringSectionStorage>(
                logger.clone(),
                UnifiedLogType::CopperList,
                1024,
            )
            .unwrap();
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

        let mut dl = mmap_reader(&file_path);
        let section = dl
            .read_next_section_type(UnifiedLogType::CopperList)
            .expect("Failed to read section")
            .expect("Missing CopperList section");

        let mut reader = SliceReader::new(&section[..]);
        let cl0: CopperList<(u32, u32, u32)> = decode_from_reader(&mut reader, standard()).unwrap();
        let cl1: CopperList<(u32, u32, u32)> = decode_from_reader(&mut reader, standard()).unwrap();
        assert_eq!(cl0.payload.1, 2);
        assert_eq!(cl1.payload.2, 6);
    }

    #[test]
    fn test_multi_slab_end2end() {
        if !io_uring_supported() {
            return;
        }

        let tmp_dir = TempDir::new().expect("could not create a tmp dir");
        let (logger, file_path) = make_a_logger(&tmp_dir, SMALL_SLAB);
        {
            let mut stream = stream_write::<CopperList<(u32, u32, u32)>, IoUringSectionStorage>(
                logger.clone(),
                UnifiedLogType::CopperList,
                1024,
            )
            .unwrap();
            let cl0 = CopperList {
                state: CopperListStateMock::Free,
                payload: (1u32, 2u32, 3u32),
            };
            for _ in 0..10_000 {
                stream.log(&cl0).unwrap();
            }
        }
        drop(logger);

        let mut dl = mmap_reader(&file_path);
        let mut total_readback = 0;
        loop {
            let section = dl.read_next_section_type(UnifiedLogType::CopperList);
            if section.is_err() {
                break;
            }
            let Some(section) = section.unwrap() else {
                break;
            };

            let mut reader = SliceReader::new(&section[..]);
            loop {
                let maybe_cl: Result<CopperList<(u32, u32, u32)>, _> =
                    decode_from_reader(&mut reader, standard());
                if maybe_cl.is_ok() {
                    total_readback += 1;
                } else {
                    break;
                }
            }
        }

        assert_eq!(total_readback, 10_000);
    }
}
