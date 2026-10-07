//! Mapped-file adapter for the byte-addressed section chain.
use crate::byte_log::{ByteLogger, ByteRegion, ByteSection, ByteStorage, read_header, read_main};
use crate::{
    ApplicationMetadata, CapacityPolicy, MainHeader, SECTION_HEADER_COMPACT_SIZE, SectionHeader,
    UnifiedLogRead,
};
use alloc::sync::Arc;
use bincode::{config::standard, decode_from_slice};
use cu29_traits::{CuError, CuResult, UnifiedLogType};
use memmap2::{Mmap, MmapMut};
use std::fs::{File, OpenOptions};
use std::io::{self, Read};
use std::path::{Path, PathBuf};

const MAX_SPANS: usize = 64;
enum Mapping {
    Read(Mmap),
    Write(core::cell::UnsafeCell<MmapMut>),
}
struct MappedFile {
    mapping: Mapping,
    pointer: *mut u8,
    length: usize,
    #[cfg(feature = "mmap-fsync")]
    file: File,
}
// SAFETY: Sections receive disjoint ranges and exclusive generation leases. Header
// access uses the same lease. The mapping never exposes borrowed payload slices.
unsafe impl Send for MappedFile {}
unsafe impl Sync for MappedFile {}
impl MappedFile {
    fn new(file: File, writable: bool) -> io::Result<Self> {
        // SAFETY: Copper owns these backing files; external mutation is unsupported.
        let (mapping, pointer, length) = unsafe {
            if writable {
                let mut mapping = MmapMut::map_mut(&file)?;
                let pointer = mapping.as_mut_ptr();
                let length = mapping.len();
                (
                    Mapping::Write(core::cell::UnsafeCell::new(mapping)),
                    pointer,
                    length,
                )
            } else {
                let mapping = Mmap::map(&file)?;
                let pointer = mapping.as_ptr().cast_mut();
                let length = mapping.len();
                (Mapping::Read(mapping), pointer, length)
            }
        };
        Ok(Self {
            mapping,
            pointer,
            length,
            #[cfg(feature = "mmap-fsync")]
            file,
        })
    }
}

/// A range keeps mappings alive even after the logger is dropped.
pub struct MmapRegion {
    maps: [Option<Arc<MappedFile>>; MAX_SPANS],
    first: usize,
    slab_size: usize,
    length: u64,
}
impl MmapRegion {
    fn visit(
        &self,
        offset: u64,
        len: usize,
        mut visit: impl FnMut(&MappedFile, usize, usize, usize) -> CuResult<()>,
    ) -> CuResult<()> {
        if offset
            .checked_add(len as u64)
            .is_none_or(|end| end > self.length)
        {
            return Err(CuError::from("Mapped range exceeds section"));
        }
        let mut cursor = self.first
            + usize::try_from(offset).map_err(|_| CuError::from("Offset exceeds address space"))?;
        let mut done = 0;
        while done < len {
            let index = cursor / self.slab_size;
            let local = cursor % self.slab_size;
            let map = self
                .maps
                .get(index)
                .and_then(Option::as_ref)
                .ok_or(CuError::from("Missing mapped slab"))?;
            let count = (len - done).min(map.length.saturating_sub(local));
            if count == 0 {
                return Err(CuError::from("Truncated mapped slab"));
            }
            visit(map, local, done, count)?;
            cursor += count;
            done += count;
        }
        Ok(())
    }
}
impl ByteRegion for MmapRegion {
    fn len(&self) -> u64 {
        self.length
    }
    fn read(&self, offset: u64, bytes: &mut [u8]) -> CuResult<()> {
        self.visit(offset, bytes.len(), |map, local, done, count| {
            // SAFETY: Reads are bounded to a leased section or read-only file mapping.
            unsafe {
                core::ptr::copy_nonoverlapping(
                    map.pointer.add(local),
                    bytes[done..].as_mut_ptr(),
                    count,
                );
            }
            Ok(())
        })
    }
    fn write(&mut self, offset: u64, bytes: &[u8]) -> CuResult<()> {
        self.visit(offset, bytes.len(), |map, local, done, count| {
            let Mapping::Write(_) = &map.mapping else {
                return Err(CuError::from("Read-only mapped storage"));
            };
            // SAFETY: The allocator gives disjoint sections exclusive generation leases.
            // A header update seals its lease before writing; reclamation cannot overlap
            // an encoder holding BUSY. No mutable Rust references to a mapping escape.
            unsafe {
                core::ptr::copy_nonoverlapping(
                    bytes[done..].as_ptr(),
                    map.pointer.add(local),
                    count,
                );
            }
            Ok(())
        })
    }
    fn commit(&mut self) -> CuResult<()> {
        Ok(())
    }
    fn flush(&mut self) -> CuResult<()> {
        for map in self.maps.iter().flatten() {
            match &map.mapping {
                Mapping::Write(mapping) => {
                    // SAFETY: Flushing observes the mapping without creating payload slices.
                    unsafe { &*mapping.get() }
                        .flush()
                        .map_err(|e| CuError::new_with_cause("Cannot flush mapped section", e))?;
                    #[cfg(feature = "mmap-fsync")]
                    map.file
                        .sync_all()
                        .map_err(|e| CuError::new_with_cause("Cannot sync backing file", e))?;
                }
                Mapping::Read(mapping) => {
                    let _ = mapping.len();
                }
            }
        }
        Ok(())
    }
}

#[doc(hidden)]
pub struct MmapStorage {
    path: PathBuf,
    slab_size: usize,
    maps: Vec<Arc<MappedFile>>,
    writable: bool,
}
fn slab_path(path: &Path, index: usize) -> io::Result<PathBuf> {
    let stem = path
        .file_stem()
        .and_then(|s| s.to_str())
        .ok_or_else(|| io::Error::other("Invalid log file name"))?;
    let extension = path
        .extension()
        .and_then(|s| s.to_str())
        .ok_or_else(|| io::Error::other("Log file requires an extension"))?;
    Ok(path.with_file_name(format!("{stem}_{index}.{extension}")))
}
impl MmapStorage {
    fn open(path: &Path, writable: bool) -> io::Result<Self> {
        let first = File::open(slab_path(path, 0)?)?;
        let slab_size = first.metadata()?.len() as usize;
        if slab_size < 512 || !slab_size.is_multiple_of(512) {
            return Err(io::Error::other("Invalid backing slab size"));
        }
        let mut maps = Vec::new();
        for index in 0.. {
            let file = match OpenOptions::new()
                .read(true)
                .write(writable)
                .open(slab_path(path, index)?)
            {
                Ok(file) => file,
                Err(e) if e.kind() == io::ErrorKind::NotFound => break,
                Err(e) => return Err(e),
            };
            if file.metadata()?.len() != slab_size as u64 {
                return Err(io::Error::other("Inconsistent backing slab sizes"));
            }
            maps.push(Arc::new(MappedFile::new(file, writable)?));
        }
        Ok(Self {
            path: path.to_path_buf(),
            slab_size,
            maps,
            writable,
        })
    }
    fn create(path: &Path, slab_size: usize) -> io::Result<Self> {
        if slab_size < 1024 || !slab_size.is_multiple_of(512) {
            return Err(io::Error::other(
                "Slab size must be a multiple of 512 bytes",
            ));
        }
        let mut storage = Self {
            path: path.to_path_buf(),
            slab_size,
            maps: Vec::new(),
            writable: true,
        };
        storage.grow(slab_size as u64).map_err(io::Error::other)?;
        // Remove old suffixes only for this explicitly replaced log.
        for index in 1.. {
            match std::fs::remove_file(slab_path(path, index)?) {
                Ok(()) => {}
                Err(e) if e.kind() == io::ErrorKind::NotFound => break,
                Err(e) => return Err(e),
            }
        }
        match std::fs::remove_file(path) {
            Ok(()) => {}
            Err(e) if e.kind() == io::ErrorKind::NotFound => {}
            Err(e) => return Err(e),
        }
        let target = slab_path(path, 0)?;
        #[cfg(unix)]
        std::os::unix::fs::symlink(target.file_name().unwrap(), path)?;
        #[cfg(not(unix))]
        std::fs::hard_link(target, path)?;
        Ok(storage)
    }
}
impl ByteStorage for MmapStorage {
    type Region = MmapRegion;
    fn len(&self) -> u64 {
        self.maps.len() as u64 * self.slab_size as u64
    }
    fn grow(&mut self, end: u64) -> CuResult<()> {
        if end <= self.len() {
            return Ok(());
        }
        if !self.writable {
            return Err(CuError::from("Read-only storage cannot grow"));
        }
        let count = end.div_ceil(self.slab_size as u64);
        while (self.maps.len() as u64) < count {
            let path = slab_path(&self.path, self.maps.len())
                .map_err(|e| CuError::new_with_cause("Invalid slab path", e))?;
            let file = OpenOptions::new()
                .read(true)
                .write(true)
                .create(true)
                .truncate(true)
                .open(path)
                .map_err(|e| CuError::new_with_cause("Cannot create backing slab", e))?;
            file.set_len(self.slab_size as u64)
                .map_err(|e| CuError::new_with_cause("Cannot allocate backing slab", e))?;
            self.maps
                .push(Arc::new(MappedFile::new(file, true).map_err(|e| {
                    CuError::new_with_cause("Cannot map backing slab", e)
                })?));
        }
        Ok(())
    }
    fn region(&self, offset: u64, length: u64) -> CuResult<MmapRegion> {
        if offset
            .checked_add(length)
            .is_none_or(|end| end > self.len())
        {
            return Err(CuError::from("Byte range exceeds backing storage"));
        }
        let begin = usize::try_from(offset / self.slab_size as u64)
            .map_err(|_| CuError::from("Offset exceeds address space"))?;
        let count =
            ((offset % self.slab_size as u64 + length).div_ceil(self.slab_size as u64)) as usize;
        if count > MAX_SPANS {
            return Err(CuError::from("Section spans more than 64 backing files"));
        }
        let mut maps = core::array::from_fn(|_| None);
        for (slot, mapping) in maps.iter_mut().zip(self.maps[begin..begin + count].iter()) {
            *slot = Some(mapping.clone());
        }
        Ok(MmapRegion {
            maps,
            first: (offset % self.slab_size as u64) as usize,
            slab_size: self.slab_size,
            length,
        })
    }
}
pub type MmapSectionStorage = ByteSection<MmapRegion>;
pub type MmapUnifiedLoggerWrite = ByteLogger<MmapStorage>;
pub enum MmapUnifiedLogger {
    Read(MmapUnifiedLoggerRead),
    Write(MmapUnifiedLoggerWrite),
}

#[derive(Default)]
pub struct MmapUnifiedLoggerBuilder {
    file_base_name: Option<PathBuf>,
    preallocated_size: Option<usize>,
    write: bool,
    create: bool,
    append: bool,
    capacity: Option<u64>,
    policy: CapacityPolicy,
}
impl MmapUnifiedLoggerBuilder {
    pub fn new() -> Self {
        Self::default()
    }
    pub fn file_base_name(mut self, path: &Path) -> Self {
        self.file_base_name = Some(path.to_path_buf());
        self
    }
    pub fn preallocated_size(mut self, size: usize) -> Self {
        self.preallocated_size = Some(size);
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
    pub fn append(mut self, append: bool) -> Self {
        self.append = append;
        self
    }
    /// Bound rotating sections by a byte capacity, retaining static metadata.
    pub fn rollover(mut self, size: usize) -> Self {
        self.capacity = Some(size as u64);
        self.policy = CapacityPolicy::OverwriteOldest;
        self
    }
    pub fn capacity(mut self, size: u64, policy: CapacityPolicy) -> Self {
        self.capacity = Some(size);
        self.policy = policy;
        self
    }
    pub fn build(self) -> io::Result<MmapUnifiedLogger> {
        let path = self
            .file_base_name
            .ok_or_else(|| io::Error::other("Log file path is required"))?;
        if !(self.write && self.create) {
            return MmapUnifiedLoggerRead::new(&path).map(MmapUnifiedLogger::Read);
        }
        let slab_size = self
            .preallocated_size
            .ok_or_else(|| io::Error::other("Preallocated size is required"))?;
        let slab_size = slab_size
            .checked_add(511)
            .ok_or_else(|| io::Error::other("Slab size overflow"))?
            / 512
            * 512;
        let slab_size = slab_size.max(1024);
        let logger = if self.append {
            let storage = MmapStorage::open(&path, true)?;
            if storage.slab_size != slab_size {
                return Err(io::Error::other("Append slab allocation size mismatch"));
            }
            let capacity = self.capacity.unwrap_or(storage.len());
            ByteLogger::append(storage, self.policy, capacity)
        } else {
            ByteLogger::new(
                MmapStorage::create(&path, slab_size)?,
                512,
                self.policy,
                self.capacity.unwrap_or(slab_size as u64),
            )
        }
        .map_err(io::Error::other)?;
        Ok(MmapUnifiedLogger::Write(logger))
    }
}

/// A byte offset relative to the beginning of the log.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct LogPosition(pub u64);
pub struct MmapUnifiedLoggerRead {
    storage: MmapStorage,
    main_header: MainHeader,
    cursor: u64,
    remaining: u64,
}
impl MmapUnifiedLoggerRead {
    pub fn new(path: &Path) -> io::Result<Self> {
        let storage = MmapStorage::open(path, false)?;
        let main_header = read_main(&storage).map_err(io::Error::other)?;
        let cursor = if main_header.metadata_offset != 0 {
            main_header.metadata_offset
        } else {
            main_header.head_section
        };
        let remaining = main_header.sections_end / 512 + 2;
        Ok(Self {
            storage,
            main_header,
            cursor,
            remaining,
        })
    }
    pub fn raw_main_header(&self) -> &MainHeader {
        &self.main_header
    }
    pub fn position(&self) -> LogPosition {
        LogPosition(self.cursor)
    }
    pub fn seek(&mut self, position: LogPosition) -> CuResult<()> {
        if position.0 != 0 {
            self.section_header(position.0)?;
        }
        self.cursor = position.0;
        self.remaining = self.main_header.sections_end / 512 + 2;
        Ok(())
    }
    pub fn application_metadata(&self) -> CuResult<Option<ApplicationMetadata>> {
        if self.main_header.metadata_offset == 0 {
            return Ok(None);
        }
        let h = self.section_header(self.main_header.metadata_offset)?;
        if h.entry_type != UnifiedLogType::ApplicationMetadata || h.is_open {
            return Err(CuError::from("Invalid static application metadata"));
        }
        let mut data = vec![0; h.used as usize];
        self.storage
            .region(self.main_header.metadata_offset + 512, h.used as u64)?
            .read(0, &mut data)?;
        let (metadata, used) = decode_from_slice::<ApplicationMetadata, _>(&data, standard())
            .map_err(|e| CuError::new_with_cause("Invalid application metadata", e))?;
        if used != data.len() || metadata.catalog_offset != h.next_section {
            return Err(CuError::from("Invalid metadata body or catalog offset"));
        }
        Ok(Some(metadata))
    }
    fn section_header(&self, offset: u64) -> CuResult<SectionHeader> {
        if !offset.is_multiple_of(512)
            || offset < self.main_header.page_size as u64
            || offset >= self.main_header.sections_end
        {
            return Err(CuError::from("Invalid section offset"));
        }
        let h = read_header(&self.storage, offset)?;
        let limit = if offset < self.main_header.sections_begin {
            self.main_header.sections_begin
        } else {
            self.main_header.sections_end
        };
        if offset
            .checked_add(h.allocated)
            .is_none_or(|end| end > limit)
        {
            return Err(CuError::from("Section exceeds logical byte bounds"));
        }
        if h.next_section != 0
            && (!h.next_section.is_multiple_of(512)
                || h.next_section < self.main_header.page_size as u64
                || h.next_section >= self.main_header.sections_end)
        {
            return Err(CuError::from("Invalid section link"));
        }
        if offset >= self.main_header.sections_begin {
            if h.next_section != 0 && h.next_section < self.main_header.sections_begin {
                return Err(CuError::from("Data links into static metadata"));
            }
            if (offset == self.main_header.tail_section) != (h.next_section == 0) {
                return Err(CuError::from("Invalid tail link"));
            }
        }
        Ok(h)
    }
    fn advance(&mut self, h: &SectionHeader) -> CuResult<()> {
        if self.remaining == 0 {
            return Err(CuError::from("Cyclic section links"));
        }
        self.remaining -= 1;
        self.cursor = if self.cursor < self.main_header.sections_begin && h.next_section == 0 {
            self.main_header.head_section
        } else {
            h.next_section
        };
        Ok(())
    }
    pub fn raw_skip_section(&mut self) -> CuResult<SectionHeader> {
        if self.cursor == 0 {
            return Ok(SectionHeader {
                entry_type: UnifiedLogType::LastEntry,
                is_open: !self.main_header.clean_close,
                allocated: 512,
                ..SectionHeader::default()
            });
        }
        let h = self.section_header(self.cursor)?;
        self.advance(&h)?;
        Ok(h)
    }
    pub fn end_of_log(&mut self) -> CuResult<LogPosition> {
        if !self.main_header.clean_close {
            return Err(CuError::from("Log was not cleanly closed"));
        }
        while self.cursor != 0 {
            if self.raw_skip_section()?.is_open {
                return Err(CuError::from("Log contains an open section"));
            }
        }
        Ok(LogPosition(0))
    }
    pub fn scan_section_bytes(&mut self, kind: UnifiedLogType) -> CuResult<u64> {
        let mut bytes = 0;
        while self.cursor != 0 {
            let h = self.raw_skip_section()?;
            if h.entry_type == kind {
                bytes += h.used as u64;
            }
        }
        Ok(bytes)
    }
}
impl UnifiedLogRead for MmapUnifiedLoggerRead {
    fn raw_read_section(&mut self) -> CuResult<(SectionHeader, Vec<u8>)> {
        if self.cursor == 0 {
            return Ok((self.raw_skip_section()?, Vec::new()));
        }
        let h = self.section_header(self.cursor)?;
        let mut bytes = vec![0; h.used as usize];
        self.storage
            .region(
                self.cursor + SECTION_HEADER_COMPACT_SIZE as u64,
                h.used as u64,
            )?
            .read(0, &mut bytes)?;
        self.advance(&h)?;
        Ok((h, bytes))
    }
    fn read_next_section_type(&mut self, kind: UnifiedLogType) -> CuResult<Option<Vec<u8>>> {
        while self.cursor != 0 {
            let h = self.section_header(self.cursor)?;
            if h.entry_type == kind {
                return self.raw_read_section().map(|(_, bytes)| Some(bytes));
            }
            self.advance(&h)?;
        }
        Ok(None)
    }
}
pub struct UnifiedLoggerIOReader {
    logger: MmapUnifiedLoggerRead,
    log_type: UnifiedLogType,
    buffer: Vec<u8>,
    buffer_pos: usize,
}
impl UnifiedLoggerIOReader {
    pub fn new(logger: MmapUnifiedLoggerRead, log_type: UnifiedLogType) -> Self {
        Self {
            logger,
            log_type,
            buffer: Vec::new(),
            buffer_pos: 0,
        }
    }
}
impl Read for UnifiedLoggerIOReader {
    fn read(&mut self, bytes: &mut [u8]) -> io::Result<usize> {
        if bytes.is_empty() {
            return Ok(0);
        }
        while self.buffer_pos == self.buffer.len() {
            match self
                .logger
                .read_next_section_type(self.log_type)
                .map_err(io::Error::other)?
            {
                Some(buffer) => {
                    self.buffer = buffer;
                    self.buffer_pos = 0;
                }
                None => return Ok(0),
            }
        }
        let count = bytes.len().min(self.buffer.len() - self.buffer_pos);
        bytes[..count].copy_from_slice(&self.buffer[self.buffer_pos..self.buffer_pos + count]);
        self.buffer_pos += count;
        Ok(count)
    }
}
