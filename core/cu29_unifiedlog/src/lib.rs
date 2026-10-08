#![cfg_attr(not(feature = "std"), no_std)]

extern crate alloc;
extern crate core;

#[doc(hidden)]
pub mod byte_log;
#[cfg(feature = "std")]
pub mod memmap;
pub mod noop;

#[cfg(feature = "std")]
mod compat {
    // backward compatibility for the std implementation
    pub use crate::memmap::LogPosition;
    pub use crate::memmap::MmapUnifiedLogger as UnifiedLogger;
    pub use crate::memmap::MmapUnifiedLoggerBuilder as UnifiedLoggerBuilder;
    pub use crate::memmap::MmapUnifiedLoggerRead as UnifiedLoggerRead;
    pub use crate::memmap::MmapUnifiedLoggerWrite as UnifiedLoggerWrite;
    pub use crate::memmap::UnifiedLoggerIOReader;
}

#[cfg(feature = "std")]
pub use compat::*;
pub use noop::{NoopLogger, NoopSectionStorage};

use alloc::string::ToString;
#[cfg(not(feature = "std"))]
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt::{Debug, Display, Formatter, Result as FmtResult};
#[cfg(not(feature = "std"))]
use spin::Mutex;
#[cfg(feature = "std")]
use std::sync::{Arc, Mutex};

use bincode::error::EncodeError;
use bincode::{Decode, Encode};
use cu29_traits::{CuError, CuResult, UnifiedLogType, WriteStream};

/// ID to spot the beginning of a Copper Log
#[allow(dead_code)]
pub const MAIN_MAGIC: [u8; 4] = [0xB4, 0xA5, 0x50, 0xFF]; // BRASS OFF

/// ID to spot a section of Copper Log
pub const SECTION_MAGIC: [u8; 2] = [0xFA, 0x57]; // FAST

/// Version of the unified log **encapsulation only**: file headers, section
/// headers, and the layout used to locate sections in a slab.
///
/// This is **never** a version of the encoded content inside sections. Changes
/// to CopperLists, payload types, keyframes, or their serialization must not bump
/// this value. Decode content with the logreader built for the exact application
/// version that produced it; this header cannot establish content compatibility.
/// Unreleased version 2 uses byte links, static metadata and construction contexts.
pub const UNIFIED_LOG_FORMAT_VERSION: u8 = 2;

pub const SECTION_HEADER_COMPACT_SIZE: u16 = 512; // Usual minimum size for a disk sector.

/// Header of the byte-addressed log. All offsets are relative to the log origin.
#[derive(Encode, Decode, Debug, Clone)]
pub struct MainHeader {
    pub magic: [u8; 4],
    pub format_version: u8,
    pub page_size: u16,
    pub metadata_offset: u64,
    pub sections_begin: u64,
    pub sections_end: u64,
    pub head_section: u64,
    pub tail_section: u64,
    pub clean_close: bool,
}

impl Display for MainHeader {
    fn fmt(&self, f: &mut Formatter<'_>) -> FmtResult {
        write!(
            f,
            "  format_version -> {}\n  alignment -> {}\n  metadata -> {}\n  data -> {}..{}\n  head -> {}\n  tail -> {}\n  clean_close -> {}",
            self.format_version,
            self.page_size,
            self.metadata_offset,
            self.sections_begin,
            self.sections_end,
            self.head_section,
            self.tail_section,
            self.clean_close
        )
    }
}

/// Construction identity shared by every section of an application instance.
#[derive(Encode, Decode, Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct SectionContext {
    pub run_id: u64,
    pub instance_id: u32,
    pub mission_index: u32,
}

/// One linked section. The allocation includes its 512-byte header reservation.
#[derive(Encode, Decode, Debug, Clone)]
pub struct SectionHeader {
    pub magic: [u8; 2],
    pub block_size: u16,
    pub entry_type: UnifiedLogType,
    pub allocated: u64,
    pub next_section: u64,
    pub used: u32,
    pub is_open: bool,
    pub context: SectionContext,
}

impl Display for SectionHeader {
    fn fmt(&self, f: &mut Formatter<'_>) -> FmtResult {
        write!(
            f,
            "    type -> {:?}\n    use -> {} / {} (open: {})\n    next -> {}\n    context -> {:?}",
            self.entry_type,
            self.used,
            self.allocated,
            self.is_open,
            self.next_section,
            self.context
        )
    }
}

impl Default for SectionHeader {
    fn default() -> Self {
        Self {
            magic: SECTION_MAGIC,
            block_size: SECTION_HEADER_COMPACT_SIZE,
            entry_type: UnifiedLogType::Empty,
            allocated: 0,
            next_section: 0,
            used: 0,
            is_open: true,
            context: SectionContext::default(),
        }
    }
}

/// Static application identity and configuration, retained outside the data ring.
#[derive(Encode, Decode, Debug, Clone, PartialEq, Eq)]
pub struct ApplicationMetadata {
    pub app_type: alloc::string::String,
    pub app_name: alloc::string::String,
    pub app_version: alloc::string::String,
    pub git_commit: Option<alloc::string::String>,
    pub git_dirty: Option<bool>,
    pub subsystem_id: Option<alloc::string::String>,
    pub subsystem_code: u16,
    pub effective_config_ron: alloc::string::String,
    pub missions: Vec<alloc::string::String>,
    pub catalog_offset: u64,
}

/// Behavior when a section needs additional storage.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum CapacityPolicy {
    #[default]
    Grow,
    OverwriteOldest,
    StopWhenFull,
}

pub enum AllocatedSection<S: SectionStorage> {
    NoMoreSpace,
    Section(SectionHandle<S>),
}

/// A Storage is an append-only structure that can update a header section.
pub trait SectionStorage: Send + Sync {
    /// This rewinds the storage, serialize the header and jumps to the beginning of the user data storage.
    fn initialize<E: Encode>(&mut self, header: &E) -> Result<usize, EncodeError>;
    /// This updates the header leaving the position to the end of the user data storage.
    fn post_update_header<E: Encode>(&mut self, header: &E) -> Result<usize, EncodeError>;
    /// Appends the entry to the user data storage.
    fn append<E: Encode>(&mut self, entry: &E) -> Result<usize, EncodeError>;
    /// Whether this handle was sealed by section reclamation.
    fn is_sealed(&self) -> bool {
        false
    }
    /// Flushes the section to the underlying storage
    fn flush(&mut self) -> CuResult<usize>;
}

/// A SectionHandle is a handle to a section in the datalogger.
/// It allows tracking the lifecycle of the section.
#[derive(Default)]
pub struct SectionHandle<S: SectionStorage> {
    header: SectionHeader, // keep a copy of the header as metadata
    storage: S,
}

impl<S: SectionStorage> SectionHandle<S> {
    pub fn create(header: SectionHeader, mut storage: S) -> CuResult<Self> {
        // Write the first version of the header.
        let _ = storage.initialize(&header).map_err(|e| e.to_string())?;
        Ok(Self { header, storage })
    }

    pub fn mark_closed(&mut self) {
        self.header.is_open = false;
    }
    pub fn append<E: Encode>(&mut self, entry: E) -> Result<usize, EncodeError> {
        self.storage.append(&entry)
    }

    pub fn get_storage(&self) -> &S {
        &self.storage
    }

    pub fn get_storage_mut(&mut self) -> &mut S {
        &mut self.storage
    }

    pub fn post_update_header(&mut self) -> Result<usize, EncodeError> {
        self.storage.post_update_header(&self.header)
    }
}

/// Basic statistics for the unified logger.
/// Note: the total_allocated_space might grow for the std implementation
pub struct UnifiedLogStatus {
    /// Bytes reserved for headers, static metadata and retained data sections.
    pub total_used_space: usize,
    /// Total logical backing capacity, including free space.
    pub total_allocated_space: usize,
}

/// Payload stored in the end-of-log section to signal whether the log was cleanly closed.
#[derive(Encode, Decode, Debug, Clone)]
pub struct EndOfLogMarker {
    pub temporary: bool,
}

/// The writing interface to the unified logger.
/// Writing is "almost" linear as various streams can allocate sections and track them until
/// they drop them.
pub trait UnifiedLogWrite<S: SectionStorage>: Send + Sync {
    /// A section is a contiguous chunk of memory that can be used to write data.
    /// It can store various types of data as specified by the entry_type.
    /// The requested_section_size is the size of the section to allocate.
    /// It returns a handle to the section that can be used to write data until
    /// it is flushed with flush_section, it is then considered unmutable.
    fn add_section(
        &mut self,
        entry_type: UnifiedLogType,
        requested_section_size: usize,
    ) -> CuResult<SectionHandle<S>>;

    /// Flush the given section to the underlying storage.
    fn flush_section(&mut self, section: &mut SectionHandle<S>);

    /// Flush a section, reporting storage failures.
    fn try_flush_section(&mut self, section: &mut SectionHandle<S>) -> CuResult<()> {
        self.flush_section(section);
        Ok(())
    }

    /// Seal or compare static metadata before initializing runtime streams.
    #[doc(hidden)]
    fn seal_metadata<C: Encode>(
        &mut self,
        _metadata: &ApplicationMetadata,
        _catalog: Option<&C>,
    ) -> CuResult<()> {
        Err(CuError::from(
            "Logger does not support application metadata",
        ))
    }
    /// Reserve a construction identity after static metadata has been compared.
    #[doc(hidden)]
    fn construction_context(
        &mut self,
        instance_id: u32,
        mission_index: u32,
    ) -> CuResult<SectionContext> {
        Ok(SectionContext {
            run_id: 0,
            instance_id,
            mission_index,
        })
    }
    /// Allocate a section belonging to one construction.
    #[doc(hidden)]
    fn add_section_with_context(
        &mut self,
        kind: UnifiedLogType,
        size: usize,
        _context: SectionContext,
    ) -> CuResult<SectionHandle<S>> {
        self.add_section(kind, size)
    }
    /// Returns the current status of the unified logger.
    fn status(&self) -> UnifiedLogStatus;
}

/// Read back a unified log linearly.
pub trait UnifiedLogRead {
    /// Read through the unified logger until it reaches the UnifiedLogType given in datalogtype.
    /// It will return the byte array of the section if found.
    fn read_next_section_type(&mut self, datalogtype: UnifiedLogType) -> CuResult<Option<Vec<u8>>>;

    /// Read through the next section entry regardless of its type.
    /// It will return the header and the byte array of the section.
    /// Note the last Entry should be of UnifiedLogType::LastEntry if the log is not corrupted.
    fn raw_read_section(&mut self) -> CuResult<(SectionHeader, Vec<u8>)>;
}

/// Create a new stream to write to the unifiedlogger.
pub fn stream_write<E: Encode, S: SectionStorage>(
    logger: Arc<Mutex<impl UnifiedLogWrite<S>>>,
    entry_type: UnifiedLogType,
    minimum_allocation_amount: usize,
) -> CuResult<impl WriteStream<E>> {
    LogStream::new(entry_type, logger, minimum_allocation_amount)
}

/// Create a stream whose sections retain their construction identity across rollover.
#[doc(hidden)]
pub fn stream_write_context<E: Encode, S: SectionStorage>(
    logger: Arc<Mutex<impl UnifiedLogWrite<S>>>,
    entry_type: UnifiedLogType,
    size: usize,
    context: SectionContext,
) -> CuResult<impl WriteStream<E>> {
    LogStream::with_context(entry_type, logger, size, context)
}

/// A wrapper around the unifiedlogger that implements the Write trait.
pub struct LogStream<S: SectionStorage, L: UnifiedLogWrite<S>> {
    entry_type: UnifiedLogType,
    parent_logger: Arc<Mutex<L>>,
    current_section: SectionHandle<S>,
    current_position: usize,
    minimum_allocation_amount: usize,
    last_log_bytes: usize,
    context: SectionContext,
}

impl<S: SectionStorage, L: UnifiedLogWrite<S>> LogStream<S, L> {
    /// Creates a concrete stream, including for borrowed canonical encoded entries.
    pub fn new(
        entry_type: UnifiedLogType,
        parent_logger: Arc<Mutex<L>>,
        minimum_allocation_amount: usize,
    ) -> CuResult<Self> {
        Self::with_context(
            entry_type,
            parent_logger,
            minimum_allocation_amount,
            SectionContext::default(),
        )
    }

    #[doc(hidden)]
    pub fn with_context(
        entry_type: UnifiedLogType,
        parent_logger: Arc<Mutex<L>>,
        minimum_allocation_amount: usize,
        context: SectionContext,
    ) -> CuResult<Self> {
        #[cfg(feature = "std")]
        let section = parent_logger
            .lock()
            .map_err(|e| {
                CuError::from("Could not lock a section at LogStream creation")
                    .add_cause(e.to_string().as_str())
            })?
            .add_section_with_context(entry_type, minimum_allocation_amount, context)?;

        #[cfg(not(feature = "std"))]
        let section = parent_logger.lock().add_section_with_context(
            entry_type,
            minimum_allocation_amount,
            context,
        )?;

        Ok(Self {
            entry_type,
            parent_logger,
            current_section: section,
            current_position: 0,
            minimum_allocation_amount,
            last_log_bytes: 0,
            context,
        })
    }
}

impl<S: SectionStorage, L: UnifiedLogWrite<S>> Debug for LogStream<S, L> {
    fn fmt(&self, f: &mut Formatter<'_>) -> FmtResult {
        write!(
            f,
            "MmapStream {{ entry_type: {:?}, current_position: {}, minimum_allocation_amount: {} }}",
            self.entry_type, self.current_position, self.minimum_allocation_amount
        )
    }
}

impl<E: Encode, S: SectionStorage, L: UnifiedLogWrite<S>> WriteStream<E> for LogStream<S, L> {
    fn log(&mut self, obj: &E) -> CuResult<()> {
        let sealed = self.current_section.storage.is_sealed();
        let result = if sealed {
            Err(EncodeError::UnexpectedEnd)
        } else {
            self.current_section.append(obj)
        };
        match result {
            Ok(nb_bytes) => {
                self.current_position += nb_bytes;
                self.current_section.header.used += nb_bytes as u32;
                self.last_log_bytes = nb_bytes;
                // Lifecycle sections are promptly closed, so sparse events cannot pin the ring.
                if self.entry_type == UnifiedLogType::RuntimeLifecycle {
                    self.current_section.storage.flush()?;
                }
                Ok(())
            }
            Err(e) => match e {
                EncodeError::UnexpectedEnd => {
                    if !sealed && self.current_section.header.used == 0 {
                        return Err(CuError::from("Entry exceeds an empty section"));
                    }
                    #[cfg(feature = "std")]
                    let logger_guard = self.parent_logger.lock();

                    #[cfg(not(feature = "std"))]
                    let mut logger_guard = self.parent_logger.lock();

                    #[cfg(feature = "std")]
                    let mut logger_guard =
                        match logger_guard {
                            Ok(g) => g,
                            Err(_) => return Err(
                                "Logger mutex poisoned while reporting EncodeError::UnexpectedEnd"
                                    .into(),
                            ), // It will retry but at least not completely crash.
                        };

                    logger_guard.try_flush_section(&mut self.current_section)?;
                    self.current_section = logger_guard.add_section_with_context(
                        self.entry_type,
                        self.minimum_allocation_amount,
                        self.context,
                    )?;

                    let result = self
                        .current_section
                        .append(obj)
                        .map_err(|e| {
                            CuError::from(
                                "Failed to encode object in a newly minted section. Unrecoverable failure.",
                            )
                            .add_cause(e.to_string().as_str())
                        })?; // If we fail just after creating a section, there is not much we can do.

                    self.current_position += result;
                    self.current_section.header.used += result as u32;
                    self.last_log_bytes = result;
                    if self.entry_type == UnifiedLogType::RuntimeLifecycle {
                        self.current_section.storage.flush()?;
                    }
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

    fn flush(&mut self) -> CuResult<()> {
        #[cfg(feature = "std")]
        let mut logger = self
            .parent_logger
            .lock()
            .map_err(|_| CuError::from("Logger mutex poisoned"))?;
        #[cfg(not(feature = "std"))]
        let mut logger = self.parent_logger.lock();
        logger.try_flush_section(&mut self.current_section)
    }

    fn last_log_bytes(&self) -> Option<usize> {
        Some(self.last_log_bytes)
    }
}

impl<S: SectionStorage, L: UnifiedLogWrite<S>> Drop for LogStream<S, L> {
    fn drop(&mut self) {
        #[cfg(feature = "std")]
        match self.parent_logger.lock() {
            Ok(mut logger_guard) => {
                logger_guard.flush_section(&mut self.current_section);
            }
            Err(_) => {
                // Only surface the warning when a real poisoning occurred.
                if !std::thread::panicking() {
                    eprintln!("⚠️ MmapStream::drop: logger mutex poisoned");
                }
            }
        }

        #[cfg(not(feature = "std"))]
        {
            let mut logger_guard = self.parent_logger.lock();
            logger_guard.flush_section(&mut self.current_section);
        }
    }
}
