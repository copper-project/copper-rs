#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(not(feature = "std"))]
extern crate alloc;
extern crate core;

#[cfg(feature = "std")]
pub mod memmap;

#[cfg(feature = "std")]
mod compat {
    // backward compatibility for the std implementation
    pub use crate::memmap::MmapUnifiedLogger as UnifiedLogger;
    pub use crate::memmap::MmapUnifiedLoggerBuilder as UnifiedLoggerBuilder;
    pub use crate::memmap::MmapUnifiedLoggerRead as UnifiedLoggerRead;
    pub use crate::memmap::MmapUnifiedLoggerWrite as UnifiedLoggerWrite;
    pub use crate::memmap::UnifiedLoggerIOReader;
}

#[cfg(feature = "std")]
pub use compat::*;

#[cfg(not(feature = "std"))]
mod imp {
    pub use alloc::string::ToString;
    pub use alloc::sync::Arc;
    pub use alloc::vec::Vec;
    pub use core::fmt::Debug;
    pub use core::fmt::Display;
    pub use core::fmt::Formatter;
    pub use core::fmt::Result as FmtResult;
    pub use spin::Mutex;
}

#[cfg(feature = "std")]
mod imp {
    pub use std::fmt::Debug;
    pub use std::fmt::Display;
    pub use std::fmt::Formatter;
    pub use std::fmt::Result as FmtResult;
    pub use std::sync::Arc;
    pub use std::sync::Mutex;
}

use imp::*;

use bincode::error::EncodeError;
use bincode::{Decode, Encode};
use cu29_traits::{CuError, CuResult, UnifiedLogType, WriteStream};

/// ID to spot the beginning of a Copper Log
#[allow(dead_code)]
pub const MAIN_MAGIC: [u8; 4] = [0xB4, 0xA5, 0x50, 0xFF]; // BRASS OFF

/// ID to spot a section of Copper Log
pub const SECTION_MAGIC: [u8; 2] = [0xFA, 0x57]; // FAST

pub const SECTION_HEADER_COMPACT_SIZE: u16 = 512; // Usual minimum size for a disk sector.

/// The main file header of the datalogger.
#[derive(Encode, Decode, Debug)]
pub struct MainHeader {
    pub magic: [u8; 4],            // Magic number to identify the file.
    pub first_section_offset: u16, // This is to align with a page at write time.
    pub page_size: u16,
}

impl Display for MainHeader {
    fn fmt(&self, f: &mut Formatter<'_>) -> FmtResult {
        writeln!(
            f,
            "  Magic -> {:2x}{:2x}{:2x}{:2x}",
            self.magic[0], self.magic[1], self.magic[2], self.magic[3]
        )?;
        writeln!(f, "  first_section_offset -> {}", self.first_section_offset)?;
        writeln!(f, "  page_size -> {}", self.page_size)
    }
}

/// Each concurrent sublogger is tracked through a section header.
/// They form a linked list of sections.
/// The entry type is used to identify the type of data in the section.
#[derive(Encode, Decode, Debug)]
pub struct SectionHeader {
    pub magic: [u8; 2],  // Magic number to identify the section.
    pub block_size: u16, // IMPORTANT: we assume this header fits in this block size.
    pub entry_type: UnifiedLogType,
    pub offset_to_next_section: u32, // offset from the first byte of this header to the first byte of the next header (MAGIC to MAGIC).
    pub used: u32,                   // how much of the section is filled.
    pub is_open: bool,               // true while being written, false once closed.
}

impl Display for SectionHeader {
    fn fmt(&self, f: &mut Formatter<'_>) -> FmtResult {
        writeln!(f, "    Magic -> {:2x}{:2x}", self.magic[0], self.magic[1])?;
        writeln!(f, "    type -> {:?}", self.entry_type)?;
        write!(
            f,
            "    use  -> {} / {} (open: {})",
            self.used, self.offset_to_next_section, self.is_open
        )
    }
}

impl Default for SectionHeader {
    fn default() -> Self {
        Self {
            magic: SECTION_MAGIC,
            block_size: 512,
            entry_type: UnifiedLogType::Empty,
            offset_to_next_section: 0,
            used: 0,
            is_open: true,
        }
    }
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
    pub total_used_space: usize,
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

/// A wrapper around the unifiedlogger that implements the Write trait.
struct LogStream<S: SectionStorage, L: UnifiedLogWrite<S>> {
    entry_type: UnifiedLogType,
    parent_logger: Arc<Mutex<L>>,
    current_section: SectionHandle<S>,
    current_position: usize,
    minimum_allocation_amount: usize,
}

impl<S: SectionStorage, L: UnifiedLogWrite<S>> LogStream<S, L> {
    fn new(
        entry_type: UnifiedLogType,
        parent_logger: Arc<Mutex<L>>,
        minimum_allocation_amount: usize,
    ) -> CuResult<Self> {
        #[cfg(feature = "std")]
        let section = parent_logger
            .lock()
            .expect("Could not lock a section at MmapStream creation")
            .add_section(entry_type, minimum_allocation_amount)?;

        #[cfg(not(feature = "std"))]
        let section = parent_logger
            .lock()
            .add_section(entry_type, minimum_allocation_amount)?;

        Ok(Self {
            entry_type,
            parent_logger,
            current_section: section,
            current_position: 0,
            minimum_allocation_amount,
        })
    }
}

impl<S: SectionStorage, L: UnifiedLogWrite<S>> Debug for LogStream<S, L> {
    fn fmt(&self, f: &mut Formatter<'_>) -> FmtResult {
        write!(f, "MmapStream {{ entry_type: {:?}, current_position: {}, minimum_allocation_amount: {} }}", self.entry_type, self.current_position, self.minimum_allocation_amount)
    }
}

impl<E: Encode, S: SectionStorage, L: UnifiedLogWrite<S>> WriteStream<E> for LogStream<S, L> {
    fn log(&mut self, obj: &E) -> CuResult<()> {
        //let dst = self.current_section.get_user_buffer();
        // let result = encode_into_slice(obj, dst, standard());
        let result = self.current_section.append(obj);
        match result {
            Ok(nb_bytes) => {
                self.current_position += nb_bytes;
                self.current_section.header.used += nb_bytes as u32;
                Ok(())
            }
            Err(e) => match e {
                EncodeError::UnexpectedEnd => {
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

                    logger_guard.flush_section(&mut self.current_section);
                    self.current_section = logger_guard
                        .add_section(self.entry_type, self.minimum_allocation_amount)?;

                    let result = self.current_section.append(obj).expect(
                        "Failed to encode object in a newly minted section. Unrecoverable failure.",
                    ); // If we fail just after creating a section, there is not much we can do, we need to bail.

                    self.current_position += result;
                    self.current_section.header.used += result as u32;
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

impl<S: SectionStorage, L: UnifiedLogWrite<S>> Drop for LogStream<S, L> {
    fn drop(&mut self) {
        #[cfg(feature = "std")]
        let logger_guard = self.parent_logger.lock();

        #[cfg(not(feature = "std"))]
        let mut logger_guard = self.parent_logger.lock();

        #[cfg(feature = "std")]
        let mut logger_guard = match logger_guard {
            Ok(g) => g,
            Err(_) => return,
        };
        logger_guard.flush_section(&mut self.current_section);
        #[cfg(feature = "std")]
        if !std::thread::panicking() {
            eprintln!("⚠️ MmapStream::drop: logger mutex poisoned");
        }
    }
}
