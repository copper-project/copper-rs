#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(not(feature = "std"))]
extern crate alloc;
extern crate core;

#[cfg(feature = "std")]
mod memmap;

use core::mem;

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

use bincode::config::standard;
use bincode::error::EncodeError;
use bincode::{encode_into_slice, Decode, Encode};
use cu29_traits::{CuError, CuResult, UnifiedLogType, WriteStream};

/// ID to spot the beginning of a Copper Log
#[allow(dead_code)]
pub const MAIN_MAGIC: [u8; 4] = [0xB4, 0xA5, 0x50, 0xFF]; // BRASS OFF

/// ID to spot a section of Copper Log
pub const SECTION_MAGIC: [u8; 2] = [0xFA, 0x57]; // FAST

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
    pub magic: [u8; 2], // Magic number to identify the section.
    pub entry_type: UnifiedLogType,
    pub section_size: u32, // offset from the first byte of this header to the first byte of the next header (MAGIC to MAGIC).
    pub filled_size: u32,  // how much of the section is filled.
}

const MAX_HEADER_SIZE: usize = mem::size_of::<SectionHeader>() + 3usize; // 3 == additional worse case scenario for the 3 int variable encoding

impl Display for SectionHeader {
    fn fmt(&self, f: &mut Formatter<'_>) -> FmtResult {
        writeln!(f, "    Magic -> {:2x}{:2x}", self.magic[0], self.magic[1])?;
        writeln!(f, "    type -> {:?}", self.entry_type)?;
        write!(
            f,
            "    use  -> {} / {}",
            self.filled_size, self.section_size
        )
    }
}

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

pub enum AllocatedSection {
    NoMoreSpace,
    Section(SectionHandle),
}

/// A SectionHandle is a handle to a section in the datalogger.
/// It allows to track the lifecycle of a section of the datalogger.
#[derive(Default)]
pub struct SectionHandle {
    section_header: SectionHeader,
    buffer: &'static mut [u8], // This includes the encoded header for end of section patching.
    used: u32,                 // this is the size of the used part of the buffer.
}

// This is for a placeholder to unsure an orderly cleanup as we dodge the borrow checker.

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

    pub fn update_header(&mut self) {
        // no need to do anything if we never used the section.
        if self.section_header.entry_type == UnifiedLogType::Empty || self.used == 0 {
            return;
        }
        self.section_header.filled_size = self.used;
    }
}

/// Basic statistics for the unified logger.
/// Note: the total_allocated_space might grow for the std implementation
pub struct UnifiedLogStatus {
    pub total_used_space: usize,
    pub total_allocated_space: usize,
}

/// The writing interface to the unified logger.
/// Writing is "almost" linear as various streams can allocate sections and track them until
/// they drop them.
pub trait UnifiedLogWrite: Send + Sync {
    /// A section is a contiguous chunk of memory that can be used to write data.
    /// It can store various types of data as specified by the entry_type.
    /// The requested_section_size is the size of the section to allocate.
    /// It returns a handle to the section that can be used to write data until
    /// it is flushed with flush_section, it is then considered unmutable.
    fn add_section(
        &mut self,
        entry_type: UnifiedLogType,
        requested_section_size: usize,
    ) -> SectionHandle;

    /// Flush the given section to the underlying storage.
    fn flush_section(&mut self, section: &mut SectionHandle);

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
pub fn stream_write<E: Encode>(
    logger: Arc<Mutex<impl UnifiedLogWrite>>,
    entry_type: UnifiedLogType,
    minimum_allocation_amount: usize,
) -> impl WriteStream<E> {
    LogStream::new(entry_type, logger.clone(), minimum_allocation_amount)
}

/// A wrapper around the unifiedlogger that implements the Write trait.
struct LogStream<L: UnifiedLogWrite> {
    entry_type: UnifiedLogType,
    parent_logger: Arc<Mutex<L>>,
    current_section: SectionHandle,
    current_position: usize,
    minimum_allocation_amount: usize,
}

impl<L: UnifiedLogWrite> LogStream<L> {
    fn new(
        entry_type: UnifiedLogType,
        parent_logger: Arc<Mutex<L>>,
        minimum_allocation_amount: usize,
    ) -> Self {
        #[cfg(feature = "std")]
        let section = parent_logger
            .lock()
            .expect("Could not lock a section at MmapStream creation")
            .add_section(entry_type, minimum_allocation_amount);

        #[cfg(not(feature = "std"))]
        let section = parent_logger
            .lock()
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

impl<L: UnifiedLogWrite> Debug for LogStream<L> {
    fn fmt(&self, f: &mut Formatter<'_>) -> FmtResult {
        write!(f, "MmapStream {{ entry_type: {:?}, current_position: {}, minimum_allocation_amount: {} }}", self.entry_type, self.current_position, self.minimum_allocation_amount)
    }
}

impl<E: Encode, L: UnifiedLogWrite> WriteStream<E> for LogStream<L> {
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

impl<L: UnifiedLogWrite> Drop for LogStream<L> {
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
