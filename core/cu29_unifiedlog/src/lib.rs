#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(not(feature = "std"))]
extern crate alloc;

#[cfg(feature = "std")]
mod memmap;

use core::mem;

#[cfg(feature = "std")]
mod compat {
    // backward compatibility for the std implementation
    pub use crate::memmap::mmap_stream_write as stream_write;
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
    pub use alloc::vec::Vec;
    pub use core::fmt::Display;
    pub use core::fmt::Formatter;
    pub use core::fmt::Result as FmtResult;
}

#[cfg(feature = "std")]
mod imp {
    pub use core::fmt::Display;
    pub use std::fmt::Formatter;
    pub use std::fmt::Result as FmtResult;
}

use imp::*;

use bincode::{Decode, Encode};
use cu29_traits::{CuResult, UnifiedLogType};

#[allow(dead_code)] // TODO(gbin): Can be removed once no-std implementation is done.
const MAIN_MAGIC: [u8; 4] = [0xB4, 0xA5, 0x50, 0xFF];

const SECTION_MAGIC: [u8; 2] = [0xFA, 0x57];

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
pub trait UnifiedLogWrite {
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
