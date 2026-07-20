//! # Byte Buffer Abstractions
//!
//! This module provides zero-copy, `no_std` compatible traits for reading and writing
//! byte buffers. These traits are the foundation of the Zenoh protocol codec system.
//!
//! ## Overview
//!
//! The module defines two primary trait families:
//!
//! - **Writing**: [`ZWriteable`] and optionally [`ZStoreable`] for capturing written data
//! - **Reading**: [`ZReadable`] for consuming byte data
//!
//! ## Writing Data
//!
//! The [`ZWriteable`] trait provides methods for writing bytes into a buffer:
//!
//! ```ignore
//! let mut buffer = [0u8; 128];
//! let mut writer = &mut buffer[..];
//!
//! writer.write_u8(42)?;
//! writer.write_exact(&[1, 2, 3, 4])?;
//! ```
//!
//! ## Reading Data
//!
//! The [`ZReadable`] trait provides methods for reading bytes from a buffer:
//!
//! ```ignore
//! let data = &[42u8, 1, 2, 3, 4][..];
//! let mut reader = data;
//!
//! let value = reader.read_u8()?;
//! let slice = reader.read_slice(4)?;
//! ```
//!
//! ## Implementations
//!
//! Both traits are implemented for mutable byte slices (`&mut [u8]`) and
//! immutable byte slices (`&[u8]`), providing efficient cursor-based operations.

/// Trait for writing bytes into a buffer in a streaming fashion.
///
/// This trait provides methods for writing data sequentially into a byte buffer.
/// The buffer automatically advances as data is written, maintaining a cursor position.
///
/// # Implementation Note
///
/// The standard implementation for `&mut [u8]` uses pointer arithmetic to advance
/// the slice as data is written, providing zero-overhead abstraction.
///
/// # Examples
///
/// ```ignore
/// let mut buffer = [0u8; 16];
/// let mut writer = &mut buffer[..];
///
/// writer.write_u8(0x42)?;
/// writer.write_exact(&[1, 2, 3])?;
///
/// assert_eq!(writer.remaining(), 12);
/// ```
pub trait ZWriteable {
    /// Returns the number of bytes remaining in the buffer.
    ///
    /// This indicates how much more data can be written before the buffer is full.
    fn remaining(&self) -> usize;

    /// Writes bytes from the source slice into the buffer.
    ///
    /// This method writes as many bytes as possible (up to `src.len()` or the
    /// remaining buffer space, whichever is smaller) and returns the number
    /// of bytes actually written.
    ///
    /// # Parameters
    ///
    /// - `src`: The source data to write
    ///
    /// # Returns
    ///
    /// The number of bytes successfully written.
    ///
    /// # Errors
    ///
    /// Returns [`BytesError`](crate::BytesError) if writing fails.
    fn write(&mut self, src: &'_ [u8]) -> core::result::Result<usize, crate::BytesError>;

    /// Writes a single byte to the buffer.
    ///
    /// # Parameters
    ///
    /// - `value`: The byte value to write
    ///
    /// # Errors
    ///
    /// Returns [`BytesError::DstIsFull`](crate::BytesError::DstIsFull) if the buffer has no space remaining.
    fn write_u8(&mut self, value: u8) -> core::result::Result<(), crate::BytesError>;

    /// Writes all bytes from the source slice, failing if buffer is too small.
    ///
    /// Unlike [`write`](Self::write), this method ensures all bytes are written
    /// or returns an error.
    ///
    /// # Parameters
    ///
    /// - `src`: The source data to write completely
    ///
    /// # Errors
    ///
    /// Returns [`BytesError::DstIsTooSmall`](crate::BytesError::DstIsTooSmall) if the buffer cannot hold all bytes.
    fn write_exact(&mut self, src: &'_ [u8]) -> core::result::Result<(), crate::BytesError> {
        let len = src.len();
        let written = self.write(src)?;
        if written < len {
            crate::error!(
                "dst (len: {}) is too small to write exact {} bytes - {}",
                self.remaining(),
                len,
                crate::zctx!()
            );

            crate::zbail!(crate::BytesError::DstIsTooSmall);
        }

        Ok(())
    }

    /// Writes data by providing a mutable slice for the writer function to fill.
    ///
    /// This method allocates a slot of `len` bytes in the buffer and calls the
    /// provided `writer` function with that slot. The function should write data
    /// and return the actual number of bytes written.
    ///
    /// # Parameters
    ///
    /// - `len`: The maximum number of bytes to allocate
    /// - `writer`: Function that fills the buffer and returns bytes written
    ///
    /// # Returns
    ///
    /// The actual number of bytes written by the writer function.
    ///
    /// # Errors
    ///
    /// Returns [`BytesError::DstIsTooSmall`](crate::BytesError::DstIsTooSmall) if the buffer has insufficient space
    /// or if the writer function reports writing more bytes than allocated.
    fn write_slot(
        &mut self,
        len: usize,
        writer: impl FnOnce(&mut [u8]) -> usize,
    ) -> core::result::Result<usize, crate::BytesError>;
}

/// Trait for capturing written data as borrowed slices (test-only).
///
/// This trait extends [`ZWriteable`] to allow capturing written data as references
/// with a specific lifetime. It's used in tests to verify encoding output without
/// allocating separate buffers.
///
/// # Safety
///
/// The methods in this trait are unsafe because they rely on maintaining proper
/// lifetime relationships between the buffer and returned slices.
#[cfg(test)]
pub trait ZStoreable<'a>: ZWriteable {
    /// Marker type for tracking buffer positions.
    type Mark;

    /// Creates a marker for the current buffer position.
    fn mark(&self) -> Self::Mark;

    /// Returns a slice from the marked position to the current position.
    ///
    /// # Safety
    ///
    /// The caller must ensure the mark is valid and the buffer hasn't been moved.
    unsafe fn slice(&self, mark: &Self::Mark) -> &'a [u8];

    /// Writes data into the buffer and returns it as a borrowed slice.
    ///
    /// This combines writing with capturing the written data.
    ///
    /// # Parameters
    ///
    /// - `len`: Maximum bytes to write
    /// - `data`: Function that writes data and returns bytes written
    ///
    /// # Returns
    ///
    /// A borrowed slice containing the data that was written.
    ///
    /// # Safety
    ///
    /// The caller must ensure the returned slice lifetime is valid.
    unsafe fn store(
        &mut self,
        len: usize,
        data: impl FnOnce(&mut [u8]) -> usize,
    ) -> core::result::Result<&'a [u8], crate::BytesError> {
        let mark = self.mark();
        let written = self.write_slot(len, data)?;
        Ok(unsafe { &self.slice(&mark)[..written] })
    }

    /// Writes a string into the buffer and returns it as a borrowed string slice.
    ///
    /// # Safety
    ///
    /// The caller must ensure the returned slice lifetime is valid.
    unsafe fn store_str(&mut self, s: &str) -> core::result::Result<&'a str, crate::BytesError> {
        let bytes = s.as_bytes();
        let slot = unsafe {
            self.store(bytes.len(), |buf| {
                buf[..bytes.len()].copy_from_slice(bytes);
                bytes.len()
            })?
        };

        Ok(core::str::from_utf8(slot)
            .expect("Stored string is not valid UTF-8, this should never happen"))
    }
}

/// Trait for reading bytes from a buffer in a streaming fashion.
///
/// This trait provides methods for reading data sequentially from a byte buffer.
/// The buffer automatically advances as data is read, maintaining a cursor position.
///
/// # Zero-Copy Design
///
/// Methods like [`read_slice`](Self::read_slice) return borrowed slices directly
/// from the buffer without copying, enabling efficient zero-copy parsing.
///
/// # Examples
///
/// ```ignore
/// let data = &[0x42, 1, 2, 3, 4][..];
/// let mut reader = data;
///
/// let byte = reader.read_u8()?;
/// assert_eq!(byte, 0x42);
///
/// let slice = reader.read_slice(4)?;
/// assert_eq!(slice, &[1, 2, 3, 4]);
/// ```
pub trait ZReadable<'a> {
    /// Returns the number of bytes remaining in the buffer.
    fn remaining(&self) -> usize;

    /// Peeks at the next byte without consuming it.
    ///
    /// # Errors
    ///
    /// Returns [`BytesError::SrcIsEmpty`](crate::BytesError::SrcIsEmpty) if the buffer is empty.
    fn peek(&self) -> core::result::Result<u8, crate::BytesError>;

    /// Returns a borrowed slice of `len` bytes and advances the cursor.
    ///
    /// This is a zero-copy operation that returns a reference to the buffer's data.
    ///
    /// # Parameters
    ///
    /// - `len`: The number of bytes to read
    ///
    /// # Errors
    ///
    /// Returns [`BytesError::SrcIsTooSmall`](crate::BytesError::SrcIsTooSmall) if the buffer has fewer than `len` bytes.
    fn read_slice(&mut self, len: usize) -> core::result::Result<&'a [u8], crate::BytesError>;

    /// Returns true if there are bytes remaining to read.
    fn can_read(&self) -> bool {
        self.remaining().gt(&0)
    }

    /// Reads a single byte and advances the cursor.
    ///
    /// # Errors
    ///
    /// Returns [`BytesError::SrcIsEmpty`](crate::BytesError::SrcIsEmpty) if the buffer is empty.
    fn read_u8(&mut self) -> core::result::Result<u8, crate::BytesError>;

    /// Reads exactly `dst.len()` bytes into the destination slice.
    ///
    /// # Parameters
    ///
    /// - `dst`: The destination buffer to fill
    ///
    /// # Errors
    ///
    /// Returns [`BytesError::SrcIsTooSmall`](crate::BytesError::SrcIsTooSmall) if the buffer has insufficient bytes.
    fn read_exact(&mut self, dst: &'_ mut [u8]) -> core::result::Result<(), crate::BytesError> {
        let len = dst.len();
        if self.remaining() < len {
            crate::trace!(
                "src (len: {}) is too small to read exact {} bytes - {}",
                self.remaining(),
                len,
                crate::zctx!()
            );

            crate::zbail!(crate::BytesError::SrcIsTooSmall);
        }

        let bytes = self.read_slice(len)?;
        dst.copy_from_slice(bytes);

        Ok(())
    }
}

/// Implementation of [`ZWriteable`] for mutable byte slices.
///
/// This implementation uses unsafe pointer arithmetic to advance the slice
/// as data is written, providing zero-copy, zero-overhead cursor-based writing.
impl ZWriteable for &mut [u8] {
    fn remaining(&self) -> usize {
        self.len()
    }

    fn write_u8(&mut self, value: u8) -> core::result::Result<(), crate::BytesError> {
        if self.remaining() == 0 {
            crate::trace!("dst (len: 0) is full, cannot write u8 - {}", crate::zctx!());
            crate::zbail!(crate::BytesError::DstIsFull);
        }

        unsafe {
            *self.get_unchecked_mut(0) = value;
            *self = core::mem::take(self).get_unchecked_mut(1..);
        }

        Ok(())
    }

    fn write(&mut self, src: &'_ [u8]) -> core::result::Result<usize, crate::BytesError> {
        let len = src.len().min(self.len());
        let (head, tail) = unsafe { core::mem::take(self).split_at_mut_unchecked(len) };

        head.copy_from_slice(&src[..len]);
        *self = tail;

        Ok(len)
    }

    fn write_slot(
        &mut self,
        len: usize,
        writer: impl FnOnce(&mut [u8]) -> usize,
    ) -> core::result::Result<usize, crate::BytesError> {
        if self.len() < len {
            crate::trace!(
                "dst (len: {}) is too small to write slot of len {} - {}",
                self.len(),
                len,
                crate::zctx!()
            );
            crate::zbail!(crate::BytesError::DstIsTooSmall);
        }

        let written = writer(&mut self[..len]);

        if written > len {
            crate::trace!(
                "writer wrote {} bytes which exceeds the allocated slot size of {} - {}",
                written,
                len,
                crate::zctx!()
            );
            crate::zbail!(crate::BytesError::DstIsTooSmall);
        }

        let (_, tail) = unsafe { core::mem::take(self).split_at_mut_unchecked(written) };
        *self = tail;

        Ok(written)
    }
}

/// Marker type for tracking positions in a byte slice buffer.
#[cfg(test)]
pub struct SliceMark<'a> {
    ptr: *const u8,
    len: usize,
    _marker: core::marker::PhantomData<&'a u8>,
}

/// Implementation of [`ZStoreable`] for mutable byte slices (test-only).
#[cfg(test)]
impl<'a> ZStoreable<'a> for &'a mut [u8] {
    type Mark = SliceMark<'a>;
    fn mark(&self) -> Self::Mark {
        SliceMark {
            ptr: self.as_ptr(),
            len: self.len(),
            _marker: core::marker::PhantomData,
        }
    }

    unsafe fn slice(&self, mark: &Self::Mark) -> &'a [u8] {
        unsafe { core::slice::from_raw_parts(mark.ptr as *mut u8, mark.len) }
    }
}

/// Implementation of [`ZReadable`] for immutable byte slices.
///
/// This implementation uses unsafe pointer arithmetic to advance the slice
/// as data is read, providing zero-copy, zero-overhead cursor-based reading.
impl<'a> ZReadable<'a> for &'a [u8] {
    fn remaining(&self) -> usize {
        self.len()
    }

    fn peek(&self) -> core::result::Result<u8, crate::BytesError> {
        if !self.can_read() {
            crate::trace!("src (len: 0) is empty, cannot peek u8 - {}", crate::zctx!());
            crate::zbail!(crate::BytesError::SrcIsEmpty)
        }

        Ok(unsafe { *self.get_unchecked(0) })
    }

    fn read_u8(&mut self) -> core::result::Result<u8, crate::BytesError> {
        if !self.can_read() {
            crate::trace!("src (len: 0) is empty, cannot read u8 - {}", crate::zctx!());
            crate::zbail!(crate::BytesError::SrcIsEmpty);
        }

        let value = unsafe { *self.get_unchecked(0) };
        *self = unsafe { self.get_unchecked(1..) };

        Ok(value)
    }

    fn read_slice(&mut self, len: usize) -> core::result::Result<&'a [u8], crate::BytesError> {
        if self.remaining() < len {
            crate::trace!(
                "src (len: {}) is too small to read {} bytes - {}",
                self.remaining(),
                len,
                crate::zctx!()
            );
            crate::zbail!(crate::BytesError::SrcIsTooSmall);
        }

        let (head, tail) = unsafe { self.split_at_unchecked(len) };
        *self = tail;

        Ok(head)
    }
}
