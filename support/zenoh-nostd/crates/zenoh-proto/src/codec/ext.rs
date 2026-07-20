//! # Protocol Extension System
//!
//! This module implements the Zenoh protocol extension mechanism, which allows
//! messages to include optional, versioned extensions in a backward-compatible way.
//!
//! ## Extension Kinds
//!
//! Extensions come in three kinds, encoded in the extension header:
//!
//! - **Unit**: A flag-only extension with no associated data (e.g., presence markers)
//! - **U64**: An extension containing a variable-length encoded 64-bit unsigned integer
//! - **ZStruct**: An extension containing a complex structured type with its own encoding
//!
//! ## Extension Header Format
//!
//! Each extension is prefixed with a header byte with the following layout:
//!
//! - **MORE (bit 7)**: Indicates if more extensions follow
//! - **KIND (bits 5-6)**: The extension kind (Unit=00, U64=01, ZStruct=10)
//! - **MAND (bit 4)**: Marks the extension as mandatory (decoder must understand it)
//! - **EXT ID (bits 0-3)**: Extension identifier (0-15)
//!
//! ## Usage
//!
//! Extensions are typically derived using the `#[derive(ZExt)]` macro:
//!
//! ```ignore
//! #[derive(ZExt, Debug, PartialEq)]
//! pub struct MyExtension {
//!     pub value: u64,
//! }
//! ```
//!
//! The macro automatically infers the extension kind based on the structure:
//! - Empty structs → Unit
//! - Single integer field → U64
//! - Everything else → ZStruct
//!
//! ## Encoding Extensions
//!
//! Extensions are encoded with their ID and flags using [`zext_encode`]:
//!
//! ```ignore
//! zext_encode::<0x1, true>(&ext, &mut writer, false)?;
//! //            ^^^  ^^^^              ^^^^   ^^^^^
//! //            ID   MAND             writer   more
//! ```
//!
//! ## Decoding Extensions
//!
//! During decoding:
//! 1. Read the extension header with [`decode_ext_header`]
//! 2. Check the extension ID and whether it's mandatory
//! 3. Decode known extensions with [`zext_decode`]
//! 4. Skip unknown extensions with [`skip_ext`] (if not mandatory)

use crate::*;

/// Bit mask for extracting the extension kind from the header byte.
const KIND_MASK: u8 = 0b0110_0000;

/// Extension kind identifier encoded in the extension header.
///
/// The kind determines how the extension data is interpreted and encoded.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ZExtKind {
    /// Unit extension - no associated data, acts as a presence flag.
    Unit = 0b00 << 5,
    /// U64 extension - contains a variable-length encoded 64-bit unsigned integer.
    U64 = 0b01 << 5,
    /// ZStruct extension - contains a structured type with its own encoding.
    ZStruct = 0b10 << 5,
}

impl From<ZExtKind> for u8 {
    fn from(kind: ZExtKind) -> Self {
        kind as u8
    }
}

impl TryFrom<u8> for ZExtKind {
    type Error = crate::CodecError;

    fn try_from(value: u8) -> core::result::Result<Self, crate::CodecError> {
        match value & KIND_MASK {
            0b0000_0000 => Ok(ZExtKind::Unit),
            0b0010_0000 => Ok(ZExtKind::U64),
            0b0100_0000 => Ok(ZExtKind::ZStruct),
            _ => Err(crate::CodecError::CouldNotParseHeader),
        }
    }
}

/// Trait for types that can be used as Zenoh protocol extensions.
///
/// This trait combines the encoding/decoding traits ([`ZLen`], [`ZEncode`], [`ZDecode`])
/// with an associated extension kind constant. It is typically derived using the
/// `#[derive(ZExt)]` macro.
///
/// # Examples
///
/// ```ignore
/// #[derive(ZExt, Debug, PartialEq)]
/// pub struct MyExtension {
///     pub timestamp: u64,
/// }
/// ```
pub trait ZExt<'a>: ZLen + ZEncode + ZDecode<'a> {
    /// The kind of this extension (Unit, U64, or ZStruct).
    const KIND: ZExtKind;
}

/// Trait for resolving the extension kind at runtime.
///
/// This trait solves the issue where multiple extensions might share the same ID
/// but have different kinds. It allows retrieving the kind from both the extension
/// type and `Option<T>` wrappers.
pub trait ZExtResolveKind {
    /// Returns the extension kind for this type.
    fn ext_kind(&self) -> ZExtKind;
}

impl<'a, T: ZExt<'a>> ZExtResolveKind for T {
    fn ext_kind(&self) -> ZExtKind {
        T::KIND
    }
}

impl<'a, T: ZExt<'a>> ZExtResolveKind for Option<T> {
    fn ext_kind(&self) -> ZExtKind {
        T::KIND
    }
}

/// Bit flag indicating the extension is mandatory (bit 4 of header).
const FLAG_MANDATORY: u8 = 1 << 4;
/// Bit flag indicating more extensions follow (bit 7 of header).
const FLAG_MORE: u8 = 1 << 7;
/// Bit mask for extracting the extension ID from the header (bits 0-3).
const ID_MASK: u8 = 0b0000_1111;

/// Encodes an extension ID and kind into a single byte.
///
/// Combines the extension ID (lower 4 bits) with the extension kind (bits 5-6).
///
/// # Type Parameters
///
/// - `ID`: The extension identifier (0-15)
/// - `T`: The extension type implementing [`ZExt`]
pub const fn zext_enc_id<'a, const ID: u8, T: ZExt<'a>>() -> u8 {
    ID | T::KIND as u8
}

/// Creates an extension header byte without the MORE flag.
///
/// This combines the extension ID, kind, and mandatory flag into a header byte.
///
/// # Type Parameters
///
/// - `ID`: The extension identifier (0-15)
/// - `MANDATORY`: Whether this extension is mandatory
/// - `T`: The extension type implementing [`ZExt`]
pub const fn zext_eheader<'a, const ID: u8, const MANDATORY: bool, T: ZExt<'a>>() -> u8 {
    zext_enc_id::<ID, T>() | if MANDATORY { FLAG_MANDATORY } else { 0 }
}

/// Creates a complete extension header byte.
///
/// This combines the extension ID, kind, mandatory flag, and more flag.
///
/// # Parameters
///
/// - `more`: Whether more extensions follow this one
///
/// # Type Parameters
///
/// - `ID`: The extension identifier (0-15)
/// - `MANDATORY`: Whether this extension is mandatory
/// - `T`: The extension type implementing [`ZExt`]
pub const fn zext_header<'a, const ID: u8, const MANDATORY: bool, T: ZExt<'a>>(more: bool) -> u8 {
    zext_eheader::<ID, MANDATORY, T>() | if more { FLAG_MORE } else { 0 }
}

/// Calculates the total encoded length of an extension including its header.
///
/// The length includes:
/// - 1 byte for the extension header
/// - For Unit/U64: the body length directly
/// - For ZStruct: the length prefix + the body length
///
/// # Parameters
///
/// - `x`: The extension value to measure
///
/// # Type Parameters
///
/// - `T`: The extension type implementing [`ZExt`]
pub fn zext_len<'a, T: ZExt<'a>>(x: &T) -> usize {
    1 + match T::KIND {
        ZExtKind::Unit | ZExtKind::U64 => x.z_len(),
        ZExtKind::ZStruct => <usize as ZLen>::z_len(&x.z_len()) + x.z_len(),
    }
}

/// Encodes an extension with its header and optional length prefix.
///
/// The encoding consists of:
/// 1. The extension header byte (ID, kind, mandatory flag, more flag)
/// 2. For ZStruct extensions: a variable-length encoded size prefix
/// 3. The extension body data
///
/// # Parameters
///
/// - `x`: The extension value to encode
/// - `w`: The writable buffer to encode into
/// - `more`: Whether more extensions follow this one
///
/// # Type Parameters
///
/// - `T`: The extension type implementing [`ZExt`]
/// - `ID`: The extension identifier (0-15)
/// - `MANDATORY`: Whether this extension is mandatory
///
/// # Errors
///
/// Returns [`CodecError`] if encoding fails (e.g., buffer too small).
pub fn zext_encode<'a, T: ZExt<'a>, const ID: u8, const MANDATORY: bool>(
    x: &T,
    w: &mut impl crate::ZWriteable,
    more: bool,
) -> core::result::Result<(), crate::CodecError> {
    let header: u8 = zext_header::<ID, MANDATORY, T>(more);

    <u8 as ZEncode>::z_encode(&header, w)?;

    if T::KIND == ZExtKind::ZStruct {
        <usize as ZEncode>::z_encode(&x.z_len(), w)?;
    }

    <T as ZEncode>::z_encode(x, w)
}

/// Decodes an extension from a readable buffer.
///
/// This function:
/// 1. Reads and discards the extension header byte (already processed by caller)
/// 2. For ZStruct extensions: reads the length prefix and creates a limited slice
/// 3. Decodes the extension body
///
/// # Parameters
///
/// - `r`: The readable buffer to decode from
///
/// # Type Parameters
///
/// - `T`: The extension type implementing [`ZExt`]
///
/// # Errors
///
/// Returns [`CodecError`] if decoding fails (e.g., invalid data, buffer underflow).
pub fn zext_decode<'a, T: ZExt<'a>>(
    r: &mut impl crate::ZReadable<'a>,
) -> core::result::Result<T, crate::CodecError> {
    let _ = <u8 as ZDecode>::z_decode(r)?;

    if T::KIND == ZExtKind::ZStruct {
        let len = <usize as ZDecode>::z_decode(r)?;
        <T as ZDecode>::z_decode(&mut r.read_slice(len)?)
    } else {
        <T as ZDecode>::z_decode(r)
    }
}

/// Skips over an unknown extension without decoding it.
///
/// This is used when encountering an extension that the decoder doesn't recognize.
/// If the extension is not marked as mandatory, it can be safely skipped.
///
/// The function reads and discards:
/// 1. The extension header byte
/// 2. For U64: 8 bytes (variable-length encoded)
/// 3. For ZStruct: the length prefix + that many bytes
/// 4. For Unit: nothing (no body data)
///
/// # Parameters
///
/// - `r`: The readable buffer to skip data from
/// - `kind`: The kind of extension to skip
///
/// # Errors
///
/// Returns [`CodecError`] if the buffer doesn't contain enough data.
pub fn skip_ext<'a>(
    r: &mut impl crate::ZReadable<'a>,
    kind: ZExtKind,
) -> core::result::Result<(), crate::CodecError> {
    let _ = <u8 as ZDecode>::z_decode(r)?;

    match kind {
        ZExtKind::Unit => {}
        ZExtKind::U64 => {
            let _ = <u64 as ZDecode>::z_decode(r)?;
        }
        ZExtKind::ZStruct => {
            let len = <usize as ZDecode>::z_decode(r)?;
            let _ = r.read_slice(len)?;
        }
    }

    Ok(())
}

/// Decodes and parses an extension header byte without consuming it.
///
/// This function peeks at the next byte in the buffer and extracts:
/// - Extension ID (bits 0-3)
/// - Extension kind (bits 5-6)
/// - Mandatory flag (bit 4)
/// - More flag (bit 7)
///
/// The byte is not consumed from the buffer, allowing the caller to decide
/// how to proceed based on the header information.
///
/// # Parameters
///
/// - `r`: The readable buffer to peek from
///
/// # Returns
///
/// A tuple containing `(id, kind, mandatory, more)`:
/// - `id`: The extension identifier (0-15)
/// - `kind`: The extension kind
/// - `mandatory`: Whether the extension must be understood
/// - `more`: Whether additional extensions follow
///
/// # Errors
///
/// Returns [`CodecError`] if the buffer is empty or the kind bits are invalid.
pub fn decode_ext_header<'a>(
    r: &mut impl crate::ZReadable<'a>,
) -> core::result::Result<(u8, ZExtKind, bool, bool), crate::CodecError> {
    let header = r.peek()?;

    let id = header & ID_MASK;
    let kind = ZExtKind::try_from(header & KIND_MASK)?;
    let mandatory = (header & FLAG_MANDATORY) != 0;
    let more = (header & FLAG_MORE) != 0;

    Ok((id, kind, mandatory, more))
}
