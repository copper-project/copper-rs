//! # Structured Type Codec
//!
//! This module provides traits and implementations for encoding and decoding
//! structured types in the Zenoh protocol. It defines the core codec traits
//! that are used throughout the protocol implementation.
//!
//! ## Overview
//!
//! The struct codec system separates concerns into several layers:
//!
//! - **Body traits** ([`ZBodyLen`], [`ZBodyEncode`], [`ZBodyDecode`]): Handle the body data only
//! - **Full traits** ([`ZLen`], [`ZEncode`], [`ZDecode`]): Handle complete encoding including headers
//! - **Header trait** ([`ZHeader`]): For types with a header byte
//! - **Extension trait** ([`ZExtCount`]): For counting protocol extensions
//!
//! ## Encoding Strategy
//!
//! Types can be encoded in two ways:
//!
//! 1. **Header + Body**: Types with a header byte use [`ZHeader`] to compute the header,
//!    then encode the header followed by the body.
//! 2. **Body Only**: Types without headers directly encode their body data.
//!
//! ## Context in Decoding
//!
//! The [`ZBodyDecode`] trait supports a context type (`Ctx`) that can be passed
//! during decoding. This allows decoders to receive additional information needed
//! to properly decode the body (e.g., a header byte that was already read).
//!
//! ## Usage with Derive Macros
//!
//! The `#[derive(ZStruct)]` macro automatically implements these traits:
//!
//! ```ignore
//! #[derive(ZStruct, Debug, PartialEq)]
//! #[zenoh(header = "_:3|ID:5=0x01")]
//! pub struct MyMessage {
//!     pub field1: u32,
//!     pub field2: u64,
//! }
//! ```

mod array;
mod bytes;
mod str;
mod uint;

/// Calculates the encoded length of a type's body data (excluding header).
///
/// This trait is implemented for types that have a body that can be encoded.
///
/// # Examples
///
/// ```ignore
/// let value = 42u64;
/// let len = value.z_body_len(); // Returns the variable-length encoded size
/// ```
pub trait ZBodyLen {
    /// Returns the encoded length of this type's body in bytes.
    fn z_body_len(&self) -> usize;
}

/// Encodes a type's body data (excluding header) into a writable buffer.
///
/// This trait handles the core encoding logic for a type, writing its data
/// to the provided buffer in the Zenoh protocol format.
///
/// # Examples
///
/// ```ignore
/// let value = 42u64;
/// let mut buffer = [0u8; 16];
/// value.z_body_encode(&mut &mut buffer[..]).unwrap();
/// ```
pub trait ZBodyEncode {
    /// Encodes this type's body into the writable buffer.
    ///
    /// # Errors
    ///
    /// Returns [`CodecError`](crate::CodecError) if encoding fails (e.g., buffer too small).
    fn z_body_encode(
        &self,
        w: &mut impl crate::ZWriteable,
    ) -> core::result::Result<(), crate::CodecError>;
}

/// Decodes a type's body data (excluding header) from a readable buffer.
///
/// This trait handles the core decoding logic for a type, reading its data
/// from the provided buffer.
///
/// The associated `Ctx` type allows passing contextual information (such as
/// a previously-read header byte) to guide the decoding process.
///
/// # Type Parameters
///
/// - `'a`: Lifetime of the borrowed data (for zero-copy decoding)
///
/// # Examples
///
/// ```ignore
/// let buffer = &[42u8, 0, 0, 0][..];
/// let value = u64::z_body_decode(&mut buffer, ()).unwrap();
/// ```
pub trait ZBodyDecode<'a>: Sized {
    /// Context type passed to the decoder (often `()` or `u8` for header).
    type Ctx;

    /// Decodes this type's body from the readable buffer.
    ///
    /// # Parameters
    ///
    /// - `r`: The readable buffer to decode from
    /// - `ctx`: Contextual information for decoding
    ///
    /// # Errors
    ///
    /// Returns [`CodecError`](crate::CodecError) if decoding fails (e.g., invalid data, buffer underflow).
    fn z_body_decode(
        r: &mut impl crate::ZReadable<'a>,
        ctx: Self::Ctx,
    ) -> core::result::Result<Self, crate::CodecError>;
}

/// Computes a header byte for types that have one.
///
/// Many protocol messages include a header byte that encodes metadata about
/// the message (e.g., message type, flags, field presence indicators).
///
/// This trait is typically implemented by the `#[derive(ZStruct)]` macro
/// when a `#[zenoh(header = "...")]` attribute is specified.
pub trait ZHeader {
    /// Returns the header byte for this value.
    fn z_header(&self) -> u8;
}

/// Calculates the total encoded length including header (if any).
///
/// This trait extends [`ZBodyLen`] and represents the complete encoded size
/// of a type, including any header bytes.
///
/// For types without headers, this is typically the same as [`ZBodyLen::z_body_len()`].
/// For types with headers, this adds the header size (usually 1 byte).
pub trait ZLen: ZBodyLen {
    /// Returns the total encoded length in bytes.
    fn z_len(&self) -> usize;
}

/// Encodes a complete type including header (if any).
///
/// This trait extends [`ZBodyEncode`] and handles encoding the complete
/// representation of a type, typically by first encoding a header (if present)
/// and then the body.
pub trait ZEncode: ZBodyEncode {
    /// Encodes this type completely into the writable buffer.
    ///
    /// # Errors
    ///
    /// Returns [`CodecError`](crate::CodecError) if encoding fails (e.g., buffer too small).
    fn z_encode(
        &self,
        w: &mut impl crate::ZWriteable,
    ) -> core::result::Result<(), crate::CodecError>;
}

/// Decodes a complete type including header (if any).
///
/// This trait extends [`ZBodyDecode`] and handles decoding the complete
/// representation of a type, typically by first reading a header (if present)
/// and then the body.
///
/// # Type Parameters
///
/// - `'a`: Lifetime of the borrowed data (for zero-copy decoding)
pub trait ZDecode<'a>: Sized + ZBodyDecode<'a> {
    /// Decodes this type completely from the readable buffer.
    ///
    /// # Errors
    ///
    /// Returns [`CodecError`](crate::CodecError) if decoding fails (e.g., invalid data, buffer underflow).
    fn z_decode(r: &mut impl crate::ZReadable<'a>)
    -> core::result::Result<Self, crate::CodecError>;
}

/// Counts the number of protocol extensions in a message.
///
/// This trait is used by messages that can contain variable numbers of
/// extensions. The count is used during encoding to properly set the
/// "more" flag in extension headers.
pub trait ZExtCount {
    /// Returns the number of extensions in this message.
    fn z_ext_count(&self) -> usize;
}

macro_rules! derive_zstruct_with_body {
    (lt, $($ty:ty),*) => {
        $(
            impl<'a> $crate::ZLen for $ty {
                fn z_len(&self) -> usize {
                    <Self as $crate::ZBodyLen>::z_body_len(self)
                }
            }

            impl<'a> $crate::ZEncode for $ty {
                fn z_encode(&self, w: &mut impl $crate::ZWriteable) -> core::result::Result<(), crate::CodecError> {
                    <Self as $crate::ZBodyEncode>::z_body_encode(self, w)
                }
            }

            impl<'a> $crate::ZDecode<'a> for $ty {
                fn z_decode(r: &mut impl $crate::ZReadable<'a>) -> core::result::Result<Self, crate::CodecError> {
                    <Self as $crate::ZBodyDecode>::z_body_decode(r, ())
                }
            }
        )*
    };

    ($($ty:ty),*) => {
        $(
            impl $crate::ZLen for $ty {
                fn z_len(&self) -> usize {
                    <Self as $crate::ZBodyLen>::z_body_len(self)
                }
            }

            impl $crate::ZEncode for $ty {
                fn z_encode(&self, w: &mut impl $crate::ZWriteable) -> core::result::Result<(), crate::CodecError> {
                    <Self as $crate::ZBodyEncode>::z_body_encode(self, w)
                }
            }

            impl<'a> $crate::ZDecode<'a> for $ty {
                fn z_decode(r: &mut impl $crate::ZReadable<'a>) -> core::result::Result<Self, crate::CodecError> {
                    <Self as $crate::ZBodyDecode>::z_body_decode(r, ())
                }
            }
        )*
    };
}

pub(crate) use derive_zstruct_with_body;
