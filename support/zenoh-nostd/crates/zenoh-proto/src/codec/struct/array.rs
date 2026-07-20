//! # Array Encoding Implementation
//!
//! This module provides codec implementations for fixed-size arrays of bytes.
//!
//! ## Encoding Strategy
//!
//! Fixed-size byte arrays are encoded differently from slices:
//!
//! - **No length prefix**: The size is known at compile time
//! - **Raw bytes**: Direct copy of the array data
//!
//! This is more efficient than encoding slices when the size is fixed and known.

use crate::*;

impl<const N: usize> ZBodyLen for [u8; N] {
    fn z_body_len(&self) -> usize {
        N
    }
}

impl<const N: usize> ZBodyEncode for [u8; N] {
    fn z_body_encode(
        &self,
        w: &mut impl crate::ZWriteable,
    ) -> core::result::Result<(), crate::CodecError> {
        Ok(w.write_exact(self.as_slice())?)
    }
}

impl<'a, const N: usize> ZBodyDecode<'a> for [u8; N] {
    type Ctx = ();

    fn z_body_decode(
        r: &mut impl crate::ZReadable<'a>,
        _: (),
    ) -> core::result::Result<Self, crate::CodecError> {
        let mut dst = [0u8; N];
        r.read_exact(dst.as_mut_slice())?;
        Ok(dst)
    }
}

impl<const N: usize> ZLen for [u8; N] {
    fn z_len(&self) -> usize {
        <Self as ZBodyLen>::z_body_len(self)
    }
}

impl<const N: usize> ZEncode for [u8; N] {
    fn z_encode(
        &self,
        w: &mut impl crate::ZWriteable,
    ) -> core::result::Result<(), crate::CodecError> {
        <Self as ZBodyEncode>::z_body_encode(self, w)
    }
}

impl<'a, const N: usize> ZDecode<'a> for [u8; N] {
    fn z_decode(
        r: &mut impl crate::ZReadable<'a>,
    ) -> core::result::Result<Self, crate::CodecError> {
        <Self as ZBodyDecode>::z_body_decode(r, ())
    }
}
