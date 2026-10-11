//! Selective ULEB128 integer encoding, independent of the enclosing configuration.

use crate::de::{Decoder, read::Reader};
use crate::enc::{Encoder, write::Writer};
use crate::error::{DecodeError, EncodeError};
use crate::{Decode, Encode};

/// Encodes just this integer as unsigned LEB128, using zigzag for signed values.
///
/// Each byte carries seven low-order bits and a continuation bit. This wrapper
/// ignores the configuration's integer encoding and endianness; ordinary values
/// alongside it continue to use the enclosing configuration. Decode with the same
/// wrapper and integer type. All Rust signed and unsigned integer types are supported.
///
/// ```
/// use cu_bincode::{config, decode_from_slice, encode_into_slice, Uleb128};
/// let mut bytes = [0; 16];
/// let len = encode_into_slice((300u32, Uleb128(-150i128)), &mut bytes, config::standard()).unwrap();
/// assert_eq!(&bytes[..len], &[251, 44, 1, 171, 2]);
/// let (decoded, used) = decode_from_slice::<(u32, Uleb128<i128>), _>(&bytes[..len], config::standard()).unwrap();
/// assert_eq!(decoded, (300, Uleb128(-150)));
/// assert_eq!(used, len);
/// ```
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(transparent)]
pub struct Uleb128<T>(pub T);

#[inline]
fn encode<E: Encoder>(mut value: u128, encoder: &mut E) -> Result<(), EncodeError> {
    loop {
        let mut byte = (value & 0x7f) as u8;
        value >>= 7;
        if value != 0 {
            byte |= 0x80;
        }
        encoder.writer().write(&[byte])?;
        if value == 0 {
            return Ok(());
        }
    }
}

#[inline]
fn decode<D: Decoder>(bits: u32, decoder: &mut D) -> Result<u128, DecodeError> {
    let mut value = 0u128;
    for shift in (0..bits).step_by(7) {
        let mut byte = [0];
        decoder.reader().read(&mut byte)?;
        let chunk = u128::from(byte[0] & 0x7f);
        let remaining = bits - shift;
        if remaining < 7 && chunk >= (1u128 << remaining) {
            return Err(DecodeError::Other("ULEB128 integer overflow"));
        }
        value |= chunk << shift;
        if byte[0] & 0x80 == 0 {
            return Ok(value);
        }
    }
    Err(DecodeError::Other("ULEB128 integer is too long"))
}

macro_rules! unsigned {
    ($($ty:ty),* $(,)?) => {$ (
        impl Encode for Uleb128<$ty> {
            #[inline]
            fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
                encode(self.0 as u128, encoder)
            }
        }

        impl<Context> Decode<Context> for Uleb128<$ty> {
            #[inline]
            fn decode<D: Decoder<Context = Context>>(decoder: &mut D) -> Result<Self, DecodeError> {
                decoder.claim_bytes_read(core::mem::size_of::<$ty>())?;
                decode(<$ty>::BITS, decoder).map(|value| Self(value as $ty))
            }
        }
        crate::impl_borrow_decode!(Uleb128<$ty>);
    )*};
}

macro_rules! signed {
    ($($ty:ty => $unsigned:ty),* $(,)?) => {$ (
        impl Encode for Uleb128<$ty> {
            #[inline]
            fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
                let value = self.0;
                let zigzag = ((value as $unsigned) << 1) ^ ((value >> (<$ty>::BITS - 1)) as $unsigned);
                encode(zigzag as u128, encoder)
            }
        }

        impl<Context> Decode<Context> for Uleb128<$ty> {
            #[inline]
            fn decode<D: Decoder<Context = Context>>(decoder: &mut D) -> Result<Self, DecodeError> {
                decoder.claim_bytes_read(core::mem::size_of::<$ty>())?;
                let value = decode(<$ty>::BITS, decoder)? as $unsigned;
                Ok(Self(((value >> 1) as $ty) ^ -((value & 1) as $ty)))
            }
        }
        crate::impl_borrow_decode!(Uleb128<$ty>);
    )*};
}

unsigned!(u8, u16, u32, u64, u128, usize);
signed!(i8 => u8, i16 => u16, i32 => u32, i64 => u64, i128 => u128, isize => usize);
