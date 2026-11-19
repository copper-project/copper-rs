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

macro_rules! word_codec {
    ($word:ty, $encode:ident, $decode:ident) => {
        #[inline]
        fn $encode<E: Encoder>(mut value: $word, encoder: &mut E) -> Result<(), EncodeError> {
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
        fn $decode<D: Decoder>(bits: u32, decoder: &mut D) -> Result<$word, DecodeError> {
            let mut value = 0 as $word;
            for shift in (0..bits).step_by(7) {
                let mut byte = [0];
                decoder.reader().read(&mut byte)?;
                let chunk = <$word>::from(byte[0] & 0x7f);
                let remaining = bits - shift;
                if remaining < 7 && chunk >= ((1 as $word) << remaining) {
                    return Err(DecodeError::Other("ULEB128 integer overflow"));
                }
                value |= chunk << shift;
                if byte[0] & 0x80 == 0 {
                    return Ok(value);
                }
            }
            Err(DecodeError::Other("ULEB128 integer is too long"))
        }
    };
}

word_codec!(u32, encode32, decode32);
word_codec!(u64, encode64, decode64);
word_codec!(u128, encode128, decode128);

macro_rules! unsigned {
    ($enc:ident, $dec:ident; $($ty:ty),* $(,)?) => {$ (
        impl Encode for Uleb128<$ty> {
            #[inline]
            fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
                $enc(self.0 as _, encoder)
            }
        }

        impl<Context> Decode<Context> for Uleb128<$ty> {
            #[inline]
            fn decode<D: Decoder<Context = Context>>(decoder: &mut D) -> Result<Self, DecodeError> {
                decoder.claim_bytes_read(core::mem::size_of::<$ty>())?;
                $dec(<$ty>::BITS, decoder).map(|value| Self(value as $ty))
            }
        }
        crate::impl_borrow_decode!(Uleb128<$ty>);
    )*};
}

macro_rules! signed {
    ($enc:ident, $dec:ident; $($ty:ty => $unsigned:ty),* $(,)?) => {$ (
        impl Encode for Uleb128<$ty> {
            #[inline]
            fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
                let value = self.0;
                let zigzag = ((value as $unsigned) << 1) ^ ((value >> (<$ty>::BITS - 1)) as $unsigned);
                $enc(zigzag as _, encoder)
            }
        }

        impl<Context> Decode<Context> for Uleb128<$ty> {
            #[inline]
            fn decode<D: Decoder<Context = Context>>(decoder: &mut D) -> Result<Self, DecodeError> {
                decoder.claim_bytes_read(core::mem::size_of::<$ty>())?;
                let value = $dec(<$ty>::BITS, decoder)? as $unsigned;
                Ok(Self(((value >> 1) as $ty) ^ -((value & 1) as $ty)))
            }
        }
        crate::impl_borrow_decode!(Uleb128<$ty>);
    )*};
}

unsigned!(encode32, decode32; u8, u16, u32);
unsigned!(encode64, decode64; u64);
unsigned!(encode128, decode128; u128);
signed!(encode32, decode32; i8 => u8, i16 => u16, i32 => u32);
signed!(encode64, decode64; i64 => u64);
signed!(encode128, decode128; i128 => u128);
#[cfg(target_pointer_width = "64")]
unsigned!(encode64, decode64; usize);
#[cfg(not(target_pointer_width = "64"))]
unsigned!(encode32, decode32; usize);
#[cfg(target_pointer_width = "64")]
signed!(encode64, decode64; isize => usize);
#[cfg(not(target_pointer_width = "64"))]
signed!(encode32, decode32; isize => usize);
