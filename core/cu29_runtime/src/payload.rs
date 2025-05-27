/// This module is a collection of Copper friendly data structures for message payloads.
///
/// The constraint on the messages is that they can be part of a copper list, fixed sized and bincode serializable.
use arrayvec::ArrayVec;
use bincode::de::{BorrowDecoder, Decoder};
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::BorrowDecode;
use bincode::{Decode, Encode};

/// Copper friendly wrapper for a fixed size array.
#[derive(Clone, Debug, Default)]
pub struct CuArray<T, const N: usize> {
    inner: ArrayVec<T, N>,
}

impl<T, const N: usize> CuArray<T, N> {
    pub fn new() -> Self {
        Self {
            inner: ArrayVec::new(),
        }
    }

    pub fn fill_from_iter<I>(&mut self, iter: I)
    where
        I: IntoIterator<Item = T>,
    {
        self.inner.clear(); // Clear existing data
        for value in iter.into_iter().take(N) {
            self.inner.push(value);
        }
    }

    pub fn len(&self) -> usize {
        self.inner.len()
    }

    pub fn is_empty(&self) -> bool {
        self.inner.len() == 0
    }

    pub fn as_slice(&self) -> &[T] {
        &self.inner
    }

    pub fn capacity(&self) -> usize {
        N
    }
}

impl<T, const N: usize> Encode for CuArray<T, N>
where
    T: Encode,
{
    fn encode<E: bincode::enc::Encoder>(
        &self,
        encoder: &mut E,
    ) -> Result<(), bincode::error::EncodeError> {
        // Encode the length first
        (self.inner.len() as u32).encode(encoder)?;

        // Encode elements in the `ArrayVec`
        for elem in &self.inner {
            elem.encode(encoder)?;
        }

        Ok(())
    }
}

impl<T, const N: usize> Decode<()> for CuArray<T, N>
where
    T: Decode<()>,
{
    fn decode<D: bincode::de::Decoder<Context = ()>>(
        decoder: &mut D,
    ) -> Result<Self, bincode::error::DecodeError> {
        // Decode the length first
        let len = u32::decode(decoder)? as usize;
        if len > N {
            return Err(bincode::error::DecodeError::OtherString(format!(
                "Decoded length {len} exceeds maximum capacity {N}"
            )));
        }

        // Decode elements into a new `ArrayVec`
        let mut inner = ArrayVec::new();
        for _ in 0..len {
            inner.push(T::decode(decoder)?);
        }

        Ok(Self { inner })
    }
}

/// A Copper-friendly wrapper around ArrayVec with bincode serialization support.
///
/// This provides a fixed-capacity, stack-allocated vector that can be efficiently
/// serialized and deserialized. It is particularly useful for message payloads that
/// need to avoid heap allocations while supporting varying lengths of data up to a maximum.
///
/// Unlike standard Vec, CuArrayVec will never reallocate or use the heap for the elements storage.
#[derive(Debug, Clone)]
pub struct CuArrayVec<T, const N: usize>(pub ArrayVec<T, N>);

impl<T, const N: usize> Default for CuArrayVec<T, N> {
    fn default() -> Self {
        Self(ArrayVec::new())
    }
}

impl<T, const N: usize> Encode for CuArrayVec<T, N>
where
    T: Encode + 'static,
{
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        let CuArrayVec(inner) = self;
        inner.as_slice().encode(encoder)
    }
}

impl<T, const N: usize> Decode<()> for CuArrayVec<T, N>
where
    T: Decode<()> + 'static,
{
    fn decode<D: Decoder<Context = ()>>(decoder: &mut D) -> Result<Self, DecodeError> {
        let inner = Vec::<T>::decode(decoder)?;
        let actual_len = inner.len();
        if actual_len > N {
            return Err(DecodeError::ArrayLengthMismatch {
                required: N,
                found: actual_len,
            });
        }

        let mut array_vec = ArrayVec::new();
        for item in inner {
            array_vec.push(item); // Push elements one by one
        }
        Ok(CuArrayVec(array_vec))
    }
}

impl<'de, T, const N: usize> BorrowDecode<'de, ()> for CuArrayVec<T, N>
where
    T: BorrowDecode<'de, ()> + 'static,
{
    fn borrow_decode<D: BorrowDecoder<'de, Context = ()>>(
        decoder: &mut D,
    ) -> Result<Self, DecodeError> {
        let inner = Vec::<T>::borrow_decode(decoder)?;
        let actual_len = inner.len();
        if actual_len > N {
            return Err(DecodeError::ArrayLengthMismatch {
                required: N,
                found: actual_len,
            });
        }

        let mut array_vec = ArrayVec::new();
        for item in inner {
            array_vec.push(item); // Push elements one by one
        }
        Ok(CuArrayVec(array_vec))
    }
}
