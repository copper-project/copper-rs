/// This module is a collection of Copper friendly data structures for message payloads.
///
/// The constraint on the messages is that they can be part of a copper list, fixed sized and bincode serializable.
use arrayvec::ArrayVec;
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

impl<T, const N: usize> Decode for CuArray<T, N>
where
    T: Decode,
{
    fn decode<D: bincode::de::Decoder>(
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
