//! Copper-friendly payload helpers used in task messages and task-local caches.
//!
//! Payload types need to stay compatible with Copper's preallocated message buffers,
//! deterministic logging, and `no_std` targets. This module therefore focuses on
//! fixed-capacity containers and explicit state-transition payloads such as
//! [`CuLatchedStateUpdate`] and [`CuLatchedState`].
//!
//! A latched state is useful when a value changes rarely, but consumers still need a
//! deterministic view of it during live execution, logging, and replay. Producers send
//! updates with [`CuLatchedStateUpdate`], and each consumer keeps its own local cache in
//! [`CuLatchedState`].
use crate::reflect::Reflect;
use arrayvec::ArrayVec;
#[cfg(feature = "reflect")]
use bevy_reflect;
use bincode::BorrowDecode;
use bincode::de::{BorrowDecoder, Decoder};
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{Decode, Encode};
use serde_derive::{Deserialize, Serialize};

#[cfg(not(feature = "std"))]
pub use alloc::format;
#[cfg(not(feature = "std"))]
pub use alloc::vec::Vec;

/// Copper friendly wrapper for a fixed size array.
/// `T: Clone` is required because this type derives `Reflect`, and
/// the reflection path requires the reflected value to be cloneable.
#[derive(Clone, Debug, Default, Serialize, Deserialize, Reflect)]
#[reflect(opaque, from_reflect = false, no_field_bounds)]
pub struct CuArray<T: Clone, const N: usize> {
    inner: ArrayVec<T, N>,
}

impl<T: Clone, const N: usize> CuArray<T, N> {
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
    T: Encode + Clone,
{
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
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
    T: Decode<()> + Clone,
{
    fn decode<D: Decoder<Context = ()>>(decoder: &mut D) -> Result<Self, DecodeError> {
        // Decode the length first
        let len = u32::decode(decoder)? as usize;
        if len > N {
            return Err(DecodeError::OtherString(format!(
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

/// Producer-side update for a stateful value cached by downstream consumers.
///
/// Use this when a value is logically "sticky" across cycles, but you still want its
/// evolution to be explicit in the message stream. A typical producer pattern is:
///
/// - emit [`Self::Set`] when the value first becomes available or changes
/// - emit [`Self::NoChange`] on cycles where the previously latched value remains valid
/// - emit [`Self::Clear`] when the cached value must be invalidated
///
/// Each consumer that cares about the value should keep a local [`CuLatchedState`] and
/// apply incoming updates to it. Copper does not implicitly retain or replay the previous
/// payload for you; the state transition is part of the payload itself.
///
/// `NoChange` is intentionally the first variant so its bincode discriminant is zero.
///
/// # Examples
///
/// ```
/// use cu29_runtime::payload::{CuLatchedState, CuLatchedStateUpdate};
///
/// let mut calibration = CuLatchedState::default();
///
/// calibration.update(&CuLatchedStateUpdate::Set(42u32));
/// assert_eq!(calibration.get(), Some(&42));
///
/// calibration.update(&CuLatchedStateUpdate::NoChange);
/// assert_eq!(calibration.get(), Some(&42));
///
/// calibration.update_owned(CuLatchedStateUpdate::Clear);
/// assert!(calibration.is_unset());
/// ```
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize, Encode, Decode, Reflect)]
#[reflect(opaque, from_reflect = false, no_field_bounds)]
pub enum CuLatchedStateUpdate<T: Clone> {
    /// Leave the consumer-side cache unchanged for this cycle.
    #[default]
    NoChange,
    /// Replace the consumer-side cache with a new value.
    Set(T),
    /// Remove the consumer-side cached value.
    Clear,
}

impl<T: Clone> CuLatchedStateUpdate<T> {
    /// Returns `true` when this update leaves the cached value unchanged.
    pub fn is_no_change(&self) -> bool {
        matches!(self, Self::NoChange)
    }

    /// Returns `true` when this update carries a replacement value.
    pub fn is_set(&self) -> bool {
        matches!(self, Self::Set(_))
    }

    /// Returns `true` when this update clears the cached value.
    pub fn is_clear(&self) -> bool {
        matches!(self, Self::Clear)
    }
}

impl<T: Clone> From<T> for CuLatchedStateUpdate<T> {
    fn from(value: T) -> Self {
        Self::Set(value)
    }
}

/// Consumer-side cache updated by [`CuLatchedStateUpdate`].
///
/// This is typically stored in a task struct and updated as messages arrive. It is not a
/// runtime-managed global store; each consumer keeps its own copy of the latched state so
/// replay and deterministic execution follow the same update sequence as live execution.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize, Encode, Decode, Reflect)]
#[reflect(opaque, from_reflect = false, no_field_bounds)]
pub enum CuLatchedState<T: Clone> {
    /// No value has been latched yet, or the previous value was cleared.
    #[default]
    Unset,
    /// The most recently latched value.
    Set(T),
}

impl<T: Clone> CuLatchedState<T> {
    /// Creates an empty latched state.
    pub fn new() -> Self {
        Self::Unset
    }

    /// Returns `true` when a value is currently latched.
    pub fn is_set(&self) -> bool {
        matches!(self, Self::Set(_))
    }

    /// Returns `true` when no value is currently latched.
    pub fn is_unset(&self) -> bool {
        matches!(self, Self::Unset)
    }

    /// Returns the currently latched value, if any.
    pub fn get(&self) -> Option<&T> {
        match self {
            Self::Unset => None,
            Self::Set(value) => Some(value),
        }
    }

    /// Returns the currently latched value as a shared reference, if any.
    ///
    /// This is equivalent to [`Self::get`].
    pub fn as_ref(&self) -> Option<&T> {
        self.get()
    }

    /// Replaces the currently latched value.
    pub fn set(&mut self, value: T) {
        *self = Self::Set(value);
    }

    /// Clears the currently latched value.
    pub fn clear(&mut self) {
        *self = Self::Unset;
    }

    /// Removes and returns the currently latched value, leaving the state unset.
    pub fn take(&mut self) -> Option<T> {
        let previous = core::mem::take(self);
        match previous {
            Self::Unset => None,
            Self::Set(value) => Some(value),
        }
    }

    /// Applies an owned update without cloning the payload value.
    pub fn update_owned(&mut self, update: CuLatchedStateUpdate<T>) {
        match update {
            CuLatchedStateUpdate::NoChange => {}
            CuLatchedStateUpdate::Set(value) => self.set(value),
            CuLatchedStateUpdate::Clear => self.clear(),
        }
    }
}

impl<T: Clone> CuLatchedState<T> {
    /// Applies a borrowed update, cloning only when the update contains a new value.
    pub fn update(&mut self, update: &CuLatchedStateUpdate<T>) {
        match update {
            CuLatchedStateUpdate::NoChange => {}
            CuLatchedStateUpdate::Set(value) => self.set(value.clone()),
            CuLatchedStateUpdate::Clear => self.clear(),
        }
    }
}
