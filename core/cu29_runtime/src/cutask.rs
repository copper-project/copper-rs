//! This module contains all the main definition of the traits you need to implement
//! or interact with to create a Copper task.

use crate::config::ComponentConfig;
use crate::reflect::Reflect;
use bincode::de::{Decode, Decoder};
use bincode::enc::{Encode, Encoder};
use bincode::error::{DecodeError, EncodeError};
use compact_str::{CompactString, ToCompactString};
use core::any::{TypeId, type_name};
use cu29_clock::{PartialCuTimeRange, RobotClock, Tov};
use cu29_traits::{
    COMPACT_STRING_CAPACITY, CuCompactString, CuError, CuMsgMetadataTrait, CuResult,
    ErasedCuStampedData, Metadata,
};
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

use alloc::format;
use core::fmt::{Debug, Display, Formatter, Result as FmtResult};

/// The state of a task.
// Everything that is stateful in copper for zero copy constraints need to be restricted to this trait.
pub trait CuMsgPayload:
    Default + Debug + Clone + Encode + Decode<()> + Serialize + DeserializeOwned + Sized
{
}

pub trait CuMsgPack {}

// Also anything that follows this contract can be a payload (blanket implementation)
impl<T> CuMsgPayload for T where
    T: Default + Debug + Clone + Encode + Decode<()> + Serialize + DeserializeOwned + Sized
{
}

macro_rules! impl_cu_msg_pack {
    ($($name:ident),+) => {
        impl<'cl, $($name),+> CuMsgPack for ($(&CuMsg<$name>,)+)
        where
            $($name: CuMsgPayload),+
        {}
    };
}

impl<T: CuMsgPayload> CuMsgPack for CuMsg<T> {}
impl<T: CuMsgPayload> CuMsgPack for &CuMsg<T> {}
impl<T: CuMsgPayload> CuMsgPack for (&CuMsg<T>,) {}
impl CuMsgPack for () {}

// Apply the macro to generate implementations for tuple sizes up to 5
impl_cu_msg_pack!(T1, T2);
impl_cu_msg_pack!(T1, T2, T3);
impl_cu_msg_pack!(T1, T2, T3, T4);
impl_cu_msg_pack!(T1, T2, T3, T4, T5);

// A convenience macro to get from a payload or a list of payloads to a proper CuMsg or CuMsgPack
// declaration for your tasks used for input messages.
#[macro_export]
macro_rules! input_msg {
    ($lt:lifetime, $first:ty, $($rest:ty),+) => {
        ( & $lt CuMsg<$first>, $( & $lt CuMsg<$rest> ),+ )
    };
    ($lt:lifetime, $ty:ty) => {
        CuMsg<$ty>   // This is for backward compatibility
    };
    ($ty:ty) => {
        CuMsg<$ty>
    };
}

// A convenience macro to get from a payload to a proper CuMsg used as output.
#[macro_export]
macro_rules! output_msg {
    ($lt:lifetime, $first:ty, $($rest:ty),+) => {
        ( CuMsg<$first>, $( CuMsg<$rest> ),+ )
    };
    ($first:ty, $($rest:ty),+) => {
        ( CuMsg<$first>, $( CuMsg<$rest> ),+ )
    };
    ($ty:ty) => {
        CuMsg<$ty>
    };
    ($lt:lifetime, $ty:ty) => {
        CuMsg<$ty>  // This is for backward compatibility
    };
}

/// CuMsgMetadata is a structure that contains metadata common to all CuStampedDataSet.
#[derive(Debug, Clone, bincode::Encode, bincode::Decode, Serialize, Deserialize)]
pub struct CuMsgMetadata {
    /// The time range used for the processing of this message
    pub process_time: PartialCuTimeRange,
    /// A small string for real time feedback purposes.
    /// This is useful for to display on the field when the tasks are operating correctly.
    pub status_txt: CuCompactString,
}

impl Metadata for CuMsgMetadata {}

impl CuMsgMetadata {
    pub fn set_status(&mut self, status: impl ToCompactString) {
        self.status_txt = CuCompactString(status.to_compact_string());
    }
}

impl CuMsgMetadataTrait for CuMsgMetadata {
    fn process_time(&self) -> PartialCuTimeRange {
        self.process_time
    }

    fn status_txt(&self) -> &CuCompactString {
        &self.status_txt
    }
}

impl Display for CuMsgMetadata {
    fn fmt(&self, f: &mut Formatter<'_>) -> FmtResult {
        write!(
            f,
            "process_time start: {}, process_time end: {}",
            self.process_time.start, self.process_time.end
        )
    }
}

/// CuMsg is the envelope holding the msg payload and the metadata between tasks.
#[derive(Default, Debug, Clone, bincode::Encode, bincode::Decode, Serialize, Deserialize)]
#[serde(bound(
    serialize = "T: Serialize, M: Serialize",
    deserialize = "T: DeserializeOwned, M: DeserializeOwned"
))]
pub struct CuStampedData<T, M>
where
    T: CuMsgPayload,
    M: Metadata,
{
    /// This payload is the actual data exchanged between tasks.
    payload: Option<T>,

    /// The time of validity of the message.
    /// It can be undefined (None), one measure point or a range of measures (TimeRange).
    pub tov: Tov,

    /// This metadata is the data that is common to all messages.
    pub metadata: M,
}

impl Default for CuMsgMetadata {
    fn default() -> Self {
        CuMsgMetadata {
            process_time: PartialCuTimeRange::default(),
            status_txt: CuCompactString(CompactString::with_capacity(COMPACT_STRING_CAPACITY)),
        }
    }
}

impl<T, M> CuStampedData<T, M>
where
    T: CuMsgPayload,
    M: Metadata,
{
    pub fn new(payload: Option<T>) -> Self {
        CuStampedData {
            payload,
            tov: Tov::default(),
            metadata: M::default(),
        }
    }
    pub fn payload(&self) -> Option<&T> {
        self.payload.as_ref()
    }

    pub fn set_payload(&mut self, payload: T) {
        self.payload = Some(payload);
    }

    pub fn clear_payload(&mut self) {
        self.payload = None;
    }

    pub fn payload_mut(&mut self) -> &mut Option<T> {
        &mut self.payload
    }
}

impl<T, M> ErasedCuStampedData for CuStampedData<T, M>
where
    T: CuMsgPayload,
    M: CuMsgMetadataTrait + Metadata,
{
    fn payload(&self) -> Option<&dyn erased_serde::Serialize> {
        self.payload
            .as_ref()
            .map(|p| p as &dyn erased_serde::Serialize)
    }

    fn tov(&self) -> Tov {
        self.tov
    }

    fn metadata(&self) -> &dyn CuMsgMetadataTrait {
        &self.metadata
    }
}

/// This is the robotics message type for Copper with the correct Metadata type
/// that will be used by the runtime.
pub type CuMsg<T> = CuStampedData<T, CuMsgMetadata>;

impl<T: CuMsgPayload> CuStampedData<T, CuMsgMetadata> {
    /// Reinterprets the payload type carried by this message.
    ///
    /// # Safety
    ///
    /// The caller must guarantee that the message really contains a payload of type `U`. Failing
    /// to do so is undefined behaviour.
    pub unsafe fn assume_payload<U: CuMsgPayload>(&self) -> &CuMsg<U> {
        // SAFETY: Caller guarantees that the underlying payload is of type U.
        unsafe { &*(self as *const CuMsg<T> as *const CuMsg<U>) }
    }

    /// Mutable variant of [`assume_payload`](Self::assume_payload).
    ///
    /// # Safety
    ///
    /// The caller must guarantee that mutating the returned message is sound for the actual
    /// payload type stored in the buffer.
    pub unsafe fn assume_payload_mut<U: CuMsgPayload>(&mut self) -> &mut CuMsg<U> {
        // SAFETY: Caller guarantees that the underlying payload is of type U.
        unsafe { &mut *(self as *mut CuMsg<T> as *mut CuMsg<U>) }
    }
}

impl<T: CuMsgPayload + 'static> CuStampedData<T, CuMsgMetadata> {
    fn downcast_err<U: CuMsgPayload + 'static>() -> CuError {
        CuError::from(format!(
            "CuMsg payload mismatch: {} cannot be reinterpreted as {}",
            type_name::<T>(),
            type_name::<U>()
        ))
    }

    /// Attempts to view this message as carrying payload `U`.
    pub fn downcast_ref<U: CuMsgPayload + 'static>(&self) -> CuResult<&CuMsg<U>> {
        if TypeId::of::<T>() == TypeId::of::<U>() {
            // SAFETY: We just proved that T == U.
            Ok(unsafe { self.assume_payload::<U>() })
        } else {
            Err(Self::downcast_err::<U>())
        }
    }

    /// Mutable variant of [`downcast_ref`](Self::downcast_ref).
    pub fn downcast_mut<U: CuMsgPayload + 'static>(&mut self) -> CuResult<&mut CuMsg<U>> {
        if TypeId::of::<T>() == TypeId::of::<U>() {
            // SAFETY: We just proved that T == U.
            Ok(unsafe { self.assume_payload_mut::<U>() })
        } else {
            Err(Self::downcast_err::<U>())
        }
    }
}

/// The internal state of a task needs to be serializable
/// so the framework can take a snapshot of the task graph.
pub trait Freezable {
    /// This method is called by the framework when it wants to save the task state.
    /// The default implementation is to encode nothing (stateless).
    /// If you have a state, you need to implement this method.
    fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&(), encoder) // default is stateless
    }

    /// This method is called by the framework when it wants to restore the task to a specific state.
    /// Here it is similar to Decode but the framework will give you a new instance of the task (the new method will be called)
    fn thaw<D: Decoder>(&mut self, _decoder: &mut D) -> Result<(), DecodeError> {
        Ok(())
    }
}

/// Bincode Adapter for Freezable tasks
/// This allows the use of the bincode API directly to freeze and thaw tasks.
pub struct BincodeAdapter<'a, T: Freezable + ?Sized>(pub &'a T);

impl<'a, T: Freezable + ?Sized> Encode for BincodeAdapter<'a, T> {
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        self.0.freeze(encoder)
    }
}

/// A Src Task is a task that only produces messages. For example drivers for sensors are Src Tasks.
/// They are in push mode from the runtime.
/// To set the frequency of the pulls and align them to any hw, see the runtime configuration.
/// Note: A source has the privilege to have a clock passed to it vs a frozen clock.
pub trait CuSrcTask: Freezable + Reflect {
    type Output<'m>: CuMsgPayload;
    /// Resources required by the task.
    type Resources<'r>;

    /// Here you need to initialize everything your task will need for the duration of its lifetime.
    /// The config allows you to access the configuration of the task.
    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized;

    /// Start is called between the creation of the task and the first call to pre/process.
    fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
        Ok(())
    }

    /// This is a method called by the runtime before "process". This is a kind of best effort,
    /// as soon as possible call to give a chance for the task to do some work before to prepare
    /// to make "process" as short as possible.
    fn preprocess(&mut self, _clock: &RobotClock) -> CuResult<()> {
        Ok(())
    }

    /// Process is the most critical execution of the task.
    /// The goal will be to produce the output message as soon as possible.
    /// Use preprocess to prepare the task to make this method as short as possible.
    fn process<'o>(&mut self, clock: &RobotClock, new_msg: &mut Self::Output<'o>) -> CuResult<()>;

    /// This is a method called by the runtime after "process". It is best effort a chance for
    /// the task to update some state after process is out of the way.
    /// It can be use for example to maintain statistics etc. that are not time-critical for the robot.
    fn postprocess(&mut self, _clock: &RobotClock) -> CuResult<()> {
        Ok(())
    }

    /// Called to stop the task. It signals that the *process method won't be called until start is called again.
    fn stop(&mut self, _clock: &RobotClock) -> CuResult<()> {
        Ok(())
    }
}

/// This is the most generic Task of copper. It is a "transform" task deriving an output from an input.
pub trait CuTask: Freezable + Reflect {
    type Input<'m>: CuMsgPack;
    type Output<'m>: CuMsgPayload;
    /// Resources required by the task.
    type Resources<'r>;

    /// Here you need to initialize everything your task will need for the duration of its lifetime.
    /// The config allows you to access the configuration of the task.
    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized;

    /// Start is called between the creation of the task and the first call to pre/process.
    fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
        Ok(())
    }

    /// This is a method called by the runtime before "process". This is a kind of best effort,
    /// as soon as possible call to give a chance for the task to do some work before to prepare
    /// to make "process" as short as possible.
    fn preprocess(&mut self, _clock: &RobotClock) -> CuResult<()> {
        Ok(())
    }

    /// Process is the most critical execution of the task.
    /// The goal will be to produce the output message as soon as possible.
    /// Use preprocess to prepare the task to make this method as short as possible.
    fn process<'i, 'o>(
        &mut self,
        _clock: &RobotClock,
        input: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> CuResult<()>;

    /// This is a method called by the runtime after "process". It is best effort a chance for
    /// the task to update some state after process is out of the way.
    /// It can be use for example to maintain statistics etc. that are not time-critical for the robot.
    fn postprocess(&mut self, _clock: &RobotClock) -> CuResult<()> {
        Ok(())
    }

    /// Called to stop the task. It signals that the *process method won't be called until start is called again.
    fn stop(&mut self, _clock: &RobotClock) -> CuResult<()> {
        Ok(())
    }
}

/// A Sink Task is a task that only consumes messages. For example drivers for actuators are Sink Tasks.
pub trait CuSinkTask: Freezable + Reflect {
    type Input<'m>: CuMsgPack;
    /// Resources required by the task.
    type Resources<'r>;

    /// Here you need to initialize everything your task will need for the duration of its lifetime.
    /// The config allows you to access the configuration of the task.
    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized;

    /// Start is called between the creation of the task and the first call to pre/process.
    fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
        Ok(())
    }

    /// This is a method called by the runtime before "process". This is a kind of best effort,
    /// as soon as possible call to give a chance for the task to do some work before to prepare
    /// to make "process" as short as possible.
    fn preprocess(&mut self, _clock: &RobotClock) -> CuResult<()> {
        Ok(())
    }

    /// Process is the most critical execution of the task.
    /// The goal will be to produce the output message as soon as possible.
    /// Use preprocess to prepare the task to make this method as short as possible.
    fn process<'i>(&mut self, _clock: &RobotClock, input: &Self::Input<'i>) -> CuResult<()>;

    /// This is a method called by the runtime after "process". It is best effort a chance for
    /// the task to update some state after process is out of the way.
    /// It can be use for example to maintain statistics etc. that are not time-critical for the robot.
    fn postprocess(&mut self, _clock: &RobotClock) -> CuResult<()> {
        Ok(())
    }

    /// Called to stop the task. It signals that the *process method won't be called until start is called again.
    fn stop(&mut self, _clock: &RobotClock) -> CuResult<()> {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bincode::{config, decode_from_slice, encode_to_vec};

    #[test]
    fn test_cucompactstr_encode_decode() {
        let cstr = CuCompactString(CompactString::from("hello"));
        let config = config::standard();
        let encoded = encode_to_vec(&cstr, config).expect("Encoding failed");
        let (decoded, _): (CuCompactString, usize) =
            decode_from_slice(&encoded, config).expect("Decoding failed");
        assert_eq!(cstr.0, decoded.0);
    }
}
