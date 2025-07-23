//! This module contains all the main definition of the traits you need to implement
//! or interact with to create a Copper task.

use crate::config::ComponentConfig;
use bincode::de::{Decode, Decoder};
use bincode::enc::{Encode, Encoder};
use bincode::error::{DecodeError, EncodeError};
use compact_str::{CompactString, ToCompactString};
use cu29_clock::{PartialCuTimeRange, RobotClock, Tov};
use cu29_traits::{
    CuCompactString, CuMsgMetadataTrait, CuResult, ErasedCuStampedData, Metadata,
    COMPACT_STRING_CAPACITY,
};
use serde::{Deserialize, Serialize};
use std::fmt;
use std::fmt::{Debug, Display, Formatter};

// Everything that is stateful in copper for zero copy constraints need to be restricted to this trait.
pub trait CuMsgPayload: Default + Debug + Clone + Encode + Decode<()> + Serialize + Sized {}

pub trait CuMsgPack<'cl> {}

// Also anything that follows this contract can be a payload (blanket implementation)
impl<T: Default + Debug + Clone + Encode + Decode<()> + Serialize + Sized> CuMsgPayload for T {}

macro_rules! impl_cu_msg_pack {
    ($( ( $( ($ty:ident, $md:ident) ),* ) ),*) => {
        $(
            impl<'cl, $( $ty: CuMsgPayload + 'cl, $md: cu29_traits::Metadata ),*> CuMsgPack<'cl> for (
                $( &'cl CuStampedData<$ty, $md>, )*
            ) {}
        )*
    };
}

impl<'cl, T: CuMsgPayload, M: Metadata> CuMsgPack<'cl> for (&'cl CuStampedData<T, M>,) {}
impl<'cl, T: CuMsgPayload, M: Metadata> CuMsgPack<'cl> for &'cl CuStampedData<T, M> {}
impl<'cl, T: CuMsgPayload, M: Metadata> CuMsgPack<'cl> for (&'cl mut CuStampedData<T, M>,) {}
impl<'cl, T: CuMsgPayload, M: Metadata> CuMsgPack<'cl> for &'cl mut CuStampedData<T, M> {}
impl CuMsgPack<'_> for () {}

// Apply the macro to generate implementations for tuple sizes up to 5
impl_cu_msg_pack! {
    ((T1, M1), (T2, M2)),
    ((T1, M1), (T2, M2), (T3, M3)),
    ((T1, M1), (T2, M2), (T3, M3), (T4, M4)),
    ((T1, M1), (T2, M2), (T3, M3), (T4, M4), (T5, M5))
}

// A convenience macro to get from a payload or a list of payloads to a proper CuMsg or CuMsgPack
// declaration for your tasks used for input messages.
#[macro_export]
macro_rules! input_msg {
    ($lifetime:lifetime, $ty:ty) => {
        &$lifetime CuMsg<$ty>
    };
    ($lifetime:lifetime, $($ty:ty),*) => {
        (
            $( &$lifetime CuMsg<$ty>, )*
        )
    };
}

// A convenience macro to get from a payload to a proper CuMsg used as output.
#[macro_export]
macro_rules! output_msg {
    ($lifetime:lifetime, $ty:ty) => {
        &$lifetime mut CuMsg<$ty>
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
    /// The ID of the task that generated this message (index into TASKS_IDS)
    pub task_id: u16,
    /// The name of the task that generated this message
    pub task_name: CuCompactString,
}

impl cu29_traits::Metadata for CuMsgMetadata {}

impl CuMsgMetadata {
    pub fn set_status(&mut self, status: impl ToCompactString) {
        self.status_txt = CuCompactString(status.to_compact_string());
    }
}

impl cu29_traits::CuMsgMetadataTrait for CuMsgMetadata {
    fn process_time(&self) -> PartialCuTimeRange {
        self.process_time
    }

    fn status_txt(&self) -> &CuCompactString {
        &self.status_txt
    }

    fn task_id(&self) -> u16 {
        self.task_id
    }

    fn task_name(&self) -> &CuCompactString {
        &self.task_name
    }
}

impl Display for CuMsgMetadata {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "process_time start: {}, process_time end: {}",
            self.process_time.start, self.process_time.end
        )
    }
}

/// CuMsg is the envelope holding the msg payload and the metadata between tasks.
#[derive(Default, Debug, Clone, bincode::Encode, bincode::Decode, Serialize)]
pub struct CuStampedData<T, M>
where
    T: CuMsgPayload,
    M: cu29_traits::Metadata,
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
            task_id: 0,
            task_name: CuCompactString(CompactString::with_capacity(COMPACT_STRING_CAPACITY)),
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

    fn clear_payload(&mut self) {
        self.payload = None;
    }
}

/// This is the robotics message type for Copper with the correct Metadata type
/// that will be used by the runtime.
pub type CuMsg<T> = CuStampedData<T, CuMsgMetadata>;

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
pub trait CuSrcTask<'cl>: Freezable {
    type Output: CuMsgPack<'cl>;

    /// Here you need to initialize everything your task will need for the duration of its lifetime.
    /// The config allows you to access the configuration of the task.
    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
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
    fn process(&mut self, clock: &RobotClock, new_msg: Self::Output) -> CuResult<()>;

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
pub trait CuTask<'cl>: Freezable {
    type Input: CuMsgPack<'cl>;
    type Output: CuMsgPack<'cl>;

    /// Here you need to initialize everything your task will need for the duration of its lifetime.
    /// The config allows you to access the configuration of the task.
    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
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
    fn process(
        &mut self,
        _clock: &RobotClock,
        input: Self::Input,
        output: Self::Output,
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
pub trait CuSinkTask<'cl>: Freezable {
    type Input: CuMsgPack<'cl>;

    /// Here you need to initialize everything your task will need for the duration of its lifetime.
    /// The config allows you to access the configuration of the task.
    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
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
    fn process(&mut self, _clock: &RobotClock, input: Self::Input) -> CuResult<()>;

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
