//! This module contains all the main definition of the traits you need to implement
//! or interact with to create a Copper task.

use crate::config::ComponentConfig;
use bincode::de::Decoder;
use bincode::de::{BorrowDecoder, Decode};
use bincode::enc::Encode;
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::BorrowDecode;
use compact_str::{CompactString, ToCompactString};
use cu29_clock::{PartialCuTimeRange, RobotClock, Tov};
use cu29_traits::CuResult;
use serde_derive::{Deserialize, Serialize};
use std::fmt;
use std::fmt::{Debug, Display, Formatter};

// Everything that is stateful in copper for zero copy constraints need to be restricted to this trait.
pub trait CuMsgPayload: Default + Debug + Clone + Encode + Decode<()> + Sized {}

pub trait CuMsgPack<'cl> {}

// Also anything that follows this contract can be a payload (blanket implementation)
impl<T: Default + Debug + Clone + Encode + Decode<()> + Sized> CuMsgPayload for T {}

macro_rules! impl_cu_msg_pack {
    ($(($($ty:ident),*)),*) => {
        $(
            impl<'cl, $($ty: CuMsgPayload + 'cl),*> CuMsgPack<'cl> for ( $( &'cl CuMsg<$ty>, )* ) {}
        )*
    };
}

impl<'cl, T: CuMsgPayload> CuMsgPack<'cl> for (&'cl CuMsg<T>,) {}
impl<'cl, T: CuMsgPayload> CuMsgPack<'cl> for &'cl CuMsg<T> {}
impl<'cl, T: CuMsgPayload> CuMsgPack<'cl> for (&'cl mut CuMsg<T>,) {}
impl<'cl, T: CuMsgPayload> CuMsgPack<'cl> for &'cl mut CuMsg<T> {}
impl CuMsgPack<'_> for () {}

// Apply the macro to generate implementations for tuple sizes up to 5
impl_cu_msg_pack! {
    (T1, T2), (T1, T2, T3), (T1, T2, T3, T4), (T1, T2, T3, T4, T5) // TODO: continue if necessary
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

// MAX_SIZE from their repr module is not accessible so we need to copy paste their definition for 24
// which is the maximum size for inline allocation (no heap)
const COMPACT_STRING_CAPACITY: usize = size_of::<String>();

#[derive(Debug, Clone, Default, Serialize, Deserialize, PartialEq, Eq)]
pub struct CuCompactString(pub CompactString);

impl Encode for CuCompactString {
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        let CuCompactString(ref compact_string) = self;
        let bytes = compact_string.as_bytes();
        bytes.encode(encoder)
    }
}

impl<Context> Decode<Context> for CuCompactString {
    fn decode<D: Decoder>(decoder: &mut D) -> Result<Self, DecodeError> {
        let bytes = <Vec<u8> as Decode<D::Context>>::decode(decoder)?; // Decode into a byte buffer
        let compact_string =
            CompactString::from_utf8(bytes).map_err(|e| DecodeError::Utf8 { inner: e })?;
        Ok(CuCompactString(compact_string))
    }
}

impl<'de, Context> BorrowDecode<'de, Context> for CuCompactString {
    fn borrow_decode<D: BorrowDecoder<'de>>(decoder: &mut D) -> Result<Self, DecodeError> {
        CuCompactString::decode(decoder)
    }
}

/// CuMsgMetadata is a structure that contains metadata common to all CuMsgs.
#[derive(Debug, Clone, bincode::Encode, bincode::Decode, Serialize, Deserialize)]
pub struct CuMsgMetadata {
    /// The time range used for the processing of this message
    pub process_time: PartialCuTimeRange,
    /// The time of validity of the message.
    /// It can be undefined (None), one measure point or a range of measures (TimeRange).
    pub tov: Tov,
    /// A small string for real time feedback purposes.
    /// This is useful for to display on the field when the tasks are operating correctly.
    pub status_txt: CuCompactString,
}

impl CuMsgMetadata {
    pub fn set_status(&mut self, status: impl ToCompactString) {
        self.status_txt = CuCompactString(status.to_compact_string());
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
#[derive(Default, Debug, Clone, bincode::Encode, bincode::Decode)]
pub struct CuMsg<T>
where
    T: CuMsgPayload,
{
    /// This payload is the actual data exchanged between tasks.
    payload: Option<T>,

    /// This metadata is the data that is common to all messages.
    pub metadata: CuMsgMetadata,
}

impl Default for CuMsgMetadata {
    fn default() -> Self {
        CuMsgMetadata {
            process_time: PartialCuTimeRange::default(),
            tov: Tov::default(),
            status_txt: CuCompactString(CompactString::with_capacity(COMPACT_STRING_CAPACITY)),
        }
    }
}

impl<T> CuMsg<T>
where
    T: CuMsgPayload,
{
    pub fn new(payload: Option<T>) -> Self {
        CuMsg {
            payload,
            metadata: CuMsgMetadata::default(),
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
    #[allow(unused_variables)]
    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        Ok(())
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
