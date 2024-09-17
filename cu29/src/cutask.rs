//! This module contains all the main definition of the traits you need to implement
//! or interact with to create a Copper task.

use crate::clock::OptionCuTime;
use crate::config::NodeInstanceConfig;
use crate::CuResult;
use bincode::de::Decode;
use bincode::de::Decoder;
use bincode::enc::Encode;
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use cu29_clock::RobotClock;
use std::fmt;
use std::fmt::{Display, Formatter};

// Everything that is stateful in copper for zero copy constraints need to be restricted to this trait.
pub trait CuMsgPayload: Default + Encode + Decode + Sized {}

// Also anything that follows this contract can be a payload (blanket implementation)
impl<T> CuMsgPayload for T where T: Default + Encode + Decode + Sized {}

/// CuMsgMetadata is a structure that contains metadata common to all CuMsgs.
#[derive(Debug, Default, bincode::Encode, bincode::Decode)]
pub struct CuMsgMetadata {
    /// The time before the process method is called.
    pub before_process: OptionCuTime,
    /// The time after the process method is called.
    pub after_process: OptionCuTime,
}

impl Display for CuMsgMetadata {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "before_process: {}, after_process: {}",
            self.before_process, self.after_process
        )
    }
}

/// CuMsg is the envelope holding the msg payload and the metadata between tasks.
#[derive(Debug, bincode::Encode, bincode::Decode)]
pub struct CuMsg<T>
where
    T: CuMsgPayload,
{
    /// This payload is the actual data exchanged between tasks.
    payload: Option<T>,

    /// This metadata is the data that is common to all messages.
    pub metadata: CuMsgMetadata,
}

impl<T> CuMsg<T>
where
    T: CuMsgPayload,
{
    pub fn new(payload: Option<T>) -> Self {
        CuMsg {
            payload,
            metadata: CuMsgMetadata {
                before_process: OptionCuTime::none(),
                after_process: OptionCuTime::none(),
            },
        }
    }
    pub fn payload(&self) -> Option<&T> {
        self.payload.as_ref()
    }

    pub fn set_payload(&mut self, payload: T) {
        self.payload = Some(payload);
    }

    pub fn payload_mut(&mut self) -> &mut Option<T> {
        &mut self.payload
    }
}

/// The internal state of a task needs to be serializable
/// so the framework can take a snapshop of the task graph.
pub trait Freezable {
    /// This method is called by the framework when it wants to save the task state.
    /// The default implementation is to encode nothing (stateless).
    /// If you have a state, you need to implement this method.
    fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&(), encoder) // default is stateless
    }

    /// This method is called by the framework when it wants to restore the task to a specific state.
    /// Here it is similar to Decode but the framework will give you a new instance of the task (the new method will be called)
    ///
    #[allow(unused_variables)]
    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        Ok(())
    }
}

/// The CuTaskLifecycle trait is the base trait for all tasks in Copper.
/// It defines the lifecycle of a task.
/// It provides a default empty implementation as all those execution steps are optional.
pub trait CuTaskLifecycle: Freezable {
    fn new(config: Option<&NodeInstanceConfig>) -> CuResult<Self>
    where
        Self: Sized;

    /// Start is called once for a long period of time.
    /// Here you need to initialize everything your task will need for the duration of its lifetime.
    fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
        Ok(())
    }

    /// This is a method called by the runtime before "process". This is a kind of best effort,
    /// as soon as possible call to give a chance for the task to do some work before to prepare
    /// to make "process" as short as possible.
    fn preprocess(&mut self, _clock: &RobotClock) -> CuResult<()> {
        Ok(())
    }

    /// This is a method called by the runtime after "process". It is best effort a chance for
    /// the task to update some state after process is out of the way.
    /// It can be use for example to maintain statistics etc. that are not time critical for the robot.
    fn postprocess(&mut self, _clock: &RobotClock) -> CuResult<()> {
        Ok(())
    }

    /// Call at the end of the lifecycle of the task.
    fn stop(&mut self, _clock: &RobotClock) -> CuResult<()> {
        Ok(())
    }
}

/// A Src Task is a task that only produces messages. For example drivers for sensors are Src Tasks.
/// They are in push mode from the runtime.
/// To set the frequency of the pulls and align them to any hw, see the runtime configuration.
pub trait CuSrcTask: CuTaskLifecycle {
    type Output: CuMsgPayload;

    /// Process is the most critical execution of the task.
    /// The goal will be to produce the output message as soon as possible.
    /// Use preprocess to prepare the task to make this method as short as possible.
    fn process(&mut self, clock: &RobotClock, new_msg: &mut CuMsg<Self::Output>) -> CuResult<()>;
}

/// This is the most generic Task of copper. It is a "transform" task deriving an output from an input.
pub trait CuTask: CuTaskLifecycle {
    type Input: CuMsgPayload;
    type Output: CuMsgPayload;

    /// Process is the most critical execution of the task.
    /// The goal will be to produce the output message as soon as possible.
    /// Use preprocess to prepare the task to make this method as short as possible.
    fn process(
        &mut self,
        clock: &RobotClock,
        input: &CuMsg<Self::Input>,
        output: &mut CuMsg<Self::Output>,
    ) -> CuResult<()>;
}

/// A Sink Task is a task that only consumes messages. For example drivers for actuators are Sink Tasks.
pub trait CuSinkTask: CuTaskLifecycle {
    type Input: CuMsgPayload;

    /// Process is the most critical execution of the task.
    /// The goal will be to produce the output message as soon as possible.
    /// Use preprocess to prepare the task to make this method as short as possible.
    fn process(&mut self, clock: &RobotClock, input: &CuMsg<Self::Input>) -> CuResult<()>;
}
