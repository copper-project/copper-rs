use copper_clock::OptionCuTime;
use serde::{Deserialize, Serialize};

use crate::config::NodeInstanceConfig;
use crate::CuResult;

// Everything that is stateful in copper for zero copy constraints need to be restricted to this trait.
pub trait CuMsgPayload: Default + Serialize + for<'a> Deserialize<'a> + Sized {}

// Also anything that follows this contract can be a payload (blanket implementation)
impl<T> CuMsgPayload for T where T: Default + Serialize + for<'a> Deserialize<'a> + Sized {}

const DEFAULT_MAX_RECEIVERS: usize = 2;

pub struct CuMsg<T>
where
    T: CuMsgPayload,
{
    pub payload: T,

    // Runtime statistics
    pub sent_time: OptionCuTime,
    pub received_times: [OptionCuTime; DEFAULT_MAX_RECEIVERS],
}

impl<T> CuMsg<T>
where
    T: CuMsgPayload,
{
    pub fn new(payload: T) -> Self {
        CuMsg {
            payload,
            sent_time: OptionCuTime::none(),
            received_times: [OptionCuTime::none(); DEFAULT_MAX_RECEIVERS],
        }
    }
}

/// The CuTaskLifecycle trait is the base trait for all tasks in Copper.
/// It defines the lifecycle of a task.
/// It provides a default empty implementation as all those execution steps are optional.
pub trait CuTaskLifecycle {
    fn new(config: Option<&NodeInstanceConfig>) -> CuResult<Self>
    where
        Self: Sized;

    /// Start is called once for a long period of time.
    /// Here you need to initialize everything your task will need for the duration of its lifetime.
    fn start(&mut self) -> CuResult<()> {
        Ok(())
    }

    /// This is a method called by the runtime before "process". This is a kind of best effort,
    /// as soon as possible call to give a chance for the task to do some work before to prepare
    /// to make "process" as short as possible.
    fn preprocess(&mut self) -> CuResult<()> {
        Ok(())
    }

    /// This is a method called by the runtime after "process". It is best effort a chance for
    /// the task to update some state after process is out of the way.
    /// It can be use for example to maintain statistics etc. that are not time critical for the robot.
    fn postprocess(&mut self) -> CuResult<()> {
        Ok(())
    }

    /// Call at the end of the lifecycle of the task.
    fn stop(&mut self) -> CuResult<()> {
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
    fn process(&mut self, new_msg: &mut CuMsg<Self::Output>) -> CuResult<()>;
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
    fn process(&mut self, input: &CuMsg<Self::Input>) -> CuResult<()>;
}
