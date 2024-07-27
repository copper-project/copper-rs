use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{Decode, Encode};

use cu29::clock::{OptionCuTime, RobotClock};
use cu29::config::NodeInstanceConfig;
use cu29::cutask::{CuMsg, CuSrcTask, CuSinkTask, CuTask, CuTaskLifecycle, Freezable};
use cu29::CuResult;

use cu29_log_derive::debug;


// Define a message type
#[derive(Default, Debug, Clone, Encode, Decode)]
pub struct MyMsg {
    value: i32,
}


// Defines a source (ie. driver)
#[derive(Default)]
pub struct MySource {}

// Needs to be fully implemented if you want to have a stateful task.
impl Freezable for MySource {}

impl CuTaskLifecycle for MySource {
    fn new(_config: Option<&NodeInstanceConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self { })
    }
    // don't forget the other lifecycle methods if you need them: start, stop, preprocess, postprocess
}

impl CuSrcTask for MySource {
    type Output = MyMsg;

    fn process(&mut self, clock: &RobotClock, output: &mut CuMsg<Self::Output>) -> CuResult<()> {
        // Generated a 42 message.
        output.payload = MyMsg {
            value: 42,
        };
        Ok(())
    }

}


// Defines a processing task
pub struct MyTask {
    // if you add some task state here, you need to implement the Freezable trait
}

// Needs to be fully implemented if you want to have a stateful task.
impl Freezable for MyTask {}

impl CuTaskLifecycle for MyTask {
    fn new(_config: Option<&NodeInstanceConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        // add the task state initialization here
        Ok(Self {})
    }

    // don't forget the other lifecycle methods if you need them: start, stop, preprocess, postprocess
}

impl CuTask for MyTask {
    type Input = MyMsg;
    type Output = MyMsg;

    fn process(
        &mut self,
        _clock: &RobotClock,
        input: &CuMsg<Self::Input>,
        output: &mut CuMsg<Self::Output>,
    ) -> CuResult<()> {
        debug!("Received message: {}", input.payload.value);
        output.payload.value = 43;
        Ok(()) // outputs another message for downstream
    }
}

// Defines a sink (ie. actualtion)
#[derive(Default)]
pub struct MySink {}

// Needs to be fully implemented if you want to have a stateful task.
impl Freezable for MySink {}

impl CuTaskLifecycle for MySink {
    fn new(_config: Option<&NodeInstanceConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self { })
    }
    // don't forget the other lifecycle methods if you need them: start, stop, preprocess, postprocess
}

impl CuSinkTask for MySink {
    type Input = MyMsg;

    fn process(&mut self, _clock: &RobotClock, input: &mut CuMsg<Self::Input>) -> CuResult<()> {
        debug!("Sink Received message: {}", input.payload.value);
        Ok(())
    }

}
