use bincode::{Decode, Encode};
use cu29::prelude::*;

// Define a message type
#[derive(Default, Debug, Clone, Encode, Decode, Serialize)]
pub struct MyPayload {
    value: i32,
}

// Defines a source (ie. driver)
#[derive(Default)]
pub struct MySource {}

// Needs to be fully implemented if you want to have a stateful task.
impl Freezable for MySource {}

impl CuSrcTask for MySource {
    type Output<'m> = output_msg!(MyPayload);

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {})
    }

    // don't forget the other lifecycle methods if you need them: start, stop, preprocess, postprocess

    fn process(&mut self, _clock: &RobotClock, output: &mut Self::Output<'_>) -> CuResult<()> {
        // Generated a 42 message.
        output.set_payload(MyPayload { value: 42 });
        Ok(())
    }
}

// Defines a processing task
pub struct MyTask {
    // if you add some task state here, you need to implement the Freezable trait
}

// Needs to be fully implemented if you want to have a stateful task.
impl Freezable for MyTask {}

impl CuTask for MyTask {
    type Input<'m> = input_msg!(MyPayload);
    type Output<'m> = output_msg!(MyPayload);

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        // add the task state initialization here
        Ok(Self {})
    }

    // don't forget the other lifecycle methods if you need them: start, stop, preprocess, postprocess

    fn process(
        &mut self,
        _clock: &RobotClock,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        debug!("Received message: {}", input.payload().unwrap().value);
        output.set_payload(MyPayload { value: 43 });
        Ok(()) // outputs another message for downstream
    }
}

// Defines a sink (ie. actualtion)
#[derive(Default)]
pub struct MySink {}

// Needs to be fully implemented if you want to have a stateful task.
impl Freezable for MySink {}

impl CuSinkTask for MySink {
    type Input<'m> = input_msg!('m, MyPayload, MyPayload);

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {})
    }
    // don't forget the other lifecycle methods if you need them: start, stop, preprocess, postprocess

    fn process(&mut self, _clock: &RobotClock, input: &Self::Input<'_>) -> CuResult<()> {
        debug!(
            "Sink Received message 0: {}",
            input.0.payload().unwrap().value
        );
        debug!(
            "Sink Received message 1: {}",
            input.1.payload().unwrap().value
        );
        Ok(())
    }
}
