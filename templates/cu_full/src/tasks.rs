use cu29::prelude::*;
use bincode::{Decode, Encode};

// Define a message type
#[derive(Default, Debug, Clone, Encode, Decode)]
pub struct MyPayload {
    value: i32,
}

// Defines a source (ie. driver)
#[derive(Default)]
pub struct MySource {}

// Needs to be fully implemented if you want to have a stateful task.
impl Freezable for MySource {}

impl<'cl> CuSrcTask<'cl> for MySource {
    type Output = output_msg!('cl, MyPayload);

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {})
    }

    // don't forget the other lifecycle methods if you need them: start, stop, preprocess, postprocess

    fn process(&mut self, _clock: &RobotClock, output: Self::Output) -> CuResult<()> {
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

impl<'cl> CuTask<'cl> for MyTask {
    type Input = input_msg!('cl, MyPayload);
    type Output = output_msg!('cl, MyPayload);

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
        input: Self::Input,
        output: Self::Output,
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

impl<'cl> CuSinkTask<'cl> for MySink {
    type Input = input_msg!('cl, MyPayload);

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {})
    }
    // don't forget the other lifecycle methods if you need them: start, stop, preprocess, postprocess

    fn process(&mut self, _clock: &RobotClock, input: Self::Input) -> CuResult<()> {
        debug!("Sink Received message: {}", input.payload().unwrap().value);
        Ok(())
    }
}
