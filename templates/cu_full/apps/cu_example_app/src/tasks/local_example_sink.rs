use cu29::prelude::*;

use crate::messages::MyPayload;

// Defines a sink (ie. actualtion)
#[derive(Default)]
pub struct MySink {}

// Needs to be fully implemented if you want to have a stateful task.
impl Freezable for MySink {}

impl CuSinkTask for MySink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(MyPayload);

    fn new_with(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {})
    }

    // don't forget the other lifecycle methods if you need them: start, stop, preprocess, postprocess

    fn process(&mut self, _clock: &RobotClock, input: &Self::Input<'_>) -> CuResult<()> {
        debug!("Sink Received message: {}", input.payload().unwrap().value);
        Ok(())
    }
}
