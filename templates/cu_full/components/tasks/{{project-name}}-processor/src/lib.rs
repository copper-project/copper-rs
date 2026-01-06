use cu29::prelude::*;

use {{project-name|snake_case}}_types::MyPayload;

// Defines a processing task.
pub struct MyTask {
    // if you add some task state here, you need to implement the Freezable trait
}

// Needs to be fully implemented if you want to have a stateful task.
impl Freezable for MyTask {}

impl CuTask for MyTask {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(MyPayload);
    type Output<'m> = output_msg!(MyPayload);

    fn new_with(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
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
        let next = input.payload().unwrap().value + 1;
        debug!("Received message: {}", input.payload().unwrap().value);
        output.set_payload(MyPayload { value: next });
        Ok(()) // outputs another message for downstream
    }
}
