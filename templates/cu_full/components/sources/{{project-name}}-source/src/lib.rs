use cu29::prelude::*;

use {{project-name|snake_case}}_types::MyPayload;

// Defines a source (ie. driver).
#[derive(Default)]
pub struct MySource {}

// Needs to be fully implemented if you want to have a stateful task.
impl Freezable for MySource {}

impl CuSrcTask for MySource {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(MyPayload);

    fn new_with(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {})
    }

    // don't forget the other lifecycle methods if you need them: start, stop, preprocess, postprocess

    fn process(&mut self, _clock: &RobotClock, output: &mut Self::Output<'_>) -> CuResult<()> {
        // Generate a 42 message.
        output.set_payload(MyPayload { value: 42 });
        Ok(())
    }
}
