use cu29::prelude::*;

pub struct ExampleSrc {}

impl Freezable for ExampleSrc {}

impl CuSrcTask for ExampleSrc {
    type Output = output_msg!(i32);

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {})
    }

    fn process(&mut self, _clock: &RobotClock, new_msg: &mut Self::Output) -> CuResult<()> {
        new_msg.set_payload(42);
        Ok(())
    }
}

pub struct ExampleTaskA {}

impl Freezable for ExampleTaskA {}

impl CuTask for ExampleTaskA {
    type Input<'m> = input_msg!(i32);
    type Output = output_msg!(i32);

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {})
    }

    fn process(
        &mut self,
        _clock: &RobotClock,
        input: &Self::Input<'_>,
        output: &mut Self::Output,
    ) -> CuResult<()> {
        debug!("Processing from Mission A.");
        output.set_payload(input.payload().unwrap() + 1);
        Ok(())
    }
}

pub struct ExampleTaskB {}

impl Freezable for ExampleTaskB {}

impl CuTask for ExampleTaskB {
    type Input<'m> = input_msg!(i32);
    type Output = output_msg!(i32);

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {})
    }

    fn process(
        &mut self,
        _clock: &RobotClock,
        input: &Self::Input<'_>,
        output: &mut Self::Output,
    ) -> CuResult<()> {
        debug!("Processing from Mission B.");
        output.set_payload(input.payload().unwrap() + 1);
        Ok(())
    }
}

pub struct ExampleSink {}

impl Freezable for ExampleSink {}

impl CuSinkTask for ExampleSink {
    type Input<'m> = input_msg!(i32);

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {})
    }

    fn process(&mut self, _clock: &RobotClock, _input: &Self::Input<'_>) -> CuResult<()> {
        Ok(())
    }
}
