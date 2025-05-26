use cu29::prelude::*;

pub struct ExampleSrc {}

impl Freezable for ExampleSrc {}

impl<'cl> CuSrcTask<'cl> for ExampleSrc {
    type Output = output_msg!('cl, i32);

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {})
    }

    fn process(&mut self, _clock: &RobotClock, new_msg: Self::Output) -> CuResult<()> {
        new_msg.set_payload(42);
        Ok(())
    }
}

pub struct ExampleTaskA {}

impl Freezable for ExampleTaskA {}

impl<'cl> CuTask<'cl> for ExampleTaskA {
    type Input = input_msg!('cl, i32);
    type Output = output_msg!('cl, i32);

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {})
    }

    fn process(
        &mut self,
        _clock: &RobotClock,
        input: Self::Input,
        output: Self::Output,
    ) -> CuResult<()> {
        debug!("Processing from Mission A.");
        output.set_payload(input.payload().unwrap() + 1);
        Ok(())
    }
}

pub struct ExampleTaskB {}

impl Freezable for ExampleTaskB {}

impl<'cl> CuTask<'cl> for ExampleTaskB {
    type Input = input_msg!('cl, i32);
    type Output = output_msg!('cl, i32);

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {})
    }

    fn process(
        &mut self,
        _clock: &RobotClock,
        input: Self::Input,
        output: Self::Output,
    ) -> CuResult<()> {
        debug!("Processing from Mission B.");
        output.set_payload(input.payload().unwrap() + 1);
        Ok(())
    }
}

pub struct ExampleSink {}

impl Freezable for ExampleSink {}

impl<'cl> CuSinkTask<'cl> for ExampleSink {
    type Input = input_msg!('cl, i32);

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {})
    }

    fn process(&mut self, _clock: &RobotClock, _input: Self::Input) -> CuResult<()> {
        Ok(())
    }
}
