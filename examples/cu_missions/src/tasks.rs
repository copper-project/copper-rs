use cu29::prelude::*;

pub struct ExampleSrc;

impl Freezable for ExampleSrc {}

impl CuSrcTask for ExampleSrc {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(i32);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self)
    }

    fn process(&mut self, _clock: &RobotClock, new_msg: &mut Self::Output<'_>) -> CuResult<()> {
        new_msg.set_payload(42);
        Ok(())
    }
}

pub struct ExampleTask {
    label: String,
}

impl Freezable for ExampleTask {}

impl CuTask for ExampleTask {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(i32);
    type Output<'m> = output_msg!(i32);

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let label = config
            .and_then(|cfg| cfg.get::<String>("label").ok().flatten())
            .unwrap_or_else(|| "Mission".to_string());
        Ok(Self { label })
    }

    fn process(
        &mut self,
        _clock: &RobotClock,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        debug!("Processing from {}.", &self.label);
        output.set_payload(input.payload().unwrap() + 1);
        Ok(())
    }
}

pub struct ExampleSink;

impl Freezable for ExampleSink {}

impl CuSinkTask for ExampleSink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(i32);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self)
    }

    fn process(&mut self, _clock: &RobotClock, _input: &Self::Input<'_>) -> CuResult<()> {
        Ok(())
    }
}
