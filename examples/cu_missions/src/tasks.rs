use cu29::prelude::*;

#[derive(Reflect)]
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

    fn process(&mut self, _ctx: &CuContext, new_msg: &mut Self::Output<'_>) -> CuResult<()> {
        new_msg.set_payload(42);
        Ok(())
    }
}

#[derive(Reflect)]
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
        _ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        debug!("Processing from {}.", &self.label);
        output.set_payload(input.payload().unwrap() + 1);
        Ok(())
    }
}

#[derive(Reflect)]
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

    fn process(&mut self, _ctx: &CuContext, _input: &Self::Input<'_>) -> CuResult<()> {
        Ok(())
    }
}

#[derive(Reflect)]
pub struct ExampleMultiSrc;

impl Freezable for ExampleMultiSrc {}

impl CuSrcTask for ExampleMultiSrc {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(i32, u32);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self)
    }

    fn process(&mut self, _ctx: &CuContext, new_msg: &mut Self::Output<'_>) -> CuResult<()> {
        new_msg.0.set_payload(42);
        new_msg.1.set_payload(42);
        Ok(())
    }
}

#[derive(Reflect)]
pub struct MultiInputTask {
    label: String,
}

impl Freezable for MultiInputTask {}

impl CuTask for MultiInputTask {
    type Resources<'r> = ();
    type Input<'m> = input_msg!('m, i32, u32);
    type Output<'o> = output_msg!(i32);

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
        _ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        debug!("Processing from {}.", &self.label);
        if let Some(payload) = input.0.payload() {
            output.set_payload(payload.wrapping_add(1));
        }
        Ok(())
    }
}
