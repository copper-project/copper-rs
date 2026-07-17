use cu29::prelude::*;

#[derive(Reflect)]
pub struct HeartbeatSource;

impl Freezable for HeartbeatSource {}

impl CuSrcTask for HeartbeatSource {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(u64);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, _ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        output.set_payload(1);
        Ok(())
    }
}

#[derive(Reflect)]
pub struct HeartbeatSink;

impl Freezable for HeartbeatSink {}

impl CuSinkTask for HeartbeatSink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(u64);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, _ctx: &CuContext, _input: &Self::Input<'_>) -> CuResult<()> {
        Ok(())
    }
}
