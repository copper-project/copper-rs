use cu29::prelude::*;
use cu29_derive::copper_runtime;

#[derive(Reflect)]
struct StatelessSource;

impl Freezable for StatelessSource {}

impl CuSrcTask for StatelessSource {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(u32);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, _ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        output.set_payload(21);
        Ok(())
    }
}

#[derive(Reflect)]
struct StatelessTransform;

impl Freezable for StatelessTransform {}

impl CuStatelessTask for StatelessTransform {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(u32);
    type Output<'m> = output_msg!(u32);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn preprocess(&self, _ctx: &CuContext) -> CuResult<()> {
        Ok(())
    }

    fn process(
        &self,
        _ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        output.set_payload(input.payload().copied().unwrap_or_default() * 2);
        Ok(())
    }

    fn postprocess(&self, _ctx: &CuContext) -> CuResult<()> {
        Ok(())
    }
}

#[copper_runtime(
    config = "config/stateless_task_valid.ron",
    sim_mode = true,
    ignore_resources = true
)]
struct App {}

fn main() {}
