use cu29::prelude::*;
use cu29_derive::copper_runtime;

#[derive(Reflect)]
struct MultiSource;

impl Freezable for MultiSource {}

impl CuSrcTask for MultiSource {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(i32, bool);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, _ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        output.0.set_payload(42);
        output.1.set_payload(true);
        Ok(())
    }
}

#[derive(Reflect)]
struct BoolSink;

impl Freezable for BoolSink {}

impl CuSinkTask for BoolSink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(bool);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, _ctx: &CuContext, _input: &Self::Input<'_>) -> CuResult<()> {
        Ok(())
    }
}

#[copper_runtime(
    config = "config/multi_output_source_non_first_connected_valid.ron",
    sim_mode = true,
    ignore_resources = true
)]
struct App {}

fn main() {}
