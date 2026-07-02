use cu29::prelude::*;
use cu29_derive::copper_runtime;

#[derive(Reflect)]
struct Source;
impl Freezable for Source {}
impl CuSrcTask for Source {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(i32);
    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }
    fn process(&mut self, _ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        output.set_payload(0);
        Ok(())
    }
}

#[derive(Reflect)]
struct Worker;
impl Freezable for Worker {}
impl CuTask for Worker {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(i32);
    type Output<'m> = output_msg!(i32);
    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }
    fn process(
        &mut self,
        _ctx: &CuContext,
        _input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        output.set_payload(0);
        Ok(())
    }
}

#[derive(Reflect)]
struct Sink;
impl Freezable for Sink {}
impl CuSinkTask for Sink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(i32);
    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }
    fn process(&mut self, _ctx: &CuContext, _input: &Self::Input<'_>) -> CuResult<()> {
        Ok(())
    }
}

#[copper_runtime(
    config = "config/ignore_resources_background_valid.ron",
    sim_mode = true,
    ignore_resources = true
)]
struct App {}

fn main() {}
