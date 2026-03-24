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
        output.set_payload(1);
        Ok(())
    }
}

#[derive(Reflect)]
struct Proc;

impl Freezable for Proc {}

impl CuTask for Proc {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(i32);
    type Output<'m> = output_msg!(i32);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        if let Some(value) = input.payload() {
            output.set_payload(*value);
        }
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

#[copper_runtime(config = "config/background_modes_valid.ron")]
struct App {}

fn main() {}
