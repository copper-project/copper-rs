use cu29::prelude::*;
use cu29_derive::copper_runtime;

#[derive(Reflect)]
struct SourceA;

impl Freezable for SourceA {}

impl CuSrcTask for SourceA {
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
struct SourceB;

impl Freezable for SourceB {}

impl CuSrcTask for SourceB {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(i32);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, _ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        output.set_payload(2);
        Ok(())
    }
}

#[derive(Reflect)]
struct JoinTask;

impl Freezable for JoinTask {}

impl CuTask for JoinTask {
    type Resources<'r> = ();
    type Input<'m> = input_msg!('m, i32, i32);
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
        let left = input.0.payload().copied().unwrap_or_default();
        let right = input.1.payload().copied().unwrap_or_default();
        output.set_payload(left + right);
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

#[copper_runtime(config = "config/background_multi_input_invalid.ron")]
struct App {}

fn main() {}
