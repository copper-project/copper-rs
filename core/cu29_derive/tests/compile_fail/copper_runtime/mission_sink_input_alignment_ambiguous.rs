use cu29::prelude::*;
use cu29_derive::copper_runtime;

#[derive(Reflect)]
struct IntSource;

impl Freezable for IntSource {}

impl CuSrcTask for IntSource {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(i32);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, _ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        output.set_payload(7);
        Ok(())
    }
}

#[derive(Reflect)]
struct BoolSource;

impl Freezable for BoolSource {}

impl CuSrcTask for BoolSource {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(bool);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, _ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        output.set_payload(true);
        Ok(())
    }
}

#[derive(Reflect)]
struct AmbiguousSink;

impl Freezable for AmbiguousSink {}

impl CuSinkTask for AmbiguousSink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!('m, i32, i32, bool);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, _ctx: &CuContext, input: &Self::Input<'_>) -> CuResult<()> {
        let _ = input.0.payload();
        let _ = input.1.payload();
        let _ = input.2.payload();
        Ok(())
    }
}

#[copper_runtime(config = "config/mission_sink_input_alignment_ambiguous_invalid.ron")]
struct App {}

fn main() {}
