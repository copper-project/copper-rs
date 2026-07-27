//! A quality knob (`max_stall`) on a `Quality = ()` task pins the emitted
//! policy to the shared `Quality` scale: must fail to compile, with the
//! `AnytimePolicy` on_unimplemented note naming the fix.

use cu29::cutask_anytime::{AnytimeStatus, CuAnytimeTask};
use cu29::prelude::*;

#[derive(Reflect)]
struct IntSource;

impl Freezable for IntSource {}

impl CuSrcTask for IntSource {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(u32);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, _ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        output.set_payload(1);
        Ok(())
    }
}

#[derive(Reflect)]
struct UnscoredAnytime;

impl Freezable for UnscoredAnytime {}

impl CuAnytimeTask for UnscoredAnytime {
    type Input<'m> = input_msg!(u32);
    type Output<'m> = output_msg!(u32);
    type Resources<'r> = ();
    type Quality = ();

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn base(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<AnytimeStatus<()>> {
        output.set_payload(input.payload().copied().unwrap_or(0));
        Ok(AnytimeStatus::Improved(()))
    }

    fn refine(
        &mut self,
        _ctx: &CuContext,
        output: &mut Self::Output<'_>,
    ) -> CuResult<AnytimeStatus<()>> {
        let bumped = output.payload().copied().unwrap_or(0) + 1;
        output.set_payload(bumped);
        Ok(AnytimeStatus::Improved(()))
    }
}

#[derive(Reflect)]
struct IntSink;

impl Freezable for IntSink {}

impl CuSinkTask for IntSink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(u32);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, _ctx: &CuContext, _input: &Self::Input<'_>) -> CuResult<()> {
        Ok(())
    }
}

#[copper_runtime(config = "config/anytime_quality_knob_on_unscored_invalid.ron")]
struct App {}

fn main() {}
