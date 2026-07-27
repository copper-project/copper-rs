//! Sim-mode expansion of the anytime runner: the sim callback fires once per
//! node in the base step, and on `ExecutedBySim` the job local stays `None`
//! so every refine step no-ops.
use cu29::cutask_anytime::{AnytimeStatus, CuAnytimeTask, Quality};
use cu29::prelude::*;
use cu29_derive::copper_runtime;

#[derive(Reflect)]
struct AnytimeSrc;

impl Freezable for AnytimeSrc {}

impl CuSrcTask for AnytimeSrc {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(u32);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, _ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        output.set_payload(3);
        Ok(())
    }
}

#[derive(Reflect)]
struct AnytimeRefiner;

impl Freezable for AnytimeRefiner {}

impl CuAnytimeTask for AnytimeRefiner {
    type Input<'m> = input_msg!(u32);
    type Output<'m> = output_msg!(u32);
    type Resources<'r> = ();
    type Quality = Quality;

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn base(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<AnytimeStatus<Quality>> {
        output.set_payload(input.payload().copied().unwrap_or(0));
        Ok(AnytimeStatus::Improved(cu29::cutask_anytime::quality_from_f32(0.5)))
    }

    fn refine(
        &mut self,
        _ctx: &CuContext,
        output: &mut Self::Output<'_>,
    ) -> CuResult<AnytimeStatus<Quality>> {
        let bumped = output.payload().copied().unwrap_or(0) + 1;
        output.set_payload(bumped);
        Ok(AnytimeStatus::Converged(
            cu29::cutask_anytime::quality_from_f32(1.0),
        ))
    }
}

#[derive(Reflect)]
struct AnytimeCounter;

impl Freezable for AnytimeCounter {}

impl CuAnytimeTask for AnytimeCounter {
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
struct AnytimeSink;

impl Freezable for AnytimeSink {}

impl CuSinkTask for AnytimeSink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(u32);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, _ctx: &CuContext, _input: &Self::Input<'_>) -> CuResult<()> {
        Ok(())
    }
}

#[copper_runtime(config = "config/anytime_task_valid.ron", sim_mode = true)]
struct App {}

fn main() {}
