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

/// Full policy surface: comparable quality, target/floor/stall knobs.
#[derive(Reflect)]
struct AnytimeRefiner {
    target: u32,
    acc: u32,
}

impl Freezable for AnytimeRefiner {}

impl CuAnytimeTask for AnytimeRefiner {
    type Input<'m> = input_msg!(u32);
    type Output<'m> = output_msg!(u32);
    type Resources<'r> = ();
    type Quality = Quality;

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self { target: 0, acc: 0 })
    }

    fn base(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<AnytimeStatus<Quality>> {
        self.target = input.payload().copied().unwrap_or(0);
        self.acc = 0;
        output.set_payload(self.acc);
        Ok(AnytimeStatus::Improved(cu29::cutask_anytime::quality_from_f32(0.0)))
    }

    fn refine(
        &mut self,
        _ctx: &CuContext,
        output: &mut Self::Output<'_>,
    ) -> CuResult<AnytimeStatus<Quality>> {
        if self.acc >= self.target {
            return Ok(AnytimeStatus::Converged(
                cu29::cutask_anytime::quality_from_f32(1.0),
            ));
        }
        self.acc += 1;
        output.set_payload(self.acc);
        Ok(AnytimeStatus::Improved(
            cu29::cutask_anytime::quality_from_f32(self.acc as f32 / self.target as f32),
        ))
    }
}

/// Quality-less anytime task: only hard bounds are expressible.
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

#[copper_runtime(config = "config/anytime_task_valid.ron")]
struct App {}

fn main() {}
