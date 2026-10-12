//! Ordinary tasks share the PTP timeline through CuContext.

use cu29::prelude::*;

/// Emits a placeholder measurement stamped with the shared clock.
#[derive(Reflect)]
pub struct Sample;

impl Freezable for Sample {}

impl CuSrcTask for Sample {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(u64);

    fn new(_: Option<&ComponentConfig>, _: ()) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        output.set_payload(42);
        output.tov = Tov::Time(ctx.now());
        Ok(())
    }
}

/// Terminates the graph; the measurement and its timestamp are recorded by Copper.
#[derive(Reflect)]
pub struct Consume;

impl Freezable for Consume {}

impl CuSinkTask for Consume {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(u64);

    fn new(_: Option<&ComponentConfig>, _: ()) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, _: &CuContext, _: &Self::Input<'_>) -> CuResult<()> {
        Ok(())
    }
}
