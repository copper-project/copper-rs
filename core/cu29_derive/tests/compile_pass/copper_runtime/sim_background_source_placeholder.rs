use cu29::prelude::*;
use cu29_derive::copper_runtime;

#[derive(Reflect)]
struct BackgroundSource;

impl Freezable for BackgroundSource {}

impl CuSrcTask for BackgroundSource {
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

#[copper_runtime(
    config = "config/sim_background_source_placeholder_valid.ron",
    sim_mode = true
)]
struct App {}

fn main() {}
