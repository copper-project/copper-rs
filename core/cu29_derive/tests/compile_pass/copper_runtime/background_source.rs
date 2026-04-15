use cu29::prelude::*;
use cu29_derive::copper_runtime;

#[derive(Reflect)]
struct BackgroundSrc;

impl Freezable for BackgroundSrc {}

impl CuSrcTask for BackgroundSrc {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(u32);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, _ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        output.set_payload(42);
        Ok(())
    }
}

#[copper_runtime(config = "config/background_source_valid.ron")]
struct App {}

fn main() {}
