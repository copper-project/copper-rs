use cu29::prelude::*;
use cu29_derive::copper_runtime;

#[derive(Reflect)]
struct SingleSource;

impl Freezable for SingleSource {}

impl CuSrcTask for SingleSource {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(i32);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, _ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        output.set_payload(42);
        Ok(())
    }
}

#[copper_runtime(
    config = "config/explicit_task_kind_source_no_outputs_valid.ron",
    sim_mode = true,
    ignore_resources = true
)]
struct App {}

fn main() {}
