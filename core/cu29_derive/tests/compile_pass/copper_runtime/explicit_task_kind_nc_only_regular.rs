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
struct NcOnlyRegular;

impl Freezable for NcOnlyRegular {}

impl CuTask for NcOnlyRegular {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(i32);
    type Output<'m> = output_msg!(bool);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(
        &mut self,
        _ctx: &CuContext,
        _input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        output.set_payload(true);
        Ok(())
    }
}

#[copper_runtime(
    config = "config/explicit_task_kind_nc_only_regular_valid.ron",
    sim_mode = true,
    ignore_resources = true
)]
struct App {}

fn main() {}
