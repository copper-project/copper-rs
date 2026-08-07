use cu29::prelude::*;
use cu29_derive::copper_runtime;

#[derive(Reflect)]
struct ConstantSource;

impl Freezable for ConstantSource {}

impl CuSrcTask for ConstantSource {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(());

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, _ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        output.set_payload(());
        Ok(())
    }
}

#[copper_runtime(config = "config/constants_expression_type_mismatch.ron")]
struct App {}

fn main() {}
