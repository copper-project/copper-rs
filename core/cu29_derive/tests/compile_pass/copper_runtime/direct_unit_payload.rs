use cu29::prelude::*;
use cu29::units::si::angle::radian;
use cu29::units::si::f32::Angle;
use cu29_derive::copper_runtime;

#[derive(Reflect)]
struct AngleSource;

impl Freezable for AngleSource {}

impl CuSrcTask for AngleSource {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(Angle);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, _ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        output.set_payload(Angle::new::<radian>(0.0));
        Ok(())
    }
}

#[copper_runtime(config = "config/direct_unit_payload_valid.ron")]
struct App {}

fn main() {}
