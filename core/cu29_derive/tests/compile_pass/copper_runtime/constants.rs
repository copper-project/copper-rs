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

#[copper_runtime(
    config = "config/constants_valid.ron",
    sim_mode = true,
    ignore_resources = true
)]
struct App {}

const _: usize = COUNT;
const _: [cu29::units::si::f32::Length; 3] = LENGTH_DEFAULT;
const _: [cu29::units::si::f32::Length; 3] = LENGTH_MM;
const _: cu29::units::si::f32::Mass = MASS_KG;
const _: cu29::units::si::f64::ThermodynamicTemperature = TEMPERATURE_C;

fn main() {}
