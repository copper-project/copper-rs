use cu29::prelude::*;
use cu29_derive::copper_runtime;

#[derive(Clone, Copy)]
pub struct ConstPair {
    pub count: usize,
    pub rate: u32,
}

impl ConstPair {
    pub const fn new(count: usize, rate: u32) -> Self {
        Self { count, rate }
    }
}

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

const _: usize = constants::COUNT;
const _: [cu29::units::si::f32::Length; 3] = constants::LENGTH_DEFAULT;
const _: [cu29::units::si::f32::Length; 3] = constants::LENGTH_MM;
const _: cu29::units::si::f32::Mass = constants::MASS_KG;
const _: cu29::units::si::f64::ThermodynamicTemperature = constants::TEMPERATURE_C;
const _: u32 = robot::sensors::MAX_RATE;
const _: u16 = A::MISSION_LIMIT;
const _: u16 = B::MISSION_LIMIT;
const _: f32 = A::drive::GAIN;
const _: ConstPair = constructed::PAIR;
const _: [(); constants::COUNT] = [(); constructed::PAIR.count];

mod task_family {
    const _: usize = super::constants::COUNT;
    const _: u32 = super::robot::sensors::MAX_RATE;
}

fn main() {}
