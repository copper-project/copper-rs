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
struct PairSink;

impl Freezable for PairSink {}

impl CuSinkTask for PairSink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!('m, i32, i32);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, _ctx: &CuContext, input: &Self::Input<'_>) -> CuResult<()> {
        let _ = input.0.payload();
        let _ = input.1.payload();
        Ok(())
    }
}

#[copper_runtime(config = "config/mission_sink_reused_id_stable_positions_valid.ron")]
struct App {}

fn main() {
    let clock = RobotClock::default();

    let _ = A::App::builder().with_clock(clock.clone()).build();
    let _ = B::App::builder().with_clock(clock).build();
}
