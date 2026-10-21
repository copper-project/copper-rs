//! Small deterministic task graph used by the live monitor example.
use cu29::prelude::*;

#[derive(Reflect)]
pub struct Tick;
impl Freezable for Tick {}
impl CuSrcTask for Tick {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(f64);
    fn new(_config: Option<&ComponentConfig>, _resources: ()) -> CuResult<Self> {
        Ok(Self)
    }
    fn process(&mut self, ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        output.set_payload(ctx.cl_id() as f64);
        output.tov = Tov::Time(ctx.now());
        Ok(())
    }
}

#[derive(Reflect)]
pub struct Scale {
    gain: f64,
}
impl Freezable for Scale {}
impl CuTask for Scale {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(f64);
    type Output<'m> = output_msg!(f64);
    fn new(config: Option<&ComponentConfig>, _resources: ()) -> CuResult<Self> {
        let gain = config
            .map(|config| config.get::<f64>("gain"))
            .transpose()?
            .flatten()
            .unwrap_or(1.0);
        Ok(Self { gain })
    }
    fn process(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        if let Some(value) = input.payload() {
            output.set_payload(value * self.gain);
        } else {
            output.clear_payload();
        }
        output.tov = input.tov;
        Ok(())
    }
}

#[derive(Reflect)]
pub struct Output;
impl Freezable for Output {}
impl CuSinkTask for Output {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(f64);
    fn new(_config: Option<&ComponentConfig>, _resources: ()) -> CuResult<Self> {
        Ok(Self)
    }
    fn process(&mut self, _ctx: &CuContext, _input: &Self::Input<'_>) -> CuResult<()> {
        Ok(())
    }
}
