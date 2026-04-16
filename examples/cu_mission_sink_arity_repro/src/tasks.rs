use cu29::prelude::*;

#[derive(Reflect)]
pub struct IntSource;

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
pub struct BoolSource;

impl Freezable for BoolSource {}

impl CuSrcTask for BoolSource {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(bool);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, _ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        output.set_payload(true);
        Ok(())
    }
}

#[derive(Reflect)]
pub struct CombinedSink;

impl Freezable for CombinedSink {}

impl CuSinkTask for CombinedSink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!('m, i32, bool);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, _ctx: &CuContext, _input: &Self::Input<'_>) -> CuResult<()> {
        Ok(())
    }
}
