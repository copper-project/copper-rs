use cu29::prelude::*;
use cu_sim_depthsense::VirtPointCloud;

pub struct Sink {}

impl Freezable for Sink {}

impl<'cl> CuSinkTask<'cl> for Sink {
    type Input = input_msg!('cl, VirtPointCloud);

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {})
    }

    fn process(&mut self, _clock: &RobotClock, input: Self::Input) -> CuResult<()> {
        Ok(())
    }
}
