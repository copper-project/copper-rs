use bincode::de::Decoder;
use bincode::enc::Encoder;
use cu29::clock::RobotClock;
use cu29::config::NodeInstanceConfig;
use cu29::cutask::{CuMsg, CuSrcTask, CuTask, CuTaskLifecycle, Freezable};
use cu29::CuResult;
use cu29_log_derive::debug;
use cu_ads7883::ADSReadingMsg;
use cu_rp_sn754410::MotorMsg;

pub struct PIDTask {}

impl Freezable for PIDTask {}

impl CuTaskLifecycle for PIDTask {
    fn new(_config: Option<&NodeInstanceConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {})
    }
}

impl CuTask for PIDTask {
    type Input = ADSReadingMsg;
    type Output = MotorMsg;

    fn process(
        &mut self,
        _clock: &RobotClock,
        input: &CuMsg<Self::Input>,
        output: &mut CuMsg<Self::Output>,
    ) -> CuResult<()> {
        debug!("PIDTask processing input: {}", input.payload);
        Ok(())
    }
}
