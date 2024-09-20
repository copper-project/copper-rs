use cu29::clock::RobotClock;
use cu29::config::NodeInstanceConfig;
use cu29::cutask::{CuSinkTask, CuTaskLifecycle};
use cu29::cutask::{CuSrcTask, Freezable};

use cu29::cutask::CuMsg;
use cu29::{input_msg, output_msg};
use cu29_traits::CuResult;

pub struct Src1Task {
    pub value: i32,
}

impl Freezable for Src1Task {}

impl CuTaskLifecycle for Src1Task {
    fn new(_config: Option<&NodeInstanceConfig>) -> CuResult<Self> {
        Ok(Self { value: 42 })
    }
}

impl<'cl> CuSrcTask<'cl> for Src1Task {
    type Output = output_msg!('cl, i32);

    fn process(&mut self, _clock: &RobotClock, output: Self::Output) -> CuResult<()> {
        self.value += 1;
        output.set_payload(self.value);
        Ok(())
    }
}

pub struct Src2Task {
    pub value: f32,
}

impl Freezable for Src2Task {}

impl CuTaskLifecycle for Src2Task {
    fn new(_config: Option<&NodeInstanceConfig>) -> CuResult<Self> {
        Ok(Self { value: 24.0 })
    }
}

impl<'cl> CuSrcTask<'cl> for Src2Task {
    type Output = output_msg!('cl, f32);

    fn process(&mut self, _clock: &RobotClock, output: Self::Output) -> CuResult<()> {
        self.value += 1.0;
        output.set_payload(self.value);
        Ok(())
    }
}

pub struct SinkTask {}

impl Freezable for SinkTask {}

impl CuTaskLifecycle for SinkTask {
    fn new(_config: Option<&NodeInstanceConfig>) -> CuResult<Self> {
        Ok(Self {})
    }
}

impl<'cl> CuSinkTask<'cl> for SinkTask {
    type Input = input_msg!('cl, i32, f32);

    fn process(&mut self, _clock: &RobotClock, input: Self::Input) -> CuResult<()> {
        let (i, f) = input;
        println!(
            "DstTask received: {}, {}",
            i.payload().unwrap(),
            f.payload().unwrap()
        );
        Ok(())
    }
}
