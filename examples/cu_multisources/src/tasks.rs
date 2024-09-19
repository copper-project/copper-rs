use cu29::clock::RobotClock;
use cu29::config::NodeInstanceConfig;
use cu29::cutask::{CuSinkTask, CuTaskLifecycle};
use cu29::cutask::{CuSrcTask, Freezable};

use cu29::cutask::CuMsg;
use cu29::{input_msg, output_msg};
use cu29_traits::CuResult;

struct Src1Task {
    pub value: i32,
}

impl Freezable for Src1Task {}

impl CuTaskLifecycle for Src1Task {
    fn new(_config: Option<&NodeInstanceConfig>) -> CuResult<Self> {
        Ok(Self { value: 42 })
    }
}

impl CuSrcTask for Src1Task {
    type Output = output_msg!(i32);

    fn process(&mut self, _clock: &RobotClock, output: Self::Output) -> CuResult<()> {
        self.value += 1;
        output.payload = self.value;
        Ok(())
    }
}

struct Src2Task {
    pub value: f32,
}

impl Freezable for Src2Task {}

impl CuTaskLifecycle for Src2Task {
    fn new(_config: Option<&NodeInstanceConfig>) -> CuResult<Self> {
        Ok(Self { value: 24.0 })
    }
}

impl CuSrcTask for Src2Task {
    type Output = output_msg!(f32);

    fn process(&mut self, _clock: &RobotClock, output: Self::Output) -> CuResult<()> {
        self.value += 1.0;
        output.payload = self.value;
        Ok(())
    }
}

struct SinkTask {}

impl Freezable for SinkTask {}

impl CuTaskLifecycle for SinkTask {
    fn new(_config: Option<&NodeInstanceConfig>) -> CuResult<Self> {
        Ok(Self {})
    }
}

impl CuSinkTask for SinkTask {
    type Input = input_msg!(i32, f32);

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
