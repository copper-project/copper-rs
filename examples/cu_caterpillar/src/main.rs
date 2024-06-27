use copper::clock::{ClockProvider, OptionCuTime, RobotClock};
use copper::cutask::{CuMsg, CuSrcTask, CuTask, CuTaskLifecycle};
use copper::CuResult;
use copper_derive::copper_runtime;
use copper_helpers::basic_logger_runtime_setup;
use copper_log_derive::debug;
use cu_rp_gpio::RPGpioMsg;
use serde::{Deserialize, Serialize};
use std::path::PathBuf;
use std::thread::sleep;
use std::time::Duration;

#[copper_runtime(config = "copperconfig.ron")]
struct TheVeryHungryCaterpillar {}

#[derive(Serialize, Deserialize, Default)]
pub struct CaterpillarMsg(bool);

pub struct CaterpillarSource {
    state: bool,
}

impl CuTaskLifecycle for CaterpillarSource {
    fn new(_config: Option<&copper::config::NodeInstanceConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self { state: true })
    }
}

impl CuSrcTask for CaterpillarSource {
    type Output = RPGpioMsg;

    fn process(&mut self, clock: &RobotClock, output: &mut CuMsg<Self::Output>) -> CuResult<()> {
        // forward the state to the next task
        self.state = !self.state;
        output.payload = RPGpioMsg {
            on: self.state,
            creation: clock.now().into(),
            actuation: OptionCuTime::none(),
        };
        Ok(())
    }
}

pub struct CaterpillarTask {}

impl CuTaskLifecycle for CaterpillarTask {
    fn new(_config: Option<&copper::config::NodeInstanceConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {})
    }
}

impl CuTask for CaterpillarTask {
    type Input = RPGpioMsg;
    type Output = RPGpioMsg;

    fn process(
        &mut self,
        _clock: &RobotClock,
        input: &CuMsg<Self::Input>,
        output: &mut CuMsg<Self::Output>,
    ) -> CuResult<()> {
        // forward the state to the next task
        output.payload = input.payload;
        Ok(())
    }
}

fn main() {
    let logger_runtime =
        basic_logger_runtime_setup(&PathBuf::from("/tmp/caterpillar.copper"), true)
            .expect("Failed to setup logger.");
    let clock = logger_runtime.get_clock();
    debug!("Application created.");
    let mut application =
        TheVeryHungryCaterpillar::new(clock.clone()).expect("Failed to create runtime.");
    debug!("Running... starting clock: {}.", clock.now());
    application.run(2).expect("Failed to run application.");
    debug!("End of program: {}.", clock.now());
    sleep(Duration::from_secs(1));
}
