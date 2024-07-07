use copper::clock::RobotClock;
use copper::config::NodeInstanceConfig;
use copper::cutask::{CuMsg, CuSinkTask, CuTaskLifecycle};
use copper::CuResult;
use copper_derive::copper_runtime;
use copper_helpers::basic_copper_setup;
use copper_log_derive::debug;
use cu_wt901::PositionalReadings;
use std::path::PathBuf;
use std::thread::sleep;
use std::time::Duration;

struct WT910TestSink {}

impl CuTaskLifecycle for WT910TestSink {
    fn new(_config: Option<&NodeInstanceConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {})
    }
}

impl CuSinkTask for WT910TestSink {
    type Input = PositionalReadings;

    fn process(&mut self, clock: &RobotClock, new_msg: &mut CuMsg<Self::Input>) -> CuResult<()> {
        debug!("Received: {}", &new_msg.payload);
        Ok(())
    }
}

#[copper_runtime(config = "copperconfig.ron")]
struct WT910Tester {}

fn main() {
    let logger_path = "/tmp/caterpillar.copper";
    let copper_ctx =
        basic_copper_setup(&PathBuf::from(logger_path), true).expect("Failed to setup logger.");
    debug!("Logger created at {}.", logger_path);
    let clock = copper_ctx.clock;
    debug!("Creating application... ");
    let mut application = WT910Tester::new(clock.clone(), copper_ctx.unified_logger.clone())
        .expect("Failed to create runtime.");
    debug!("Running... starting clock: {}.", clock.now());
    application.run(100).expect("Failed to run application.");
    debug!("End of program: {}.", clock.now());
    sleep(Duration::from_secs(1));
}
