use cu29::clock::RobotClock;
use cu29::config::ComponentConfig;
use cu29::cutask::{CuMsg, CuSinkTask, CuTaskLifecycle, Freezable};
use cu29::{input_msg, CuResult};
use cu29_derive::copper_runtime;
use cu29_helpers::basic_copper_setup;
use cu29_log_derive::debug;
use cu_ads7883_new::ADSReadingPayload;
use std::path::PathBuf;

#[copper_runtime(config = "tests/copperconfig.ron")]
struct ADS78883Tester {}

pub struct ADS78883TestSink {}

impl Freezable for ADS78883TestSink {}

impl CuTaskLifecycle for ADS78883TestSink {
    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {})
    }
}

impl<'cl> CuSinkTask<'cl> for ADS78883TestSink {
    type Input = input_msg!('cl, ADSReadingPayload);

    fn process(&mut self, _clock: &RobotClock, new_msg: Self::Input) -> CuResult<()> {
        debug!("Received: {}", &new_msg.payload());
        Ok(())
    }
}

#[test]
#[ignore] // As this needs the real hardware to run.
fn test_driver_with_hardware() {
    let logger_path = "/tmp/caterpillar.copper";
    let copper_ctx = basic_copper_setup(&PathBuf::from(logger_path), None, true, None)
        .expect("Failed to setup logger.");
    debug!("Logger created at {}.", logger_path);
    let clock = copper_ctx.clock;
    debug!("Creating application... ");
    let mut application = ADS78883Tester::new(clock.clone(), copper_ctx.unified_logger.clone())
        .expect("Failed to create runtime.");
    debug!("Running... starting clock: {}.", clock.now());
    for _ in 0..1000 {
        application
            .run_one_iteration()
            .expect("Failed to run application.");
    }
    debug!("End of program: {}.", clock.now());
}