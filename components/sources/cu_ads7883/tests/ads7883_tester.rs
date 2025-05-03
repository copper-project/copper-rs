use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;
use cu_ads7883_new::ADSReadingPayload;

#[copper_runtime(config = "tests/copperconfig.ron")]
struct ADS78883Tester {}

pub struct ADS78883TestSink {}

impl Freezable for ADS78883TestSink {}

impl<'cl> CuSinkTask<'cl> for ADS78883TestSink {
    type Input = input_msg!('cl, ADSReadingPayload);
    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {})
    }

    fn process(&mut self, _clock: &RobotClock, new_msg: Self::Input) -> CuResult<()> {
        debug!("Received: {}", &new_msg.payload());
        Ok(())
    }
}

#[test]
#[ignore] // As this needs the real hardware to run.
fn test_driver_with_hardware() {
    let tmp_dir = tempfile::TempDir::new().expect("could not create a tmp dir");
    let logger_path = tmp_dir.path().join("caterpillar.copper");
    let copper_ctx =
        basic_copper_setup(&logger_path, None, true, None).expect("Failed to setup logger.");
    debug!("Logger created at {}.", logger_path);
    let clock = copper_ctx.clock;
    debug!("Creating application... ");
    let mut application =
        ADS78883Tester::new(clock.clone(), copper_ctx.unified_logger.clone(), None)
            .expect("Failed to create runtime.");
    debug!("Running... starting clock: {}.", clock.now());
    for _ in 0..1000 {
        application
            .run_one_iteration()
            .expect("Failed to run application.");
    }
    debug!("End of program: {}.", clock.now());
}
