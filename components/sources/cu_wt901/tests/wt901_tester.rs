use cu_wt901::PositionalReadingsPayload;
use cu29::prelude::*;

use std::thread::sleep;
use std::time::Duration;

#[derive(Reflect)]
struct WT910TestSink;

impl Freezable for WT910TestSink {}

impl CuSinkTask for WT910TestSink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(PositionalReadingsPayload);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self)
    }

    fn process(&mut self, _ctx: &CuContext, new_msg: &Self::Input<'_>) -> CuResult<()> {
        debug!("Received: {}", &new_msg.payload());
        Ok(())
    }
}

#[copper_runtime(config = "tests/copperconfig.ron")]
struct WT910Tester {}

fn main() {
    let tmp_dir = tempfile::TempDir::new().expect("could not create a tmp dir");
    let logger_path = tmp_dir.path().join("caterpillar.copper");
    debug!("Logger created at {}.", &logger_path);
    let clock = RobotClock::default();
    debug!("Creating application... ");
    let mut application = WT910Tester::builder()
        .with_clock(clock.clone())
        .with_log_path(&logger_path, None)
        .expect("Failed to setup logger.")
        .build()
        .expect("Failed to create runtime.");
    debug!("Running... starting clock: {}.", clock.now());
    for _ in 0..1000 {
        application
            .run_one_iteration()
            .expect("Failed to run application.");
    }
    debug!("End of program: {}.", clock.now());
    sleep(Duration::from_secs(1));
}
