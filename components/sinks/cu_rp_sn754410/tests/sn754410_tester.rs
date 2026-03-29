use cu29::prelude::*;

#[copper_runtime(config = "tests/copperconfig.ron")]
struct SN754410Tester {}

#[test]
#[ignore] // As this needs the real hardware to run.
fn test_driver_with_hardware() {
    let tmp_dir = tempfile::TempDir::new().expect("could not create a tmp dir");
    let logger_path = tmp_dir.path().join("caterpillar.copper");
    debug!("Logger created at {}.", &logger_path);
    let clock = RobotClock::default();
    debug!("Creating application... ");
    let mut application = SN754410Tester::builder()
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
}
