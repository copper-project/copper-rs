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
        .build_app()
        .expect("Failed to create runtime.")
        .start_all_tasks()
        .expect("Failed to start tasks.");
    debug!("Running... starting clock: {}.", clock.now());
    for _ in 0..1000 {
        application
            .run_one_iteration()
            .expect("Failed to run application.");
    }
    application.stop_all_tasks().expect("Failed to stop tasks.");
    debug!("End of program: {}.", clock.now());
}
