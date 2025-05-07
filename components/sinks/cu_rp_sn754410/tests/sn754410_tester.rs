use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;

#[copper_runtime(config = "tests/copperconfig.ron")]
struct SN754410Tester {}

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
        SN754410Tester::new(clock.clone(), copper_ctx.unified_logger.clone(), None)
            .expect("Failed to create runtime.");
    debug!("Running... starting clock: {}.", clock.now());
    for _ in 0..1000 {
        application
            .run_one_iteration()
            .expect("Failed to run application.");
    }
    debug!("End of program: {}.", clock.now());
}
