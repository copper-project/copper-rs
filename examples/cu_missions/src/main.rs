use cu29::prelude::*;

mod tasks;

#[copper_runtime(config = "copperconfig.ron")]
struct App {}

// Get the applications for each mission
use A::App as MissionAApp;
use B::App as MissionBApp;

const SLAB_SIZE: Option<usize> = Some(10 * 1024 * 1024);

fn run_once<App>(app: &mut App) -> CuResult<()>
where
    App: CuApplication<memmap::MmapSectionStorage, UnifiedLoggerWrite>,
{
    app.start_all_tasks()?;
    app.run_one_iteration()?;
    app.stop_all_tasks()?;
    Ok(())
}

fn main() {
    let tmp_dir = tempfile::TempDir::new().expect("could not create a tmp dir");
    let logger_path = tmp_dir.path().join("monitor.copper");

    debug!("Logger created at {}.", path = &logger_path);
    debug!("Creating application... ");
    let clock = RobotClock::default();
    debug!("Running... starting clock: {}.", clock.now());
    debug!("Starting Mission A.");
    {
        let mut application_a = MissionAApp::builder()
            .with_clock(clock.clone())
            .with_log_path(&logger_path, SLAB_SIZE)
            .expect("Failed to setup logger.")
            .build()
            .expect("Failed to create runtime");
        run_once(&mut application_a).expect("Failed to run mission A.");
    }

    debug!("Starting Mission B.");
    // Note if you don't end the tasks, you'll be up for a bad time because tasks can hog some resources.
    {
        let mut application_b = MissionBApp::builder()
            .with_clock(clock.clone())
            .with_log_path(&logger_path, SLAB_SIZE)
            .expect("Failed to setup logger.")
            .build()
            .expect("Failed to create runtime");
        run_once(&mut application_b).expect("Failed to run mission B.");
    }

    debug!("End of program: {}.", clock.now());
}
