use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;

mod tasks;

#[copper_runtime(config = "copperconfig.ron")]
struct App {}

// Get the applications for each mission
use A::AppBuilder as AppBuilderA;
use B::AppBuilder as AppBuilderB;

const SLAB_SIZE: Option<usize> = Some(1024 * 1024);

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

    let copper_ctx =
        basic_copper_setup(&logger_path, SLAB_SIZE, true, None).expect("Failed to setup logger.");
    debug!("Logger created at {}.", path = logger_path);
    debug!("Creating application... ");
    let mut application_a = AppBuilderA::new()
        .with_context(&copper_ctx)
        .build()
        .expect("Failed to create runtime");
    let mut application_b = AppBuilderB::new()
        .with_context(&copper_ctx)
        .build()
        .expect("Failed to create runtime");

    let clock = copper_ctx.clock;
    debug!("Running... starting clock: {}.", clock.now());
    debug!("Starting Mission A.");
    run_once(&mut application_a).expect("Failed to run mission A.");

    debug!("Starting Mission B.");
    // Note if you don't end the tasks, you'll be up for a bad time because tasks can hog some resources.
    run_once(&mut application_b).expect("Failed to run mission B.");

    debug!("End of program: {}.", clock.now());
}
