use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;
use std::fs;
use std::path::{Path, PathBuf};

pub mod tasks {
    use cu29::prelude::*;
    use std::thread::sleep;

    pub struct ExampleSrc;

    impl Freezable for ExampleSrc {}

    impl CuSrcTask for ExampleSrc {
        type Resources<'r> = ();
        type Output<'m> = output_msg!(i32);

        fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self)
        }

        fn process(&mut self, _clock: &RobotClock, new_msg: &mut Self::Output<'_>) -> CuResult<()> {
            new_msg.set_payload(42);
            Ok(())
        }
    }

    pub struct ExampleTask {
        sleep_duration: std::time::Duration,
    }

    impl Freezable for ExampleTask {}

    impl CuTask for ExampleTask {
        type Resources<'r> = ();
        type Input<'m> = input_msg!(i32);
        type Output<'m> = output_msg!(i32);

        fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            let config = config.ok_or_else(|| CuError::from("Missing config"))?;
            let sleep_duration_ms = config
                .get::<u64>("sleep_duration_ms")?
                .ok_or_else(|| CuError::from("Missing sleep_duration_ms"))?;
            let sleep_duration = std::time::Duration::from_millis(sleep_duration_ms);
            Ok(Self { sleep_duration })
        }

        fn process(
            &mut self,
            _clock: &RobotClock,
            input: &Self::Input<'_>,
            output: &mut Self::Output<'_>,
        ) -> CuResult<()> {
            let Some(payload) = input.payload() else {
                return Ok(());
            };
            // Emulate a long-running task
            debug!(
                "Task is tasking a {}ms time to process input: {}",
                self.sleep_duration.as_millis(),
                &payload
            );
            sleep(self.sleep_duration);
            debug!("Task ({}ms) done.", self.sleep_duration.as_millis());
            output.set_payload(payload + 1);
            Ok(())
        }
    }

    pub struct ExampleSink;

    impl Freezable for ExampleSink {}

    impl CuSinkTask for ExampleSink {
        type Resources<'r> = ();
        type Input<'m> = input_msg!(i32);

        fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self)
        }

        fn process(&mut self, _clock: &RobotClock, _input: &Self::Input<'_>) -> CuResult<()> {
            Ok(())
        }
    }
}

#[copper_runtime(config = "copperconfig.ron")]
struct App {}

const SLAB_SIZE: Option<usize> = Some(150 * 1024 * 1024);
fn main() {
    let logger_path = "logs/background.copper";
    if let Some(parent) = Path::new(logger_path).parent()
        && !parent.exists()
    {
        fs::create_dir_all(parent).expect("Failed to create logs directory");
    }
    let copper_ctx = basic_copper_setup(&PathBuf::from(logger_path), SLAB_SIZE, true, None)
        .expect("Failed to setup logger.");

    debug!("Logger created at {}.", path = &logger_path);
    let clock = copper_ctx.clock;
    debug!("Creating application... ");
    let mut application = App::new(clock.clone(), copper_ctx.unified_logger.clone(), None)
        .expect("Failed to create application.");
    debug!("Running... starting clock: {}.", clock.now());
    application
        .start_all_tasks()
        .expect("Failed to start application.");
    application.run().expect("Failed to run application.");
    application
        .stop_all_tasks()
        .expect("Failed to stop application.");
    debug!("End of program: {}.", clock.now());
    // check if the logger file is at least 1 section in length
}
