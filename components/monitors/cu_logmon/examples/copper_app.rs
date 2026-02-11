use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;
use std::fs;
use std::path::PathBuf;
use std::time::Duration;

pub mod tasks {
    use cu29::prelude::*;

    #[derive(Reflect)]
    pub struct Src {
        counter: u32,
    }

    impl Freezable for Src {}

    impl CuSrcTask for Src {
        type Resources<'r> = ();
        type Output<'m> = output_msg!(u32);

        fn new(
            _config: Option<&ComponentConfig>,
            _resources: Self::Resources<'_>,
        ) -> CuResult<Self> {
            Ok(Self { counter: 0 })
        }

        fn process(&mut self, _clock: &RobotClock, new_msg: &mut Self::Output<'_>) -> CuResult<()> {
            self.counter = self.counter.wrapping_add(1);
            new_msg.set_payload(self.counter);
            Ok(())
        }
    }

    #[derive(Reflect)]
    pub struct Step;

    impl Freezable for Step {}

    impl CuTask for Step {
        type Resources<'r> = ();
        type Input<'m> = input_msg!(u32);
        type Output<'m> = output_msg!(u32);

        fn new(
            _config: Option<&ComponentConfig>,
            _resources: Self::Resources<'_>,
        ) -> CuResult<Self> {
            Ok(Self)
        }

        fn process(
            &mut self,
            _clock: &RobotClock,
            input: &Self::Input<'_>,
            output: &mut Self::Output<'_>,
        ) -> CuResult<()> {
            let payload = input.payload().unwrap();
            output.set_payload(payload + 10);
            Ok(())
        }
    }

    #[derive(Reflect)]
    pub struct Sink;

    impl Freezable for Sink {}

    impl CuSinkTask for Sink {
        type Resources<'r> = ();
        type Input<'m> = input_msg!(u32);

        fn new(
            _config: Option<&ComponentConfig>,
            _resources: Self::Resources<'_>,
        ) -> CuResult<Self> {
            Ok(Self)
        }

        fn process(&mut self, _clock: &RobotClock, _input: &Self::Input<'_>) -> CuResult<()> {
            // Do nothing, just exercise the pipeline
            Ok(())
        }
    }
}

#[copper_runtime(config = "copperconfig.ron")]
struct App {}

// Keep modest but large enough to satisfy section sizes in copperconfig.ron.
const SLAB_SIZE: Option<usize> = Some(16 * 1024 * 1024);

fn main() {
    let logger_path =
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("logs/logmon_copper_app.copper");
    if let Some(parent) = logger_path.parent() {
        let _ = fs::create_dir_all(parent);
    }

    let copper_ctx =
        basic_copper_setup(&logger_path, SLAB_SIZE, true, None).expect("Failed to setup logger.");
    info!("Logger created at {}", logger_path);

    let mut application = AppBuilder::new()
        .with_context(&copper_ctx)
        .build()
        .expect("Failed to create runtime");

    let clock = copper_ctx.clock;
    info!("Starting app at {}", clock.now());
    application
        .start_all_tasks()
        .expect("Failed to start application.");

    // Run a fixed number of iterations so the monitor has time to emit a few lines,
    // then exit cleanly.
    for _ in 0..120 {
        application
            .run_one_iteration()
            .expect("Failed to run one iteration.");
        std::thread::sleep(Duration::from_millis(50));
    }

    application
        .stop_all_tasks()
        .expect("Failed to stop application.");
    info!("App stopped at {}", clock.now());
}
