use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;

pub mod tasks {
    use cu29::prelude::*;

    pub struct ExampleSrc {}

    impl Freezable for ExampleSrc {}

    impl CuSrcTask for ExampleSrc {
        type Output<'m> = output_msg!(i32);

        fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self {})
        }

        fn process(&mut self, _clock: &RobotClock, new_msg: &mut Self::Output<'_>) -> CuResult<()> {
            new_msg.set_payload(42);
            Ok(())
        }
    }

    pub struct ExampleBridge {
        counter: i32,
    }

    impl Freezable for ExampleBridge {}

    impl CuBridge for ExampleBridge {
        type Input<'m> = input_msg!(i32);
        type Output<'m> = output_msg!(i32);

        fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self { counter: 0 })
        }

        fn send<'i>(&mut self, clock: &RobotClock, msg: &Self::Input<'i>) -> CuResult<()> {
            debug!(
                "Sending through the bridge {} at {}",
                msg.payload(),
                clock.now()
            );
            Ok(())
        }

        fn receive<'o>(&mut self, clock: &RobotClock, msg: &mut Self::Output<'o>) -> CuResult<()> {
            self.counter += 1;
            msg.set_payload(self.counter);
            msg.tov = clock.now().into();
            debug!(
                "Receiving from the bridge {} at {}",
                msg.payload(),
                clock.now()
            );
            Ok(())
        }
    }

    pub struct ExampleSink {}

    impl Freezable for ExampleSink {}

    impl CuSinkTask for ExampleSink {
        type Input<'m> = input_msg!(i32);

        fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self {})
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
    let tmp_dir = tempfile::TempDir::new().expect("Could not create temporary directory");
    let logger_path = tmp_dir.path().join("logger.copper");
    let copper_ctx =
        basic_copper_setup(&logger_path, SLAB_SIZE, true, None).expect("Failed to setup logger.");
    let clock = copper_ctx.clock;
    let mut application = App::new(clock.clone(), copper_ctx.unified_logger.clone(), None)
        .expect("Failed to create application.");
    application
        .start_all_tasks()
        .expect("Failed to start application.");
    application
        .run_one_iteration()
        .expect("Failed to run application.");
    application
        .run_one_iteration()
        .expect("Failed to run application.");
    application
        .stop_all_tasks()
        .expect("Failed to stop application.");
}
