use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;

pub mod cu_zenoh_ros {
    use cu_zenoh_ros_sink::ZenohRosSink;

    pub type ExamplePublisher = ZenohRosSink<i8>;
}

pub mod tasks {
    use cu29::prelude::*;
    use std::time::Duration;

    pub struct ExampleSrc {}

    impl Freezable for ExampleSrc {}

    impl<'cl> CuSrcTask<'cl> for ExampleSrc {
        type Output = output_msg!('cl, i8);

        fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self {})
        }

        fn process(&mut self, _clock: &RobotClock, new_msg: Self::Output) -> CuResult<()> {
            std::thread::sleep(Duration::from_secs(1));
            debug!("Sending message to ROS");
            new_msg.set_payload(42);
            Ok(())
        }
    }
}

#[copper_runtime(config = "copperconfig.ron")]
struct App {}

fn main() {
    let tmp_dir = tempfile::TempDir::new().expect("could not create a tmp dir");
    let logger_path = tmp_dir.path().join("zenoh.copper");
    let copper_ctx =
        basic_copper_setup(&logger_path, None, true, None).expect("Failed to setup logger.");
    debug!("Logger created at {}.", path = &logger_path);
    let clock = copper_ctx.clock;
    debug!("Creating application... ");
    let mut application = App::new(clock.clone(), copper_ctx.unified_logger.clone(), None)
        .expect("Failed to create application.");
    debug!("Running... starting clock: {}.", clock.now());

    let outcome = application.run();
    match outcome {
        Ok(_result) => {}
        Err(error) => {
            debug!("Application Ended: {}", error)
        }
    }
}
