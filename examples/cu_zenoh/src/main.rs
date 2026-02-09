use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;

pub mod cu_zenoh {
    use cu_zenoh_sink::ZenohSink;

    pub type ExampleSink = ZenohSink<i32>;
}
pub mod tasks {
    use std::time::Duration;

    use cu29::prelude::*;

    #[derive(Reflect)]
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
            std::thread::sleep(Duration::from_secs(1));
            debug!("Sending value");
            new_msg.set_payload(42);
            Ok(())
        }
    }
}

#[copper_runtime(config = "copperconfig.ron")]
struct App {}

const SLAB_SIZE: Option<usize> = Some(150 * 1024 * 1024);

fn main() {
    let tmp_dir = tempfile::TempDir::new().expect("could not create a tmp dir");
    let logger_path = tmp_dir.path().join("zenoh.copper");
    let copper_ctx =
        basic_copper_setup(&logger_path, SLAB_SIZE, true, None).expect("Failed to setup logger.");
    debug!("Logger created at {}.", path = &logger_path);
    let clock = copper_ctx.clock;
    debug!("Creating application... ");
    let mut application = App::new(clock.clone(), copper_ctx.unified_logger.clone(), None)
        .expect("Failed to create application.");
    debug!("Running... starting clock: {}.", clock.now());

    if let Err(error) = application.run() {
        debug!("Application Ended: {}", error)
    }
}
