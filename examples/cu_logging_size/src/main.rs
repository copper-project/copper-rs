use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;
use std::fs::metadata;

pub mod tasks {
    use cu29::prelude::*;

    pub struct ExampleSrc {}

    impl Freezable for ExampleSrc {}

    impl<'cl> CuSrcTask<'cl> for ExampleSrc {
        type Output = output_msg!('cl, i32);

        fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self {})
        }

        fn process(&mut self, _clock: &RobotClock, new_msg: Self::Output) -> CuResult<()> {
            new_msg.set_payload(42);
            Ok(())
        }
    }

    pub struct ExampleTask {}

    impl Freezable for ExampleTask {}

    impl<'cl> CuTask<'cl> for ExampleTask {
        type Input = input_msg!('cl, i32);
        type Output = output_msg!('cl, i32);

        fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self {})
        }

        fn process(
            &mut self,
            _clock: &RobotClock,
            input: Self::Input,
            output: Self::Output,
        ) -> CuResult<()> {
            output.set_payload(input.payload().unwrap() + 1);
            Ok(())
        }
    }

    pub struct ExampleSink {}

    impl Freezable for ExampleSink {}

    impl<'cl> CuSinkTask<'cl> for ExampleSink {
        type Input = input_msg!('cl, i32);

        fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self {})
        }

        fn process(&mut self, _clock: &RobotClock, _input: Self::Input) -> CuResult<()> {
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
    debug!("Logger created at {}.", path = &logger_path);
    let clock = copper_ctx.clock;
    debug!("Creating application... ");
    let mut application = App::new(clock.clone(), copper_ctx.unified_logger.clone(), None)
        .expect("Failed to create application.");
    debug!("Running... starting clock: {}.", clock.now());
    application
        .start_all_tasks()
        .expect("Failed to start application.");
    application
        .run_one_iteration()
        .expect("Failed to run application.");
    application
        .stop_all_tasks()
        .expect("Failed to stop application.");
    debug!("End of program: {}.", clock.now());
    // check if the logger file is at least 1 section in length

    // change the end of the logger_path from copper to _0.copper
    let logger_first_path = tmp_dir.path().join("logger_0.copper"); // get the first slab

    match metadata(&logger_first_path) {
        Ok(meta) => {
            let file_size = meta.len();
            assert!(file_size >= SLAB_SIZE.unwrap() as u64);
        }
        Err(e) => {
            eprintln!("Failed to get file metadata: {e}");
        }
    }
    let (current_slab_used, _current_slab_offsets, _back_slab_in_flight) =
        copper_ctx.unified_logger.lock().unwrap().stats();
    assert!(current_slab_used > 100 * 1024 * 1024); // in the ron file we said:  section_size_mib: 100 so at least that amount should be used before it the logger is closed and trimmed
}
