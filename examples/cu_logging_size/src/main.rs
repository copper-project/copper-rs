use cu29::prelude::*;
use std::fs::metadata;
use std::sync::{Arc, Mutex};

pub mod tasks {
    use cu29::prelude::*;

    #[derive(Reflect)]
    pub struct ExampleSrc {}

    impl Freezable for ExampleSrc {}

    impl CuSrcTask for ExampleSrc {
        type Resources<'r> = ();
        type Output<'m> = output_msg!(i32);

        fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self {})
        }

        fn process(&mut self, _ctx: &CuContext, new_msg: &mut Self::Output<'_>) -> CuResult<()> {
            new_msg.set_payload(42);
            Ok(())
        }
    }

    #[derive(Reflect)]
    pub struct ExampleTask {}

    impl Freezable for ExampleTask {}

    impl CuTask for ExampleTask {
        type Resources<'r> = ();
        type Input<'m> = input_msg!(i32);
        type Output<'m> = output_msg!(i32);

        fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self {})
        }

        fn process(
            &mut self,
            _ctx: &CuContext,
            input: &Self::Input<'_>,
            output: &mut Self::Output<'_>,
        ) -> CuResult<()> {
            output.set_payload(input.payload().unwrap() + 1);
            Ok(())
        }
    }

    #[derive(Reflect)]
    pub struct ExampleSink {}

    impl Freezable for ExampleSink {}

    impl CuSinkTask for ExampleSink {
        type Resources<'r> = ();
        type Input<'m> = input_msg!(i32);

        fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self {})
        }

        fn process(&mut self, _ctx: &CuContext, _input: &Self::Input<'_>) -> CuResult<()> {
            Ok(())
        }
    }
}

#[copper_runtime(config = "copperconfig.ron")]
struct App {}

const SLAB_SIZE_BYTES: usize = 150 * 1024 * 1024;
const MIN_USED_BYTES: usize = 100 * 1024 * 1024;
fn main() {
    let tmp_dir = tempfile::TempDir::new().expect("Could not create temporary directory");
    let logger_path = tmp_dir.path().join("logger.copper");
    let logger = UnifiedLoggerBuilder::new()
        .write(true)
        .create(true)
        .file_base_name(&logger_path)
        .preallocated_size(SLAB_SIZE_BYTES)
        .build()
        .expect("Failed to setup logger.");
    let UnifiedLogger::Write(logger) = logger else {
        panic!("UnifiedLoggerBuilder did not create a write-capable logger");
    };
    let unified_logger = Arc::new(Mutex::new(logger));
    debug!("Logger created at {}.", path = &logger_path);
    debug!("Creating application... ");
    let mut application = App::builder()
        .with_logger::<memmap::MmapSectionStorage, UnifiedLoggerWrite>(unified_logger.clone())
        .build()
        .expect("Failed to create application.");
    debug!("Running... starting clock: {}.", application.clock().now());
    application
        .start_all_tasks()
        .expect("Failed to start application.");
    application
        .run_one_iteration()
        .expect("Failed to run application.");
    application
        .stop_all_tasks()
        .expect("Failed to stop application.");
    debug!("End of program: {}.", application.clock().now());
    // check if the logger file is at least 1 section in length

    // change the end of the logger_path from copper to _0.copper
    let logger_first_path = tmp_dir.path().join("logger_0.copper"); // get the first slab

    match metadata(&logger_first_path) {
        Ok(meta) => {
            let file_size = meta.len();
            assert!(file_size >= SLAB_SIZE_BYTES as u64);
        }
        Err(e) => {
            eprintln!("Failed to get file metadata: {e}");
        }
    }
    let (current_slab_used, _current_slab_offsets, _back_slab_in_flight) =
        unified_logger.lock().unwrap().stats();
    assert!(current_slab_used > MIN_USED_BYTES); // in the ron file we said:  section_size_mib: 100 so at least that amount should be used before it the logger is closed and trimmed
}
