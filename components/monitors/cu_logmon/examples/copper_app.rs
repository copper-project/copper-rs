use cu29::prelude::*;
use simplelog::{Config as LogConfig, SimpleLogger};
use std::fs;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::time::Duration;

pub mod tasks {
    use cu29::prelude::*;

    pub struct Src {
        counter: u32,
    }

    impl Freezable for Src {}

    impl CuSrcTask for Src {
        type Output<'m> = output_msg!(u32);

        fn new(_config: Option<&ComponentConfig>) -> CuResult<Self> {
            Ok(Self { counter: 0 })
        }

        fn process(&mut self, _clock: &RobotClock, new_msg: &mut Self::Output<'_>) -> CuResult<()> {
            self.counter = self.counter.wrapping_add(1);
            new_msg.set_payload(self.counter);
            Ok(())
        }
    }

    pub struct Step;

    impl Freezable for Step {}

    impl CuTask for Step {
        type Input<'m> = input_msg!(u32);
        type Output<'m> = output_msg!(u32);

        fn new(_config: Option<&ComponentConfig>) -> CuResult<Self> {
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

    pub struct Sink;

    impl Freezable for Sink {}

    impl CuSinkTask for Sink {
        type Input<'m> = input_msg!(u32);

        fn new(_config: Option<&ComponentConfig>) -> CuResult<Self> {
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

fn setup_logging(log_path: &PathBuf) -> CuResult<CopperContext> {
    let preallocated_size = SLAB_SIZE.unwrap_or(1024 * 1024 * 10);
    let UnifiedLogger::Write(logger) = UnifiedLoggerBuilder::new()
        .write(true)
        .create(true)
        .file_base_name(log_path)
        .preallocated_size(preallocated_size)
        .build()
        .expect("Failed to create logger")
    else {
        panic!("Failed to create logger")
    };

    let unified_logger = Arc::new(Mutex::new(logger));
    let structured_stream = stream_write(
        unified_logger.clone(),
        UnifiedLogType::StructuredLogLine,
        4096 * 10,
    )?;

    let text_logger = SimpleLogger::new(simplelog::LevelFilter::Debug, LogConfig::default());

    let clock = RobotClock::new();
    let structured_logging =
        LoggerRuntime::init(clock.clone(), structured_stream, Some(text_logger));
    Ok(CopperContext {
        unified_logger,
        logger_runtime: structured_logging,
        clock,
    })
}

fn main() {
    let logger_path =
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("logs/logmon_copper_app.copper");
    if let Some(parent) = logger_path.parent() {
        let _ = fs::create_dir_all(parent);
    }

    let copper_ctx = setup_logging(&logger_path).expect("Failed to setup logger.");
    debug!("Logger created at {}", logger_path);

    let mut application = AppBuilder::new()
        .with_context(&copper_ctx)
        .build()
        .expect("Failed to create runtime");

    let clock = copper_ctx.clock;
    debug!("Starting app at {}", clock.now());
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
    debug!("App stopped at {}", clock.now());
}
