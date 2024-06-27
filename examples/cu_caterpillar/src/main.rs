use copper::clock::{OptionCuTime, RobotClock};
use copper::cutask::{CuMsg, CuSrcTask, CuTask, CuTaskLifecycle};
use copper::{CuResult, DataLogType};
use copper_datalogger::{stream_write, DataLogger, DataLoggerBuilder};
use copper_derive::copper_runtime;
use copper_log::default_log_index_dir;
use copper_log_derive::debug;
use copper_log_runtime::{ExtraTextLogger, LoggerRuntime};
use cu_rp_gpio::RPGpioMsg;
use serde::{Deserialize, Serialize};
use simplelog::{ColorChoice, Config, LevelFilter, TermLogger, TerminalMode};
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::thread::sleep;
use std::time::Duration;

#[copper_runtime(config = "copperconfig.ron")]
struct TheVeryHungryCaterpillar {}

#[derive(Serialize, Deserialize, Default)]
pub struct CaterpillarMsg(bool);

pub struct CaterpillarSource {
    state: bool,
}

impl CuTaskLifecycle for CaterpillarSource {
    fn new(_config: Option<&copper::config::NodeInstanceConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self { state: true })
    }
}

impl CuSrcTask for CaterpillarSource {
    type Output = RPGpioMsg;

    fn process(&mut self, clock: &RobotClock, output: &mut CuMsg<Self::Output>) -> CuResult<()> {
        // forward the state to the next task
        self.state = !self.state;
        output.payload = RPGpioMsg {
            on: self.state,
            creation: clock.now().into(),
            actuation: OptionCuTime::none(),
        };
        Ok(())
    }
}

pub struct CaterpillarTask {}

impl CuTaskLifecycle for CaterpillarTask {
    fn new(_config: Option<&copper::config::NodeInstanceConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {})
    }
}

impl CuTask for CaterpillarTask {
    type Input = RPGpioMsg;
    type Output = RPGpioMsg;

    fn process(
        &mut self,
        _clock: &RobotClock,
        input: &CuMsg<Self::Input>,
        output: &mut CuMsg<Self::Output>,
    ) -> CuResult<()> {
        // forward the state to the next task
        output.payload = input.payload;
        Ok(())
    }
}

fn main() {
    let path: PathBuf = PathBuf::from("/tmp/caterpillar.copper");
    let DataLogger::Write(logger) = DataLoggerBuilder::new()
        .write(true)
        .create(true)
        .file_path(&path)
        .preallocated_size(100000)
        .build()
        .expect("Failed to create logger")
    else {
        panic!("Failed to create logger")
    };
    let data_logger = Arc::new(Mutex::new(logger));
    let stream = stream_write(data_logger.clone(), DataLogType::StructuredLogLine, 1024);

    let slow_text_logger = TermLogger::new(
        LevelFilter::Debug,
        Config::default(),
        TerminalMode::Mixed,
        ColorChoice::Auto,
    );

    // This is the path to the index file that was created at build time.
    // depending if we build with debug pick it from debug or release:
    let log_index_path = default_log_index_dir();

    let extra: ExtraTextLogger = ExtraTextLogger::new(log_index_path, slow_text_logger);
    let clock = RobotClock::default();
    let _needed = LoggerRuntime::init(clock.clone(), stream, Some(extra)); // with the slow textual logger on top.
    debug!("Application created.");
    let mut application =
        TheVeryHungryCaterpillar::new(clock.clone()).expect("Failed to create runtime.");
    debug!("Running... starting clock: {}.", clock.now());
    application.run(2).expect("Failed to run application.");
    debug!("End of program.");
    drop(_needed);
    sleep(Duration::from_secs(1));
}
