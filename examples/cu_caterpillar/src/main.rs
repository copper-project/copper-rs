use copper::clock::{OptionCuTime, RobotClock};
use copper::cutask::{CuMsg, CuSrcTask, CuTask, CuTaskLifecycle};
use copper::{CuResult, DataLogType};
use copper_datalogger::{stream, DataLogger};
use copper_derive::copper_runtime;
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
    let data_logger = Arc::new(Mutex::new(
        DataLogger::create(path.as_path(), Some(100000)).expect("Failed to create logger"),
    ));
    let stream = stream(data_logger.clone(), DataLogType::StructuredLogLine, 1024);

    //
    let slow_text_logger = TermLogger::new(
        LevelFilter::Debug,
        Config::default(),
        TerminalMode::Mixed,
        ColorChoice::Auto,
    );

    // This is the path to the index file that was created at build time.
    let log_index_path = PathBuf::from("../..").join("target/debug/copper_log_index");
    println!("log_index_path: {:?}", log_index_path);

    //    .join("target/debug/copper_log_index");

    let extra: ExtraTextLogger = ExtraTextLogger::new(log_index_path, slow_text_logger);
    let _needed = LoggerRuntime::init(stream, Some(extra));
    debug!("Application created.");
    let mut application = TheVeryHungryCaterpillar::new().expect("Failed to create runtime.");
    debug!(
        "Running... starting clock: {}.",
        application.copper_runtime.clock.now()
    );
    application.run(2).expect("Failed to run application.");
    debug!("End of program.");
    sleep(Duration::from_secs(1));
}
