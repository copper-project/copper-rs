use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use copper::cutask::{CuMsg, CuSrcTask, CuTask, CuTaskLifecycle};
use copper::{CuResult, DataLogType};
use copper_derive::copper_runtime;
use copper_log_derive::debug;
use serde::{Deserialize, Serialize};
use copper::clock::{OptionCuTime, RobotClock};
use copper_datalogger::{DataLogger, stream};
use copper_log_runtime::LoggerRuntime;
use cu_rp_gpio::RPGpioMsg;

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
        DataLogger::new(path.as_path(), Some(100000)).expect("Failed to create logger"),
    ));
    let stream = stream(data_logger.clone(), DataLogType::StructuredLogLine, 1024);
    let _needed = LoggerRuntime::init(stream);
    debug!("Application created.");
    let mut application = TheVeryHungryCaterpillar::new().expect("Failed to create runtime.");
    debug!("Running... starting clock: {}.", application.copper_runtime.clock.now());
    application.run(2).expect("Failed to run application.");
    debug!("End of program.");
}

/// adds a panic handler that logs the panic message
fn setup_panic_hook() {
    std::panic::set_hook(Box::new(|info| {
        debug!("Panic: {}", panic_msg = info.to_string());
    }));
}
