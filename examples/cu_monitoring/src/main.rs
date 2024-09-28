use cu29::config::ComponentConfig;
use cu29::cutask::CuMsgMetadata;
use cu29::monitoring::{CuMonitor, CuTaskState, Decision};
use cu29_derive::copper_runtime;
use cu29_helpers::basic_copper_setup;
use cu29_log_derive::debug;
use cu29_traits::{CuError, CuResult};
use std::path::PathBuf;

pub mod tasks {
    use cu29::clock::RobotClock;
    use cu29::config::ComponentConfig;
    use cu29::cutask::{CuMsg, CuSinkTask, CuSrcTask, CuTask, CuTaskLifecycle, Freezable};
    use cu29::{input_msg, output_msg};
    use cu29_traits::CuResult;

    pub struct ExampleSrc {}

    impl CuTaskLifecycle for ExampleSrc {
        fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self {})
        }
    }

    impl Freezable for ExampleSrc {}

    impl<'cl> CuSrcTask<'cl> for ExampleSrc {
        type Output = output_msg!('cl, i32);

        fn process(&mut self, _clock: &RobotClock, new_msg: Self::Output) -> CuResult<()> {
            new_msg.set_payload(42);
            Ok(())
        }
    }

    pub struct ExampleTask {}

    impl CuTaskLifecycle for ExampleTask {
        fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self {})
        }
    }

    impl Freezable for ExampleTask {}

    impl<'cl> CuTask<'cl> for ExampleTask {
        type Input = input_msg!('cl, i32);
        type Output = output_msg!('cl, i32);

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

    impl CuTaskLifecycle for ExampleSink {
        fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self {})
        }
    }

    impl Freezable for ExampleSink {}

    impl<'cl> CuSinkTask<'cl> for ExampleSink {
        type Input = input_msg!('cl, i32);

        fn process(&mut self, _clock: &RobotClock, _input: Self::Input) -> CuResult<()> {
            Ok(())
        }
    }
}

struct ExampleMonitor {
    tasks: &'static [&'static str],
}

#[copper_runtime(config = "copperconfig.ron")]
struct App {}

impl CuMonitor for ExampleMonitor {
    fn new(_config: Option<&ComponentConfig>, taskids: &'static [&str]) -> CuResult<Self> {
        debug!("Monitoring: created: {}", taskids);
        Ok(ExampleMonitor { tasks: taskids })
    }

    fn start(&mut self, clock: &_RobotClock) -> CuResult<()> {
        debug!("Monitoring: started: {}", clock.now());
        Ok(())
    }

    fn process_copperlist(&self, msgs: &[&CuMsgMetadata]) -> CuResult<()> {
        debug!("Monitoring: Processing copperlist...");
        for t in msgs.iter().enumerate() {
            let (taskid, metadata) = t;
            debug!("Task: {} -> {}", taskid, metadata);
        }
        Ok(())
    }

    fn process_error(&self, taskid: usize, step: CuTaskState, error: &CuError) -> Decision {
        debug!(
            "Monitoring: Processing error task: {} step: {} error: {}",
            self.tasks[taskid], step, error
        );
        Decision::Ignore
    }

    fn stop(&mut self, clock: &_RobotClock) -> CuResult<()> {
        debug!("Monitoring: stopped: {}", clock.now());
        Ok(())
    }
}

const SLAB_SIZE: Option<usize> = Some(1024 * 1024 * 1);
fn main() {
    let logger_path = "monitor.copper";
    let copper_ctx = basic_copper_setup(&PathBuf::from(logger_path), SLAB_SIZE, true)
        .expect("Failed to setup logger.");
    debug!("Logger created at {}.", path = logger_path);
    let clock = copper_ctx.clock;
    debug!("Creating application... ");
    let mut application = App::new(clock.clone(), copper_ctx.unified_logger.clone())
        .expect("Failed to create runtime.");
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
}
