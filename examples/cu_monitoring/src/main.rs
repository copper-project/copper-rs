use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;
use std::path::PathBuf;

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

    pub struct ExampleTask {}

    impl Freezable for ExampleTask {}

    impl CuTask for ExampleTask {
        type Input<'m> = input_msg!(i32);
        type Output<'m> = output_msg!(i32);

        fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self {})
        }

        fn process(
            &mut self,
            _clock: &RobotClock,
            input: &Self::Input<'_>,
            output: &mut Self::Output<'_>,
        ) -> CuResult<()> {
            output.set_payload(input.payload().unwrap() + 1);
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

struct ExampleMonitor {
    tasks: &'static [&'static str],
}

#[copper_runtime(config = "copperconfig.ron")]
struct App {}

impl CuMonitor for ExampleMonitor {
    fn new(_config: &CuConfig, taskids: &'static [&str]) -> CuResult<Self> {
        debug!("Monitoring: created: {}", taskids);
        Ok(ExampleMonitor { tasks: taskids })
    }

    fn start(&mut self, clock: &RobotClock) -> CuResult<()> {
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

    fn stop(&mut self, clock: &RobotClock) -> CuResult<()> {
        debug!("Monitoring: stopped: {}", clock.now());
        Ok(())
    }
}
const SLAB_SIZE: Option<usize> = None;
fn main() {
    let logger_path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("logs/monitor.copper");

    let copper_ctx =
        basic_copper_setup(&logger_path, SLAB_SIZE, true, None).expect("Failed to setup logger.");
    debug!("Logger created at {}.", path = logger_path);
    debug!("Creating application... ");
    let mut application = AppBuilder::new()
        .with_context(&copper_ctx)
        .build()
        .expect("Failed to create runtime");

    let clock = copper_ctx.clock;
    debug!("Running... starting clock: {}.", clock.now());
    application
        .start_all_tasks()
        .expect("Failed to start application.");
    application.run().expect("Failed to run application.");
    application
        .stop_all_tasks()
        .expect("Failed to stop application.");
    debug!("End of program: {}.", clock.now());
}
