use cu29::config::ComponentConfig;
use cu29::cutask::CuMsgMetadata;
use cu29::monitoring::{CuMonitor, CuTaskState, Decision};
use cu29_derive::copper_runtime;
use cu29_log_derive::debug;
use cu29_traits::{CuError, CuResult};

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
        Ok(ExampleMonitor { tasks: taskids })
    }

    fn start(&mut self, _clock: &_RobotClock) -> CuResult<()> {
        Ok(())
    }

    fn process_copperlist(&self, msgs: &[&CuMsgMetadata]) -> CuResult<()> {
        for t in msgs.iter().enumerate() {
            let (taskid, metadata) = t;
            println!("Task: {} -> {}", taskid, metadata);
        }
        Ok(())
    }

    fn process_error(&self, taskid: usize, step: CuTaskState, error: CuError) -> Decision {
        println!(
            "Task: {} step: {:?} error: {}",
            self.tasks[taskid], step, error
        );
        Decision::ContinueWithNoOuput
    }

    fn stop(&mut self) -> CuResult<()> {
        println!("Stopped");
        Ok(())
    }
}

fn main() {
    println!("Hello, world!");
}
