use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;
use std::sync::atomic::{AtomicBool, AtomicI32, Ordering};

static LAST_I32: AtomicI32 = AtomicI32::new(0);
static LAST_BOOL: AtomicBool = AtomicBool::new(false);

pub mod tasks {
    use super::{LAST_BOOL, LAST_I32};
    use cu29::prelude::*;
    use std::sync::atomic::Ordering;

    pub struct MultiOutSrc;

    impl Freezable for MultiOutSrc {}

    impl CuSrcTask for MultiOutSrc {
        type Resources<'r> = ();
        type Output<'m> = output_msg!(i32, bool);

        fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self)
        }

        fn process(&mut self, clock: &RobotClock, output: &mut Self::Output<'_>) -> CuResult<()> {
            output.0.tov = Tov::Time(clock.now());
            output.0.set_payload(42);
            output.1.set_payload(true);
            Ok(())
        }
    }

    pub struct IntSink;

    impl Freezable for IntSink {}

    impl CuSinkTask for IntSink {
        type Resources<'r> = ();
        type Input<'m> = input_msg!(i32);

        fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self)
        }

        fn process(&mut self, _clock: &RobotClock, input: &Self::Input<'_>) -> CuResult<()> {
            if let Some(value) = input.payload() {
                LAST_I32.store(*value, Ordering::SeqCst);
            }
            Ok(())
        }
    }

    pub struct BoolSink;

    impl Freezable for BoolSink {}

    impl CuSinkTask for BoolSink {
        type Resources<'r> = ();
        type Input<'m> = input_msg!(bool);

        fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self)
        }

        fn process(&mut self, _clock: &RobotClock, input: &Self::Input<'_>) -> CuResult<()> {
            if let Some(value) = input.payload() {
                LAST_BOOL.store(*value, Ordering::SeqCst);
            }
            Ok(())
        }
    }

    pub struct BothSink;

    impl Freezable for BothSink {}

    impl CuSinkTask for BothSink {
        type Resources<'r> = ();
        type Input<'m> = input_msg!('m, i32, bool);

        fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self)
        }

        fn process(&mut self, _clock: &RobotClock, input: &Self::Input<'_>) -> CuResult<()> {
            let (int_msg, bool_msg): (&CuMsg<i32>, &CuMsg<bool>) = *input;
            if let Some(value) = int_msg.payload() {
                LAST_I32.store(*value, Ordering::SeqCst);
            }
            if let Some(value) = bool_msg.payload() {
                LAST_BOOL.store(*value, Ordering::SeqCst);
            }
            Ok(())
        }
    }

    pub struct DropBoolSink;

    impl Freezable for DropBoolSink {}

    impl CuSinkTask for DropBoolSink {
        type Resources<'r> = ();
        type Input<'m> = input_msg!(bool);

        fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self)
        }

        fn process(&mut self, _clock: &RobotClock, input: &Self::Input<'_>) -> CuResult<()> {
            let _ = input.payload();
            Ok(())
        }
    }

    pub struct LoopbackTask;

    impl Freezable for LoopbackTask {}

    impl CuTask for LoopbackTask {
        type Resources<'r> = ();
        type Input<'m> = input_msg!(i32);
        type Output<'m> = output_msg!(i32);

        fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self)
        }

        fn process(
            &mut self,
            _clock: &RobotClock,
            input: &Self::Input<'_>,
            output: &mut Self::Output<'_>,
        ) -> CuResult<()> {
            if let Some(value) = input.payload() {
                output.set_payload(*value);
            } else {
                output.clear_payload();
            }
            Ok(())
        }
    }
}

#[copper_runtime(config = "copperconfig.ron")]
struct App {}

fn main() {
    let tmp_dir = tempfile::TempDir::new().expect("Could not create temporary directory");
    let logger_path = tmp_dir.path().join("logger.copper");
    let copper_ctx =
        basic_copper_setup(&logger_path, None, true, None).expect("Failed to setup logger.");
    let clock = copper_ctx.clock.clone();

    let mut application = App::new(clock.clone(), copper_ctx.unified_logger.clone(), None)
        .expect("Failed to create application.");
    application
        .start_all_tasks()
        .expect("Failed to start application.");
    application
        .run_one_iteration()
        .expect("Failed to run application.");
    application
        .stop_all_tasks()
        .expect("Failed to stop application.");

    assert_eq!(LAST_I32.load(Ordering::SeqCst), 42);
    assert!(LAST_BOOL.load(Ordering::SeqCst));
}
