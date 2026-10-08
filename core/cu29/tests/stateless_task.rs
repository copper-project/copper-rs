#![cfg(all(test, feature = "std"))]

use cu29::bincode::Decode;
use cu29::bincode::de::Decoder;
use cu29::bincode::enc::Encoder;
use cu29::bincode::error::{DecodeError, EncodeError};
use cu29::prelude::*;
use cu29_unifiedlog::{UnifiedLogger, UnifiedLoggerBuilder, UnifiedLoggerIOReader};
use std::sync::atomic::{AtomicU32, AtomicUsize, Ordering};

static PROCESS_CALLS: AtomicUsize = AtomicUsize::new(0);
static SINK_VALUE: AtomicU32 = AtomicU32::new(0);

#[derive(Reflect)]
struct Source {
    next: u32,
}

impl Freezable for Source {
    fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        self.next.encode(encoder)
    }

    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        self.next = Decode::decode(decoder)?;
        Ok(())
    }
}

impl CuSrcTask for Source {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(u32);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self { next: 21 })
    }

    fn process(&mut self, _ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        output.set_payload(self.next);
        self.next += 1;
        Ok(())
    }
}

#[derive(Reflect)]
struct Double;

impl CuStatelessTask for Double {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(u32);
    type Output<'m> = output_msg!(u32);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(
        &self,
        _ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        PROCESS_CALLS.fetch_add(1, Ordering::Relaxed);
        output.set_payload(input.payload().copied().unwrap_or_default() * 2);
        Ok(())
    }
}

#[derive(Reflect)]
struct Sink {
    sum: u32,
}

impl Freezable for Sink {
    fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        self.sum.encode(encoder)
    }

    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        self.sum = Decode::decode(decoder)?;
        Ok(())
    }
}

impl CuSinkTask for Sink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(u32);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self { sum: 0 })
    }

    fn process(&mut self, _ctx: &CuContext, input: &Self::Input<'_>) -> CuResult<()> {
        self.sum += input.payload().copied().unwrap_or_default();
        SINK_VALUE.store(self.sum, Ordering::Relaxed);
        Ok(())
    }
}

#[copper_runtime(config = "tests/stateless_task.ron")]
struct StatelessTaskApp {}

mod replay {
    use super::{Double, Sink, Source};
    use cu29::prelude::*;

    #[copper_runtime(config = "tests/stateless_task.ron", sim_mode = true)]
    struct StatelessReplayApp {}

    pub(super) fn restore_and_run(keyframe: &KeyFrame) -> CuResult<()> {
        let (clock, _mock) = RobotClock::mock();
        let mut callback = |_step: default::SimStep<'_>| SimOverride::ExecuteByRuntime;
        let mut app = StatelessReplayApp::builder()
            .with_clock(clock)
            .with_sim_callback(&mut callback)
            .build()?;
        app.restore_keyframe(keyframe)?;
        let mut running = app.start(&mut callback)?;
        running.run_one_iteration(&mut callback)?;
        running.stop(&mut callback)?;

        Ok(())
    }
}

#[test]
fn stateless_task_executes_and_preserves_keyframe_restore_order() -> CuResult<()> {
    PROCESS_CALLS.store(0, Ordering::Relaxed);
    SINK_VALUE.store(0, Ordering::Relaxed);

    let (clock, _mock) = RobotClock::mock();
    let log_dir = tempfile::tempdir_in(concat!(env!("CARGO_MANIFEST_DIR"), "/../../target"))
        .expect("create stateless task log directory");
    let log_path = log_dir.path().join("stateless.copper");
    let app = StatelessTaskApp::builder()
        .with_clock(clock)
        .with_log_path(&log_path, Some(1024 * 1024))?
        .build()?;
    let mut running = app.start()?;
    running.run_one_iteration()?;
    assert_eq!(SINK_VALUE.load(Ordering::Relaxed), 42);
    running.run_one_iteration()?;
    assert_eq!(SINK_VALUE.load(Ordering::Relaxed), 86);
    drop(running.stop()?);

    let UnifiedLogger::Read(logger) = UnifiedLoggerBuilder::new()
        .file_base_name(&log_path)
        .build()
        .expect("open stateless task log")
    else {
        panic!("expected a log reader");
    };
    let mut reader = UnifiedLoggerIOReader::new(logger, UnifiedLogType::FrozenTasks);
    let keyframe: KeyFrame =
        cu29::bincode::decode_from_std_read(&mut reader, cu29::bincode::config::standard())
            .expect("read first keyframe");

    replay::restore_and_run(&keyframe)?;

    assert_eq!(PROCESS_CALLS.load(Ordering::Relaxed), 3);
    assert_eq!(SINK_VALUE.load(Ordering::Relaxed), 42);
    Ok(())
}
