#![cfg(all(test, feature = "std"))]

use cu29::prelude::*;
use std::sync::atomic::{AtomicU32, AtomicUsize, Ordering};

static PROCESS_CALLS: AtomicUsize = AtomicUsize::new(0);
static SINK_VALUE: AtomicU32 = AtomicU32::new(0);

#[derive(Reflect)]
struct Source;

impl Freezable for Source {}

impl CuSrcTask for Source {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(u32);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, _ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        output.set_payload(21);
        Ok(())
    }
}

#[derive(Reflect)]
struct Double;

impl Freezable for Double {}

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
struct Sink;

impl Freezable for Sink {}

impl CuSinkTask for Sink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(u32);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, _ctx: &CuContext, input: &Self::Input<'_>) -> CuResult<()> {
        SINK_VALUE.store(
            input.payload().copied().unwrap_or_default(),
            Ordering::Relaxed,
        );
        Ok(())
    }
}

#[copper_runtime(config = "tests/stateless_task.ron")]
struct StatelessTaskApp {}

#[test]
fn stateless_task_executes_in_the_configured_runtime() -> CuResult<()> {
    PROCESS_CALLS.store(0, Ordering::Relaxed);
    SINK_VALUE.store(0, Ordering::Relaxed);

    let (clock, _mock) = RobotClock::mock();
    let app = StatelessTaskApp::builder().with_clock(clock).build()?;
    let mut running = app.start()?;
    running.run_one_iteration()?;
    running.stop()?;

    assert_eq!(PROCESS_CALLS.load(Ordering::Relaxed), 1);
    assert_eq!(SINK_VALUE.load(Ordering::Relaxed), 42);
    Ok(())
}
