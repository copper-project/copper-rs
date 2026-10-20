#![cfg(feature = "std")]
use cu29::prelude::*;
use std::sync::atomic::{AtomicU8, Ordering};
static FAILURE: AtomicU8 = AtomicU8::new(0);
fn fail(operation: u8) -> CuResult<()> {
    if FAILURE.load(Ordering::Relaxed) == operation {
        Err(CuError::from("injected lifecycle failure"))
    } else {
        Ok(())
    }
}
#[derive(Reflect)]
struct Source;
impl Freezable for Source {}
impl CuSrcTask for Source {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(u32);
    fn new(_: Option<&ComponentConfig>, _: ()) -> CuResult<Self> {
        fail(1)?;
        Ok(Self)
    }
    fn start(&mut self, _: &CuContext) -> CuResult<()> {
        fail(2)
    }
    fn stop(&mut self, _: &CuContext) -> CuResult<()> {
        fail(3)
    }
    fn process(&mut self, _: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        assert_ne!(FAILURE.load(Ordering::Relaxed), 5, "injected process panic");
        fail(4)?;
        output.set_payload(42);
        Ok(())
    }
}
#[derive(Reflect)]
struct Sink;
impl Freezable for Sink {}
impl CuSinkTask for Sink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(u32);
    fn new(_: Option<&ComponentConfig>, _: ()) -> CuResult<Self> {
        Ok(Self)
    }
    fn process(&mut self, _: &CuContext, _: &Self::Input<'_>) -> CuResult<()> {
        Ok(())
    }
}
struct FatalMonitor;
impl CuMonitor for FatalMonitor {
    fn new(_: CuMonitoringMetadata, _: CuMonitoringRuntime) -> CuResult<Self> {
        Ok(Self)
    }
    fn process_copperlist(&self, _: &CuContext, _: CopperListView<'_>) -> CuResult<()> {
        Ok(())
    }
    fn process_error(&self, _: ComponentId, _: CuComponentState, _: &CuError) -> Decision {
        Decision::Shutdown
    }
}
#[copper_runtime(config = "tests/iteration_cleanup.ron")]
struct App {}
#[test]
fn failed_and_panicking_iterations_allow_shutdown_and_slot_reuse() {
    let dir = tempfile::tempdir().unwrap();
    for failure in [4, 5] {
        FAILURE.store(0, Ordering::Relaxed);
        let path = dir.path().join(format!("failure-{failure}.copper"));
        let mut app = App::builder()
            .with_log_path(&path, Some(65536))
            .unwrap()
            .build()
            .unwrap()
            .start()
            .unwrap();
        app.run_one_iteration().unwrap();
        FAILURE.store(failure, Ordering::Relaxed);
        if failure == 4 {
            assert!(app.run_one_iteration().is_err());
        } else {
            assert!(
                std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    app.run_one_iteration().unwrap();
                }))
                .is_err()
            );
        }
        FAILURE.store(0, Ordering::Relaxed);
        let stopped = app.stop().unwrap();
        let mut restarted = stopped.start().unwrap();
        restarted.run_one_iteration().unwrap();
        drop(restarted.stop().unwrap());
    }
}
