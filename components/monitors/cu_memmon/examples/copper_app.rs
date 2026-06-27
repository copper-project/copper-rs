//! Minimal end-to-end demo for `cu_memmon`.
//!
//! Build with `--features cu29/memory_monitoring` to install the counting
//! allocator and see real per-task allocation totals at shutdown; without the
//! feature, the monitor reports zeros and warns once at startup.
//!
//! The intermediate `Step` task deliberately allocates and immediately drops a
//! small Vec on every iteration so a leaking pattern is easy to spot at the
//! `stop` summary (alloc should equal dealloc -> "balanced").

use cu29::prelude::*;
use std::fs;
use std::path::PathBuf;
use std::time::Duration;

pub mod tasks {
    use cu29::prelude::*;

    #[derive(Reflect)]
    pub struct Src {
        counter: u32,
    }

    impl Freezable for Src {}

    impl CuSrcTask for Src {
        type Resources<'r> = ();
        type Output<'m> = output_msg!(u32);

        fn new(
            _config: Option<&ComponentConfig>,
            _resources: Self::Resources<'_>,
        ) -> CuResult<Self> {
            Ok(Self { counter: 0 })
        }

        fn process(&mut self, _ctx: &CuContext, new_msg: &mut Self::Output<'_>) -> CuResult<()> {
            self.counter = self.counter.wrapping_add(1);
            new_msg.set_payload(self.counter);
            Ok(())
        }
    }

    #[derive(Reflect)]
    pub struct Step;

    impl Freezable for Step {}

    impl CuTask for Step {
        type Resources<'r> = ();
        type Input<'m> = input_msg!(u32);
        type Output<'m> = output_msg!(u32);

        fn new(
            _config: Option<&ComponentConfig>,
            _resources: Self::Resources<'_>,
        ) -> CuResult<Self> {
            Ok(Self)
        }

        fn process(
            &mut self,
            _ctx: &CuContext,
            input: &Self::Input<'_>,
            output: &mut Self::Output<'_>,
        ) -> CuResult<()> {
            // Deliberately allocate-and-drop to exercise the monitor: the
            // lifetime alloc/dealloc totals should balance, but process()
            // shows nonzero per-call allocation.
            let scratch: Vec<u32> = (0..16).collect();
            let payload = input.payload().copied().unwrap_or(0) + scratch.iter().sum::<u32>();
            output.set_payload(payload);
            Ok(())
        }
    }

    #[derive(Reflect)]
    pub struct Sink;

    impl Freezable for Sink {}

    impl CuSinkTask for Sink {
        type Resources<'r> = ();
        type Input<'m> = input_msg!(u32);

        fn new(
            _config: Option<&ComponentConfig>,
            _resources: Self::Resources<'_>,
        ) -> CuResult<Self> {
            Ok(Self)
        }

        fn process(&mut self, _ctx: &CuContext, _input: &Self::Input<'_>) -> CuResult<()> {
            Ok(())
        }
    }
}

#[copper_runtime(config = "copperconfig.ron")]
struct App {}

const SLAB_SIZE: Option<usize> = Some(16 * 1024 * 1024);

fn main() {
    let logger_path =
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("logs/memmon_copper_app.copper");
    if let Some(parent) = logger_path.parent() {
        let _ = fs::create_dir_all(parent);
    }

    info!("Logger created at {}", &logger_path);

    let mut application = App::builder()
        .with_log_path(&logger_path, SLAB_SIZE)
        .expect("Failed to setup logger.")
        .build()
        .expect("Failed to create runtime");
    info!("Starting app at {}", application.clock().now());
    application
        .start_all_tasks()
        .expect("Failed to start application.");

    for _ in 0..60 {
        application
            .run_one_iteration()
            .expect("Failed to run one iteration.");
        std::thread::sleep(Duration::from_millis(25));
    }

    application
        .stop_all_tasks()
        .expect("Failed to stop application.");
    info!("App stopped at {}", application.clock().now());
}
