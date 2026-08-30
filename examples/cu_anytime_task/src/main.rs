use cu29::prelude::*;
use std::fs;
use std::path::Path;

pub mod tasks {
    use cu29::cutask_anytime::{AnytimeStatus, CuAnytimeTask, Quality, quality_from_f32};
    use cu29::prelude::*;
    use std::sync::Mutex;

    /// What the sink observed, for the assertions in `main`.
    pub static RECORDED: Mutex<Vec<(u32, String)>> = Mutex::new(Vec::new());

    #[derive(Reflect)]
    pub struct TargetSrc;

    impl Freezable for TargetSrc {}

    impl CuSrcTask for TargetSrc {
        type Resources<'r> = ();
        type Output<'m> = output_msg!(u32);

        fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self)
        }

        fn process(&mut self, ctx: &CuContext, new_msg: &mut Self::Output<'_>) -> CuResult<()> {
            new_msg.set_payload(5);
            // A fresh Tov: the planner's max_age anchors on it.
            new_msg.tov = Tov::Time(ctx.clock.now());
            Ok(())
        }
    }

    /// Anytime node with a measurable quality: `base()` publishes 0 and each
    /// `refine()` commits one more increment toward the input target, so the
    /// published quality climbs to 1.0 and the configured quality_target
    /// stops refinement early.
    #[derive(Reflect)]
    pub struct IncrementalPlanner {
        target: u32,
        acc: u32,
    }

    impl Freezable for IncrementalPlanner {}

    impl CuAnytimeTask for IncrementalPlanner {
        type Input<'m> = input_msg!(u32);
        type Output<'m> = output_msg!(u32);
        type Resources<'r> = ();
        type Quality = Quality;

        fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self { target: 0, acc: 0 })
        }

        fn base(
            &mut self,
            _ctx: &CuContext,
            input: &Self::Input<'_>,
            output: &mut Self::Output<'_>,
        ) -> CuResult<AnytimeStatus<Quality>> {
            self.target = input.payload().copied().ok_or("planner: no input")?;
            self.acc = 0;
            output.set_payload(self.acc);
            Ok(AnytimeStatus::Improved(quality_from_f32(0.0)))
        }

        fn refine(
            &mut self,
            _ctx: &CuContext,
            output: &mut Self::Output<'_>,
        ) -> CuResult<AnytimeStatus<Quality>> {
            if self.acc >= self.target {
                return Ok(AnytimeStatus::Converged(quality_from_f32(1.0)));
            }
            self.acc += 1;
            output.set_payload(self.acc);
            Ok(AnytimeStatus::Improved(quality_from_f32(
                self.acc as f32 / self.target as f32,
            )))
        }
    }

    /// Quality-less anytime node: every quantum bumps the output, so it runs
    /// its whole emitted refine budget and stops by position (MaxRefines).
    #[derive(Reflect)]
    pub struct CountingSmoother;

    impl Freezable for CountingSmoother {}

    impl CuAnytimeTask for CountingSmoother {
        type Input<'m> = input_msg!(u32);
        type Output<'m> = output_msg!(u32);
        type Resources<'r> = ();
        type Quality = ();

        fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self)
        }

        fn base(
            &mut self,
            _ctx: &CuContext,
            input: &Self::Input<'_>,
            output: &mut Self::Output<'_>,
        ) -> CuResult<AnytimeStatus<()>> {
            output.set_payload(input.payload().copied().ok_or("smoother: no input")?);
            Ok(AnytimeStatus::Improved(()))
        }

        fn refine(
            &mut self,
            _ctx: &CuContext,
            output: &mut Self::Output<'_>,
        ) -> CuResult<AnytimeStatus<()>> {
            let bumped = output.payload().copied().unwrap_or(0) + 1;
            output.set_payload(bumped);
            Ok(AnytimeStatus::Improved(()))
        }
    }

    /// What the background chain published, when a job landed.
    pub static BACKGROUND_RECORDED: Mutex<Vec<(u32, String)>> = Mutex::new(Vec::new());

    #[derive(Reflect)]
    pub struct RecordingSink;

    impl Freezable for RecordingSink {}

    impl CuSinkTask for RecordingSink {
        type Resources<'r> = ();
        type Input<'m> = input_msg!(u32);

        fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self)
        }

        fn process(&mut self, _ctx: &CuContext, input: &Self::Input<'_>) -> CuResult<()> {
            let payload = input.payload().copied().ok_or("recorder: no input")?;
            let status = input.metadata.status_txt.0.to_string();
            RECORDED
                .lock()
                .expect("recorder poisoned")
                .push((payload, status));
            Ok(())
        }
    }
    /// Sink of the `tracker` node, which runs the same anytime task as
    /// `planner` but with `background: true`: most copperlists carry no payload
    /// because the job is still running on its worker thread.
    #[derive(Reflect)]
    pub struct BackgroundSink;

    impl Freezable for BackgroundSink {}

    impl CuSinkTask for BackgroundSink {
        type Resources<'r> = ();
        type Input<'m> = input_msg!(u32);

        fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self)
        }

        fn process(&mut self, _ctx: &CuContext, input: &Self::Input<'_>) -> CuResult<()> {
            if let Some(payload) = input.payload().copied() {
                let status = input.metadata.status_txt.0.to_string();
                BACKGROUND_RECORDED
                    .lock()
                    .expect("recorder poisoned")
                    .push((payload, status));
            }
            Ok(())
        }
    }
}

#[copper_runtime(config = "copperconfig.ron")]
struct App {}

const SLAB_SIZE: Option<usize> = Some(16 * 1024 * 1024);

fn main() {
    let logger_path = "logs/anytime.copper";
    if let Some(parent) = Path::new(logger_path).parent()
        && !parent.exists()
    {
        fs::create_dir_all(parent).expect("Failed to create logs directory");
    }
    let application = App::builder()
        .with_log_path(logger_path, SLAB_SIZE)
        .expect("Failed to setup logger.")
        .build()
        .expect("Failed to create application.");
    let mut running = application.start().expect("Failed to start application.");
    // Three copperlists for the foreground chain; keep polling until the
    // backgrounded node's first job comes back from its worker thread.
    for iteration in 0..100 {
        running
            .run_one_iteration()
            .expect("Failed to run application.");
        let landed = !tasks::BACKGROUND_RECORDED
            .lock()
            .expect("recorder poisoned")
            .is_empty();
        if iteration >= 2 && landed {
            break;
        }
    }
    running.stop().expect("Failed to stop application.");

    let background = tasks::BACKGROUND_RECORDED
        .lock()
        .expect("recorder poisoned");
    // The worker ran the whole job: five quanta reach the target of 5 and the
    // check before the sixth sees quality 1.0 >= quality_target.
    assert_eq!(
        background
            .first()
            .map(|(payload, status)| (*payload, status.as_str())),
        Some((5, "any:5it q=1.00 tgt")),
        "background anytime job did not land"
    );
    println!("background anytime OK: {:?}", background.as_slice());
    drop(background);

    let recorded = tasks::RECORDED.lock().expect("recorder poisoned");
    assert!(recorded.len() >= 3, "one recorded value per copperlist");
    for (payload, status) in recorded.iter() {
        // planner: base publishes 0, five quanta reach the target of 5, and
        // quality 1.0 >= quality_target stops it before its 8-quantum budget.
        // smoother: +1 per quantum for its whole 2-quantum budget.
        assert_eq!(*payload, 7, "planner result (5) + smoother budget (2)");
        // The smoother's status stamp: 2 quanta, stopped by position.
        assert_eq!(status, "any:2it max");
    }
    println!("anytime example OK: {:?}", recorded.as_slice());
}
