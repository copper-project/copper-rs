//! The planner under two refinement policies, in a full application.
//!
//! Both planner nodes are the same task type with the default RRT* config,
//! and both RNG resources carry the same seed, so they run the same tree.
//! Only the `anytime:` policy differs:
//!
//! | node | policy |
//! |---|---|
//! | `quick_planner` | `max_refines: 2`, `time_budget_ms: 50`, `quality_target: 0.93` |
//! | `thorough_planner` | `max_refines: 24`, `time_budget_ms: 250`, `max_stall: 4` |
//!
//! Because both nodes run job N on the same stream, the thorough tree is the
//! quick tree plus more iterations, so its path is never longer - that is the
//! anytime trade-off, measured. Keeping the planner config identical is what
//! makes the comparison valid.

use cu_rrt_star::{PlanPath, PlanRequest, Point2, World};
use cu29::prelude::*;
use std::sync::Mutex;

#[copper_runtime(config = "tests/copperconfig.ron")]
struct DualPolicyTester {}

/// Start and goal of every planning job.
const START: Point2 = Point2::new(0.5, 0.5);
const GOAL: Point2 = Point2::new(9.5, 9.5);

const SLAB_SIZE: Option<usize> = Some(16 * 1024 * 1024);
const ITERATIONS: usize = 10;

/// What the sink saw, one entry per copperlist.
static REPORTS: Mutex<Vec<PlanReport>> = Mutex::new(Vec::new());

/// Takes the reports collected so far, leaving the sink ready for a new run.
fn take_reports() -> Vec<PlanReport> {
    core::mem::take(&mut *REPORTS.lock().expect("reports poisoned"))
}

/// One copperlist as the sink saw it. A `None` path means the node published
/// nothing: no path yet, or a quality below the configured floor.
#[derive(Debug, Clone)]
struct PlanReport {
    quick: Option<PlanPath>,
    thorough: Option<PlanPath>,
}

/// Emits one planning problem per copperlist, always on the depot map.
#[derive(Default, Reflect)]
pub struct GoalSrc;

impl Freezable for GoalSrc {}

impl CuSrcTask for GoalSrc {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(PlanRequest);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, ctx: &CuContext, new_msg: &mut Self::Output<'_>) -> CuResult<()> {
        new_msg.set_payload(PlanRequest {
            world: World::depot(),
            start: START,
            goal: GOAL,
        });
        // A fresh Tov per job. An anytime node reads it as the age anchor when
        // its policy sets max_age_ms; neither planner here does, so both
        // anchor on their own job start instead.
        new_msg.tov = Tov::Time(ctx.clock.now());
        Ok(())
    }
}

/// Records what both planners published in the same copperlist.
#[derive(Default, Reflect)]
pub struct ComparisonSink;

impl Freezable for ComparisonSink {}

impl CuSinkTask for ComparisonSink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!('m, PlanPath, PlanPath);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, _ctx: &CuContext, input: &Self::Input<'_>) -> CuResult<()> {
        // Input order follows the cnx order in the RON: quick first.
        let (quick, thorough): (&CuMsg<PlanPath>, &CuMsg<PlanPath>) = *input;
        REPORTS.lock().expect("reports poisoned").push(PlanReport {
            quick: quick.payload().cloned(),
            thorough: thorough.payload().cloned(),
        });
        Ok(())
    }
}

/// Runs the whole application once and returns what the sink saw, one entry
/// per copperlist.
fn run(logger_path: &std::path::Path) -> Vec<PlanReport> {
    let mut application = DualPolicyTester::builder()
        .with_log_path(logger_path, SLAB_SIZE)
        .expect("Failed to setup logger.")
        .build()
        .expect("Failed to create application.");
    application
        .start_all_tasks()
        .expect("Failed to start application.");
    for _ in 0..ITERATIONS {
        application
            .run_one_iteration()
            .expect("Failed to run application.");
    }
    application
        .stop_all_tasks()
        .expect("Failed to stop application.");
    take_reports()
}

/// Checks the anytime contract on what the sink saw and returns how many jobs
/// both planners published.
fn check(reports: &[PlanReport]) -> usize {
    assert_eq!(reports.len(), ITERATIONS, "one report per copperlist");
    let world = World::depot();
    let lower_bound = START.distance(GOAL);
    let mut compared = 0;

    for (index, report) in reports.iter().enumerate() {
        for path in [&report.quick, &report.thorough].into_iter().flatten() {
            assert!(path.len >= 2, "job {index}: a path needs two waypoints");
            assert_eq!(path.waypoints[0], START, "job {index}");
            assert_eq!(path.waypoints[(path.len - 1) as usize], GOAL, "job {index}");
            // A published path must be drivable as published, whichever stop
            // point the policy picked.
            for pair in path.waypoints[..path.len as usize].windows(2) {
                assert!(
                    world.is_free_segment(pair[0], pair[1]),
                    "job {index}: published path crosses an obstacle"
                );
            }
            assert!(
                path.cost.raw() >= lower_bound,
                "job {index}: path shorter than the straight line"
            );
        }

        let (Some(quick), Some(thorough)) = (&report.quick, &report.thorough) else {
            continue;
        };
        compared += 1;
        // More quanta on the same stream can only shorten the path.
        assert!(
            thorough.cost.raw() <= quick.cost.raw() + 1e-3,
            "job {index}: the thorough policy published a longer path ({} vs {})",
            thorough.cost.raw(),
            quick.cost.raw()
        );
    }

    assert!(
        compared >= ITERATIONS / 2,
        "both planners published in only {compared} of {ITERATIONS} jobs"
    );
    compared
}

#[test]
fn dual_policy_refines_toward_shorter_paths() {
    let tmp_dir = tempfile::TempDir::new().expect("could not create a tmp dir");
    let reports = run(&tmp_dir.path().join("rrt_star_tester.copper"));
    check(&reports);
}
