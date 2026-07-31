//! Anytime RRT*: the same planner under two refinement policies.
//!
//! `base()` grows the tree until it has a first, crude path; every `refine()`
//! runs one more block of RRT* iterations and republishes only when the path
//! got shorter. The task reports how good the path is; the RON `anytime:`
//! policy decides how long to keep going.
//!
//! Both planner nodes are the same task type with the same RRT* `config:`, so
//! they run the same tree from the same seed. Only the policy differs:
//!
//! | node | policy | meaning |
//! |---|---|---|
//! | `quick_planner` | `max_refines: 2`, `time_budget_ms: 50`, `quality_target: 0.85` | two quanta at most, and stop early once the path is within 15% of the straight line |
//! | `thorough_planner` | `max_refines: 24`, `time_budget_ms: 250`, `max_stall: 4` | up to 24 quanta, but give up after 4 that improved nothing |
//!
//! Both carry `quality_floor: 0.05`, which drops a job that found no path at
//! all: the sink then sees no payload.
//!
//! Because both nodes start from the same seed, the thorough tree is the quick
//! tree plus more iterations, so its path is never longer - that is the anytime
//! trade-off, measured. Keeping the two `config:` blocks identical is what
//! makes the comparison below valid.

mod rrt;
mod tasks;

use cu29::prelude::*;
use std::fs;
use std::path::Path;

#[copper_runtime(config = "copperconfig.ron")]
struct App {}

const SLAB_SIZE: Option<usize> = Some(16 * 1024 * 1024);
const ITERATIONS: usize = 10;

fn main() {
    let logger_path = "logs/anytime_rrt_star.copper";
    if let Some(parent) = Path::new(logger_path).parent()
        && !parent.exists()
    {
        fs::create_dir_all(parent).expect("Failed to create logs directory");
    }
    let mut application = App::builder()
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

    let reports = tasks::REPORTS.lock().expect("reports poisoned");
    assert_eq!(reports.len(), ITERATIONS, "one report per copperlist");

    let lower_bound = tasks::START.distance(tasks::GOAL);
    println!("straight line start -> goal: {lower_bound:.2} m (quality 1.0)");
    println!("{:<5} {:>28}   {:>28}", "job", "quick", "thorough");
    let mut compared = 0;
    for (index, report) in reports.iter().enumerate() {
        println!(
            "{:<5} {:>28}   {:>28}",
            index,
            describe(&report.quick, &report.quick_status, lower_bound),
            describe(&report.thorough, &report.thorough_status, lower_bound),
        );
        let (Some(quick), Some(thorough)) = (&report.quick, &report.thorough) else {
            continue;
        };
        compared += 1;
        // More quanta on the same seed can only shorten the path.
        assert!(
            thorough.cost <= quick.cost + 1e-3,
            "job {index}: the thorough policy published a longer path ({} vs {})",
            thorough.cost,
            quick.cost
        );
        assert!(
            thorough.cost >= lower_bound,
            "job {index}: path shorter than the straight line"
        );
        assert!(thorough.len >= 2, "job {index}: a path needs two waypoints");
    }
    assert!(
        compared >= ITERATIONS / 2,
        "both planners published in only {compared} of {ITERATIONS} jobs"
    );
    println!("anytime RRT* example OK: {compared}/{ITERATIONS} jobs compared");
}

/// One cell of the table: cost, quality and the runtime's anytime stamp.
fn describe(path: &Option<tasks::PlanPath>, status: &str, lower_bound: f32) -> String {
    match path {
        Some(path) => format!(
            "{:.2}m q={:.2} [{}]",
            path.cost,
            lower_bound / path.cost,
            status
        ),
        None => format!("no path [{status}]"),
    }
}
