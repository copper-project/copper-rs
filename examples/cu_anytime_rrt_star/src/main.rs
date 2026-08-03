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
//! | `quick_planner` | `max_refines: 2`, `time_budget_ms: 50`, `quality_target: 0.93` | two quanta at most, and stop early once the path is within 7% of the straight line |
//! | `thorough_planner` | `max_refines: 24`, `time_budget_ms: 250`, `max_stall: 4` | up to 24 quanta, but give up after 4 that improved nothing |
//!
//! Both carry `quality_floor: 0.05`, which drops a job that found no path at
//! all: the sink then sees no payload.
//!
//! Because both nodes start from the same seed, the thorough tree is the quick
//! tree plus more iterations, so its path is never longer - that is the anytime
//! trade-off, measured. Keeping the two `config:` blocks identical is what
//! makes the comparison below valid.
//!
//! The RRT* `gamma` is `0.0` in the RON, which asks the planner to derive the
//! rewiring radius constant from the map. Pinning it to an arbitrary smaller
//! number is what makes an RRT* implementation quietly stop converging.

mod rrt;
mod tasks;

use cu29::prelude::*;
use std::fs;
use std::path::Path;

#[copper_runtime(config = "copperconfig.ron")]
struct App {}

const SLAB_SIZE: Option<usize> = Some(16 * 1024 * 1024);
const ITERATIONS: usize = 10;

/// Runs the whole application once and returns what the sink saw, one entry
/// per copperlist.
fn run(logger_path: &str) -> Vec<tasks::PlanReport> {
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
    tasks::take_reports()
}

/// Checks the anytime contract on what the sink saw and returns how many jobs
/// both planners published.
fn check(reports: &[tasks::PlanReport]) -> usize {
    assert_eq!(reports.len(), ITERATIONS, "one report per copperlist");
    let world = rrt::World::depot();
    let lower_bound = tasks::START.distance(tasks::GOAL);
    let mut compared = 0;

    for (index, report) in reports.iter().enumerate() {
        for path in [&report.quick, &report.thorough].into_iter().flatten() {
            assert!(path.len >= 2, "job {index}: a path needs two waypoints");
            assert_eq!(path.waypoints[0], tasks::START, "job {index}");
            assert_eq!(
                path.waypoints[(path.len - 1) as usize],
                tasks::GOAL,
                "job {index}"
            );
            // A published path must be drivable as published, whichever stop
            // point the policy picked.
            for pair in path.waypoints[..path.len as usize].windows(2) {
                assert!(
                    world.is_free_segment(pair[0], pair[1]),
                    "job {index}: published path crosses an obstacle"
                );
            }
            assert!(
                path.cost >= lower_bound,
                "job {index}: path shorter than the straight line"
            );
        }

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
    }

    assert!(
        compared >= ITERATIONS / 2,
        "both planners published in only {compared} of {ITERATIONS} jobs"
    );
    compared
}

fn main() {
    let reports = run("logs/anytime_rrt_star.copper");

    let lower_bound = tasks::START.distance(tasks::GOAL);
    println!("straight line start -> goal: {lower_bound:.2} m (quality 1.0)");
    println!("{:<5} {:>28}   {:>28}", "job", "quick", "thorough");
    for (index, report) in reports.iter().enumerate() {
        println!(
            "{:<5} {:>28}   {:>28}",
            index,
            describe(&report.quick, &report.quick_status, lower_bound),
            describe(&report.thorough, &report.thorough_status, lower_bound),
        );
    }

    let compared = check(&reports);
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

#[cfg(test)]
mod tests {
    use super::*;

    /// The self-check of `main`, as a test: CI only builds the examples, so
    /// without this nothing ever runs the anytime path.
    #[test]
    fn both_policies_publish_drivable_paths() {
        let logger_path = std::env::temp_dir().join("cu_anytime_rrt_star_test.copper");
        let reports = run(logger_path.to_str().expect("non-utf8 temp dir"));
        check(&reports);
    }
}
