//! Closed-loop navigation on the `cu-rrt-star` anytime planner, visualized
//! with Rerun.
//!
//! A simulated point robot patrols the corners of the depot map. Every
//! copperlist, `tasks::NavSim` advances the robot along the newest published
//! path and asks for a fresh plan from where the robot is now; the planner
//! refines each plan for as long as its `anytime:` policy allows; and
//! `tasks::RerunViewer` draws the world, the robot, its trail, the current
//! path and the per-cycle quality and iteration curves.
//!
//! The viewer spawns a local Rerun instance, so install it first:
//! `cargo install rerun-cli --locked` (or `pip install rerun-sdk`). Then:
//! `cargo run --release -p cu-anytime-rrt-star`, and stop with Ctrl-C.
//! To record to a file instead, set `"rrd": "demo.rrd"` in the viewer's
//! `config:` block.
//!
//! The quick-vs-thorough policy comparison this example used to carry lives
//! in `components/tasks/cu_rrt_star/tests/`.

mod tasks;

use cu29::prelude::*;
use std::fs;
use std::path::Path;

#[copper_runtime(config = "copperconfig.ron")]
struct App {}

const SLAB_SIZE: Option<usize> = Some(16 * 1024 * 1024);

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
    // Paced by `runtime.rate_target_hz` in the RON; stops on Ctrl-C.
    if let Err(error) = application.run() {
        eprintln!("Error while running: {error}");
    }
}
