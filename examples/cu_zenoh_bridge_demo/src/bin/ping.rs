use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;
use std::time::Duration;

pub mod bridges {
    pub use cu_zenoh_bridge_demo::bridges::*;
}

pub mod messages {
    pub use cu_zenoh_bridge_demo::messages::*;
}

pub mod tasks {
    pub use cu_zenoh_bridge_demo::tasks::*;
}

#[copper_runtime(config = "ping_config.ron")]
struct PingApp {}

const SLAB_SIZE: Option<usize> = Some(64 * 1024 * 1024);

fn main() {
    if let Err(err) = drive() {
        eprintln!("zenoh-ping failed: {err}");
        std::process::exit(1);
    }
}

fn drive() -> CuResult<()> {
    let tmp_dir = tempfile::TempDir::new().expect("could not create temp dir");
    let logger_path = tmp_dir.path().join("zenoh_ping.copper");
    let ctx = basic_copper_setup(&logger_path, SLAB_SIZE, true, None)?;

    let mut app = PingApp::new(ctx.clock.clone(), ctx.unified_logger.clone(), None)?;
    app.start_all_tasks()?;

    loop {
        app.run_one_iteration()?;
        std::thread::sleep(Duration::from_millis(200));
    }
}
