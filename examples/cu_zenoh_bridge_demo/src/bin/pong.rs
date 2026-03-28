use cu_zenoh_bridge_demo::parse_run_options;
use cu29::prelude::*;
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

#[copper_runtime(config = "multi_copper.ron", subsystem = "pong")]
struct PongApp {}

const SLAB_SIZE: Option<usize> = Some(64 * 1024 * 1024);

fn main() {
    if let Err(err) = drive() {
        eprintln!("zenoh-pong failed: {err}");
        std::process::exit(1);
    }
}

fn drive() -> CuResult<()> {
    let options = parse_run_options("zenoh_pong.copper")?;
    let mut app = PongApp::builder()
        .with_log_path(&options.log_path, SLAB_SIZE)?
        .with_instance_id(options.instance_id)
        .build()?;
    app.start_all_tasks()?;

    if let Some(iterations) = options.iterations {
        for _ in 0..iterations {
            app.run_one_iteration()?;
            std::thread::sleep(Duration::from_millis(200));
        }
        app.stop_all_tasks()?;
        Ok(())
    } else {
        loop {
            app.run_one_iteration()?;
            std::thread::sleep(Duration::from_millis(200));
        }
    }
}
