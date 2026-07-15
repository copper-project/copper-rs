use cu29::prelude::*;

mod autonomy_bridge;
mod compute_tasks;
mod messages;

mod tasks {
    pub use crate::compute_tasks::*;
}

#[copper_runtime(config = "multi_copper_udp.ron", subsystem = "compute")]
struct ComputeApp {}

const LOG_SLAB_SIZE: Option<usize> = Some(64 * 1024 * 1024);

fn main() {
    if let Err(err) = drive() {
        eprintln!("quad-compute failed: {err}");
        std::process::exit(1);
    }
}

fn drive() -> CuResult<()> {
    let mut app = ComputeApp::builder()
        .with_log_path("logs/compute.copper", LOG_SLAB_SIZE)?
        .build()?;
    app.run()
}
