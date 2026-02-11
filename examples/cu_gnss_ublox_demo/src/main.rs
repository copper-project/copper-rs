pub mod tasks;

use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;
use std::thread;
use std::time::Duration;

#[copper_runtime(config = "copperconfig.ron")]
struct CuGnssUbloxDemo {}

fn main() {
    if let Err(err) = run() {
        error!("cu-gnss-ublox-demo failed: {}", err);
        std::process::exit(1);
    }
}

fn run() -> CuResult<()> {
    let log_dir = tempfile::TempDir::new()
        .map_err(|e| CuError::new_with_cause("failed to create temporary log directory", e))?;
    let log_path = log_dir.path().join("cu_gnss_ublox_demo.copper");
    let ctx = basic_copper_setup(&log_path, Some(16 * 1024 * 1024), true, None)?;

    let mut app = CuGnssUbloxDemoBuilder::new().with_context(&ctx).build()?;
    app.start_all_tasks()?;

    let target_events = 20_u64;
    let timeout_ns = CuDuration::from_secs(20).as_nanos();
    let start_ns = ctx.clock.now().as_nanos();

    while tasks::state::total_events() < target_events {
        app.run_one_iteration()?;

        let elapsed_ns = ctx.clock.now().as_nanos().saturating_sub(start_ns);
        if elapsed_ns >= timeout_ns {
            break;
        }

        thread::sleep(Duration::from_millis(5));
    }

    app.stop_all_tasks()?;

    let summary = tasks::state::summary();
    println!("[gnss] final {}", summary);

    if tasks::state::total_events() == 0 {
        return Err(CuError::from(
            "no GNSS events were received; check serial mapping and receiver output",
        ));
    }

    Ok(())
}
