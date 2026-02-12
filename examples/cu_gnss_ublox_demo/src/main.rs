pub mod tasks;

use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};
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

    let running = Arc::new(AtomicBool::new(true));
    let running_for_signal = Arc::clone(&running);
    ctrlc::set_handler(move || {
        running_for_signal.store(false, Ordering::Relaxed);
    })
    .map_err(|e| CuError::new_with_cause("failed to install Ctrl-C handler", e))?;

    info!("GNSS demo started. Press Ctrl+C to stop.");
    info!(
        "This demo logs message counters and GNSS values; `sats_in_view` is a per-epoch count that can move up/down as tracking changes."
    );
    let heartbeat_period_ns = CuDuration::from_secs(5).as_nanos();
    let mut next_heartbeat_ns = ctx.clock.now().as_nanos().saturating_add(heartbeat_period_ns);

    let mut run_error: Option<CuError> = None;
    while running.load(Ordering::Relaxed) {
        if let Err(err) = app.run_one_iteration() {
            run_error = Some(err);
            break;
        }

        let now_ns = ctx.clock.now().as_nanos();
        if now_ns >= next_heartbeat_ns {
            info!("[gnss/heartbeat] {}", tasks::state::summary());
            next_heartbeat_ns = now_ns.saturating_add(heartbeat_period_ns);
        }

        thread::sleep(Duration::from_millis(5));
    }

    app.stop_all_tasks()?;

    if let Some(err) = run_error {
        return Err(err);
    }

    info!("[gnss/final] {}", tasks::state::summary());

    if tasks::state::total_events() == 0 {
        return Err(CuError::from(
            "no GNSS events were received; check serial mapping and receiver output",
        ));
    }

    Ok(())
}
