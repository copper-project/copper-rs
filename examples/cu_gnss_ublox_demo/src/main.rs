pub mod tasks;

use cu29::prelude::*;

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

    let app = CuGnssUbloxDemo::builder()
        .with_log_path(&log_path, Some(16 * 1024 * 1024))?
        .build()?;
    let clock = app.clock();
    let mut app = app.start()?;

    info!("GNSS demo started. Press Ctrl+C to stop.");
    info!(
        "This demo logs message counters and GNSS values; `sats_in_view` is a per-epoch count that can move up/down as tracking changes."
    );
    let stopped = loop {
        let next_heartbeat = clock.raw_now() + CuDuration::from_secs(5);
        match app.run_until(|| clock.raw_now() >= next_heartbeat) {
            Ok(CuRunExit::Requested) => info!("[gnss/heartbeat] {}", tasks::state::summary()),
            result => {
                let stopped = app.stop()?;
                result?;
                break stopped;
            }
        }
    };

    info!("[gnss/final] {}", tasks::state::summary());

    if tasks::state::total_events() == 0 {
        return Err(CuError::from(
            "no GNSS events were received; check serial mapping and receiver output",
        ));
    }

    stopped.into_inner().log_shutdown_completed()
}
