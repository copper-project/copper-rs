//! The legal lifecycle flows must compile: start/iterate/stop, restart from
//! `Stopped` (mission chaining), full `run_until_shutdown`, and cleanup from
//! `Faulted`.

use cu29::prelude::*;

fn start_iterate_stop_restart<A: CuStdApplication>(app: CuStdAppLifecycle<A>) -> CuResult<()> {
    let mut running = app.start()?;
    running.run_one_iteration()?;
    let stopped = running.stop()?;

    // Restarting a stopped application is legal.
    let running = stopped.start()?;
    running.stop()?;
    Ok(())
}

fn run_full_lifecycle<A: CuStdApplication>(app: CuStdAppLifecycle<A>) -> CuResult<()> {
    let stopped = app.run_until_shutdown()?;
    // A stopped application can run again.
    stopped.run_until_shutdown()?;
    Ok(())
}

fn cleanup_after_failed_start<A: CuStdApplication>(app: CuStdAppLifecycle<A>) {
    if let Err(failed) = app.start() {
        // The error hands the application back, typed Faulted: cleanup is
        // still possible and nothing is lost.
        let _ = failed.app.stop();
    }
}
