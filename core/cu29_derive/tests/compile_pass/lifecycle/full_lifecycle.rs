//! The legal lifecycle flows must compile: start/iterate/stop, restart from
//! `Stopped` (mission chaining), full `run`, and cleanup from `Faulted`.

use cu29::prelude::*;

fn start_iterate_stop_restart<A: CuStdApplication>(app: CuStdAppLifecycle<A>) -> CuResult<()> {
    let mut running = app.start_all_tasks()?;
    running.run_one_iteration()?;
    let stopped = running.stop_all_tasks()?;

    // Restarting a stopped application is legal.
    let running = stopped.start_all_tasks()?;
    running.stop_all_tasks()?;
    Ok(())
}

fn run_full_lifecycle<A: CuStdApplication>(app: CuStdAppLifecycle<A>) -> CuResult<()> {
    let stopped = app.run()?;
    // A stopped application can run again.
    stopped.run()?;
    Ok(())
}

fn cleanup_after_failed_start<A: CuStdApplication>(app: CuStdAppLifecycle<A>) {
    if let Err(failed) = app.start_all_tasks() {
        // The error hands the application back, typed Faulted: cleanup is
        // still possible and nothing is lost.
        let _ = failed.app.stop_all_tasks();
    }
}
