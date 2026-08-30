//! Pre-typestate code keeps its shape: the raw (deprecated) lifecycle API is
//! still callable on the `Initialized` handle that `build()` returns, so
//! legacy programs compile unchanged and only pick up deprecation warnings.

use cu29::prelude::*;

#[allow(deprecated)]
fn legacy_flow<A: CuStdApplication>(mut app: CuStdAppLifecycle<A>) -> CuResult<()> {
    app.start_all_tasks()?;
    app.run_one_iteration()?;
    app.stop_all_tasks()?;
    Ok(())
}

fn main() {}
