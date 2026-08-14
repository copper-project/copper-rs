//! Lifecycle transitions consume the handle: reusing the pre-start handle
//! after `start_all_tasks` must fail to compile (use of moved value).

use cu29::prelude::*;

fn reuse_after_start<A: CuStdApplication>(app: CuStdAppLifecycle<A>) {
    let _running = app.start_all_tasks();
    let _ = app.start_all_tasks();
}

fn main() {}
