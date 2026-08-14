//! Starting an application that is already `Running` must fail to compile:
//! `start_all_tasks` is only available from the `Initialized` and `Stopped`
//! typestates (see the `Startable` on_unimplemented note naming the fix).

use cu29::prelude::*;

fn double_start<A: CuStdApplication>(app: CuStdAppLifecycle<A, Running>) {
    let _ = app.start_all_tasks();
}

fn main() {}
