//! Lifecycle transitions consume the handle: reusing the pre-start handle
//! after `start` must fail to compile (use of moved value).

use cu29::prelude::*;

fn reuse_after_start<A: CuStdApplication>(app: CuStdAppLifecycle<A>) {
    let _running = app.start();
    let _ = app.start();
}

fn main() {}
