//! Waiting for output does not grant mutable access to a stopped application.

use cu29::prelude::*;

fn wait_stopped<A: CuStdApplication + CuApplicationOutput>(
    mut app: CuStdAppLifecycle<A, Stopped>,
) {
    app.wait_for_output().unwrap(); //~ E0596
}

fn main() {}
