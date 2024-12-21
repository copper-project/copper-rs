use cu29_helpers::basic_copper_setup;
use cu29_log::CuLogEntry;
use cu29_log::ANONYMOUS;
use cu29_log_derive::debug;
use cu29_value::to_value;

#[cfg(not(debug_assertions))]
use cu29_log_runtime::log;

#[cfg(debug_assertions)]
use cu29_log_runtime::log_debug_mode;

use serde::Serialize;
use tempdir::TempDir;

// Those should be in cu29_log_runtime but got moved here to avoid circular dependencies

#[test]
fn log_derive_end2end() {
    let tmp_dir = TempDir::new("teststructlog").expect("Failed to create temp dir");
    let log_path = tmp_dir.path().join("teststructlog.copper");

    let _ = basic_copper_setup(&log_path, None, true, None).expect("Failed to setup logger.");
    debug!("Logger created at {}.", log_path);

    #[derive(Serialize)]
    struct Test {
        a: i32,
        b: i32,
    }
    let mytuple = (1, "toto", 3.34f64, true, 'a');
    {
        let _gigantic_vec = vec![0u8; 1_000_000];
        debug!("Just a string {}", "zarma");
        debug!("anonymous param constants {} {}", 42u16, 43u8);
        debug!("named param constants {} {}", a = 3, b = 2);
        debug!("mixed named param constants, {} {} {}", a = 3, 54, b = 2);
        debug!("complex tuple", mytuple);
        debug!("Struct", Test { a: 3, b: 4 });
    }
}
