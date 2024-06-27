use copper::monitoring::ScopedAllocCounter;
use copper_helpers::basic_copper_setup;
use copper_log_derive::debug;
use serde::Serialize;
use std::path::PathBuf;

fn main() {
    let _ = basic_copper_setup(&PathBuf::from("/tmp/teststructlog.copper"), true)
        .expect("Failed to setup logger.");
    debug!("Logger created.");

    #[derive(Serialize)]
    struct Test {
        a: i32,
        b: i32,
    }
    let mytuple = (1, "toto", 3.34f64, true, 'a');
    {
        let hop = ScopedAllocCounter::new();
        let gigantic_vec = vec![0u8; 1_000_000];
        debug!("Just a string {}", "zarma");
        debug!("anonymous param constants {} {}", 42u16, 43u8);
        debug!("named param constants {} {}", a = 3, b = 2);
        debug!("mixed named param constants, {} {} {}", a = 3, 54, b = 2);
        debug!("complex tuple", mytuple);
        debug!("Struct", Test { a: 3, b: 4 });
    }
}
