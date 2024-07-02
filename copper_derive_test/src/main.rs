use copper::clock::RobotClock;
use copper_derive::copper_runtime;
use copper_log_derive::debug;
use copper_unifiedlog::{UnifiedLogger, UnifiedLoggerBuilder};
use std::path::Path;
use std::sync::{Arc, Mutex};

#[copper_runtime(config = "copperconfig.ron")]
struct MyApplication {}

fn main() {
    let UnifiedLogger::Write(logger) = UnifiedLoggerBuilder::new()
        .write(true)
        .create(true)
        .file_path(Path::new("/tmp/test.copper".into()))
        .preallocated_size(100000)
        .build()
        .expect("Failed to create logger")
    else {
        panic!("Failed to create logger")
    };
    let unified_logger = Arc::new(Mutex::new(logger));

    debug!("Application created.");
    let _application = MyApplication::new(RobotClock::default(), unified_logger)
        .expect("Failed to create runtime.");
    debug!("End of program.");
}
