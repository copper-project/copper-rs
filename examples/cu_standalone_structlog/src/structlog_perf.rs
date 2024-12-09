use cu29::prelude::*;
use cu29_clock::{CuTime, RobotClock};
use std::path::PathBuf;

const LOG_FILE: &str = "./logfile.bin";

fn main() {
    let clock = RobotClock::new();
    let bf = {
        let writer = SimpleFileWriter::new(&PathBuf::from(LOG_FILE)).unwrap();
        let _log_runtime = LoggerRuntime::init(clock.clone(), writer, None::<NullLog>);
        let bf: CuTime = clock.now();
        for i in 0..1_000_000 {
            debug!(
                "This is the logline {} associated with the log Logging and some more = {}, {}",
                i,
                i + 2,
                i + 3
            );
        }
        bf
    };
    // This will force the flush here for fairness.
    let af = clock.now();
    println!("Total time: {} in {}", af - bf, LOG_FILE);
}
