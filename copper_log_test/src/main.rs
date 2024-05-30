use copper::DataLogType;
use copper_datalogger::{stream, DataLogger};
use copper_log::debug;
use copper_log_runtime::LoggerRuntime;
use copper_value::to_value;
use serde::Serialize;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};

fn main() {
    let path: PathBuf = PathBuf::from("/tmp/teststructlog.copper");
    let data_logger = Arc::new(Mutex::new(
        DataLogger::new(path.as_path(), Some(100000)).expect("Failed to create logger"),
    ));
    let mut stream = stream(data_logger.clone(), DataLogType::StructuredLogLine, 1024);
    let rt = LoggerRuntime::init(stream);
    #[derive(Serialize)]
    struct Test {
        a: i32,
        b: i32,
    }
    let mytuple = (1, "toto", 3.34f64, true, 'a');
    {
        let hop = copper::monitoring::ScopedAllocCounter::new();
        let gigantic_vec = vec![0u8; 1_000_000];
        debug!("Just a string {}", "zarma");
        debug!("anonymous param constants {} {}", 42u16, 43u8);
        debug!("named param constants {} {}", a = 3, b = 2);
        debug!("mixed named param constants, {} {} {}", a = 3, 54, b = 2);
        debug!("complex tuple", mytuple);
        debug!("Struct", Test { a: 3, b: 4 });
    }
    debug!(" AFTER CLOSE {} ", "AFTER CLOSE");
    rt.close();
}
