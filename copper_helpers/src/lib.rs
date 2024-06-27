use copper_clock::RobotClock;
use copper_log::default_log_index_dir;
use copper_log_runtime::{ExtraTextLogger, LoggerRuntime};
use copper_traits::{CuResult, UnifiedLogType};
use copper_unifiedlog::{stream_write, UnifiedLogger, UnifiedLoggerBuilder};
use simplelog::{ColorChoice, Config, LevelFilter, TermLogger, TerminalMode};
use std::path::Path;
use std::sync::{Arc, Mutex};

/// This is a basic setup for a copper application to get you started.
/// Duplicate and customize as needed when your needs grow.
///
/// text_log: if true, the log will be printed to the console as a simple log.
/// It is useful to debug an application in real-time but should be set to false in production
/// as it is an order of magnitude slower than the default copper structured logging.
/// It will create a LoggerRuntime that can be used as a robot clock source too.
pub fn basic_logger_runtime_setup(
    datalogger_output_path: &Path,
    text_log: bool,
) -> CuResult<LoggerRuntime> {
    let UnifiedLogger::Write(logger) = UnifiedLoggerBuilder::new()
        .write(true)
        .create(true)
        .file_path(datalogger_output_path)
        .preallocated_size(100000)
        .build()
        .expect("Failed to create logger")
    else {
        panic!("Failed to create logger")
    };
    let data_logger = Arc::new(Mutex::new(logger));
    let stream = stream_write(data_logger.clone(), UnifiedLogType::StructuredLogLine, 1024);

    let extra = if text_log {
        let slow_text_logger = TermLogger::new(
            LevelFilter::Debug,
            Config::default(),
            TerminalMode::Mixed,
            ColorChoice::Auto,
        );

        // This is the path to the index file that was created at build time.
        // depending if we build with debug pick it from debug or release:
        let log_index_path = default_log_index_dir();

        let extra_text_logger: ExtraTextLogger =
            ExtraTextLogger::new(log_index_path, slow_text_logger);
        Some(extra_text_logger)
    } else {
        None
    };
    Ok(LoggerRuntime::init(RobotClock::default(), stream, extra))
}
