use cu29_clock::RobotClock;
use cu29_log::default_log_index_dir;
use cu29_log_runtime::{ExtraTextLogger, LoggerRuntime};
use cu29_traits::{CuResult, UnifiedLogType};
use cu29_unifiedlog::{stream_write, UnifiedLogger, UnifiedLoggerBuilder, UnifiedLoggerWrite};
use simplelog::{ColorChoice, Config, LevelFilter, TermLogger, TerminalMode};
use std::path::Path;
use std::sync::{Arc, Mutex};

/// Just a simple struct to hold the various bits needed to run a Copper application.
pub struct CopperContext {
    pub unified_logger: Arc<Mutex<UnifiedLoggerWrite>>,
    pub logger_runtime: LoggerRuntime,
    pub clock: RobotClock,
}

/// This is a basic setup for a copper application to get you started.
/// Duplicate and customize as needed when your needs grow.
///
/// text_log: if true, the log will be printed to the console as a simple log.
/// It is useful to debug an application in real-time but should be set to false in production
/// as it is an order of magnitude slower than the default copper structured logging.
/// It will create a LoggerRuntime that can be used as a robot clock source too.
///
/// preallocated_storage_size: The size of the preallocated storage for the unified logger.
pub fn basic_copper_setup(
    unifiedlogger_output_path: &Path,
    preallocated_storage_size: Option<usize>,
    text_log: bool,
) -> CuResult<CopperContext> {
    let preallocated_size = preallocated_storage_size.unwrap_or(1024 * 1024 * 10);
    let UnifiedLogger::Write(logger) = UnifiedLoggerBuilder::new()
        .write(true)
        .create(true)
        .file_path(unifiedlogger_output_path)
        .preallocated_size(preallocated_size)
        .build()
        .expect("Failed to create logger")
    else {
        panic!("Failed to create logger")
    };
    let unified_logger = Arc::new(Mutex::new(logger));
    let structured_stream = stream_write(
        unified_logger.clone(),
        UnifiedLogType::StructuredLogLine,
        4096,
    );

    #[cfg(debug_assertions)]
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

    #[cfg(not(debug_assertions))]
    let extra = None;

    let clock = RobotClock::default();
    let structured_logging = LoggerRuntime::init(clock.clone(), structured_stream, extra);
    Ok(CopperContext {
        unified_logger: unified_logger.clone(),
        logger_runtime: structured_logging,
        clock,
    })
}
