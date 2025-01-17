use cu29_clock::RobotClock;
use cu29_log_runtime::LoggerRuntime;
use cu29_runtime::curuntime::CopperContext;
use cu29_traits::{CuResult, UnifiedLogType};
use cu29_unifiedlog::{stream_write, UnifiedLogger, UnifiedLoggerBuilder};
use simplelog::TermLogger;
#[cfg(debug_assertions)]
use simplelog::{ColorChoice, Config, LevelFilter, TerminalMode};
use std::path::Path;
use std::sync::{Arc, Mutex};

/// This is a basic setup for a copper application to get you started.
/// Duplicate and customize as needed when your needs grow.
///
/// unifiedlogger_output_base_name: The base name of the log file. The logger will create a set of files based on this name
///                                  for example if named "toto.copper" it will create toto_0.copper, toto_1.copper etc.
///
/// text_log: if true, the log will be printed to the console as a simple log.
/// It is useful to debug an application in real-time but should be set to false in production
/// as it is an order of magnitude slower than the default copper structured logging.
/// It will create a LoggerRuntime that can be used as a robot clock source too.
///
/// slab_size: The logger will pre-allocate large files of those sizes. With the name of the given file _0, _1 etc.
/// clock: if you let it to None it will create a default clock otherwise you can provide your own, for example a simulation clock.
///        with let (clock , mock) = RobotClock::mock();
pub fn basic_copper_setup(
    unifiedlogger_output_base_name: &Path,
    slab_size: Option<usize>,
    _text_log: bool,
    clock: Option<RobotClock>,
) -> CuResult<CopperContext> {
    let preallocated_size = slab_size.unwrap_or(1024 * 1024 * 10);
    let UnifiedLogger::Write(logger) = UnifiedLoggerBuilder::new()
        .write(true)
        .create(true)
        .file_base_name(unifiedlogger_output_base_name)
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
        4096 * 10,
    );

    #[cfg(debug_assertions)]
    let extra: Option<TermLogger> = if _text_log {
        let slow_text_logger = TermLogger::new(
            LevelFilter::Debug,
            Config::default(),
            TerminalMode::Mixed,
            ColorChoice::Auto,
        );
        Some(*slow_text_logger)
    } else {
        None
    };

    #[cfg(not(debug_assertions))]
    let extra: Option<TermLogger> = None;

    let clock = clock.unwrap_or_default();
    let structured_logging = LoggerRuntime::init(clock.clone(), structured_stream, extra);
    Ok(CopperContext {
        unified_logger: unified_logger.clone(),
        logger_runtime: structured_logging,
        clock,
    })
}
