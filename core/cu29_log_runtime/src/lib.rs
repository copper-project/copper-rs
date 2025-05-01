use bincode::config::Configuration;
use bincode::enc::write::Writer;
use bincode::enc::Encode;
use bincode::enc::{Encoder, EncoderImpl};
use bincode::error::EncodeError;
use cu29_clock::RobotClock;
use cu29_log::{CuLogEntry, CuLogLevel};
use cu29_traits::{CuResult, WriteStream};
use log::Log;

#[cfg(debug_assertions)]
use {cu29_log::format_logline, std::collections::HashMap, std::sync::RwLock};

use std::fmt::{Debug, Formatter};
use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::PathBuf;
use std::sync::{Mutex, OnceLock};

#[derive(Debug)]
struct DummyWriteStream;

impl WriteStream<CuLogEntry> for DummyWriteStream {
    fn log(&mut self, obj: &CuLogEntry) -> CuResult<()> {
        eprintln!("Pending logs got cut: {obj:?}");
        Ok(())
    }
}
type LogWriter = Box<dyn WriteStream<CuLogEntry>>;
type WriterConfig = (Mutex<LogWriter>, RobotClock, Option<CuLogLevel>);

static LOGGER_CONFIG: OnceLock<WriterConfig> = OnceLock::new();

#[cfg(debug_assertions)]
pub static EXTRA_TEXT_LOGGER: RwLock<Option<Box<dyn Log + 'static>>> = RwLock::new(None);

pub struct NullLog;
impl Log for NullLog {
    fn enabled(&self, _metadata: &log::Metadata) -> bool {
        false
    }

    fn log(&self, _record: &log::Record) {}
    fn flush(&self) {}
}

/// The lifetime of this struct is the lifetime of the logger.
pub struct LoggerRuntime {}

impl LoggerRuntime {
    /// destination is the binary stream in which we will log the structured log.
    /// `extra_text_logger` is the logger that will log the text logs in real time. This is slow and only for debug builds.
    pub fn init(
        clock: RobotClock,
        destination: impl WriteStream<CuLogEntry> + 'static,
        #[allow(unused_variables)] extra_text_logger: Option<impl Log + 'static>,
        max_level: Option<CuLogLevel>,
    ) -> Self {
        let runtime = LoggerRuntime {};

        // If LOGGER_CONFIG is already initialized, update the inner value.
        // This should only be useful for unit testing.
        if let Some((writer, _, _)) = LOGGER_CONFIG.get() {
            let mut writer_guard = writer.lock().unwrap();
            *writer_guard = Box::new(destination);
        } else {
            LOGGER_CONFIG
                .set((Mutex::new(Box::new(destination)), clock, max_level))
                .unwrap();
        }
        #[cfg(debug_assertions)]
        if let Some(logger) = extra_text_logger {
            *EXTRA_TEXT_LOGGER.write().unwrap() = Some(Box::new(logger) as Box<dyn Log>);
        }

        runtime
    }

    pub fn flush(&self) {
        if let Some((writer, _clock, _max_level)) = LOGGER_CONFIG.get() {
            if let Ok(mut writer) = writer.lock() {
                if let Err(err) = writer.flush() {
                    eprintln!("cu29_log: Failed to flush writer: {err}");
                }
            } else {
                eprintln!("cu29_log: Failed to lock writer.");
            }
        } else {
            eprintln!("cu29_log: Logger not initialized.");
        }
    }
}

impl Drop for LoggerRuntime {
    fn drop(&mut self) {
        self.flush();
        if let Some((mutex, _clock, _max_level)) = LOGGER_CONFIG.get() {
            if let Ok(mut writer_guard) = mutex.lock() {
                // Replace the current WriteStream with a DummyWriteStream
                *writer_guard = Box::new(DummyWriteStream);
            }
        }
    }
}

/// Function called from generated code to log data.
/// It moves entry by design, it will be absorbed in the queue.
#[inline(always)]
pub fn log(entry: &mut CuLogEntry) -> CuResult<()> {
    let config = LOGGER_CONFIG
        .get()
        .map(|(writer, clock, max_level)| (writer, clock, max_level));
    if config.is_none() {
        return Err("Logger not initialized.".into());
    }
    let (writer, clock, max_level) = config.unwrap();

    // Check if this log entry should be filtered out based on max_level
    if let Some(max) = max_level {
        if &entry.level > max {
            // Skip logging if the entry's level is higher than the max configured level
            return Ok(());
        }
    }

    entry.time = clock.now();
    if let Err(err) = writer.lock().unwrap().log(entry) {
        eprintln!("Failed to log data: {err}");
    }
    // This is only for debug builds with standard textual logging implemented.
    #[cfg(debug_assertions)]
    {
        // This scope is important :).
        // if we have not passed a text logger in debug mode, it is ok just move along.
    }

    Ok(())
}

/// This version of log is only compiled in debug mode
/// This allows a normal logging framework to be bridged.
#[cfg(debug_assertions)]
pub fn log_debug_mode(
    entry: &mut CuLogEntry,
    format_str: &str, // this is the missing info at runtime.
    param_names: &[&str],
) -> CuResult<()> {
    // We call log() which will handle max_level filtering for us
    // If the log level is filtered out, log() will return Ok(()) early
    log(entry)?;

    let guarded_logger = EXTRA_TEXT_LOGGER.read().unwrap();
    if guarded_logger.is_none() {
        return Ok(());
    }
    if let Some(logger) = guarded_logger.as_ref() {
        let fstr = format_str.to_string();
        // transform the slice into a hashmap
        let params: Vec<String> = entry.params.iter().map(|v| v.to_string()).collect();
        let named_params: Vec<(&str, String)> = param_names
            .iter()
            .zip(params.iter())
            .map(|(name, value)| (*name, value.clone()))
            .collect();
        // build hashmap of string, string from named_paramgs
        let named_params: HashMap<String, String> = named_params
            .iter()
            .map(|(k, v)| (k.to_string(), v.clone()))
            .collect();
        let logline = format_logline(entry.time, &fstr, params.as_slice(), &named_params)?;

        // Map CuLogLevel to log::Level
        let log_level = match entry.level {
            CuLogLevel::Error => log::Level::Error,
            CuLogLevel::Warn => log::Level::Warn,
            CuLogLevel::Info => log::Level::Info,
            CuLogLevel::Debug => log::Level::Debug,
            CuLogLevel::Trace => log::Level::Trace,
        };

        logger.log(
            &log::Record::builder()
                .args(format_args!("{logline}"))
                .level(log_level)
                .target("cu29_log")
                .module_path_static(Some("cu29_log"))
                .file_static(Some("cu29_log"))
                .line(Some(0))
                .build(),
        );
    }
    Ok(())
}

// This is an adaptation of the Iowriter from bincode.
pub struct OwningIoWriter<W: Write> {
    writer: BufWriter<W>,
    bytes_written: usize,
}

impl<W: Write> OwningIoWriter<W> {
    pub fn new(writer: W) -> Self {
        Self {
            writer: BufWriter::new(writer),
            bytes_written: 0,
        }
    }

    pub fn bytes_written(&self) -> usize {
        self.bytes_written
    }

    pub fn flush(&mut self) -> Result<(), EncodeError> {
        self.writer.flush().map_err(|inner| EncodeError::Io {
            inner,
            index: self.bytes_written,
        })
    }
}

impl<W: Write> Writer for OwningIoWriter<W> {
    #[inline(always)]
    fn write(&mut self, bytes: &[u8]) -> Result<(), EncodeError> {
        self.writer
            .write_all(bytes)
            .map_err(|inner| EncodeError::Io {
                inner,
                index: self.bytes_written,
            })?;
        self.bytes_written += bytes.len();
        Ok(())
    }
}

/// This allows this crate to be used outside of Copper (ie. decoupling it from the unifiedlog.
pub struct SimpleFileWriter {
    path: PathBuf,
    encoder: EncoderImpl<OwningIoWriter<File>, Configuration>,
}

impl SimpleFileWriter {
    pub fn new(path: &PathBuf) -> CuResult<Self> {
        let file = std::fs::OpenOptions::new()
            .create(true)
            .truncate(true)
            .write(true)
            .open(path)
            .map_err(|e| format!("Failed to open file: {e:?}"))?;

        let writer = OwningIoWriter::new(file);
        let encoder = EncoderImpl::new(writer, bincode::config::standard());

        Ok(SimpleFileWriter {
            path: path.clone(),
            encoder,
        })
    }
}

impl Debug for SimpleFileWriter {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(f, "SimpleFileWriter for path {:?}", self.path)
    }
}

impl WriteStream<CuLogEntry> for SimpleFileWriter {
    #[inline(always)]
    fn log(&mut self, obj: &CuLogEntry) -> CuResult<()> {
        obj.encode(&mut self.encoder)
            .map_err(|e| format!("Failed to write to file: {e:?}"))?;
        Ok(())
    }

    fn flush(&mut self) -> CuResult<()> {
        self.encoder
            .writer()
            .flush()
            .map_err(|e| format!("Failed to flush file: {e:?}"))?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use crate::{CuLogEntry, CuLogLevel};
    use bincode::config::standard;
    use cu29_value::Value;
    use smallvec::smallvec;

    #[test]
    fn test_encode_decode_structured_log() {
        let log_entry = CuLogEntry {
            level: CuLogLevel::Info,
            time: 0.into(),
            msg_index: 1,
            paramname_indexes: smallvec![2, 3],
            params: smallvec![Value::String("test".to_string())],
        };
        let encoded = bincode::encode_to_vec(&log_entry, standard()).unwrap();
        let decoded_tuple: (CuLogEntry, usize) =
            bincode::decode_from_slice(&encoded, standard()).unwrap();
        assert_eq!(log_entry, decoded_tuple.0);
    }
}
