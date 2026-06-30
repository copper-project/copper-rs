#![cfg_attr(not(feature = "std"), no_std)]
#[cfg(not(feature = "std"))]
extern crate alloc;

mod sync_compat;

use core::sync::atomic::{AtomicUsize, Ordering};
use cu29_clock::RobotClock;
use cu29_log::CuLogEntry;
#[allow(unused_imports)]
use cu29_log::CuLogLevel;
use cu29_traits::{CuResult, WriteStream};
use log::Log;
use sync_compat::{Mutex, OnceLock, init_once, lock as lock_mutex};

#[cfg(not(feature = "std"))]
mod imp {
    pub use alloc::boxed::Box;
    pub use alloc::vec::Vec;
}

#[cfg(feature = "std")]
mod imp {
    pub use bincode::config::Configuration;
    pub use bincode::enc::Encode;
    pub use bincode::enc::Encoder;
    pub use bincode::enc::EncoderImpl;
    pub use bincode::enc::write::Writer;
    pub use bincode::error::EncodeError;
    pub use std::fmt::{Debug, Formatter};
    pub use std::fs::File;
    pub use std::io::{BufWriter, Write};
    pub use std::path::PathBuf;

    #[cfg(debug_assertions)]
    pub use {std::collections::HashMap, strfmt::strfmt};
}

use imp::*;

#[allow(dead_code)] // for no_std
#[derive(Debug)]
struct DummyWriteStream;

impl WriteStream<CuLogEntry> for DummyWriteStream {
    #[allow(unused_variables)] // for no_std
    fn log(&mut self, obj: &CuLogEntry) -> CuResult<()> {
        #[cfg(feature = "std")]
        eprintln!("Pending logs got cut: {obj:?}");
        Ok(())
    }
}
type LogWriter = Box<dyn WriteStream<CuLogEntry> + Send + 'static>;

/// Callback signature: receives the structured entry plus its format string and param names.
pub type LiveLogListener = Box<dyn Fn(&CuLogEntry, &str, &[&str]) + Send + Sync + 'static>;

pub type LiveLogListenerId = usize;

struct LiveLogListeners {
    next_id: LiveLogListenerId,
    listeners: Vec<(LiveLogListenerId, LiveLogListener)>,
}

impl Default for LiveLogListeners {
    fn default() -> Self {
        Self {
            next_id: 1,
            listeners: Vec::new(),
        }
    }
}

impl LiveLogListeners {
    fn insert(&mut self, listener: LiveLogListener) -> LiveLogListenerId {
        let id = self.next_id;
        self.next_id = self.next_id.saturating_add(1).max(1);
        self.listeners.push((id, listener));
        id
    }

    fn remove(&mut self, id: LiveLogListenerId) {
        self.listeners.retain(|(listener_id, _)| *listener_id != id);
    }
}

pub struct LiveLogListenerGuard {
    id: Option<LiveLogListenerId>,
}

impl Drop for LiveLogListenerGuard {
    fn drop(&mut self) {
        if let Some(id) = self.id.take() {
            unregister_live_log_listener_id(id);
        }
    }
}

#[cfg(all(feature = "std", debug_assertions))]
pub fn format_message_only(
    format_str: &str,
    params: &[String],
    named_params: &HashMap<String, String>,
) -> CuResult<String> {
    if format_str.contains("{}") {
        let mut formatted = format_str.to_string();
        for param in params.iter() {
            if !formatted.contains("{}") {
                break;
            }
            formatted = formatted.replacen("{}", param, 1);
        }
        if !named_params.is_empty() {
            let mut named = named_params.iter().collect::<Vec<_>>();
            named.sort_by(|a, b| a.0.cmp(b.0));
            for (name, value) in named {
                if formatted.contains("{}") {
                    formatted = formatted.replacen("{}", value, 1);
                }
                formatted = formatted.replace(&format!("{{{name}}}"), value);
            }
        }
        return Ok(formatted);
    }

    // Named replacement
    imp::strfmt(format_str, named_params).map_err(|e| {
        cu29_traits::CuError::new_with_cause(
            format!(
                "Failed to format log message: {format_str:?} with variables [{named_params:?}]"
            )
            .as_str(),
            e,
        )
    })
}

/// Shared logging state reachable from the macro-generated calls.
struct LoggerState {
    writer: Mutex<LogWriter>,
    clock: RobotClock,
    live_listeners: Mutex<LiveLogListeners>,
}

impl core::fmt::Debug for LoggerState {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("LoggerState")
            .field("clock", &self.clock)
            .finish_non_exhaustive()
    }
}

static LOGGER_STATE: OnceLock<LoggerState> = OnceLock::new();
static STRUCTURED_LOG_BYTES: AtomicUsize = AtomicUsize::new(0);

fn init_logger_state(state: LoggerState) {
    init_once(&LOGGER_STATE, state);
}

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
    ) -> Self {
        STRUCTURED_LOG_BYTES.store(0, Ordering::Relaxed);

        if let Some(state) = LOGGER_STATE.get() {
            let mut writer_guard = lock_mutex(&state.writer);
            *writer_guard = Box::new(destination);
        } else {
            let state = LoggerState {
                writer: Mutex::new(Box::new(destination)),
                clock,
                live_listeners: Mutex::new(LiveLogListeners::default()),
            };
            init_logger_state(state);
        }

        // If caller provided a default text logger (std + debug builds), install it as the live listener.
        #[cfg(all(feature = "std", debug_assertions))]
        if let Some(logger) = extra_text_logger {
            register_live_log_listener(move |entry, format_str, param_names| {
                // Build a text line from structured data—no parsing.
                let params: Vec<String> = entry.params.iter().map(|v| v.to_string()).collect();
                let mut named_params = HashMap::new();
                let mut param_names_iter = param_names.iter();
                for (name_index, value) in entry.paramname_indexes.iter().zip(params.iter()) {
                    if *name_index != cu29_log::ANONYMOUS {
                        let Some(name) = param_names_iter.next() else {
                            continue;
                        };
                        named_params.insert(name.to_string(), value.clone());
                    }
                }
                if let Ok(line) = format_message_only(format_str, params.as_slice(), &named_params)
                {
                    logger.log(
                        &log::Record::builder()
                            .args(format_args!("{line}"))
                            .level(match entry.level {
                                CuLogLevel::Debug => log::Level::Debug,
                                CuLogLevel::Info => log::Level::Info,
                                CuLogLevel::Warning => log::Level::Warn,
                                CuLogLevel::Error => log::Level::Error,
                                CuLogLevel::Critical => log::Level::Error,
                            })
                            .target("cu29_log")
                            .module_path_static(Some("cu29_log"))
                            .file_static(Some("cu29_log"))
                            .line(Some(0))
                            .build(),
                    );
                }
            });
        }

        LoggerRuntime {}
    }

    pub fn flush(&self) {
        // no op in no_std TODO(gbin): check if it will be needed in no_std at some point.
        if let Some(state) = LOGGER_STATE.get() {
            let mut writer = lock_mutex(&state.writer);
            let _ = writer.flush(); // ignore errors in no_std
        } else {
            #[cfg(feature = "std")]
            eprintln!("cu29_log: Logger not initialized.");
        }
    }
}

impl Drop for LoggerRuntime {
    fn drop(&mut self) {
        self.flush();
        // Assume on no-std that there is no buffering. TODO(gbin): check if this hold true.
        if let Some(state) = LOGGER_STATE.get() {
            let mut writer_guard = lock_mutex(&state.writer);
            *writer_guard = Box::new(DummyWriteStream);
        }
    }
}

/// Function called from generated code to log data.
/// It moves entry by design, it will be absorbed in the queue.
#[inline(always)]
fn log_inner(
    entry: &mut CuLogEntry,
    notify: bool,
    format_str: &str,
    param_names: &[&str],
) -> CuResult<()> {
    let Some(state) = LOGGER_STATE.get() else {
        return Err("Logger not initialized.".into());
    };
    entry.time = state.clock.now();

    let mut guard = lock_mutex(&state.writer);
    guard.log(entry)?;
    if let Some(bytes) = guard.last_log_bytes() {
        STRUCTURED_LOG_BYTES.fetch_add(bytes, Ordering::Relaxed);
    }

    // Basic notification; richer context added in log_debug_mode.
    if notify {
        notify_live_listeners(entry, format_str, param_names);
    }
    Ok(())
}

/// Public entry point used in release / no-debug paths.
#[inline(always)]
pub fn log(entry: &mut CuLogEntry) -> CuResult<()> {
    log_inner(entry, true, "", &[])
}

/// Returns the total number of bytes written to the structured log stream.
pub fn structured_log_bytes_total() -> u64 {
    STRUCTURED_LOG_BYTES.load(Ordering::Relaxed) as u64
}

/// This version of log is only compiled in debug mode
/// This allows a normal logging framework to be bridged.
#[cfg(debug_assertions)]
pub fn log_debug_mode(
    entry: &mut CuLogEntry,
    _format_str: &str, // this is the missing info at runtime.
    _param_names: &[&str],
) -> CuResult<()> {
    // Write structured log but avoid double-notifying live listeners here.
    log_inner(entry, false, "", &[])?;

    // and the bridging is only available in std.
    #[cfg(feature = "std")]
    extra_log(entry, _format_str, _param_names)?;

    Ok(())
}

#[cfg(debug_assertions)]
#[cfg(feature = "std")]
fn extra_log(entry: &mut CuLogEntry, format_str: &str, param_names: &[&str]) -> CuResult<()> {
    // Legacy text logging now goes through the live listener; keep this as a thin shim.
    notify_live_listeners(entry, format_str, param_names);

    Ok(())
}

/// Register a live log listener; subsequent logs invoke `cb`.
pub fn register_live_log_listener<F>(cb: F) -> Option<LiveLogListenerId>
where
    F: Fn(&CuLogEntry, &str, &[&str]) + Send + Sync + 'static,
{
    LOGGER_STATE.get().map(|state| {
        let mut guard = lock_mutex(&state.live_listeners);
        guard.insert(Box::new(cb))
    })
}

/// Register a scoped live log listener and remove it when the returned guard is dropped.
pub fn scoped_live_log_listener<F>(cb: F) -> LiveLogListenerGuard
where
    F: Fn(&CuLogEntry, &str, &[&str]) + Send + Sync + 'static,
{
    LiveLogListenerGuard {
        id: register_live_log_listener(cb),
    }
}

/// Remove a live log listener by id. No-op if runtime not initialized.
pub fn unregister_live_log_listener_id(id: LiveLogListenerId) {
    if let Some(state) = LOGGER_STATE.get() {
        let mut guard = lock_mutex(&state.live_listeners);
        guard.remove(id);
    }
}

/// Remove all registered live log listeners. No-op if runtime not initialized.
pub fn unregister_live_log_listener() {
    if let Some(state) = LOGGER_STATE.get() {
        let mut guard = lock_mutex(&state.live_listeners);
        guard.listeners.clear();
    }
}

/// Capture structured log entries emitted while `f` runs.
///
/// This is a scoped tee: existing live listeners still receive entries while
/// capture is active.
#[cfg(feature = "std")]
pub fn capture_live_logs<F, R>(f: F) -> (R, Vec<CuLogEntry>)
where
    F: FnOnce() -> R,
{
    let captured_logs = std::sync::Arc::new(std::sync::Mutex::new(Vec::<CuLogEntry>::new()));
    let listener_guard = LOGGER_STATE.get().map(|_| {
        let captured_logs_for_listener = captured_logs.clone();
        scoped_live_log_listener(move |entry, _, _| {
            captured_logs_for_listener
                .lock()
                .expect("live log capture poisoned")
                .push(entry.clone());
        })
    });

    let result = f();
    drop(listener_guard);
    let logs = captured_logs
        .lock()
        .expect("live log capture poisoned")
        .clone();

    (result, logs)
}

/// Notify registered listener if any.
#[allow(clippy::collapsible_if)]
pub(crate) fn notify_live_listeners(entry: &CuLogEntry, format_str: &str, param_names: &[&str]) {
    if let Some(state) = LOGGER_STATE.get() {
        let guard = lock_mutex(&state.live_listeners);
        for (_, cb) in &guard.listeners {
            cb(entry, format_str, param_names);
        }
    }
}
// This is an adaptation of the Iowriter from bincode.

#[cfg(feature = "std")]
pub struct OwningIoWriter<W: Write> {
    writer: BufWriter<W>,
    bytes_written: usize,
}

#[cfg(feature = "std")]
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

#[cfg(feature = "std")]
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
#[cfg(feature = "std")]
pub struct SimpleFileWriter {
    path: PathBuf,
    encoder: EncoderImpl<OwningIoWriter<File>, Configuration>,
}

#[cfg(feature = "std")]
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

#[cfg(feature = "std")]
impl Debug for SimpleFileWriter {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(f, "SimpleFileWriter for path {:?}", self.path)
    }
}

#[cfg(feature = "std")]
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
    use crate::CuLogEntry;
    use bincode::config::standard;
    use cu29_log::{CuLogLevel, CuLogOrigin};
    use cu29_value::Value;
    use smallvec::smallvec;

    #[cfg(not(feature = "std"))]
    use alloc::string::ToString;

    #[test]
    fn test_encode_decode_structured_log() {
        let log_entry = CuLogEntry {
            time: 0.into(),
            level: CuLogLevel::Info,
            origin: CuLogOrigin::default(),
            msg_index: 1,
            paramname_indexes: smallvec![2, 3],
            params: smallvec![Value::String("test".to_string())],
        };
        let encoded = bincode::encode_to_vec(&log_entry, standard()).unwrap();
        let decoded_tuple: (CuLogEntry, usize) =
            bincode::decode_from_slice(&encoded, standard()).unwrap();
        assert_eq!(log_entry, decoded_tuple.0);
    }

    #[cfg(all(feature = "std", debug_assertions))]
    #[test]
    fn test_format_message_only_mixes_named_and_positional_placeholders() {
        let params = vec!["event payload".to_string()];
        let mut named_params = std::collections::HashMap::new();
        named_params.insert("hash".to_string(), "0x000000000".to_string());
        named_params.insert("size".to_string(), "420".to_string());

        let formatted = crate::format_message_only(
            "File closed after hash was calculated Hash: {hash}, size: {size};\n{}",
            &params,
            &named_params,
        )
        .unwrap();

        assert_eq!(
            formatted,
            "File closed after hash was calculated Hash: 0x000000000, size: 420;\nevent payload"
        );
    }
}
