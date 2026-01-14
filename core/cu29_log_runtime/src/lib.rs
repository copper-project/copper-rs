#![cfg_attr(not(feature = "std"), no_std)]
#[cfg(not(feature = "std"))]
extern crate alloc;

use cu29_clock::RobotClock;
#[allow(unused_imports)]
use cu29_log::CuLogLevel;
use cu29_log::{CULISTID_UNKNOWN, CuLogEntry};
use cu29_traits::{CuResult, WriteStream};
use log::Log;

#[cfg(feature = "std")]
use std::cell::Cell;

#[cfg(not(feature = "std"))]
use portable_atomic::{AtomicU32, Ordering};

#[cfg(not(feature = "std"))]
mod imp {
    pub use alloc::boxed::Box;
    pub use spin::Mutex;
    pub use spin::once::Once as OnceLock;
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
    pub use std::sync::{Mutex, OnceLock};

    #[cfg(debug_assertions)]
    pub use {cu29_log::format_logline, std::collections::HashMap, std::sync::RwLock};
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
type WriterPair = (Mutex<LogWriter>, RobotClock);

static WRITER: OnceLock<WriterPair> = OnceLock::new();

#[cfg(debug_assertions)]
#[cfg(feature = "std")]
pub static EXTRA_TEXT_LOGGER: RwLock<Option<Box<dyn Log + 'static>>> = RwLock::new(None);

#[cfg(feature = "std")]
thread_local! {
    static CURRENT_CULISTID: Cell<u32> = const { Cell::new(CULISTID_UNKNOWN) };
}

#[cfg(not(feature = "std"))]
// no_std uses a global atomic; portable-atomic covers targets without native atomics.
static CURRENT_CULISTID: AtomicU32 = AtomicU32::new(CULISTID_UNKNOWN);

/// Returns the current CopperList id for this execution context.
#[inline]
pub fn current_culistid() -> u32 {
    #[cfg(feature = "std")]
    {
        CURRENT_CULISTID.with(|id| id.get())
    }
    #[cfg(not(feature = "std"))]
    {
        CURRENT_CULISTID.load(Ordering::SeqCst)
    }
}

/// Sets the current CopperList id, returning the previous value.
#[inline]
pub fn set_current_culistid(culistid: u32) -> u32 {
    #[cfg(feature = "std")]
    {
        CURRENT_CULISTID.with(|id| {
            let prev = id.get();
            id.set(culistid);
            prev
        })
    }
    #[cfg(not(feature = "std"))]
    {
        CURRENT_CULISTID.swap(culistid, Ordering::SeqCst)
    }
}

/// Guard that restores the previous CopperList id on drop.
pub struct CulistContextGuard {
    previous: u32,
}

impl CulistContextGuard {
    pub fn new(culistid: u32) -> Self {
        let previous = set_current_culistid(culistid);
        Self { previous }
    }
}

impl Drop for CulistContextGuard {
    fn drop(&mut self) {
        set_current_culistid(self.previous);
    }
}

/// Runs a closure with the provided CopperList id set as current.
#[inline]
pub fn with_culistid<R>(culistid: u32, f: impl FnOnce() -> R) -> R {
    let _guard = CulistContextGuard::new(culistid);
    f()
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
        let runtime = LoggerRuntime {};

        // If WRITER is already initialized, update the inner value.
        // This should only be useful for unit testing.
        if let Some((writer, _)) = WRITER.get() {
            #[cfg(not(feature = "std"))]
            let mut writer_guard = writer.lock();
            #[cfg(feature = "std")]
            let mut writer_guard = writer.lock().unwrap();
            *writer_guard = Box::new(destination);
        } else {
            #[cfg(not(feature = "std"))]
            WRITER.call_once(|| (Mutex::new(Box::new(destination)), clock));
            #[cfg(feature = "std")]
            WRITER
                .set((Mutex::new(Box::new(destination)), clock))
                .unwrap();
        }
        #[cfg(debug_assertions)]
        #[cfg(feature = "std")]
        if let Some(logger) = extra_text_logger {
            let mut extra_text_logger = EXTRA_TEXT_LOGGER.write().unwrap();
            *extra_text_logger = Some(Box::new(logger) as Box<dyn Log>);
        }

        runtime
    }

    pub fn flush(&self) {
        // no op in no_std TODO(gbin): check if it will be needed in no_std at some point.
        #[cfg(feature = "std")]
        if let Some((writer, _clock)) = WRITER.get() {
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
        // Assume on no-std that there is no buffering. TODO(gbin): check if this hold true.
        #[cfg(feature = "std")]
        if let Some((mutex, _clock)) = WRITER.get()
            && let Ok(mut writer_guard) = mutex.lock()
        {
            // Replace the current WriteStream with a DummyWriteStream
            *writer_guard = Box::new(DummyWriteStream);
        }
    }
}

/// Function called from generated code to log data.
/// It moves entry by design, it will be absorbed in the queue.
#[inline(always)]
pub fn log(entry: &mut CuLogEntry) -> CuResult<()> {
    let d = WRITER.get().map(|(writer, clock)| (writer, clock));
    if d.is_none() {
        return Err("Logger not initialized.".into());
    }
    let (writer, clock) = d.unwrap();
    entry.time = clock.now();
    entry.culistid = current_culistid();

    #[cfg(not(feature = "std"))]
    writer.lock().log(entry)?;

    #[cfg(feature = "std")]
    if let Err(err) = writer.lock().unwrap().log(entry) {
        eprintln!("Failed to log data: {err}");
    }
    Ok(())
}

/// This version of log is only compiled in debug mode
/// This allows a normal logging framework to be bridged.
#[cfg(debug_assertions)]
pub fn log_debug_mode(
    entry: &mut CuLogEntry,
    _format_str: &str, // this is the missing info at runtime.
    _param_names: &[&str],
) -> CuResult<()> {
    log(entry)?;

    // and the bridging is only available in std.
    #[cfg(feature = "std")]
    extra_log(entry, _format_str, _param_names)?;

    Ok(())
}

#[cfg(debug_assertions)]
#[cfg(feature = "std")]
fn extra_log(entry: &mut CuLogEntry, format_str: &str, param_names: &[&str]) -> CuResult<()> {
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

        // Convert Copper log level to the standard log level
        // Note: CuLogLevel::Critical is mapped to log::Level::Error because the `log` crate
        // does not have a `Critical` level. `Error` is the highest severity level available
        // in the `log` crate, making it the closest equivalent.
        let log_level = match entry.level {
            CuLogLevel::Debug => log::Level::Debug,
            CuLogLevel::Info => log::Level::Info,
            CuLogLevel::Warning => log::Level::Warn,
            CuLogLevel::Error => log::Level::Error,
            CuLogLevel::Critical => log::Level::Error,
        };

        let logline = format_logline(
            entry.time,
            entry.level,
            &fstr,
            params.as_slice(),
            &named_params,
        )?;
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
    use crate::CulistContextGuard;
    #[cfg(feature = "std")]
    use crate::LoggerRuntime;
    #[cfg(feature = "std")]
    use crate::NullLog;
    use crate::current_culistid;
    #[cfg(feature = "std")]
    use crate::log;
    use crate::set_current_culistid;
    use bincode::config::standard;
    #[cfg(feature = "std")]
    use cu29_clock::RobotClock;
    use cu29_log::CULISTID_UNKNOWN;
    use cu29_log::CuLogLevel;
    #[cfg(feature = "std")]
    use cu29_traits::WriteStream;
    use cu29_value::Value;
    use smallvec::smallvec;
    #[cfg(feature = "std")]
    use std::sync::{Arc, Barrier, Mutex, OnceLock};
    #[cfg(feature = "std")]
    use std::thread;

    #[cfg(not(feature = "std"))]
    use alloc::string::ToString;

    #[test]
    fn test_encode_decode_structured_log() {
        let log_entry = CuLogEntry {
            time: 0.into(),
            level: CuLogLevel::Info,
            msg_index: 1,
            paramname_indexes: smallvec![2, 3],
            params: smallvec![Value::String("test".to_string())],
            culistid: CULISTID_UNKNOWN,
        };
        let encoded = bincode::encode_to_vec(&log_entry, standard()).unwrap();
        let decoded_tuple: (CuLogEntry, usize) =
            bincode::decode_from_slice(&encoded, standard()).unwrap();
        assert_eq!(log_entry, decoded_tuple.0);
    }

    #[test]
    fn test_culist_context_guard() {
        let original = current_culistid();
        {
            let _guard = CulistContextGuard::new(42);
            assert_eq!(current_culistid(), 42);
        }
        assert_eq!(current_culistid(), original);
        let prev = set_current_culistid(7);
        assert_eq!(prev, original);
        assert_eq!(current_culistid(), 7);
        set_current_culistid(original);
    }

    #[cfg(feature = "std")]
    #[test]
    fn test_thread_local_culistid_logging() {
        static TEST_LOCK: OnceLock<Mutex<()>> = OnceLock::new();
        let _guard = TEST_LOCK.get_or_init(|| Mutex::new(())).lock().unwrap();

        #[derive(Debug)]
        struct CulistCapture {
            culistids: Arc<Mutex<Vec<u32>>>,
        }

        impl WriteStream<CuLogEntry> for CulistCapture {
            fn log(&mut self, obj: &CuLogEntry) -> cu29_traits::CuResult<()> {
                self.culistids.lock().unwrap().push(obj.culistid);
                Ok(())
            }
        }

        let culistids = Arc::new(Mutex::new(Vec::new()));
        let writer = CulistCapture {
            culistids: culistids.clone(),
        };
        let _runtime = LoggerRuntime::init(RobotClock::default(), writer, None::<NullLog>);

        let barrier = Arc::new(Barrier::new(3));
        let mut handles = Vec::new();
        for culistid in [10u32, 20u32] {
            let barrier = barrier.clone();
            let handle = thread::spawn(move || {
                set_current_culistid(culistid);
                barrier.wait();
                let mut entry = CuLogEntry::new(1, CuLogLevel::Info);
                log(&mut entry).unwrap();
            });
            handles.push(handle);
        }

        barrier.wait();
        for handle in handles {
            handle.join().unwrap();
        }

        let mut values = culistids.lock().unwrap().clone();
        values.sort();
        assert_eq!(values, vec![10, 20]);
    }
}
