use std::fs::File;
use std::sync::OnceLock;
use std::io::Write;
use cu29_clock::{ClockProvider, RobotClock};
use cu29_intern_strs::read_interned_strings;
use cu29_log::CuLogEntry;
use cu29_traits::{CuResult, WriteStream};
use kanal::{bounded, Sender};
use log::{warn, Log, Record};
use std::path::PathBuf;
use std::sync::Arc;
use std::thread;
use std::sync::Once;
use std::thread::{sleep, JoinHandle};
use std::time::Duration;
use bincode::config::Configuration;
use bincode::enc;
use bincode::enc::{Encoder, EncoderImpl};
use bincode::error::EncodeError;
use bincode::enc::write::{SliceWriter, Writer};
use bincode::enc::Encode;

// The logging system is basically a global queue.
static QUEUE_CLOCK: OnceLock<(Sender<CuLogEntry>, RobotClock)> = OnceLock::new();

/// The lifetime of this struct is the lifetime of the logger.
pub struct LoggerRuntime {
    handle: Option<JoinHandle<()>>,
    extra_text_logger: Option<ExtraTextLogger>,
}

impl LoggerRuntime {
    /// destination is the binary stream in which we will log the structured log.
    /// extra_text_logger is the logger that will log the text logs in real time. This is slow and only for debug builds.
    pub fn init(
        clock: RobotClock,
        destination: impl WriteStream<CuLogEntry> + 'static,
        extra_text_logger: Option<ExtraTextLogger>,
    ) -> Self {
        if (!cfg!(debug_assertions)) && extra_text_logger.is_some() {
            eprintln!("Extra text logger is only available in debug builds. Ignoring the extra text logger.");
        };

        let mut runtime = LoggerRuntime {
            extra_text_logger,
            handle: None,
        };
        let (s, handle) = runtime.initialize_queue(destination);
        QUEUE_CLOCK
            .set((s, clock))
            .expect("Failed to initialize the logger queue.");
        runtime.handle = Some(handle);
        runtime
    }

    fn initialize_queue(
        &self,
        mut destination: impl WriteStream<CuLogEntry> + 'static,
    ) -> (Sender<CuLogEntry>, JoinHandle<()>) {
        let (sender, receiver) = bounded::<CuLogEntry>(10000);

        #[cfg(debug_assertions)]
        let (index, extra_text_logger) = if let Some(extra) = &self.extra_text_logger {
            let index = Some(
                read_interned_strings(extra.path_to_index.as_path())
                    .expect("Failed to read the interned strings"),
            );
            let logger = Some(extra.logger.clone());
            (index, logger)
        } else {
            (None, None)
        };

        let handle = thread::spawn(move || {
            let receiver = receiver.clone();
            loop {
                if let Ok(cu_log_entry) = receiver.recv() {
                    // If the user wants to really log a clock they should add it as a structured field.
                    if let Err(err) = destination.log(&cu_log_entry) {
                        eprintln!("Failed to log data: {}", err);
                    }

                    // This is only for debug builds with standard textual logging implemented.
                    #[cfg(debug_assertions)]
                    if let Some(index) = &index {
                        if let Some(ref logger) = extra_text_logger {
                            let stringified = cu29_log::rebuild_logline(index, &cu_log_entry);
                            match stringified {
                                Ok(s) => {
                                    let s = format!("[{}] {}", cu_log_entry.time, s);
                                    logger.log(
                                        &Record::builder()
                                            // TODO: forward this info in the CuLogEntry
                                            .level(log::Level::Debug)
                                            // .target("copper")
                                            //.module_path_static(Some("cu29_log"))
                                            //.file_static(Some("cu29_log"))
                                            //.line(Some(0))
                                            .args(format_args!("{}", s))
                                            .build(),
                                    ); // DO NOT TRY to split off this statement.
                                       // format_args! has to be in the same statement per structure and scoping.
                                }
                                Err(e) => {
                                    eprintln!("Failed to rebuild log line: {}", e);
                                }
                            }
                        }
                    }
                } else {
                    // means that the sender has disconnected, we can stop the thread.
                    break;
                }
            }
        });
        (sender, handle)
    }
    pub fn is_alive(&self) -> bool {
        QUEUE_CLOCK.get().is_some()
    }

    pub fn flush(&self) {
        if let Some((queue, _)) = QUEUE_CLOCK.get() {
            loop {
                if queue.is_empty() {
                    break;
                }
                println!("Waiting for the queue to empty.");
                sleep(Duration::from_millis(1));
            }
        }
    }

    pub fn close(&mut self) {
        let queue = QUEUE_CLOCK.get();
        if queue.is_none() {
            eprintln!("Logger closed before it was initialized.");
            return;
        }
        self.flush();
        queue.unwrap().0.close();
        if let Some(handle) = self.handle.take() {
            handle.join().expect("Failed to join the logger thread.");
            self.handle = None;
        }
    }
}

impl Drop for LoggerRuntime {
    fn drop(&mut self) {
        self.close();
    }
}

/// This is to basically be able to see the logs in text format in real time.
/// THIS WILL SLOW DOWN THE LOGGING SYSTEM by an order of magnitude.
/// This will only be active for debug builds.
pub struct ExtraTextLogger {
    path_to_index: PathBuf,
    logger: Arc<dyn Log>,
}
impl ExtraTextLogger {
    pub fn new(path_to_index: PathBuf, logger: Box<dyn Log>) -> Self {
        ExtraTextLogger {
            path_to_index,
            logger: Arc::new(logger),
        }
    }
}

/// Function called from generated code to log data.
/// It moves entry by design, it will be absorbed in the queue.
#[inline(always)]
pub fn log(mut entry: CuLogEntry) -> CuResult<()> {
    if let Some((queue, clock)) = QUEUE_CLOCK.get() {
        entry.time = clock.now();
        let err = queue
            .send(entry)
            .map_err(|e| format!("Failed to send data to the logger, did you hold the reference to the logger long enough? {:?}", e).into());
        err
    } else {
        Err("Logger not initialized.".into())
    }
}

// This is an adaptation of the Iowriter from bincode.
pub struct OwningIoWriter<W: Write> {
    writer: W,
    bytes_written: usize,
}

impl<'a, W: Write> OwningIoWriter<W> {
    pub fn new(writer: W) -> Self {
        Self {
            writer,
            bytes_written: 0,
        }
    }

    pub fn bytes_written(&self) -> usize {
        self.bytes_written
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
    encoder: EncoderImpl<OwningIoWriter<File>, Configuration>,
}

impl SimpleFileWriter {
    pub fn new(path: &PathBuf) -> CuResult<Self> {
        let file = std::fs::OpenOptions::new()
            .create(true)
            .write(true)
            .open(path)
            .map_err(|e| format!("Failed to open file: {:?}", e))?;

        let writer = OwningIoWriter::new(file);
        let encoder = EncoderImpl::new(writer, bincode::config::standard());

        Ok(SimpleFileWriter { encoder})
    }
}

impl WriteStream<CuLogEntry> for SimpleFileWriter {
    #[inline(always)]
    fn log(&mut self, obj: &CuLogEntry) -> CuResult<()> {
        obj.encode(&mut self.encoder).map_err(|e| format!("Failed to write to file: {:?}", e))?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use crate::CuLogEntry;
    use bincode::config::standard;
    use cu29_log::value::Value;

    #[test]
    fn test_encode_decode_structured_log() {
        let log_entry = CuLogEntry {
            time: 0.into(),
            msg_index: 1,
            paramname_indexes: vec![2, 3],
            params: vec![Value::String("test".to_string())],
        };
        let encoded = bincode::encode_to_vec(&log_entry, standard()).unwrap();
        println!("{:?}", encoded);
        let decoded_tuple: (CuLogEntry, usize) =
            bincode::decode_from_slice(&encoded, standard()).unwrap();
        assert_eq!(log_entry, decoded_tuple.0);
    }
}
