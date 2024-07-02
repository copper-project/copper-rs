use bincode::Encode;
use copper_clock::{ClockProvider, RobotClock};
use copper_log::CuLogEntry;
use copper_log_reader::read_interned_strings;
use copper_traits::{CuResult, WriteStream};
use kanal::{bounded, Sender};
use log::{Log, Record};
use once_cell::sync::OnceCell;
use std::path::PathBuf;
use std::sync::Arc;
use std::thread;
use std::thread::{sleep, JoinHandle};
use std::time::Duration;

// The logging system is basically a global queue.
static QUEUE: OnceCell<Sender<CuLogEntry>> = OnceCell::new();

/// The lifetime of this struct is the lifetime of the logger.
pub struct LoggerRuntime {
    clock: RobotClock,
    handle: Option<JoinHandle<()>>,
    extra_text_logger: Option<ExtraTextLogger>,
}

impl ClockProvider for LoggerRuntime {
    fn get_clock(&self) -> RobotClock {
        self.clock.clone()
    }
}

impl LoggerRuntime {
    /// destination is the binary stream in which we will log the structured log.
    /// extra_text_logger is the logger that will log the text logs in real time. This is slow and only for debug builds.
    pub fn init(
        clock_source: RobotClock,
        destination: impl WriteStream<CuLogEntry> + 'static,
        extra_text_logger: Option<ExtraTextLogger>,
    ) -> Self {
        if (!cfg!(debug_assertions)) && extra_text_logger.is_some() {
            eprintln!("Extra text logger is only available in debug builds. Ignoring the extra text logger.");
        };

        let mut runtime = LoggerRuntime {
            clock: clock_source,
            extra_text_logger,
            handle: None,
        };
        let (s, handle) = runtime.initialize_queue(destination);
        QUEUE
            .set(s)
            .expect("Failed to initialize the logger queue.");
        runtime.handle = Some(handle);
        runtime
    }

    fn initialize_queue(
        &self,
        mut destination: impl WriteStream<CuLogEntry> + 'static,
    ) -> (Sender<CuLogEntry>, JoinHandle<()>) {
        let (sender, receiver) = bounded::<CuLogEntry>(100);

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
        let clock = self.clock.clone();

        let handle = thread::spawn(move || {
            let receiver = receiver.clone();
            loop {
                if let Ok(mut cu_log_entry) = receiver.recv() {
                    // We don't need to be precise on this clock.
                    // If the user wants to really log a clock they should add it as a structured field.
                    cu_log_entry.time = clock.now();
                    if let Err(err) = destination.log(&cu_log_entry) {
                        eprintln!("Failed to log data: {}", err);
                    }

                    // This is only for debug builds with standard textual logging implemented.
                    #[cfg(debug_assertions)]
                    if let Some(index) = &index {
                        if let Some(ref logger) = extra_text_logger {
                            let stringified = copper_log::rebuild_logline(index, &cu_log_entry);
                            match stringified {
                                Ok(s) => {
                                    let s = format!("[{}] {}", cu_log_entry.time, s);
                                    logger.log(
                                        &Record::builder()
                                            // TODO: forward this info in the CuLogEntry
                                            .level(log::Level::Debug)
                                            // .target("copper")
                                            //.module_path_static(Some("copper_log"))
                                            //.file_static(Some("copper_log"))
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
        QUEUE.get().is_some()
    }

    pub fn flush(&self) {
        if let Some(queue) = QUEUE.get() {
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
        let queue = QUEUE.get();
        if queue.is_none() {
            eprintln!("Logger closed before it was initialized.");
            return;
        }
        self.flush();
        queue.unwrap().close();
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
#[inline]
pub fn log(entry: CuLogEntry) -> CuResult<()> {
    if let Some(queue) = QUEUE.get() {
        let err = queue
            .send(entry)
            .map_err(|e| format!("Failed to send data to the logger, did you hold the reference to the logger long enough? {:?}", e).into());
        err
    } else {
        Err("Logger not initialized.".into())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::CuLogEntry;
    use bincode::config::standard;
    use copper_log::value::Value;

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

    #[test]
    #[ignore] // FIXME: the serde sere works but deser doesn't with the current implementation
    fn test_serialize_deserialize_structured_log() {
        // tests compatibility with std serde
        let log_entry = CuLogEntry {
            time: 0.into(),
            msg_index: 1,
            paramname_indexes: vec![2, 3],
            params: vec![Value::String("test".to_string())],
        };
        let v = bincode::serde::encode_to_vec(&log_entry, standard()).unwrap();
        // Should be [1, 2, 2, 3, 1, 12, 4, 116, 101, 115, 116]
        println!("{:?}", v);
        let decoded: (CuLogEntry, usize) =
            bincode::serde::decode_from_slice(v.as_slice(), standard()).unwrap();
        assert_eq!(log_entry, decoded.0);
    }
}
