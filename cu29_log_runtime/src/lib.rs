use std::fs::File;
use std::sync::{Mutex, OnceLock};
use std::io::{BufWriter, Write};
use cu29_clock::RobotClock;
use cu29_log::CuLogEntry;
use cu29_traits::{CuResult, WriteStream};
use log::Log;
use std::path::PathBuf;
use std::sync::Arc;
use bincode::config::Configuration;
use bincode::enc::{Encoder, EncoderImpl};
use bincode::error::EncodeError;
use bincode::enc::write::Writer;
use bincode::enc::Encode;

static WRITER: OnceLock<(Arc<Mutex<dyn WriteStream<CuLogEntry>>>, RobotClock)> = OnceLock::new();

/// The lifetime of this struct is the lifetime of the logger.
pub struct LoggerRuntime {
    #[allow(dead_code)]
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

        let runtime = LoggerRuntime {
            extra_text_logger,
        };
        if let Ok(_) = WRITER.set((Arc::new(Mutex::new(destination)), clock)) { panic!("Logger already initialized."); };
        runtime
    }


    pub fn flush(&self) {
        let d = WRITER.get().map(|(writer, clock)| (Arc::clone(writer), clock));
        if d.is_none() {
            return;
        }
        let (writer, _clock) = d.unwrap();
        writer.lock().unwrap().flush().unwrap();
    }

    pub fn close(&mut self) {
        let d = WRITER.get().map(|(writer, clock)| (Arc::clone(writer), clock));
        if d.is_none() {
            return;
        }
        let (writer, _clock) = d.unwrap();
        writer.lock().unwrap().flush().unwrap();
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
    #[allow(dead_code)] 
    path_to_index: PathBuf,
    #[allow(dead_code)]
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
    let d = WRITER.get().map(|(writer, clock)| (Arc::clone(writer), clock));
    if d.is_none() {
        return Err("Logger not initialized.".into());
    }
    let (writer, clock) = d.unwrap();
    entry.time = clock.now();

    if let Err(err) = writer.lock().unwrap().log(&entry) {
        eprintln!("Failed to log data: {}", err);
    }
    // TODO(gbin): Put that back
    // This is only for debug builds with standard textual logging implemented.
    // #[cfg(debug_assertions)]
    // let index = entry.msg_index;
    // if let Some(index) = &index {
    //     if let Some(ref logger) = self.extra_text_logger {
    //         let stringified = cu29_log::rebuild_logline(index, &cu_log_entry);
    //         match stringified {
    //             Ok(s) => {
    //                 let s = format!("[{}] {}", cu_log_entry.time, s);
    //                 logger.log(
    //                     &Record::builder()
    //                         // TODO: forward this info in the CuLogEntry
    //                         .level(log::Level::Debug)
    //                         // .target("copper")
    //                         //.module_path_static(Some("cu29_log"))
    //                         //.file_static(Some("cu29_log"))
    //                         //.line(Some(0))
    //                         .args(format_args!("{}", s))
    //                         .build(),
    //                 ); // DO NOT TRY to split off this statement.
    //                 // format_args! has to be in the same statement per structure and scoping.
    //             }
    //             Err(e) => {
    //                 eprintln!("Failed to rebuild log line: {}", e);
    //             }
    //         }
    //     }
    // }

    Ok(())
    // if let Some((queue, clock)) = QUEUE_CLOCK.get() {
    //     entry.time = clock.now();
    //     let err = queue
    //         .send(entry)
    //         .map_err(|e| format!("Failed to send data to the logger, did you hold the reference to the logger long enough? {:?}", e).into());
    //     err
    // } else {
    //     Err("Logger not initialized.".into())
    // }
}

// This is an adaptation of the Iowriter from bincode.
pub struct OwningIoWriter<W: Write> {
    writer: BufWriter<W>,
    bytes_written: usize,
}

impl<'a, W: Write> OwningIoWriter<W> {
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

    fn flush(&mut self) -> CuResult<()> {
        self.encoder.writer().flush().map_err(|e| format!("Failed to flush file: {:?}", e))?;
        Ok(())
    }
}

// #[cfg(test)]
// mod tests {
//     use crate::CuLogEntry;
//     use bincode::config::standard;
//     use cu29_log::value::Value;
//
//     #[test]
//     fn test_encode_decode_structured_log() {
//         let log_entry = CuLogEntry {
//             time: 0.into(),
//             msg_index: 1,
//             paramname_indexes: vec![2, 3],
//             params: vec![Value::String("test".to_string())],
//         };
//         let encoded = bincode::encode_to_vec(&log_entry, standard()).unwrap();
//         println!("{:?}", encoded);
//         let decoded_tuple: (CuLogEntry, usize) =
//             bincode::decode_from_slice(&encoded, standard()).unwrap();
//         assert_eq!(log_entry, decoded_tuple.0);
//     }
// }
