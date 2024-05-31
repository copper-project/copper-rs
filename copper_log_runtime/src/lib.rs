use bincode_derive::{Decode, Encode};
use copper_traits::{CuResult, Stream};
pub use copper_value as value;
use copper_value::Value;
use kanal::{bounded, Sender};
use once_cell::sync::OnceCell;
use serde::{Deserialize, Serialize};
use std::fmt::Display;
use std::io::{stdout, Write};
use std::sync::{Arc, Mutex};
use std::thread;

// The logging system is basically a global queue.
static QUEUE: OnceCell<Sender<CuLogEntry>> = OnceCell::new();

#[allow(dead_code)]
pub const ANONYMOUS: u32 = 0;

/// The lifetime of this struct is the lifetime of the logger.
pub struct LoggerRuntime {}

#[derive(Debug, Encode, Decode, Serialize, Deserialize, PartialEq)]
pub struct CuLogEntry {
    pub msg_index: u32,
    pub paramname_indexes: Vec<u32>,
    pub params: Vec<Value>,
}

impl Display for CuLogEntry {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "CuLogEntry {{ msg_index: {}, paramname_indexes: {:?}, params: {:?} }}",
            self.msg_index, self.paramname_indexes, self.params
        )
    }
}

impl CuLogEntry {
    pub fn new(msg_index: u32) -> Self {
        CuLogEntry {
            msg_index,
            paramname_indexes: Vec::new(),
            params: Vec::new(),
        }
    }

    pub fn add_param(&mut self, paramname_index: u32, param: Value) {
        self.paramname_indexes.push(paramname_index);
        self.params.push(param);
    }
}
impl LoggerRuntime {
    pub fn init(destination: impl Stream + 'static) -> Self {
        QUEUE
            .set(initialize_queue(destination))
            .expect("Failed to initialize the logger queue.");
        LoggerRuntime {}
    }

    pub fn close(&self) {
        QUEUE
            .get()
            .expect("Logger Runtime closed before beeing open.")
            .close();
    }
}

impl Drop for LoggerRuntime {
    fn drop(&mut self) {
        self.close();
    }
}

fn initialize_queue(mut destination: impl Stream + 'static) -> Sender<CuLogEntry> {
    let (sender, receiver) = bounded::<CuLogEntry>(100);
    let handle = thread::spawn(move || loop {
        if let Ok(data) = receiver.recv() {
            if let Err(err) = destination.log(&data) {
                eprintln!("Failed to log data: {}", err);
            }
        } else {
            break;
        }
    });
    let handle = Arc::new(Mutex::new(Some(handle))).clone();
    ctrlc::set_handler({
        let sender_clone = sender.clone();
        move || {
            sender_clone.close();
            let handle = handle.lock().unwrap().take().unwrap();
            handle.join().expect("Failed to join the logging thread");
            stdout().flush().expect("Failed to flush stdout");
        }
    })
    .expect("Failed to set the Ctrl-C handler");
    sender
}

/// Function called from generated code to log data.
/// It moves entry by design, it will be absorbed in the queue.
#[inline]
pub fn log(entry: CuLogEntry) -> CuResult<()> {
    if let Some(queue) = QUEUE.get() {
        queue
            .send(entry)
            .map_err(|_| "Failed to send data to the logger.".into())
    } else {
        Err("Logger not initialized.".into())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::CuLogEntry;
    use bincode::config::standard;
    use copper_value::Value;

    #[test]
    fn test_encode_decode_structured_log() {
        let log_entry = CuLogEntry {
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
