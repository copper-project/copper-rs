use bincode::enc::write::Writer;
use bincode_derive::{Decode, Encode};
pub use copper_value as value; // Part of the API, do not remove.
use copper_value::Value;
use kanal::{bounded, Sender};
use lazy_static::lazy_static;
use pretty_hex::pretty_hex;
use std::fmt::Display;
use std::io::{stdout, Write};
use std::sync::{Arc, Mutex};
use std::thread;

#[allow(dead_code)]
pub const ANONYMOUS: u32 = 0;

#[derive(Debug, Encode, Decode)]
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

lazy_static! {
    static ref QUEUE: Sender<CuLogEntry> = initialize_queue();
}

/// The lifetime of this struct is the lifetime of the logger.
pub struct LoggerRuntime {}

impl LoggerRuntime {
    pub fn init() -> Self {
        lazy_static::initialize(&QUEUE);
        LoggerRuntime {}
    }

    pub fn close(&self) {
        QUEUE.close();
    }
}

impl Drop for LoggerRuntime {
    fn drop(&mut self) {
        self.close();
    }
}

fn initialize_queue() -> Sender<CuLogEntry> {
    let (sender, receiver) = bounded::<CuLogEntry>(100);
    let config = bincode::config::standard();

    let handle = thread::spawn(move || loop {
        if let Ok(data) = receiver.recv() {
            let binary = bincode::encode_to_vec(&data, config).unwrap();
            println!("{}", pretty_hex(&binary.as_slice()));
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
#[inline]
pub fn log(entry: CuLogEntry) {
    // If the queue is closed, we just drop the data.
    if QUEUE.send(entry).is_err() {
        if !QUEUE.is_closed() {
            eprintln!("Copper: Failed to send data to the logger, some data got dropped.");
            if QUEUE.is_full() {
                eprintln!(
                    "Copper: The logger queue is full, consider increasing the size of the queue."
                );
            }
        }
    }
}
