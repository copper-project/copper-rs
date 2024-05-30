use bincode::enc::write::Writer;
use bincode_derive::{Decode, Encode};
use copper_traits::{CuResult, Stream};
pub use copper_value as value;
use copper_value::Value;
use kanal::{bounded, Sender};
use once_cell::sync::OnceCell;
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
    let config = bincode::config::standard();

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
