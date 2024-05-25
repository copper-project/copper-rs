use bincode;
use bincode::enc::write::Writer;
pub use copper_value as value;
use kanal::{bounded, Sender};
use lazy_static::lazy_static;
use pretty_hex::pretty_hex;
use std::fmt::Display;
use std::io::{stdout, Write};
use std::sync::{Arc, Mutex};
use std::thread;
use value::Value;

#[allow(dead_code)]
pub const ANONYMOUS: Value = Value::U32(0);

lazy_static! {
    static ref QUEUE: Sender<Value> = initialize_queue();
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

fn initialize_queue() -> Sender<Value> {
    let (sender, receiver) = bounded::<Value>(100);
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
pub fn log(data: Value) {
    // If the queue is closed, we just drop the data.
    if QUEUE.send(data).is_err() {
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
