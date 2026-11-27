//! Bounded handoff of canonical structured records to the runtime output worker.

use bincode::{Encode, config::standard, enc::Encoder, enc::write::Writer, error::EncodeError};
use cu29_log::CuLogEntry;
use cu29_traits::{CuError, CuResult, WriteStream};
use cu29_unifiedlog::{LogStream, SectionStorage, UnifiedLogWrite};
use std::fmt::{self, Debug};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::Thread;

// One batch can fill while the preceding batch is being written. Both are
// allocated at construction. Length prefixes are private to this handoff.
const BATCH_BYTES: usize = 64 * 1024;
const PREFIX: usize = size_of::<u32>();

fn combine_output_results(first: CuResult<()>, next: CuResult<()>) -> CuResult<()> {
    match (first, next) {
        (Err(first), Err(next)) => {
            let context = format!("{first}; output cleanup also failed: {next}");
            Err(first.add_cause(&context))
        }
        (Err(error), _) | (_, Err(error)) => Err(error),
        _ => Ok(()),
    }
}

struct Batch {
    bytes: Box<[u8]>,
    used: usize,
}

impl Batch {
    fn new() -> Self {
        Self {
            bytes: vec![0; BATCH_BYTES].into_boxed_slice(),
            used: 0,
        }
    }

    fn push(&mut self, entry: &CuLogEntry) -> Result<usize, EncodeError> {
        let record_start = self.used + PREFIX;
        let space = self
            .bytes
            .get_mut(record_start..)
            .ok_or(EncodeError::UnexpectedEnd)?;
        let len = bincode::encode_into_slice(entry, space, standard())?;
        self.bytes[self.used..record_start].copy_from_slice(&(len as u32).to_le_bytes());
        self.used = record_start + len;
        Ok(len)
    }
}

struct Record<'a>(&'a [u8]);

impl Encode for Record<'_> {
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        encoder.writer().write(self.0)
    }
}

trait Destination: Send {
    fn write(&mut self, bytes: &[u8]) -> CuResult<()>;
    fn flush(&mut self) -> CuResult<()>;
}

struct LocalDestination<S: SectionStorage, L: UnifiedLogWrite<S>, F> {
    stream: LogStream<S, L>,
    on_record: F,
}

impl<S: SectionStorage, L: UnifiedLogWrite<S>, F: FnMut(&[u8]) + Send> Destination
    for LocalDestination<S, L, F>
{
    fn write(&mut self, bytes: &[u8]) -> CuResult<()> {
        self.stream.log(&Record(bytes))?;
        (self.on_record)(bytes);
        Ok(())
    }

    fn flush(&mut self) -> CuResult<()> {
        <LogStream<S, L> as WriteStream<CuLogEntry>>::flush(&mut self.stream)
    }
}

struct Pending {
    batch: Batch,
    worker: Option<Thread>,
    dropped: u64,
}

struct Drain {
    batch: Batch,
    destination: Box<dyn Destination>,
    error: Option<CuError>,
}

struct Shared {
    pending: Mutex<Pending>,
    drain: Mutex<Drain>,
    ready: AtomicBool,
}

/// Generated local structured sink and its output-worker handle.
#[doc(hidden)]
#[derive(Clone)]
pub struct BufferedStructuredLog {
    shared: Arc<Shared>,
    last_bytes: usize,
}

impl Debug for BufferedStructuredLog {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("BufferedStructuredLog")
    }
}

impl BufferedStructuredLog {
    pub fn new<S: SectionStorage + 'static, L: UnifiedLogWrite<S> + 'static>(
        stream: LogStream<S, L>,
        on_record: impl FnMut(&[u8]) + Send + 'static,
    ) -> Self {
        Self::with_destination(Box::new(LocalDestination { stream, on_record }))
    }

    fn with_destination(destination: Box<dyn Destination>) -> Self {
        Self {
            shared: Arc::new(Shared {
                ready: AtomicBool::new(false),
                pending: Mutex::new(Pending {
                    batch: Batch::new(),
                    worker: None,
                    dropped: 0,
                }),
                drain: Mutex::new(Drain {
                    batch: Batch::new(),
                    destination,
                    error: None,
                }),
            }),
            last_bytes: 0,
        }
    }

    pub(super) fn set_worker(&self, worker: Option<Thread>) {
        self.shared.pending.lock().unwrap().worker = worker;
    }

    pub(super) fn drain_pending(&self) -> bool {
        if self.shared.ready.load(Ordering::Acquire) {
            let _ = self.drain(false);
            return true;
        }
        false
    }

    fn drain(&self, flush: bool) -> CuResult<()> {
        // Explicit flush serializes with an in-flight batch. A producer only
        // needs pending, which is released before touching the destination.
        let mut writer = self.shared.drain.lock().unwrap();
        let Drain {
            batch,
            destination,
            error,
        } = &mut *writer;
        let dropped = {
            let mut pending = self.shared.pending.lock().unwrap();
            std::mem::swap(batch, &mut pending.batch);
            self.shared.ready.store(false, Ordering::Release);
            if flush {
                std::mem::take(&mut pending.dropped)
            } else {
                0
            }
        };
        if error.is_none() {
            let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                let mut records = &batch.bytes[..batch.used];
                while !records.is_empty() {
                    let (prefix, tail) = records.split_at(PREFIX);
                    let len = u32::from_le_bytes(prefix.try_into().unwrap()) as usize;
                    let (record, rest) = tail.split_at(len);
                    destination.write(record)?;
                    records = rest;
                }
                Ok(())
            }))
            .unwrap_or_else(|_| Err(CuError::from("Structured log destination panicked")));
            *error = result.err();
        }
        batch.used = 0;
        let mut result = error.clone().map_or(Ok(()), Err);
        if flush {
            let flushed =
                std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| destination.flush()))
                    .unwrap_or_else(|_| Err(CuError::from("Structured log flush panicked")));
            result = combine_output_results(result, flushed);
            if dropped > 0 {
                result = combine_output_results(
                    result,
                    Err(CuError::from(format!(
                        "Structured log buffer overflow: {dropped} records dropped"
                    ))),
                );
            }
        }
        result
    }
}

impl WriteStream<CuLogEntry> for BufferedStructuredLog {
    fn log(&mut self, entry: &CuLogEntry) -> CuResult<()> {
        let mut pending = self.shared.pending.lock().unwrap();
        let first = pending.batch.used == 0;
        self.last_bytes = match pending.batch.push(entry) {
            Ok(bytes) => bytes,
            Err(EncodeError::UnexpectedEnd) => {
                pending.dropped = pending.dropped.saturating_add(1);
                self.last_bytes = 0;
                // Keep live listeners enabled. Explicit flush reports local loss.
                return Ok(());
            }
            Err(error) => {
                return Err(CuError::new_with_cause(
                    "Cannot encode structured log",
                    error,
                ));
            }
        };
        if first {
            self.shared.ready.store(true, Ordering::Release);
        }
        if let Some(worker) = &pending.worker {
            if first {
                worker.unpark();
            }
            return Ok(());
        }
        // During setup or after the runtime stops, there is no worker.
        drop(pending);
        self.drain(false)
    }

    fn last_log_bytes(&self) -> Option<usize> {
        Some(self.last_bytes)
    }

    fn flush(&mut self) -> CuResult<()> {
        self.drain(true)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cu29_log::CuLogLevel;

    struct Capture(Arc<Mutex<Vec<Vec<u8>>>>);

    impl Destination for Capture {
        fn write(&mut self, bytes: &[u8]) -> CuResult<()> {
            self.0.lock().unwrap().push(bytes.to_vec());
            Ok(())
        }

        fn flush(&mut self) -> CuResult<()> {
            Ok(())
        }
    }

    #[test]
    fn explicit_flush_reports_a_background_error_and_still_flushes_the_sink() {
        struct FailingSink(Arc<AtomicBool>);
        impl Destination for FailingSink {
            fn write(&mut self, _: &[u8]) -> CuResult<()> {
                Err(CuError::from("injected write failure"))
            }
            fn flush(&mut self) -> CuResult<()> {
                self.0.store(true, Ordering::Release);
                Ok(())
            }
        }
        let flushed = Arc::new(AtomicBool::new(false));
        let mut text =
            BufferedStructuredLog::with_destination(Box::new(FailingSink(flushed.clone())));
        text.set_worker(Some(std::thread::current()));
        text.log(&CuLogEntry::new(1, CuLogLevel::Info)).unwrap();
        text.drain_pending();
        assert!(
            text.flush()
                .unwrap_err()
                .to_string()
                .contains("injected write failure")
        );
        assert!(flushed.load(Ordering::Acquire));
    }

    #[test]
    fn overflow_preserves_accepted_records_and_is_reported_at_flush() {
        let saved = Arc::new(Mutex::new(Vec::new()));
        let mut sender = BufferedStructuredLog::with_destination(Box::new(Capture(saved.clone())));
        sender.set_worker(Some(std::thread::current()));
        let mut entry = CuLogEntry::new(1, CuLogLevel::Info);
        entry
            .params
            .push(cu29_value::Value::String("x".repeat(1000)));
        let mut expected = Vec::new();
        for index in 0..100 {
            entry.msg_index = index;
            sender.log(&entry).unwrap();
            if sender.last_bytes != 0 {
                expected.push(bincode::encode_to_vec(&entry, standard()).unwrap());
            }
        }
        let lost = 100 - expected.len();
        assert!(lost > 0 && lost < 100);
        sender.drain_pending();
        let error = sender.flush().unwrap_err();
        assert!(
            error
                .to_string()
                .contains(&format!("{lost} records dropped"))
        );
        assert_eq!(*saved.lock().unwrap(), expected);
        sender.log(&entry).unwrap();
        sender.flush().unwrap();
        assert_eq!(saved.lock().unwrap().len(), expected.len() + 1);
    }
}
