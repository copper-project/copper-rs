//! Static structured-log fan-out at the local encoder's byte boundary.

use bincode::{
    Encode,
    enc::{Encoder, EncoderImpl, write::Writer},
    error::EncodeError,
};
use cu29_log::CuLogEntry;
use cu29_traits::{CuResult, WriteStream};
use cu29_unifiedlog::{LogStream, SectionStorage, UnifiedLogWrite};
use std::{cell::RefCell, fmt::Debug};

/// Statically composed destinations for one local serialization attempt.
/// A local section rollover starts a fresh attempt; only successful local writes
/// are committed. Destination overflow must not fail the local write.
pub trait StructuredLogOutput: Debug + Send + Sync {
    fn begin(&mut self);
    fn write(&mut self, bytes: &[u8]);
    fn finish(&mut self, success: bool);
}

impl StructuredLogOutput for () {
    fn begin(&mut self) {}
    fn write(&mut self, _: &[u8]) {}
    fn finish(&mut self, _: bool) {}
}
impl<O: StructuredLogOutput> StructuredLogOutput for Option<O> {
    fn begin(&mut self) {
        if let Some(output) = self {
            output.begin();
        }
    }
    fn write(&mut self, bytes: &[u8]) {
        if let Some(output) = self {
            output.write(bytes);
        }
    }
    fn finish(&mut self, success: bool) {
        if let Some(output) = self {
            output.finish(success);
        }
    }
}
impl<A: StructuredLogOutput, B: StructuredLogOutput> StructuredLogOutput for (A, B) {
    fn begin(&mut self) {
        self.0.begin();
        self.1.begin();
    }
    fn write(&mut self, bytes: &[u8]) {
        self.0.write(bytes);
        self.1.write(bytes);
    }
    fn finish(&mut self, success: bool) {
        self.0.finish(success);
        self.1.finish(success);
    }
}

/// Copies locally encoded bytes into bounded destination buffers during the
/// existing serialization pass. Framing, hashing and FEC run on sender workers.
/// Local errors propagate; streamed overflow leaves the local record intact.
pub struct StructuredLogStream<S: SectionStorage, L: UnifiedLogWrite<S>, O> {
    local: LogStream<S, L>,
    outputs: O,
}
impl<S: SectionStorage, L: UnifiedLogWrite<S>, O: StructuredLogOutput>
    StructuredLogStream<S, L, O>
{
    pub fn new(local: LogStream<S, L>, outputs: O) -> Self {
        Self { local, outputs }
    }
}
impl<S: SectionStorage, L: UnifiedLogWrite<S>, O: Debug> Debug for StructuredLogStream<S, L, O> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("StructuredLogStream")
            .field("local", &self.local)
            .field("outputs", &self.outputs)
            .finish()
    }
}
impl<S: SectionStorage, L: UnifiedLogWrite<S>, O: StructuredLogOutput> WriteStream<CuLogEntry>
    for StructuredLogStream<S, L, O>
{
    fn log(&mut self, entry: &CuLogEntry) -> CuResult<()> {
        let captured = CapturedEntry {
            entry,
            outputs: RefCell::new(&mut self.outputs),
        };
        let result = self.local.log(&captured);
        self.outputs.finish(result.is_ok());
        result
    }
    fn flush(&mut self) -> CuResult<()> {
        <LogStream<S, L> as WriteStream<CuLogEntry>>::flush(&mut self.local)
    }
    fn last_log_bytes(&self) -> Option<usize> {
        <LogStream<S, L> as WriteStream<CuLogEntry>>::last_log_bytes(&self.local)
    }
}

struct CapturedEntry<'a, O> {
    entry: &'a CuLogEntry,
    outputs: RefCell<&'a mut O>,
}
impl<O: StructuredLogOutput> Encode for CapturedEntry<'_, O> {
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        let mut outputs = self.outputs.borrow_mut();
        outputs.begin();
        let config = *encoder.config();
        let writer = CapturingWriter {
            local: encoder.writer(),
            outputs: &mut **outputs,
        };
        self.entry.encode(&mut EncoderImpl::new(writer, config))
    }
}
struct CapturingWriter<'a, W, O> {
    local: &'a mut W,
    outputs: &'a mut O,
}
impl<W: Writer, O: StructuredLogOutput> Writer for CapturingWriter<'_, W, O> {
    fn write(&mut self, bytes: &[u8]) -> Result<(), EncodeError> {
        self.local.write(bytes)?;
        self.outputs.write(bytes);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cu29_log::CuLogLevel;
    use cu29_traits::UnifiedLogType;
    use cu29_unifiedlog::{NoopLogger, NoopSectionStorage};
    use std::sync::{Arc, Mutex};

    #[derive(Debug, Default)]
    struct Observation {
        attempts: usize,
        commits: usize,
        bytes: Vec<u8>,
    }
    #[derive(Debug)]
    struct Probe(Arc<Mutex<Observation>>);
    impl StructuredLogOutput for Probe {
        fn begin(&mut self) {
            let mut seen = self.0.lock().unwrap();
            seen.attempts += 1;
            seen.bytes.clear();
        }
        fn write(&mut self, bytes: &[u8]) {
            self.0.lock().unwrap().bytes.extend_from_slice(bytes);
        }
        fn finish(&mut self, success: bool) {
            self.0.lock().unwrap().commits += usize::from(success);
        }
    }

    #[test]
    fn static_fanout_observes_one_native_serialization_and_preserves_byte_accounting() {
        let first = Arc::new(Mutex::new(Observation::default()));
        let second = Arc::new(Mutex::new(Observation::default()));
        let local = LogStream::<NoopSectionStorage, _>::new(
            UnifiedLogType::StructuredLogLine,
            Arc::new(Mutex::new(NoopLogger::new())),
            1024,
        )
        .unwrap();
        let mut sink =
            StructuredLogStream::new(local, (Probe(first.clone()), Some(Probe(second.clone()))));
        let mut entry = CuLogEntry::new(47, CuLogLevel::Info);
        entry.paramname_indexes.push(0);
        entry.params.push(cu29_value::Value::U64(42));
        let canonical = bincode::encode_to_vec(&entry, bincode::config::standard()).unwrap();
        sink.log(&entry).unwrap();
        assert_eq!(sink.last_log_bytes(), Some(canonical.len()));
        for observation in [first, second] {
            let seen = observation.lock().unwrap();
            assert_eq!(seen.attempts, 1);
            assert_eq!(seen.commits, 1);
            assert_eq!(seen.bytes, canonical);
        }
    }
    #[test]
    fn section_rollover_commits_only_the_complete_retry() {
        use cu29_unifiedlog::{UnifiedLogger, UnifiedLoggerBuilder};
        let seen = Arc::new(Mutex::new(Observation::default()));
        let logs = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../examples/cu_logstream_demo/logs");
        std::fs::create_dir_all(&logs).unwrap();
        let path = logs.join(format!("structured-rollover-{}.copper", std::process::id()));
        let UnifiedLogger::Write(logger) = UnifiedLoggerBuilder::new()
            .file_base_name(&path)
            .preallocated_size(1024 * 1024)
            .write(true)
            .create(true)
            .build()
            .unwrap()
        else {
            panic!("writer")
        };
        let local = LogStream::new(
            UnifiedLogType::StructuredLogLine,
            Arc::new(Mutex::new(logger)),
            1024,
        )
        .unwrap();
        let mut sink = StructuredLogStream::new(local, Probe(seen.clone()));
        for id in 1..=100 {
            let entry = CuLogEntry::new(id, CuLogLevel::Info);
            sink.log(&entry).unwrap();
            assert_eq!(
                seen.lock().unwrap().bytes,
                bincode::encode_to_vec(&entry, bincode::config::standard()).unwrap()
            );
        }
        let seen = seen.lock().unwrap();
        assert_eq!(seen.commits, 100);
        assert!(seen.attempts > 100, "local sections never rolled over");
    }
}
