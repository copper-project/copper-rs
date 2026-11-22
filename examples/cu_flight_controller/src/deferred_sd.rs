//! Encode directly into a bounded byte ring. Publish a descriptor only after
//! the whole record fits. The consumer owns all physical Copper log streams.
use alloc::{sync::Arc, vec::Vec};

use bincode::{
    Encode,
    enc::{Encoder, write::Writer},
    error::EncodeError,
};

use cu_micoairh743::{LogStorage, Logger};
use cu29::prelude::*;
use spin::Mutex;

mod queue;
use queue::Record;
pub use queue::{Input, Output, POOL_BYTES, queue};

pub struct PendingSection {
    stream: usize,
    input: Arc<Mutex<Input>>,
    used: usize,
    limit: usize,
    closed: bool,
    kind: UnifiedLogType,
}
impl SectionStorage for PendingSection {
    fn initialize<E: Encode>(&mut self, _: &E) -> Result<usize, EncodeError> {
        Ok(0)
    }
    fn post_update_header<E: Encode>(&mut self, _: &E) -> Result<usize, EncodeError> {
        Ok(0)
    }
    fn append<E: Encode>(&mut self, entry: &E) -> Result<usize, EncodeError> {
        if self.used >= self.limit {
            return Err(EncodeError::UnexpectedEnd);
        }
        let size = self.input.lock().append(self.stream, entry)?;
        self.used += size;
        Ok(size)
    }
    fn is_sealed(&self) -> bool {
        self.closed
    }
    fn flush(&mut self) -> CuResult<usize> {
        if !self.closed && self.kind != UnifiedLogType::RuntimeLifecycle {
            self.input
                .lock()
                .records
                .push(Record::Flush {
                    stream: self.stream,
                })
                .map_err(|_| CuError::from("SD flush descriptor queue full"))?;
        }
        self.closed = true;
        Ok(self.used)
    }
}

type DiskStream = LogStream<LogStorage, Logger>;
pub struct Recorder {
    disk: Arc<Mutex<Logger>>,
    input: Arc<Mutex<Input>>,
    keys: Vec<(UnifiedLogType, usize, SectionContext)>,
    last_status: Mutex<UnifiedLogStatus>,
    running: bool,
    streams: Vec<DiskStream>,
}

pub struct Worker {
    streams: Vec<DiskStream>,
    failed: bool,
    output: Output,
    written: usize,
    input: Arc<Mutex<Input>>,
}
struct Encoded<'a>(&'a [u8], &'a [u8]);
impl Encode for Encoded<'_> {
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        encoder.writer().write(self.0)?;
        encoder.writer().write(self.1)
    }
}

impl Recorder {
    pub fn new(disk: Logger, input: Input) -> Self {
        let last_status = Mutex::new(disk.status());
        Self {
            disk: Arc::new(Mutex::new(disk)),
            last_status,
            input: Arc::new(Mutex::new(input)),
            keys: Vec::new(),
            running: false,
            streams: Vec::new(),
        }
    }
    pub fn worker(&mut self, output: Output) -> Worker {
        self.running = true;
        Worker {
            streams: core::mem::take(&mut self.streams),
            failed: false,
            output,
            written: 0,
            input: self.input.clone(),
        }
    }
}
impl Worker {
    pub fn service(&mut self) {
        let Ok(record) = self.output.records.pop() else {
            return;
        };
        let (stream, size) = match record {
            Record::Entry { stream, size } => (stream, size),
            Record::Flush { stream } => {
                if !self.failed {
                    let result =
                        <DiskStream as WriteStream<Encoded>>::flush(&mut self.streams[stream]);
                    if let Err(error) = result {
                        self.failed = true;
                        defmt::error!("SD flush failed: {}", error);
                    }
                }
                return;
            }
        };
        let bytes = self
            .output
            .bytes
            .read_chunk(size)
            .expect("published record has complete bytes");
        let (first, second) = bytes.as_slices();
        if !self.failed
            && let Err(error) = self.streams[stream].log(&Encoded(first, second))
        {
            self.failed = true;
            defmt::error!("SD recording stopped: {}", error);
        }
        if self.failed {
            self.input.lock().dropped += 1;
        } else {
            self.written += 1;
        }
        bytes.commit_all();
    }
    pub fn finish(&mut self) -> CuResult<()> {
        while !self.output.records.is_empty() {
            self.service();
        }
        if self.failed {
            return Err(CuError::from("SD recording failed"));
        }
        for stream in &mut self.streams {
            <DiskStream as WriteStream<Encoded>>::flush(stream)?;
        }
        Ok(())
    }
    pub fn counts(&self) -> (usize, usize) {
        (self.written, self.input.lock().dropped)
    }
}

impl UnifiedLogWrite<PendingSection> for Recorder {
    fn add_section(
        &mut self,
        kind: UnifiedLogType,
        size: usize,
    ) -> CuResult<SectionHandle<PendingSection>> {
        self.add_section_with_context(kind, size, SectionContext::default())
    }
    fn add_section_with_context(
        &mut self,
        kind: UnifiedLogType,
        size: usize,
        context: SectionContext,
    ) -> CuResult<SectionHandle<PendingSection>> {
        let key = (kind, size, context);
        let stream = if let Some(index) = self.keys.iter().position(|item| *item == key) {
            index
        } else {
            if self.running {
                return Err(CuError::from("unregistered flight log stream"));
            }
            let index = self.keys.len();
            self.streams.push(LogStream::with_context(
                kind,
                self.disk.clone(),
                size,
                context,
            )?);
            self.keys.push(key);
            index
        };
        SectionHandle::create(
            SectionHeader {
                entry_type: kind,
                context,
                ..Default::default()
            },
            PendingSection {
                stream,
                input: self.input.clone(),
                used: 0,
                closed: false,
                kind,
                limit: size.max(1).min(u32::MAX as usize - POOL_BYTES),
            },
        )
    }
    fn seal_metadata<C: Encode>(
        &mut self,
        metadata: &ApplicationMetadata,
        catalog: Option<&C>,
    ) -> CuResult<()> {
        self.disk.lock().seal_metadata(metadata, catalog)
    }
    fn construction_context(&mut self, instance: u32, mission: u32) -> CuResult<SectionContext> {
        self.disk.lock().construction_context(instance, mission)
    }
    fn try_flush_section(&mut self, section: &mut SectionHandle<PendingSection>) -> CuResult<()> {
        section.get_storage_mut().flush()?;
        section.mark_closed();
        Ok(())
    }
    fn flush_section(&mut self, section: &mut SectionHandle<PendingSection>) {
        if let Err(error) = self.try_flush_section(section) {
            defmt::error!("SD flush failed: {}", error);
        }
    }
    fn status(&self) -> UnifiedLogStatus {
        let mut cached = self.last_status.lock();
        if let Some(disk) = self.disk.try_lock() {
            *cached = disk.status();
        }
        UnifiedLogStatus {
            total_used_space: cached.total_used_space,
            total_allocated_space: cached.total_allocated_space,
        }
    }
}
