#![cfg(all(test, feature = "std"))]

use bincode::{Decode, Encode};
use cu29::bincode::de::Decoder;
use cu29::bincode::enc::Encoder;
use cu29::bincode::error::{DecodeError, EncodeError};
use cu29::curuntime::KeyFramePayloadReader;
use cu29::prelude::*;
use cu29_export::keyframes_reader;
use cu29_unifiedlog::memmap::{MmapSectionStorage, MmapUnifiedLoggerWrite};
use cu29_unifiedlog::{UnifiedLogger, UnifiedLoggerBuilder, UnifiedLoggerIOReader};
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::Path;
use std::sync::{Arc, Mutex};
use std::time::Duration;

#[derive(Default, Debug, Clone, Encode, Decode, Serialize, Deserialize, Reflect)]
struct WorkMsg(u32);

#[derive(Reflect)]
struct SlowSource {
    next: u32,
    #[reflect(ignore)]
    delay: Duration,
}

impl Freezable for SlowSource {
    fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.next, encoder)
    }

    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        self.next = Decode::decode(decoder)?;
        Ok(())
    }
}

impl CuSrcTask for SlowSource {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(WorkMsg);

    fn new(config: Option<&ComponentConfig>, _resources: ()) -> CuResult<Self> {
        let config = config.ok_or_else(|| CuError::from("SlowSource needs config"))?;
        let sleep_ms = config
            .get::<u64>("sleep_ms")?
            .ok_or_else(|| CuError::from("SlowSource needs sleep_ms"))?;
        Ok(Self {
            next: 0,
            delay: Duration::from_millis(sleep_ms),
        })
    }

    fn process(&mut self, _ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        std::thread::sleep(self.delay);
        self.next += 1;
        output.set_payload(WorkMsg(self.next));
        Ok(())
    }
}

#[derive(Reflect)]
struct SlowTask {
    completed: u32,
    #[reflect(ignore)]
    delay: Duration,
}

impl Freezable for SlowTask {
    fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.completed, encoder)
    }

    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        self.completed = Decode::decode(decoder)?;
        Ok(())
    }
}

impl CuTask for SlowTask {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(WorkMsg);
    type Output<'m> = output_msg!(WorkMsg);

    fn new(config: Option<&ComponentConfig>, _resources: ()) -> CuResult<Self> {
        let config = config.ok_or_else(|| CuError::from("SlowTask needs config"))?;
        let sleep_ms = config
            .get::<u64>("sleep_ms")?
            .ok_or_else(|| CuError::from("SlowTask needs sleep_ms"))?;
        Ok(Self {
            completed: 0,
            delay: Duration::from_millis(sleep_ms),
        })
    }

    fn process(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        std::thread::sleep(self.delay);
        self.completed += 1;
        output.set_payload(WorkMsg(
            input.payload().map(|payload| payload.0).unwrap_or_default() + 1,
        ));
        Ok(())
    }
}

#[derive(Reflect)]
struct OutputSink;

impl Freezable for OutputSink {}

impl CuSinkTask for OutputSink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(WorkMsg);

    fn new(_config: Option<&ComponentConfig>, _resources: ()) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, _ctx: &CuContext, _input: &Self::Input<'_>) -> CuResult<()> {
        Ok(())
    }
}

#[copper_runtime(config = "tests/async_keyframes_config.ron")]
struct AsyncKeyframeApp {}

fn build_logger(path: &Path) -> CuResult<Arc<Mutex<MmapUnifiedLoggerWrite>>> {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent)
            .map_err(|error| CuError::new_with_cause("create log directory failed", error))?;
    }
    let UnifiedLogger::Write(writer) = UnifiedLoggerBuilder::new()
        .write(true)
        .create(true)
        .preallocated_size(16 * 1024 * 1024)
        .file_base_name(path)
        .build()
        .map_err(|error| CuError::new_with_cause("logger initialization failed", error))?
    else {
        return Err(CuError::from("logger builder did not return a writer"));
    };
    Ok(Arc::new(Mutex::new(writer)))
}

#[test]
fn continuously_busy_async_workers_produce_complete_keyframes() -> CuResult<()> {
    const ITERATIONS: usize = 100;
    const COMPONENTS: usize = 4;

    let temp_dir = tempfile::tempdir()
        .map_err(|error| CuError::new_with_cause("create temp directory failed", error))?;
    let log_path = temp_dir.path().join("async_keyframes.copper");
    let logger = build_logger(&log_path)?;
    let app = AsyncKeyframeApp::builder()
        .with_logger::<MmapSectionStorage, MmapUnifiedLoggerWrite>(logger)
        .build()?;

    let mut running = app.start()?;
    for _ in 0..ITERATIONS {
        running.run_one_iteration()?;
        std::thread::sleep(Duration::from_millis(5));
    }
    let stopped = running.stop()?;
    drop(stopped);

    let UnifiedLogger::Read(reader) = UnifiedLoggerBuilder::new()
        .file_base_name(&log_path)
        .build()
        .map_err(|error| CuError::new_with_cause("open keyframe log failed", error))?
    else {
        return Err(CuError::from("logger builder did not return a reader"));
    };
    let mut reader = UnifiedLoggerIOReader::new(reader, UnifiedLogType::FrozenTasks);
    let keyframes: Vec<_> = keyframes_reader(&mut reader).collect();
    assert_eq!(keyframes.len(), ITERATIONS);
    for keyframe in &keyframes {
        let mut frames = KeyFramePayloadReader::new(keyframe)?;
        for _ in 0..COMPONENTS {
            let _ = frames.next_frame()?;
        }
        frames.finish()?;
    }
    Ok(())
}
