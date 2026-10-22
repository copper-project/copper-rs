#![cfg(all(test, feature = "std"))]

use bincode::{Decode, Encode};
use cu29::bincode::de::Decoder;
use cu29::bincode::enc::Encoder;
use cu29::bincode::error::{DecodeError, EncodeError};
use cu29::curuntime::KeyFramePayloadReader;
use cu29::prelude::*;
use cu29_export::{copperlists_reader, keyframes_reader};
use cu29_unifiedlog::memmap::{MmapSectionStorage, MmapUnifiedLoggerWrite};
use cu29_unifiedlog::{UnifiedLogger, UnifiedLoggerBuilder, UnifiedLoggerIOReader};
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::Path;
use std::sync::{Arc, Condvar, LazyLock, Mutex};
use std::time::Duration;

#[derive(Default)]
struct WorkerGateState {
    started: usize,
    permits: usize,
    released: bool,
    timed_out: bool,
}

#[derive(Default)]
struct WorkerGate {
    state: Mutex<WorkerGateState>,
    changed: Condvar,
}

static WORKER_GATES: LazyLock<[Arc<WorkerGate>; 3]> =
    LazyLock::new(|| std::array::from_fn(|_| Arc::new(WorkerGate::default())));

impl WorkerGate {
    fn from_config(config: Option<&ComponentConfig>) -> CuResult<Arc<Self>> {
        let index = config
            .ok_or_else(|| CuError::from("worker needs config"))?
            .get::<u32>("gate")?
            .ok_or_else(|| CuError::from("worker needs gate"))?;
        WORKER_GATES
            .get(index as usize)
            .cloned()
            .ok_or_else(|| CuError::from("unknown worker gate"))
    }

    fn enter(&self) -> CuResult<()> {
        let mut state = self.state.lock().unwrap();
        state.started += 1;
        self.changed.notify_all();
        // A freeze regression that waits for a blocked task must fail rather
        // than deadlock the test. Elapsed time is never a success condition.
        let (mut state, _) = self
            .changed
            .wait_timeout_while(state, Duration::from_secs(30), |state| {
                state.permits == 0 && !state.released
            })
            .unwrap();
        if state.permits == 0 && !state.released {
            state.timed_out = true;
            return Err(CuError::from("worker was not explicitly released"));
        }
        state.permits = state.permits.saturating_sub(1);
        Ok(())
    }

    fn wait_started(&self, count: usize) {
        let state = self.state.lock().unwrap();
        // This deadline only prevents a broken fixture from hanging the suite.
        let (state, _) = self
            .changed
            .wait_timeout_while(state, Duration::from_secs(10), |state| {
                state.started < count
            })
            .unwrap();
        let started = state.started;
        let timed_out = state.timed_out;
        drop(state);
        assert!(!timed_out, "worker escaped its gate without a release");
        assert_eq!(started, count, "worker did not enter the expected job");
    }

    fn release_one(&self) {
        self.state.lock().unwrap().permits += 1;
        self.changed.notify_all();
    }

    fn release_all(&self) {
        self.state
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .released = true;
        self.changed.notify_all();
    }
}

// Release blocked tasks before the application's Drop tries to stop them,
// including when an assertion or runtime call fails.
struct ReleaseWorkers;

impl Drop for ReleaseWorkers {
    fn drop(&mut self) {
        for gate in WORKER_GATES.iter() {
            gate.release_all();
        }
    }
}

#[derive(Default, Debug, Clone, Encode, Decode, Serialize, Deserialize, Reflect)]
struct WorkMsg(u32);

#[derive(Reflect)]
struct ControlledSource {
    next: u32,
    #[reflect(ignore)]
    gate: Arc<WorkerGate>,
}

impl Freezable for ControlledSource {
    fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.next, encoder)
    }

    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        self.next = Decode::decode(decoder)?;
        Ok(())
    }
}

impl CuSrcTask for ControlledSource {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(WorkMsg);

    fn new(config: Option<&ComponentConfig>, _resources: ()) -> CuResult<Self> {
        Ok(Self {
            next: 0,
            gate: WorkerGate::from_config(config)?,
        })
    }

    fn process(&mut self, _ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        self.gate.enter()?;
        self.next += 1;
        output.set_payload(WorkMsg(self.next));
        Ok(())
    }
}

#[derive(Reflect)]
struct ControlledTask {
    completed: u32,
    #[reflect(ignore)]
    gate: Arc<WorkerGate>,
}

impl Freezable for ControlledTask {
    fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.completed, encoder)
    }

    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        self.completed = Decode::decode(decoder)?;
        Ok(())
    }
}

impl CuTask for ControlledTask {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(WorkMsg);
    type Output<'m> = output_msg!(WorkMsg);

    fn new(config: Option<&ComponentConfig>, _resources: ()) -> CuResult<Self> {
        Ok(Self {
            completed: 0,
            gate: WorkerGate::from_config(config)?,
        })
    }

    fn process(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        self.gate.enter()?;
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

impl AsyncKeyframeApp {
    // Raw stepping lets this fixture drain output without changing worker state.
    #[allow(deprecated)]
    fn step_and_drain(&mut self) -> CuResult<()> {
        self.run_one_iteration()?;
        self.copper_runtime_mut()
            .copperlists_manager
            .finish_pending()
    }
}

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
#[allow(deprecated)] // Stop the raw app after releasing the test-controlled workers.
fn continuously_busy_async_workers_produce_complete_keyframes() -> CuResult<()> {
    const ITERATIONS: usize = 100;
    const COMPONENTS: usize = 4;

    let temp_dir = tempfile::tempdir()
        .map_err(|error| CuError::new_with_cause("create temp directory failed", error))?;
    let log_path = temp_dir.path().join("async_keyframes.copper");
    let logger = build_logger(&log_path)?;
    let mut app = AsyncKeyframeApp::builder()
        .with_logger::<MmapSectionStorage, MmapUnifiedLoggerWrite>(logger)
        .build()?
        .start()?
        .into_inner();
    let release_workers = ReleaseWorkers;

    // Advance completed results through the graph, then hold all three workers
    // inside process() while subsequent iterations capture their pending state.
    let [source, first, second] = &*WORKER_GATES;
    app.step_and_drain()?;
    source.wait_started(1);
    source.release_one();
    app.copper_runtime_mut().tasks.0.wait_for_job()?;

    app.step_and_drain()?;
    source.wait_started(2);
    first.wait_started(1);
    source.release_one();
    first.release_one();
    app.copper_runtime_mut().tasks.0.wait_for_job()?;
    app.copper_runtime_mut().tasks.1.wait_for_job()?;

    app.step_and_drain()?;
    source.wait_started(3);
    first.wait_started(2);
    second.wait_started(1);
    for _ in 3..ITERATIONS {
        app.step_and_drain()?;
    }
    source.wait_started(3);
    first.wait_started(2);
    second.wait_started(1);
    drop(release_workers);
    app.stop_all_tasks()?;
    let manager = &app.copper_runtime_mut().copperlists_manager;
    assert_eq!(manager.dropped_copperlists_total(), 0);
    assert_eq!(manager.dropped_keyframes_total(), 0);
    drop(app);

    let UnifiedLogger::Read(reader) = UnifiedLoggerBuilder::new()
        .file_base_name(&log_path)
        .build()
        .map_err(|error| CuError::new_with_cause("open keyframe log failed", error))?
    else {
        return Err(CuError::from("logger builder did not return a reader"));
    };
    let mut keyframe_reader = UnifiedLoggerIOReader::new(reader, UnifiedLogType::FrozenTasks);
    let keyframes: Vec<_> = keyframes_reader(&mut keyframe_reader).collect();
    assert_eq!(keyframes.len(), ITERATIONS);

    let UnifiedLogger::Read(reader) = UnifiedLoggerBuilder::new()
        .file_base_name(&log_path)
        .build()
        .map_err(|error| CuError::new_with_cause("open CopperList log failed", error))?
    else {
        return Err(CuError::from("logger builder did not return a reader"));
    };
    let copperlist_reader = UnifiedLoggerIOReader::new(reader, UnifiedLogType::CopperList);
    let copperlist_ids: Vec<_> = copperlists_reader::<default::CuStampedDataSet>(copperlist_reader)
        .map(|culist| culist.id)
        .collect();
    assert_eq!(copperlist_ids, (0..ITERATIONS as u64).collect::<Vec<_>>());
    assert_eq!(
        copperlist_ids,
        keyframes
            .iter()
            .map(|frame| frame.culistid)
            .collect::<Vec<_>>()
    );
    for keyframe in &keyframes {
        let mut frames = KeyFramePayloadReader::new(keyframe)?;
        for _ in 0..COMPONENTS {
            let _ = frames.next_frame()?;
        }
        frames.finish()?;
    }
    Ok(())
}
