#![cfg(all(test, feature = "std"))]

use bincode::{Decode, Encode};
use cu29::bincode::de::Decoder;
use cu29::bincode::enc::Encoder;
use cu29::bincode::error::{DecodeError, EncodeError};
use cu29::prelude::*;
use cu29_export::{copperlists_reader, keyframes_reader};
use cu29_runtime::app::CuSimApplication;
use cu29_unifiedlog::memmap::{MmapSectionStorage, MmapUnifiedLoggerWrite};
use cu29_unifiedlog::{UnifiedLogger, UnifiedLoggerBuilder, UnifiedLoggerIOReader};
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::Path;
use std::sync::{Arc, Mutex};

#[derive(Default, Debug, Clone, Encode, Decode, Serialize, Deserialize, Reflect)]
struct ValueMsg(u32);

#[derive(Reflect)]
struct StatefulSrc {
    next: u32,
}

impl Freezable for StatefulSrc {
    fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.next, encoder)
    }

    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        self.next = Decode::decode(decoder)?;
        Ok(())
    }
}

impl CuSrcTask for StatefulSrc {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(ValueMsg);

    fn new(_config: Option<&ComponentConfig>, _resources: ()) -> CuResult<Self> {
        Ok(Self { next: 0 })
    }

    fn process(&mut self, _ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        self.next += 1;
        output.set_payload(ValueMsg(self.next));
        Ok(())
    }
}

#[derive(Reflect)]
struct StatefulTask {
    sum: u32,
}

impl Freezable for StatefulTask {
    fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.sum, encoder)
    }

    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        self.sum = Decode::decode(decoder)?;
        Ok(())
    }
}

impl CuTask for StatefulTask {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(ValueMsg);
    type Output<'m> = output_msg!(ValueMsg);

    fn new(_config: Option<&ComponentConfig>, _resources: ()) -> CuResult<Self> {
        Ok(Self { sum: 0 })
    }

    fn process(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        self.sum += input.payload().map(|value| value.0).unwrap_or_default();
        output.set_payload(ValueMsg(self.sum));
        Ok(())
    }
}

#[derive(Reflect)]
struct StatefulSink {
    last: u32,
}

impl Freezable for StatefulSink {
    fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.last, encoder)
    }

    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        self.last = Decode::decode(decoder)?;
        Ok(())
    }
}

impl CuSinkTask for StatefulSink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(ValueMsg);

    fn new(_config: Option<&ComponentConfig>, _resources: ()) -> CuResult<Self> {
        Ok(Self { last: 0 })
    }

    fn process(&mut self, _ctx: &CuContext, input: &Self::Input<'_>) -> CuResult<()> {
        self.last = input.payload().map(|value| value.0).unwrap_or_default();
        Ok(())
    }
}

#[copper_runtime(config = "tests/sim_substituted_keyframes_config.ron", sim_mode = true)]
struct SimApp {}

mod recording {
    use super::{StatefulSink, StatefulSrc, StatefulTask, ValueMsg};
    use cu29::prelude::*;
    use cu29_unifiedlog::memmap::{MmapSectionStorage, MmapUnifiedLoggerWrite};
    use std::sync::{Arc, Mutex};

    #[copper_runtime(config = "tests/sim_substituted_keyframes_config.ron")]
    struct RealApp {}

    pub(super) fn record(logger: Arc<Mutex<MmapUnifiedLoggerWrite>>) -> CuResult<()> {
        let mut app = RealApp::builder()
            .with_logger::<MmapSectionStorage, MmapUnifiedLoggerWrite>(logger)
            .build()?;
        app.start_all_tasks()?;
        app.run_one_iteration()?;
        app.run_one_iteration()?;
        app.stop_all_tasks()?;
        Ok(())
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
fn sim_restore_skips_substituted_source_and_sink_but_thaws_regular_task() -> CuResult<()> {
    let temp_dir = tempfile::tempdir()
        .map_err(|error| CuError::new_with_cause("create temp directory failed", error))?;
    let record_path = temp_dir.path().join("record.copper");
    recording::record(build_logger(&record_path)?)?;
    let UnifiedLogger::Read(reader) = UnifiedLoggerBuilder::new()
        .file_base_name(&record_path)
        .build()
        .map_err(|error| CuError::new_with_cause("open keyframe log failed", error))?
    else {
        return Err(CuError::from("logger builder did not return a reader"));
    };
    let mut reader = UnifiedLoggerIOReader::new(reader, UnifiedLogType::FrozenTasks);
    let keyframe = keyframes_reader(&mut reader)
        .find(|keyframe| keyframe.culistid == 1)
        .ok_or_else(|| CuError::from("recorded log did not contain the CL1 keyframe"))?;

    let sim_path = temp_dir.path().join("sim.copper");
    let mut callback = |_step: default::SimStep<'_>| SimOverride::ExecuteByRuntime;
    let mut app = SimApp::builder()
        .with_logger::<MmapSectionStorage, MmapUnifiedLoggerWrite>(build_logger(&sim_path)?)
        .with_sim_callback(&mut callback)
        .build()?;
    app.start_all_tasks(&mut callback)?;
    <SimApp as CuSimApplication<MmapSectionStorage, MmapUnifiedLoggerWrite>>::restore_keyframe(
        &mut app, &keyframe,
    )?;
    assert_eq!(app.copper_runtime_mut().tasks.1.sum, 1);

    let UnifiedLogger::Read(reader) = UnifiedLoggerBuilder::new()
        .file_base_name(&record_path)
        .build()
        .map_err(|error| CuError::new_with_cause("open CopperList log failed", error))?
    else {
        return Err(CuError::from("logger builder did not return a reader"));
    };
    let mut reader = UnifiedLoggerIOReader::new(reader, UnifiedLogType::CopperList);
    let recorded_cl1 = copperlists_reader::<default::CuStampedDataSet>(&mut reader)
        .find(|copperlist| copperlist.id == 1)
        .ok_or_else(|| CuError::from("recorded log did not contain CopperList 1"))?;
    let mut replay_callback =
        |step: default::SimStep<'_>| default::recorded_debug_replay_step(step, &recorded_cl1);
    app.run_one_iteration(&mut replay_callback)?;
    assert_eq!(
        app.copper_runtime_mut().tasks.1.sum,
        3,
        "replay from the nonzero CL1 keyframe must match linear task state"
    );
    app.stop_all_tasks(&mut callback)?;
    Ok(())
}
