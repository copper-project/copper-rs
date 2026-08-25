#![cfg(all(test, feature = "std"))]

use bincode::{Decode, Encode};
use cu29::cubridge::{BridgeChannel, CuBridge};
use cu29::cutask::{CuMsg, CuMsgPayload, CuSinkTask, CuSrcTask, Freezable};
use cu29::prelude::copper_runtime;
use cu29::prelude::error;
use cu29::prelude::*;
use cu29::prelude::{ComponentConfig, CuResult, RobotClock};
use cu29::reflect::Reflect;
use cu29::rx_channels;
use cu29::simulation::{CuTaskCallbackState, SimOverride};
use cu29::tx_channels;
use cu29_export::keyframes_reader;
use cu29_runtime::app::CuSimApplication;
use cu29_unifiedlog::memmap::{MmapSectionStorage, MmapUnifiedLoggerWrite};
use cu29_unifiedlog::{UnifiedLogger, UnifiedLoggerBuilder, UnifiedLoggerIOReader};
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use tempfile::TempDir;

static BRIDGE_TX_CALLED: AtomicUsize = AtomicUsize::new(0);
static BRIDGE_RX_CALLED: AtomicUsize = AtomicUsize::new(0);
static BRIDGE_NEW_CALLED: AtomicUsize = AtomicUsize::new(0);

#[derive(Default, Debug, Clone, Encode, Decode, Serialize, Deserialize, PartialEq, Reflect)]
struct Ping {
    v: u8,
}

#[derive(Default, Debug, Clone, Encode, Decode, Serialize, Deserialize, PartialEq, Reflect)]
struct Pong {
    v: u8,
}

tx_channels! {
    tx => Ping,
}

rx_channels! {
    rx => Pong,
}

#[derive(Default, Reflect)]
struct DummyBridge {
    pub tx_called: usize,
    pub rx_called: usize,
}

impl Freezable for DummyBridge {
    fn freeze<E: bincode::enc::Encoder>(
        &self,
        encoder: &mut E,
    ) -> Result<(), bincode::error::EncodeError> {
        Encode::encode(&self.tx_called, encoder)?;
        Encode::encode(&self.rx_called, encoder)?;
        Ok(())
    }

    fn thaw<D: bincode::de::Decoder>(
        &mut self,
        decoder: &mut D,
    ) -> Result<(), bincode::error::DecodeError> {
        self.tx_called = Decode::decode(decoder)?;
        self.rx_called = Decode::decode(decoder)?;
        Ok(())
    }
}

impl CuBridge for DummyBridge {
    type Tx = TxChannels;
    type Rx = RxChannels;
    type Resources<'r> = ();

    fn new(
        _config: Option<&ComponentConfig>,
        _tx_channels: &[cu29::cubridge::BridgeChannelConfig<
            <Self::Tx as cu29::cubridge::BridgeChannelSet>::Id,
        >],
        _rx_channels: &[cu29::cubridge::BridgeChannelConfig<
            <Self::Rx as cu29::cubridge::BridgeChannelSet>::Id,
        >],
        _resources: Self::Resources<'_>,
    ) -> CuResult<Self> {
        BRIDGE_NEW_CALLED.fetch_add(1, Ordering::Relaxed);
        Ok(Self::default())
    }

    fn send<'a, Payload>(
        &mut self,
        _ctx: &CuContext,
        _channel: &'static BridgeChannel<
            <Self::Tx as cu29::cubridge::BridgeChannelSet>::Id,
            Payload,
        >,
        _msg: &CuMsg<Payload>,
    ) -> CuResult<()>
    where
        Payload: CuMsgPayload + 'a,
    {
        self.tx_called += 1;
        BRIDGE_TX_CALLED.fetch_add(1, Ordering::Relaxed);
        Ok(())
    }

    fn receive<'a, Payload>(
        &mut self,
        _ctx: &CuContext,
        _channel: &'static BridgeChannel<
            <Self::Rx as cu29::cubridge::BridgeChannelSet>::Id,
            Payload,
        >,
        msg: &mut CuMsg<Payload>,
    ) -> CuResult<()>
    where
        Payload: CuMsgPayload + 'a,
    {
        self.rx_called += 1;
        BRIDGE_RX_CALLED.fetch_add(1, Ordering::Relaxed);
        // For test determinism set a payload when real path is taken
        // (normally SimOverride will short-circuit).
        if msg.payload().is_none() {
            // cannot create default generically; just leave None.
        }
        Ok(())
    }
}

#[derive(Default, Reflect)]
struct MySrc;

impl Freezable for MySrc {}

impl CuSrcTask for MySrc {
    type Output<'m> = CuMsg<Ping>;
    type Resources<'r> = ();

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self)
    }

    fn process<'o>(&mut self, _ctx: &CuContext, out: &mut Self::Output<'o>) -> CuResult<()> {
        out.set_payload(Ping { v: 7 });
        Ok(())
    }
}

#[derive(Default, Reflect)]
struct MySink;

impl Freezable for MySink {}

impl CuSinkTask for MySink {
    type Input<'m> = CuMsg<Pong>;
    type Resources<'r> = ();

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self)
    }

    fn process<'i>(&mut self, _ctx: &CuContext, _input: &Self::Input<'i>) -> CuResult<()> {
        Ok(())
    }
}

#[copper_runtime(config = "tests/sim_bridge_config.ron", sim_mode = true)]
struct App {}

mod recording {
    use super::{DummyBridge, MySink, MySrc, Ping, Pong};
    use cu29::prelude::*;
    use cu29_unifiedlog::memmap::{MmapSectionStorage, MmapUnifiedLoggerWrite};
    use std::sync::{Arc, Mutex};

    #[copper_runtime(config = "tests/sim_bridge_config.ron")]
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

mod run_in_sim {
    use super::{DummyBridge, MySink, MySrc, Ping, Pong, build_logger, recording};
    use cu29::prelude::*;
    use cu29_export::{copperlists_reader, keyframes_reader};
    use cu29_runtime::app::CuSimApplication;
    use cu29_unifiedlog::memmap::{MmapSectionStorage, MmapUnifiedLoggerWrite};
    use cu29_unifiedlog::{UnifiedLogger, UnifiedLoggerBuilder, UnifiedLoggerIOReader};

    #[copper_runtime(config = "tests/sim_bridge_run_in_sim_config.ron", sim_mode = true)]
    struct RunInSimApp {}

    #[test]
    fn nonzero_bridge_keyframe_matches_linear_debug_replay() -> CuResult<()> {
        let temp_dir = tempfile::tempdir()
            .map_err(|error| CuError::new_with_cause("create temp log dir failed", error))?;
        let record_path = temp_dir.path().join("bridge_replay_record.copper");
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

        let replay_path = temp_dir.path().join("bridge_replay.copper");
        let mut callback = |_step: default::SimStep<'_>| SimOverride::ExecuteByRuntime;
        let mut app = RunInSimApp::builder()
            .with_logger::<MmapSectionStorage, MmapUnifiedLoggerWrite>(build_logger(&replay_path)?)
            .with_sim_callback(&mut callback)
            .build()?;
        app.start_all_tasks(&mut callback)?;
        <RunInSimApp as CuSimApplication<
            MmapSectionStorage,
            MmapUnifiedLoggerWrite,
        >>::restore_keyframe(&mut app, &keyframe)?;
        assert_eq!(app.copper_runtime_mut().bridges.0.tx_called, 2);
        assert_eq!(app.copper_runtime_mut().bridges.0.rx_called, 2);

        let mut replay_callback =
            |step: default::SimStep<'_>| default::recorded_debug_replay_step(step, &recorded_cl1);
        app.run_one_iteration(&mut replay_callback)?;
        assert_eq!(app.copper_runtime_mut().bridges.0.tx_called, 2);
        assert_eq!(app.copper_runtime_mut().bridges.0.rx_called, 2);
        app.stop_all_tasks(&mut callback)?;
        Ok(())
    }
}

fn build_logger(path: &Path) -> CuResult<Arc<Mutex<MmapUnifiedLoggerWrite>>> {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent)
            .map_err(|e| cu29::CuError::new_with_cause("create log dir failed", e))?;
    }
    // Minimal logger for tests: create a tiny write-capable mmap logger on disk.
    let UnifiedLogger::Write(writer) = UnifiedLoggerBuilder::new()
        .write(true)
        .create(true)
        .preallocated_size(16 * 1024 * 1024)
        .file_base_name(path)
        .build()
        .map_err(|e| cu29::CuError::new_with_cause("logger init failed", e))?
    else {
        return Err(cu29::CuError::from("logger builder did not return writer"));
    };
    Ok(Arc::new(Mutex::new(writer)))
}

fn build_test_logger() -> CuResult<(TempDir, PathBuf, Arc<Mutex<MmapUnifiedLoggerWrite>>)> {
    let temp_dir = tempfile::tempdir()
        .map_err(|e| cu29::CuError::new_with_cause("create temp log dir failed", e))?;
    let log_path = temp_dir.path().join("sim_bridge.log");
    Ok((temp_dir, log_path.clone(), build_logger(&log_path)?))
}

fn read_first_keyframe(path: &Path) -> CuResult<KeyFrame> {
    let UnifiedLogger::Read(reader) = UnifiedLoggerBuilder::new()
        .file_base_name(path)
        .build()
        .map_err(|error| CuError::new_with_cause("open keyframe log failed", error))?
    else {
        return Err(CuError::from("logger builder did not return a reader"));
    };
    let mut reader = UnifiedLoggerIOReader::new(reader, UnifiedLogType::FrozenTasks);
    keyframes_reader(&mut reader)
        .next()
        .ok_or_else(|| CuError::from("recorded log did not contain a keyframe"))
}

#[test]
fn sim_restore_skips_substituted_bridge_frame() -> CuResult<()> {
    let temp_dir = tempfile::tempdir()
        .map_err(|error| CuError::new_with_cause("create temp log dir failed", error))?;
    let record_path = temp_dir.path().join("real_bridge.copper");
    let logger = build_logger(&record_path)?;
    recording::record(logger)?;
    let keyframe = read_first_keyframe(&record_path)?;

    let sim_path = temp_dir.path().join("sim_bridge.copper");
    let logger = build_logger(&sim_path)?;
    let mut callback = |_step: default::SimStep<'_>| SimOverride::ExecuteByRuntime;
    let mut sim = App::builder()
        .with_logger::<MmapSectionStorage, MmapUnifiedLoggerWrite>(logger)
        .with_sim_callback(&mut callback)
        .build()?;
    sim.start_all_tasks(&mut callback)?;
    <App as CuSimApplication<MmapSectionStorage, MmapUnifiedLoggerWrite>>::restore_keyframe(
        &mut sim, &keyframe,
    )?;
    sim.stop_all_tasks(&mut callback)?;
    Ok(())
}

#[test]
fn bridge_sim_callbacks_fire_and_override() -> CuResult<()> {
    // Keep an explicit construction so `dead_code` doesn't trigger in clippy:
    // this bridge is otherwise resolved indirectly from config type names.
    let _dummy_bridge_marker = DummyBridge::default();

    BRIDGE_NEW_CALLED.store(0, Ordering::Relaxed);
    BRIDGE_TX_CALLED.store(0, Ordering::Relaxed);
    BRIDGE_RX_CALLED.store(0, Ordering::Relaxed);

    let (_temp_dir, _log_path, logger) = build_test_logger()?;
    let (robot_clock, _mock) = RobotClock::mock();

    let mut lifecycle_calls = 0usize;
    let mut tx_calls = 0usize;
    let mut rx_calls = 0usize;

    let mut sim_cb = |step: <App as CuSimApplication<
        MmapSectionStorage,
        MmapUnifiedLoggerWrite,
    >>::Step<'_>|
     -> SimOverride {
        match step {
            default::SimStep::BridgeBridge(
                CuBridgeLifecycleState::Start | CuBridgeLifecycleState::Stop,
            ) => {
                lifecycle_calls += 1;
                SimOverride::ExecuteByRuntime
            }
            default::SimStep::BridgeTxTx { msg, .. } => {
                tx_calls += 1;
                // skip real send
                assert_eq!(msg.payload().unwrap().v, 7);
                SimOverride::ExecutedBySim
            }
            default::SimStep::BridgeRxRx { msg, .. } => {
                rx_calls += 1;
                msg.set_payload(Pong { v: 42 });
                SimOverride::ExecutedBySim
            }
            default::SimStep::Src(CuTaskCallbackState::Process(_, _))
            | default::SimStep::Sink(CuTaskCallbackState::Process(_, _)) => {
                SimOverride::ExecuteByRuntime
            }
            _ => SimOverride::ExecuteByRuntime,
        }
    };

    let app = App::builder()
        .with_clock(robot_clock.clone())
        .with_logger::<MmapSectionStorage, MmapUnifiedLoggerWrite>(logger)
        .with_sim_callback(&mut sim_cb)
        .build()?;

    let mut running = app.start(&mut sim_cb)?;
    running.run_one_iteration(&mut sim_cb)?;
    running.stop(&mut sim_cb)?;

    // Bridge lifecycle start+stop observed
    assert_eq!(lifecycle_calls, 2);
    // Bridge I/O callbacks triggered once each, runtime implementation skipped
    assert_eq!(tx_calls, 1);
    assert_eq!(rx_calls, 1);

    // Ensure real bridge was not instantiated nor used (sim placeholder path).
    assert_eq!(BRIDGE_NEW_CALLED.load(Ordering::Relaxed), 0);
    assert_eq!(BRIDGE_TX_CALLED.load(Ordering::Relaxed), 0);
    assert_eq!(BRIDGE_RX_CALLED.load(Ordering::Relaxed), 0);
    Ok(())
}
