#![cfg(all(test, feature = "std"))]

use bincode::{Decode, Encode};
use cu29::cubridge::{BridgeChannel, CuBridge};
use cu29::cutask::{CuMsg, CuMsgPayload, CuSinkTask, CuSrcTask, Freezable};
use cu29::prelude::copper_runtime;
use cu29::prelude::error;
use cu29::prelude::*;
use cu29::prelude::{ComponentConfig, CuResult, RobotClock};
use cu29::rx_channels;
use cu29::simulation::{CuTaskCallbackState, SimOverride};
use cu29::tx_channels;
use cu29_runtime::app::CuSimApplication;
use cu29_unifiedlog::memmap::{MmapSectionStorage, MmapUnifiedLoggerWrite};
use cu29_unifiedlog::{UnifiedLogger, UnifiedLoggerBuilder};
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::Path;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};

static BRIDGE_TX_CALLED: AtomicUsize = AtomicUsize::new(0);
static BRIDGE_RX_CALLED: AtomicUsize = AtomicUsize::new(0);

#[derive(Default, Debug, Clone, Encode, Decode, Serialize, Deserialize, PartialEq)]
struct Ping {
    v: u8,
}

#[derive(Default, Debug, Clone, Encode, Decode, Serialize, Deserialize, PartialEq)]
struct Pong {
    v: u8,
}

tx_channels! {
    tx => Ping,
}

rx_channels! {
    rx => Pong,
}

#[derive(Default)]
struct DummyBridge {
    pub tx_called: usize,
    pub rx_called: usize,
}

impl Freezable for DummyBridge {}

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
        Ok(Self::default())
    }

    fn send<'a, Payload>(
        &mut self,
        _clock: &RobotClock,
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
        _clock: &RobotClock,
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

#[derive(Default)]
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

    fn process<'o>(&mut self, _clock: &RobotClock, out: &mut Self::Output<'o>) -> CuResult<()> {
        out.set_payload(Ping { v: 7 });
        Ok(())
    }
}

#[derive(Default)]
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

    fn process<'i>(&mut self, _clock: &RobotClock, _input: &Self::Input<'i>) -> CuResult<()> {
        Ok(())
    }
}

#[copper_runtime(config = "tests/sim_bridge_config.ron", sim_mode = true)]
struct App {}

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

#[test]
fn bridge_sim_callbacks_fire_and_override() -> CuResult<()> {
    let log_path = Path::new("target/test-logs/sim_bridge.log");
    let logger = build_logger(log_path)?;
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
            default::SimStep::BridgeBridge(cu29::simulation::CuBridgeLifecycleState::Start) => {
                lifecycle_calls += 1;
                SimOverride::ExecuteByRuntime
            }
            default::SimStep::BridgeBridge(cu29::simulation::CuBridgeLifecycleState::Stop) => {
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
            default::SimStep::Src(CuTaskCallbackState::Process(_, _)) => {
                SimOverride::ExecuteByRuntime
            }
            default::SimStep::Sink(CuTaskCallbackState::Process(_, _)) => {
                SimOverride::ExecuteByRuntime
            }
            _ => SimOverride::ExecuteByRuntime,
        }
    };

    let mut app = AppBuilder::new()
        .with_clock(robot_clock.clone())
        .with_unified_logger(logger)
        .with_sim_callback(&mut sim_cb)
        .build()?;

    app.start_all_tasks(&mut sim_cb)?;
    app.run_one_iteration(&mut sim_cb)?;
    app.stop_all_tasks(&mut sim_cb)?;

    // Bridge lifecycle start+stop observed
    assert_eq!(lifecycle_calls, 2);
    // Bridge I/O callbacks triggered once each, runtime implementation skipped
    assert_eq!(tx_calls, 1);
    assert_eq!(rx_calls, 1);

    // Ensure real bridge was not used (counts remain zero)
    assert_eq!(BRIDGE_TX_CALLED.load(Ordering::Relaxed), 0);
    assert_eq!(BRIDGE_RX_CALLED.load(Ordering::Relaxed), 0);
    Ok(())
}
