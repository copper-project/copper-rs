use clap::ValueEnum;
use cu29::prelude::*;
use std::sync::{LazyLock, Mutex};

#[derive(Copy, Clone, Debug, ValueEnum)]
pub enum MissionArg {
    #[value(name = "BridgeOnlyAB")]
    BridgeOnlyAb,
    #[value(name = "BridgeLoopback")]
    BridgeLoopback,
    #[value(name = "SourceToBridge")]
    SourceToBridge,
    #[value(name = "BridgeToSink")]
    BridgeToSink,
    #[value(name = "BridgeTaskSame")]
    BridgeTaskSame,
    #[value(name = "BridgeFanout")]
    BridgeFanout,
}

mod events {
    use super::*;

    pub static EVENT_LOG: LazyLock<Mutex<Vec<&'static str>>> =
        LazyLock::new(|| Mutex::new(Vec::new()));

    pub fn record(event: &'static str) {
        // debug!("Event: {}", event);
        EVENT_LOG.lock().unwrap().push(event);
    }
    #[cfg(test)]
    pub fn reset() {
        EVENT_LOG.lock().unwrap().clear();
    }
    #[cfg(test)]
    pub fn take() -> Vec<&'static str> {
        EVENT_LOG.lock().unwrap().drain(..).collect()
    }
}

pub mod messages {
    use bincode::{Decode, Encode};
    use cu29::prelude::*;
    use serde::{Deserialize, Serialize};

    #[derive(
        Default, Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Encode, Decode, Reflect,
    )]
    pub struct IngressMsg {
        pub sequence: u32,
    }

    #[derive(
        Default, Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Encode, Decode, Reflect,
    )]
    pub struct LoopbackMsg;

    #[derive(
        Default, Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Encode, Decode, Reflect,
    )]
    pub struct FromSource {
        pub tag: u32,
    }

    #[derive(
        Default, Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Encode, Decode, Reflect,
    )]
    pub struct SinkPayload;

    #[derive(
        Default, Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Encode, Decode, Reflect,
    )]
    pub struct ChainPayload {
        pub hops: u32,
    }
}

pub mod tasks {
    use super::events;
    use super::messages;
    use cu29::prelude::*;

    #[derive(Default, Reflect)]
    pub struct SourceToBridge {
        next: u32,
    }

    impl Freezable for SourceToBridge {}

    impl CuSrcTask for SourceToBridge {
        type Resources<'r> = ();
        type Output<'m> = CuMsg<messages::FromSource>;

        fn new(
            _config: Option<&ComponentConfig>,
            _resources: Self::Resources<'_>,
        ) -> CuResult<Self> {
            Ok(Self { next: 1 })
        }

        fn process<'o>(&mut self, _ctx: &CuContext, output: &mut Self::Output<'o>) -> CuResult<()> {
            events::record("source_to_bridge.process");
            output.set_payload(messages::FromSource { tag: self.next });
            self.next += 1;
            Ok(())
        }
    }

    #[derive(Default, Reflect)]
    pub struct PassthroughChain;

    impl Freezable for PassthroughChain {}

    impl CuTask for PassthroughChain {
        type Resources<'r> = ();
        type Input<'m> = CuMsg<messages::ChainPayload>;
        type Output<'m> = CuMsg<messages::ChainPayload>;

        fn new(
            _config: Option<&ComponentConfig>,
            _resources: Self::Resources<'_>,
        ) -> CuResult<Self> {
            Ok(Self)
        }

        fn process<'i, 'o>(
            &mut self,
            _ctx: &CuContext,
            input: &Self::Input<'i>,
            output: &mut Self::Output<'o>,
        ) -> CuResult<()> {
            events::record("passthrough_chain.process");
            if let Some(payload) = input.payload() {
                output.set_payload(payload.clone());
            } else {
                output.clear_payload();
            }
            Ok(())
        }
    }

    #[derive(Default, Reflect)]
    pub struct SinkFromBridge;

    impl Freezable for SinkFromBridge {}

    impl CuSinkTask for SinkFromBridge {
        type Resources<'r> = ();
        type Input<'m> = CuMsg<messages::SinkPayload>;

        fn new(
            _config: Option<&ComponentConfig>,
            _resources: Self::Resources<'_>,
        ) -> CuResult<Self> {
            Ok(Self)
        }

        fn process<'i>(&mut self, _ctx: &CuContext, input: &Self::Input<'i>) -> CuResult<()> {
            events::record("sink_from_bridge.process");
            assert!(
                input.payload().is_some(),
                "sink should receive payloads from bridge"
            );
            Ok(())
        }
    }
}

pub mod bridges {
    use super::events;
    use super::messages;
    use cu29::prelude::*;

    // `loop_out` ships with an inline default route while `chain_out` lets the config decide.
    tx_channels! {
        pub struct AlphaTxChannels : AlphaTxId {
            loop_out => messages::LoopbackMsg = "alpha/default_loop_out",
            chain_out => messages::ChainPayload,
        }
    }

    rx_channels! {
        pub struct AlphaRxChannels : AlphaRxId {
            ingress => messages::IngressMsg = "alpha/ingress",
            loop_in => messages::LoopbackMsg,
            to_sink => messages::SinkPayload = "alpha/to_sink",
            chain_in => messages::ChainPayload,
        }
    }

    #[derive(Default, Reflect)]
    pub struct AlphaBridge;

    impl Freezable for AlphaBridge {}

    impl CuBridge for AlphaBridge {
        type Resources<'r> = ();
        type Tx = AlphaTxChannels;
        type Rx = AlphaRxChannels;

        fn new(
            _config: Option<&ComponentConfig>,
            _tx: &[BridgeChannelConfig<<Self::Tx as BridgeChannelSet>::Id>],
            _rx: &[BridgeChannelConfig<<Self::Rx as BridgeChannelSet>::Id>],
            _resources: Self::Resources<'_>,
        ) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self)
        }

        fn send<'a, Payload>(
            &mut self,
            _ctx: &CuContext,
            channel: &'static BridgeChannel<AlphaTxId, Payload>,
            msg: &CuMsg<Payload>,
        ) -> CuResult<()>
        where
            Payload: CuMsgPayload + 'a,
        {
            match channel.id() {
                AlphaTxId::LoopOut => events::record("alpha.tx.loop"),
                AlphaTxId::ChainOut => events::record("alpha.tx.chain"),
            }
            assert!(msg.payload().is_some(), "alpha bridge tx expects payloads");
            Ok(())
        }

        fn receive<'a, Payload>(
            &mut self,
            _ctx: &CuContext,
            channel: &'static BridgeChannel<AlphaRxId, Payload>,
            msg: &mut CuMsg<Payload>,
        ) -> CuResult<()>
        where
            Payload: CuMsgPayload + 'a,
        {
            match channel.id() {
                AlphaRxId::Ingress => events::record("alpha.rx.ingress"),
                AlphaRxId::LoopIn => events::record("alpha.rx.loop"),
                AlphaRxId::ToSink => events::record("alpha.rx.to_sink"),
                AlphaRxId::ChainIn => events::record("alpha.rx.chain"),
            }
            msg.set_payload(Payload::default());
            Ok(())
        }
    }

    tx_channels! {
        pub struct BetaTxChannels : BetaTxId {
            egress => messages::IngressMsg,
            from_src => messages::FromSource = "beta/default_from_src",
            from_alpha_ingress => messages::IngressMsg = "beta/from_alpha_ingress",
            from_alpha_loop => messages::LoopbackMsg = "beta/from_alpha_loop",
            from_alpha_sink => messages::SinkPayload = "beta/from_alpha_sink",
            from_alpha_chain => messages::ChainPayload = "beta/from_alpha_chain",
        }
    }

    #[derive(Copy, Clone, Debug, Eq, PartialEq)]
    pub enum BetaRxId {}

    pub struct BetaRxChannels;

    impl BridgeChannelSet for BetaRxChannels {
        type Id = BetaRxId;

        const STATIC_CHANNELS: &'static [&'static dyn BridgeChannelInfo<Self::Id>] = &[];
    }

    #[derive(Default, Reflect)]
    pub struct BetaBridge;

    impl Freezable for BetaBridge {}

    impl CuBridge for BetaBridge {
        type Resources<'r> = ();
        type Tx = BetaTxChannels;
        type Rx = BetaRxChannels;

        fn new(
            _config: Option<&ComponentConfig>,
            _tx: &[BridgeChannelConfig<<Self::Tx as BridgeChannelSet>::Id>],
            _rx: &[BridgeChannelConfig<<Self::Rx as BridgeChannelSet>::Id>],
            _resources: Self::Resources<'_>,
        ) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self)
        }

        fn send<'a, Payload>(
            &mut self,
            _ctx: &CuContext,
            channel: &'static BridgeChannel<BetaTxId, Payload>,
            msg: &CuMsg<Payload>,
        ) -> CuResult<()>
        where
            Payload: CuMsgPayload + 'a,
        {
            match channel.id() {
                BetaTxId::Egress => events::record("beta.tx.egress"),
                BetaTxId::FromSrc => events::record("beta.tx.from_src"),
                BetaTxId::FromAlphaIngress => events::record("beta.tx.from_alpha_ingress"),
                BetaTxId::FromAlphaLoop => events::record("beta.tx.from_alpha_loop"),
                BetaTxId::FromAlphaSink => events::record("beta.tx.from_alpha_sink"),
                BetaTxId::FromAlphaChain => events::record("beta.tx.from_alpha_chain"),
            }
            assert!(msg.payload().is_some(), "beta bridge tx expects payloads");
            Ok(())
        }

        fn receive<'a, Payload>(
            &mut self,
            _ctx: &CuContext,
            _channel: &'static BridgeChannel<BetaRxId, Payload>,
            _msg: &mut CuMsg<Payload>,
        ) -> CuResult<()>
        where
            Payload: CuMsgPayload + 'a,
        {
            Ok(())
        }
    }
}

#[copper_runtime(config = "copperconfig.ron")]
struct BridgeSchedulerApp {}

// Expose the generated mission applications to the bin target.
pub type BridgeLoopbackApp = BridgeLoopback::BridgeSchedulerApp;
pub type BridgeOnlyABApp = BridgeOnlyAB::BridgeSchedulerApp;
pub type BridgeTaskSameApp = BridgeTaskSame::BridgeSchedulerApp;
pub type BridgeToSinkApp = BridgeToSink::BridgeSchedulerApp;
pub type SourceToBridgeApp = SourceToBridge::BridgeSchedulerApp;
pub type BridgeFanoutApp = BridgeFanout::BridgeSchedulerApp;

#[cfg(test)]
mod tests {
    use super::events;
    use super::messages;
    use cu29::prelude::*;
    use cu29_export::copperlists_reader;
    use cu29_unifiedlog::{
        UnifiedLogger, UnifiedLoggerBuilder, UnifiedLoggerIOReader, UnifiedLoggerWrite,
        memmap::MmapSectionStorage,
    };
    use std::sync::{LazyLock, Mutex};
    use tempfile::TempDir;

    fn run_mission<AppBuilderFn, App>(builder: AppBuilderFn) -> Vec<&'static str>
    where
        AppBuilderFn: FnOnce(&std::path::Path, RobotClock) -> CuResult<App>,
        App: CuApplication<MmapSectionStorage, UnifiedLoggerWrite>,
    {
        let temp_dir = TempDir::new().expect("temp dir");
        let log_path = temp_dir.path().join("bridge_sched.copper");
        let clock = RobotClock::default();

        events::reset();
        let mut app = builder(&log_path, clock).expect("build app");
        <App as CuApplication<MmapSectionStorage, UnifiedLoggerWrite>>::start_all_tasks(&mut app)
            .expect("start");
        <App as CuApplication<MmapSectionStorage, UnifiedLoggerWrite>>::run_one_iteration(&mut app)
            .expect("run");
        <App as CuApplication<MmapSectionStorage, UnifiedLoggerWrite>>::stop_all_tasks(&mut app)
            .expect("stop");
        events::take()
    }

    fn assert_process_time_populated(label: &str, msg: &CuMsg<messages::ChainPayload>) {
        assert!(
            !msg.metadata.process_time.start.is_none(),
            "{label} missing process_time.start"
        );
        assert!(
            !msg.metadata.process_time.end.is_none(),
            "{label} missing process_time.end"
        );

        let start = msg.metadata.process_time.start.unwrap();
        let end = msg.metadata.process_time.end.unwrap();
        assert!(
            start <= end,
            "{label} has invalid process_time range: start={start}, end={end}"
        );
    }

    static TEST_MUTEX: LazyLock<Mutex<()>> = LazyLock::new(|| Mutex::new(()));

    #[test]
    fn bridge_only_ab_orders_bridges_like_sources_and_sinks() {
        let _guard = TEST_MUTEX.lock().unwrap();
        let events = run_mission(|log_path, clock| {
            super::BridgeOnlyABApp::builder()
                .with_clock(clock)
                .with_log_path(log_path, Some(32 * 1024 * 1024))
                .expect("logger")
                .build()
        });
        assert_eq!(events, vec!["alpha.rx.ingress", "beta.tx.egress"]);
    }

    #[test]
    fn bridge_loopback_stays_on_one_bridge() {
        let _guard = TEST_MUTEX.lock().unwrap();
        let events = run_mission(|log_path, clock| {
            super::BridgeLoopbackApp::builder()
                .with_clock(clock)
                .with_log_path(log_path, Some(32 * 1024 * 1024))
                .expect("logger")
                .build()
        });
        assert_eq!(events, vec!["alpha.rx.loop", "alpha.tx.loop"]);
    }

    #[test]
    fn source_to_bridge_executes_source_before_tx() {
        let _guard = TEST_MUTEX.lock().unwrap();
        let events = run_mission(|log_path, clock| {
            super::SourceToBridgeApp::builder()
                .with_clock(clock)
                .with_log_path(log_path, Some(32 * 1024 * 1024))
                .expect("logger")
                .build()
        });
        assert_eq!(events, vec!["source_to_bridge.process", "beta.tx.from_src"]);
    }

    #[test]
    fn bridge_to_sink_executes_rx_before_sink() {
        let _guard = TEST_MUTEX.lock().unwrap();
        let events = run_mission(|log_path, clock| {
            super::BridgeToSinkApp::builder()
                .with_clock(clock)
                .with_log_path(log_path, Some(32 * 1024 * 1024))
                .expect("logger")
                .build()
        });
        assert_eq!(events, vec!["alpha.rx.to_sink", "sink_from_bridge.process"]);
    }

    #[test]
    fn bridge_task_bridge_same_bridge_is_linearized() {
        let _guard = TEST_MUTEX.lock().unwrap();
        let events = run_mission(|log_path, clock| {
            super::BridgeTaskSameApp::builder()
                .with_clock(clock)
                .with_log_path(log_path, Some(32 * 1024 * 1024))
                .expect("logger")
                .build()
        });
        assert_eq!(
            events,
            vec![
                "alpha.rx.chain",
                "passthrough_chain.process",
                "alpha.tx.chain"
            ],
        );
    }

    #[test]
    fn bridge_task_loopback_populates_process_time_both_directions() {
        let _guard = TEST_MUTEX.lock().unwrap();
        let temp_dir = TempDir::new().expect("temp dir");
        let log_path = temp_dir.path().join("bridge_sched_process_time.copper");
        let clock = RobotClock::default();

        {
            let mut app = super::BridgeTaskSameApp::builder()
                .with_clock(clock.clone())
                .with_log_path(&log_path, Some(32 * 1024 * 1024))
                .expect("logger")
                .build()
                .expect("build app");
            app.start_all_tasks().expect("start");
            app.run_one_iteration().expect("run");
            app.stop_all_tasks().expect("stop");
        }

        let UnifiedLogger::Read(read_logger) = UnifiedLoggerBuilder::new()
            .file_base_name(&log_path)
            .build()
            .expect("open log for read")
        else {
            panic!("expected read logger");
        };

        let mut reader = UnifiedLoggerIOReader::new(read_logger, UnifiedLogType::CopperList);
        let mut copperlists =
            copperlists_reader::<super::BridgeTaskSame::CuStampedDataSet>(&mut reader);

        let loop_entry = copperlists.next().expect("expected one copperlist");
        assert!(
            copperlists.next().is_none(),
            "expected exactly one copperlist for one loop"
        );

        // BridgeTaskSame graph is: alpha/chain_in -> passthrough -> alpha/chain_out.
        let bridge_to_task = &loop_entry.msgs.0.0;
        let task_to_bridge = &loop_entry.msgs.0.1;

        assert!(
            bridge_to_task.payload().is_some(),
            "bridge->task payload missing"
        );
        assert!(
            task_to_bridge.payload().is_some(),
            "task->bridge payload missing"
        );
        assert_process_time_populated("bridge->task", bridge_to_task);
        assert_process_time_populated("task->bridge", task_to_bridge);
    }

    #[test]
    fn missions_can_switch_bridge_topologies() {
        let _guard = TEST_MUTEX.lock().unwrap();
        let temp_dir = TempDir::new().expect("temp dir");
        let log_path = temp_dir.path().join("bridge_sched_switch.copper");
        let clock = RobotClock::default();

        events::reset();
        {
            let mut app = super::BridgeOnlyABApp::builder()
                .with_clock(clock.clone())
                .with_log_path(&log_path, Some(32 * 1024 * 1024))
                .expect("logger")
                .build()
                .unwrap();
            app.start_all_tasks().unwrap();
            app.run_one_iteration().unwrap();
            app.stop_all_tasks().unwrap();
        }
        let first = events::take();
        assert_eq!(first, vec!["alpha.rx.ingress", "beta.tx.egress"]);

        {
            let mut app = super::BridgeLoopbackApp::builder()
                .with_clock(clock)
                .with_log_path(&log_path, Some(32 * 1024 * 1024))
                .expect("logger")
                .build()
                .unwrap();
            app.start_all_tasks().unwrap();
            app.run_one_iteration().unwrap();
            app.stop_all_tasks().unwrap();
        }
        let second = events::take();
        assert_eq!(second, vec!["alpha.rx.loop", "alpha.tx.loop"]);
    }
}
