use cu29::cubridge::CuBridge;
use cu29::prelude::*;
use once_cell::sync::Lazy;
use std::sync::Mutex;

mod events {
    use super::*;

    pub static EVENT_LOG: Lazy<Mutex<Vec<&'static str>>> = Lazy::new(|| Mutex::new(Vec::new()));

    pub fn record(event: &'static str) {
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
    use serde::{Deserialize, Serialize};

    #[derive(Default, Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Encode, Decode)]
    pub struct IngressMsg {
        pub sequence: u32,
    }

    #[derive(Default, Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Encode, Decode)]
    pub struct LoopbackMsg;

    #[derive(Default, Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Encode, Decode)]
    pub struct FromSource {
        pub tag: u32,
    }

    #[derive(Default, Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Encode, Decode)]
    pub struct SinkPayload;

    #[derive(Default, Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Encode, Decode)]
    pub struct ChainPayload {
        pub hops: u32,
    }
}

pub mod tasks {
    use super::events;
    use super::messages;
    use cu29::prelude::*;

    #[derive(Default)]
    pub struct SourceToBridge {
        next: u32,
    }

    impl Freezable for SourceToBridge {}

    impl CuSrcTask for SourceToBridge {
        type Output<'m> = CuMsg<messages::FromSource>;

        fn new(_config: Option<&ComponentConfig>) -> CuResult<Self> {
            Ok(Self { next: 1 })
        }

        fn process<'o>(
            &mut self,
            _clock: &RobotClock,
            output: &mut Self::Output<'o>,
        ) -> CuResult<()> {
            events::record("source_to_bridge.process");
            output.set_payload(messages::FromSource { tag: self.next });
            self.next += 1;
            Ok(())
        }
    }

    #[derive(Default)]
    pub struct PassthroughChain;

    impl Freezable for PassthroughChain {}

    impl CuTask for PassthroughChain {
        type Input<'m> = CuMsg<messages::ChainPayload>;
        type Output<'m> = CuMsg<messages::ChainPayload>;

        fn new(_config: Option<&ComponentConfig>) -> CuResult<Self> {
            Ok(Self)
        }

        fn process<'i, 'o>(
            &mut self,
            _clock: &RobotClock,
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

    #[derive(Default)]
    pub struct SinkFromBridge;

    impl Freezable for SinkFromBridge {}

    impl CuSinkTask for SinkFromBridge {
        type Input<'m> = CuMsg<messages::SinkPayload>;

        fn new(_config: Option<&ComponentConfig>) -> CuResult<Self> {
            Ok(Self)
        }

        fn process<'i>(&mut self, _clock: &RobotClock, input: &Self::Input<'i>) -> CuResult<()> {
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

    #[derive(Default)]
    pub struct AlphaBridge;

    impl Freezable for AlphaBridge {}

    impl CuBridge for AlphaBridge {
        type Tx = AlphaTxChannels;
        type Rx = AlphaRxChannels;

        fn new(
            _config: Option<&ComponentConfig>,
            _tx: &[BridgeChannelConfig<<Self::Tx as BridgeChannelSet>::Id>],
            _rx: &[BridgeChannelConfig<<Self::Rx as BridgeChannelSet>::Id>],
        ) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self)
        }

        fn send<'a, Payload>(
            &mut self,
            _clock: &RobotClock,
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
            _clock: &RobotClock,
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
        }
    }

    #[derive(Copy, Clone, Debug, Eq, PartialEq)]
    pub enum BetaRxId {}

    pub struct BetaRxChannels;

    impl BridgeChannelSet for BetaRxChannels {
        type Id = BetaRxId;

        const STATIC_CHANNELS: &'static [&'static dyn BridgeChannelInfo<Self::Id>] = &[];
    }

    #[derive(Default)]
    pub struct BetaBridge;

    impl Freezable for BetaBridge {}

    impl CuBridge for BetaBridge {
        type Tx = BetaTxChannels;
        type Rx = BetaRxChannels;

        fn new(
            _config: Option<&ComponentConfig>,
            _tx: &[BridgeChannelConfig<<Self::Tx as BridgeChannelSet>::Id>],
            _rx: &[BridgeChannelConfig<<Self::Rx as BridgeChannelSet>::Id>],
        ) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self)
        }

        fn send<'a, Payload>(
            &mut self,
            _clock: &RobotClock,
            channel: &'static BridgeChannel<BetaTxId, Payload>,
            msg: &CuMsg<Payload>,
        ) -> CuResult<()>
        where
            Payload: CuMsgPayload + 'a,
        {
            match channel.id() {
                BetaTxId::Egress => events::record("beta.tx.egress"),
                BetaTxId::FromSrc => events::record("beta.tx.from_src"),
            }
            assert!(msg.payload().is_some(), "beta bridge tx expects payloads");
            Ok(())
        }

        fn receive<'a, Payload>(
            &mut self,
            _clock: &RobotClock,
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

// Expose builders for the generated mission runtimes to external callers.
pub type BridgeLoopbackBuilder = BridgeLoopback::BridgeSchedulerAppBuilder;
pub type BridgeOnlyABBuilder = BridgeOnlyAB::BridgeSchedulerAppBuilder;
pub type BridgeTaskSameBuilder = BridgeTaskSame::BridgeSchedulerAppBuilder;
pub type BridgeToSinkBuilder = BridgeToSink::BridgeSchedulerAppBuilder;
pub type SourceToBridgeBuilder = SourceToBridge::BridgeSchedulerAppBuilder;

#[cfg(test)]
mod tests {
    use super::events;
    use cu29::curuntime::CopperContext;
    use cu29::prelude::{CuApplication, CuResult};
    use cu29_helpers::basic_copper_setup;
    use cu29_unifiedlog::{memmap::MmapSectionStorage, UnifiedLoggerWrite};
    use once_cell::sync::Lazy;
    use std::sync::Mutex;
    use tempfile::TempDir;

    use super::BridgeLoopback::BridgeSchedulerAppBuilder as BridgeLoopbackBuilder;
    use super::BridgeOnlyAB::BridgeSchedulerAppBuilder as BridgeOnlyBuilder;
    use super::BridgeTaskSame::BridgeSchedulerAppBuilder as BridgeTaskBuilder;
    use super::BridgeToSink::BridgeSchedulerAppBuilder as BridgeToSinkBuilder;
    use super::SourceToBridge::BridgeSchedulerAppBuilder as SourceToBridgeBuilder;

    fn run_mission<AppBuilderFn, App>(builder: AppBuilderFn) -> Vec<&'static str>
    where
        AppBuilderFn: FnOnce(&CopperContext) -> CuResult<App>,
        App: CuApplication<MmapSectionStorage, UnifiedLoggerWrite>,
    {
        let temp_dir = TempDir::new().expect("temp dir");
        let log_path = temp_dir.path().join("bridge_sched.copper");
        let ctx =
            basic_copper_setup(&log_path, Some(32 * 1024 * 1024), false, None).expect("context");

        events::reset();
        let mut app = builder(&ctx).expect("build app");
        <App as CuApplication<MmapSectionStorage, UnifiedLoggerWrite>>::start_all_tasks(&mut app)
            .expect("start");
        <App as CuApplication<MmapSectionStorage, UnifiedLoggerWrite>>::run_one_iteration(&mut app)
            .expect("run");
        <App as CuApplication<MmapSectionStorage, UnifiedLoggerWrite>>::stop_all_tasks(&mut app)
            .expect("stop");
        events::take()
    }

    static TEST_MUTEX: Lazy<Mutex<()>> = Lazy::new(|| Mutex::new(()));

    #[test]
    fn bridge_only_ab_orders_bridges_like_sources_and_sinks() {
        let _guard = TEST_MUTEX.lock().unwrap();
        let events = run_mission(|ctx| BridgeOnlyBuilder::new().with_context(ctx).build());
        assert_eq!(events, vec!["alpha.rx.ingress", "beta.tx.egress"]);
    }

    #[test]
    fn bridge_loopback_stays_on_one_bridge() {
        let _guard = TEST_MUTEX.lock().unwrap();
        let events = run_mission(|ctx| BridgeLoopbackBuilder::new().with_context(ctx).build());
        assert_eq!(events, vec!["alpha.rx.loop", "alpha.tx.loop"]);
    }

    #[test]
    fn source_to_bridge_executes_source_before_tx() {
        let _guard = TEST_MUTEX.lock().unwrap();
        let events = run_mission(|ctx| SourceToBridgeBuilder::new().with_context(ctx).build());
        assert_eq!(events, vec!["source_to_bridge.process", "beta.tx.from_src"]);
    }

    #[test]
    fn bridge_to_sink_executes_rx_before_sink() {
        let _guard = TEST_MUTEX.lock().unwrap();
        let events = run_mission(|ctx| BridgeToSinkBuilder::new().with_context(ctx).build());
        assert_eq!(events, vec!["alpha.rx.to_sink", "sink_from_bridge.process"]);
    }

    #[test]
    fn bridge_task_bridge_same_bridge_is_linearized() {
        let _guard = TEST_MUTEX.lock().unwrap();
        let events = run_mission(|ctx| BridgeTaskBuilder::new().with_context(ctx).build());
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
    fn missions_can_switch_bridge_topologies() {
        let _guard = TEST_MUTEX.lock().unwrap();
        let temp_dir = TempDir::new().expect("temp dir");
        let log_path = temp_dir.path().join("bridge_sched_switch.copper");
        let ctx =
            basic_copper_setup(&log_path, Some(32 * 1024 * 1024), false, None).expect("context");

        events::reset();
        {
            let mut app = BridgeOnlyBuilder::new().with_context(&ctx).build().unwrap();
            app.start_all_tasks().unwrap();
            app.run_one_iteration().unwrap();
            app.stop_all_tasks().unwrap();
        }
        let first = events::take();
        assert_eq!(first, vec!["alpha.rx.ingress", "beta.tx.egress"]);

        {
            let mut app = BridgeLoopbackBuilder::new()
                .with_context(&ctx)
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
