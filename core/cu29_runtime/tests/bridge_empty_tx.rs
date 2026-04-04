#![cfg(all(test, feature = "std"))]

use bincode::{Decode, Encode};
use cu29::cubridge::{BridgeChannel, BridgeChannelConfig, BridgeChannelSet, CuBridge};
use cu29::cutask::{CuMsg, CuMsgPayload, CuSrcTask, Freezable};
use cu29::prelude::copper_runtime;
use cu29::prelude::*;
use serde::{Deserialize, Serialize};
use std::sync::atomic::{AtomicUsize, Ordering};

static EMPTY_DEFAULT_SENDS: AtomicUsize = AtomicUsize::new(0);
static EMPTY_PUBLISH_SENDS: AtomicUsize = AtomicUsize::new(0);
static PAYLOAD_DEFAULT_SENDS: AtomicUsize = AtomicUsize::new(0);
static EMPTY_PAYLOAD_SENDS: AtomicUsize = AtomicUsize::new(0);

#[derive(Default, Debug, Clone, Encode, Decode, Serialize, Deserialize, PartialEq, Reflect)]
struct BridgePayload {
    v: u8,
}

tx_channels! {
    empty_default => BridgePayload,
    [publish_empty] empty_publish => BridgePayload,
    payload_default => BridgePayload,
}

rx_channels! {
    unused => BridgePayload,
}

#[derive(Default, Reflect)]
struct EmptyDefaultSrc;

impl Freezable for EmptyDefaultSrc {}

impl CuSrcTask for EmptyDefaultSrc {
    type Output<'m> = CuMsg<BridgePayload>;
    type Resources<'r> = ();

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self)
    }

    fn process<'o>(&mut self, _ctx: &CuContext, output: &mut Self::Output<'o>) -> CuResult<()> {
        output.clear_payload();
        Ok(())
    }
}

#[derive(Default, Reflect)]
struct EmptyPublishSrc;

impl Freezable for EmptyPublishSrc {}

impl CuSrcTask for EmptyPublishSrc {
    type Output<'m> = CuMsg<BridgePayload>;
    type Resources<'r> = ();

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self)
    }

    fn process<'o>(&mut self, _ctx: &CuContext, output: &mut Self::Output<'o>) -> CuResult<()> {
        output.clear_payload();
        Ok(())
    }
}

#[derive(Default, Reflect)]
struct PayloadDefaultSrc;

impl Freezable for PayloadDefaultSrc {}

impl CuSrcTask for PayloadDefaultSrc {
    type Output<'m> = CuMsg<BridgePayload>;
    type Resources<'r> = ();

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self)
    }

    fn process<'o>(&mut self, _ctx: &CuContext, output: &mut Self::Output<'o>) -> CuResult<()> {
        output.set_payload(BridgePayload { v: 7 });
        Ok(())
    }
}

#[derive(Default, Reflect)]
struct RecordingBridge;

impl Freezable for RecordingBridge {}

impl CuBridge for RecordingBridge {
    type Tx = TxChannels;
    type Rx = RxChannels;
    type Resources<'r> = ();

    fn new(
        _config: Option<&ComponentConfig>,
        _tx_channels: &[BridgeChannelConfig<<Self::Tx as BridgeChannelSet>::Id>],
        _rx_channels: &[BridgeChannelConfig<<Self::Rx as BridgeChannelSet>::Id>],
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
        channel: &'static BridgeChannel<<Self::Tx as BridgeChannelSet>::Id, Payload>,
        msg: &CuMsg<Payload>,
    ) -> CuResult<()>
    where
        Payload: CuMsgPayload + 'a,
    {
        if msg.payload().is_none() {
            EMPTY_PAYLOAD_SENDS.fetch_add(1, Ordering::Relaxed);
        }
        match channel.id {
            TxId::EmptyDefault => {
                EMPTY_DEFAULT_SENDS.fetch_add(1, Ordering::Relaxed);
            }
            TxId::EmptyPublish => {
                EMPTY_PUBLISH_SENDS.fetch_add(1, Ordering::Relaxed);
            }
            TxId::PayloadDefault => {
                PAYLOAD_DEFAULT_SENDS.fetch_add(1, Ordering::Relaxed);
            }
        }
        Ok(())
    }

    fn receive<'a, Payload>(
        &mut self,
        _ctx: &CuContext,
        _channel: &'static BridgeChannel<<Self::Rx as BridgeChannelSet>::Id, Payload>,
        msg: &mut CuMsg<Payload>,
    ) -> CuResult<()>
    where
        Payload: CuMsgPayload + 'a,
    {
        msg.clear_payload();
        Ok(())
    }
}

#[copper_runtime(config = "tests/bridge_empty_tx_config.ron")]
struct BridgeEmptyTxApp {}

#[test]
fn bridge_tx_empty_messages_are_skipped_by_default() -> CuResult<()> {
    EMPTY_DEFAULT_SENDS.store(0, Ordering::Relaxed);
    EMPTY_PUBLISH_SENDS.store(0, Ordering::Relaxed);
    PAYLOAD_DEFAULT_SENDS.store(0, Ordering::Relaxed);
    EMPTY_PAYLOAD_SENDS.store(0, Ordering::Relaxed);

    let (clock, _mock) = RobotClock::mock();
    let mut app = BridgeEmptyTxApp::builder().with_clock(clock).build()?;

    app.start_all_tasks()?;
    app.run_one_iteration()?;
    app.stop_all_tasks()?;

    assert_eq!(EMPTY_DEFAULT_SENDS.load(Ordering::Relaxed), 0);
    assert_eq!(EMPTY_PUBLISH_SENDS.load(Ordering::Relaxed), 1);
    assert_eq!(PAYLOAD_DEFAULT_SENDS.load(Ordering::Relaxed), 1);
    assert_eq!(EMPTY_PAYLOAD_SENDS.load(Ordering::Relaxed), 1);
    Ok(())
}
