use crate::resources::{GlobalLog, SharedBus};
use cu29::cubridge::{BridgeChannel, BridgeChannelConfig, BridgeChannelSet, CuBridge};
use cu29::prelude::*;
use cu29::resources;
use serde::{Deserialize, Serialize};

#[derive(
    Default, Debug, Clone, Serialize, Deserialize, bincode::Encode, bincode::Decode, Reflect,
)]
pub struct BusReading {
    pub tag: String,
    pub value: i64,
}

rx_channels! {
    stats_rx => BusReading
}

tx_channels! {
    stats_tx => BusReading
}

mod stats_bridge_resources {
    use super::*;

    resources!({
        bus => Shared<SharedBus>,
        tag => Shared<String>,
        global => Shared<GlobalLog>,
    });
}

#[derive(Reflect)]
#[reflect(from_reflect = false)]
pub struct StatsBridge {
    bus: SharedBus,
    tag: String,
    global: GlobalLog,
}

impl Freezable for StatsBridge {}

impl CuBridge for StatsBridge {
    type Tx = TxChannels;
    type Rx = RxChannels;
    type Resources<'r> = StatsBridgeResources<'r>;

    fn new(
        _config: Option<&ComponentConfig>,
        _tx_channels: &[BridgeChannelConfig<<Self::Tx as BridgeChannelSet>::Id>],
        _rx_channels: &[BridgeChannelConfig<<Self::Rx as BridgeChannelSet>::Id>],
        resources: Self::Resources<'_>,
    ) -> CuResult<Self>
    where
        Self: Sized,
    {
        let stats_bridge_resources::Resources { bus, tag, global } = resources;
        Ok(Self {
            bus: bus.0.clone(),
            tag: tag.0.clone(),
            global: global.0.clone(),
        })
    }

    fn send<'a, Payload>(
        &mut self,
        _clock: &RobotClock,
        _channel: &'static BridgeChannel<<Self::Tx as BridgeChannelSet>::Id, Payload>,
        _msg: &CuMsg<Payload>,
    ) -> CuResult<()>
    where
        Payload: CuMsgPayload + 'a,
    {
        Ok(())
    }

    fn receive<'a, Payload>(
        &mut self,
        _clock: &RobotClock,
        channel: &'static BridgeChannel<<Self::Rx as BridgeChannelSet>::Id, Payload>,
        msg: &mut CuMsg<Payload>,
    ) -> CuResult<()>
    where
        Payload: CuMsgPayload + 'a,
    {
        if matches!(channel.id(), RxId::StatsRx) {
            let reading = BusReading {
                tag: self.tag.clone(),
                value: self.bus.get(),
            };
            let payload: &mut CuMsg<BusReading> = msg.downcast_mut()?;
            payload.set_payload(reading.clone());
            self.global.push(format!(
                "bridge emitted {} from {}",
                reading.value, reading.tag
            ));
        }
        Ok(())
    }
}

type StatsBridgeResources<'r> = stats_bridge_resources::Resources<'r>;
