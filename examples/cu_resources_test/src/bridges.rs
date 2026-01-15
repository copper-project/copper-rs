use crate::resources::{GlobalLog, SharedBus};
use cu29::cubridge::{BridgeChannel, BridgeChannelConfig, BridgeChannelSet, CuBridge};
use cu29::prelude::*;
use serde::{Deserialize, Serialize};
use std::sync::Arc;

#[derive(Default, Debug, Clone, Serialize, Deserialize, bincode::Encode, bincode::Decode)]
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

pub struct StatsBridgeResources {
    pub bus: Arc<SharedBus>,
    pub tag: Arc<String>,
    pub global: Arc<GlobalLog>,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Binding {
    Bus,
    Tag,
    Global,
}

impl ResourceBindings<'_> for StatsBridgeResources {
    type Binding = Binding;

    fn from_bindings(
        manager: &mut ResourceManager,
        mapping: Option<&ResourceBindingMap<Self::Binding>>,
    ) -> CuResult<Self> {
        let mapping = mapping.ok_or_else(|| CuError::from("missing bridge bindings"))?;
        let bus = manager.borrow(mapping.get(Binding::Bus).unwrap().typed::<Arc<SharedBus>>())?;
        let tag = manager.borrow(mapping.get(Binding::Tag).unwrap().typed::<Arc<String>>())?;
        let global = manager.borrow(
            mapping
                .get(Binding::Global)
                .unwrap()
                .typed::<Arc<GlobalLog>>(),
        )?;
        Ok(Self {
            bus: bus.0.clone(),
            tag: tag.0.clone(),
            global: global.0.clone(),
        })
    }
}

pub struct StatsBridge {
    bus: Arc<SharedBus>,
    tag: Arc<String>,
    global: Arc<GlobalLog>,
}

impl Freezable for StatsBridge {}

impl CuBridge for StatsBridge {
    type Tx = TxChannels;
    type Rx = RxChannels;
    type Resources<'r> = StatsBridgeResources;

    fn new(
        _config: Option<&ComponentConfig>,
        _tx_channels: &[BridgeChannelConfig<<Self::Tx as BridgeChannelSet>::Id>],
        _rx_channels: &[BridgeChannelConfig<<Self::Rx as BridgeChannelSet>::Id>],
        resources: Self::Resources<'_>,
    ) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {
            bus: resources.bus,
            tag: resources.tag,
            global: resources.global,
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
                tag: (*self.tag).clone(),
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
