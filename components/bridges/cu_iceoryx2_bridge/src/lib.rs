#![cfg_attr(not(feature = "std"), no_std)]

extern crate alloc;

use alloc::boxed::Box;
use alloc::format;
use alloc::string::String;
use alloc::vec::Vec;
use core::any::Any;
use core::marker::PhantomData;

use cu29::cubridge::{BridgeChannel, BridgeChannelConfig, BridgeChannelSet, CuBridge};
use cu29::prelude::*;
use iceoryx2::node::node_name::NodeName;
use iceoryx2::node::{Node, NodeBuilder};
use iceoryx2::port::publisher::Publisher;
use iceoryx2::port::subscriber::Subscriber;
use iceoryx2::service::service_name::ServiceName;

#[cfg(feature = "std")]
use iceoryx2::service::ipc;
#[cfg(not(feature = "std"))]
use iceoryx2::service::local;

#[cfg(feature = "std")]
type IceoryxService = ipc::Service;
#[cfg(not(feature = "std"))]
type IceoryxService = local::Service;

fn encode_message<Payload: CuMsgPayload>(msg: &CuMsg<Payload>) -> CuResult<Vec<u8>> {
    bincode::encode_to_vec(msg, bincode::config::standard())
        .map_err(|e| CuError::new_with_cause("Iceoryx2Bridge: bincode encode failed", e))
}

fn decode_message<Payload: CuMsgPayload>(bytes: &[u8]) -> CuResult<CuMsg<Payload>> {
    let (decoded, _): (CuMsg<Payload>, usize) =
        bincode::decode_from_slice(bytes, bincode::config::standard())
            .map_err(|e| CuError::new_with_cause("Iceoryx2Bridge: bincode decode failed", e))?;
    Ok(decoded)
}

#[derive(Clone, Debug)]
struct IceoryxChannelConfig<Id: Copy> {
    id: Id,
    service: String,
    max_payload_bytes: usize,
}

struct IceoryxTxChannel<Payload>
where
    Payload: CuMsgPayload + 'static,
{
    service_name: ServiceName,
    publisher: Publisher<IceoryxService, [u8], ()>,
    max_payload_bytes: usize,
    _payload: PhantomData<Payload>,
}

struct IceoryxRxChannel<Payload>
where
    Payload: CuMsgPayload + 'static,
{
    service_name: ServiceName,
    subscriber: Subscriber<IceoryxService, [u8], ()>,
    _payload: PhantomData<Payload>,
}

struct IceoryxTxChannelEntry<Id: Copy> {
    id: Id,
    channel: Box<dyn Any>,
}

struct IceoryxRxChannelEntry<Id: Copy> {
    id: Id,
    channel: Box<dyn Any>,
}

struct IceoryxContext<TxId: Copy, RxId: Copy> {
    node: Node<IceoryxService>,
    tx_channels: Vec<IceoryxTxChannelEntry<TxId>>,
    rx_channels: Vec<IceoryxRxChannelEntry<RxId>>,
}

#[derive(Reflect)]
#[reflect(from_reflect = false, no_field_bounds, type_path = false)]
pub struct Iceoryx2Bridge<Tx, Rx>
where
    Tx: BridgeChannelSet + 'static,
    Rx: BridgeChannelSet + 'static,
    Tx::Id: Send + Sync + 'static,
    Rx::Id: Send + Sync + 'static,
{
    #[reflect(ignore)]
    node_name: Option<NodeName>,
    #[reflect(ignore)]
    tx_channels: Vec<IceoryxChannelConfig<Tx::Id>>,
    #[reflect(ignore)]
    rx_channels: Vec<IceoryxChannelConfig<Rx::Id>>,
    #[reflect(ignore)]
    ctx: Option<usize>,
}

impl<Tx, Rx> Freezable for Iceoryx2Bridge<Tx, Rx>
where
    Tx: BridgeChannelSet + 'static,
    Rx: BridgeChannelSet + 'static,
    Tx::Id: Send + Sync + 'static,
    Rx::Id: Send + Sync + 'static,
{
}

impl<Tx, Rx> cu29::reflect::TypePath for Iceoryx2Bridge<Tx, Rx>
where
    Tx: BridgeChannelSet + 'static,
    Rx: BridgeChannelSet + 'static,
    Tx::Id: Send + Sync + 'static,
    Rx::Id: Send + Sync + 'static,
{
    fn type_path() -> &'static str {
        "cu_iceoryx2_bridge::Iceoryx2Bridge"
    }

    fn short_type_path() -> &'static str {
        "Iceoryx2Bridge"
    }

    fn type_ident() -> Option<&'static str> {
        Some("Iceoryx2Bridge")
    }

    fn crate_name() -> Option<&'static str> {
        Some("cu_iceoryx2_bridge")
    }

    fn module_path() -> Option<&'static str> {
        Some("cu_iceoryx2_bridge")
    }
}

impl<Tx, Rx> Iceoryx2Bridge<Tx, Rx>
where
    Tx: BridgeChannelSet + 'static,
    Rx: BridgeChannelSet + 'static,
    Tx::Id: Send + Sync + 'static,
    Rx::Id: Send + Sync + 'static,
{
    fn ctx_mut(&mut self) -> CuResult<&mut IceoryxContext<Tx::Id, Rx::Id>> {
        let Some(raw) = self.ctx else {
            return Err(CuError::from("Iceoryx2Bridge: Context not initialized"));
        };
        let ptr = raw as *mut IceoryxContext<Tx::Id, Rx::Id>;
        // SAFETY:
        // `ptr` comes from `Box::into_raw` in `start()` and remains valid until `stop()`/`drop()`.
        // Access is serialized through `&mut self`.
        Ok(unsafe { &mut *ptr })
    }

    fn parse_default_max_payload(config: Option<&ComponentConfig>) -> CuResult<usize> {
        if let Some(config) = config
            && let Some(value) = config.get::<u64>("max_payload_bytes")?
        {
            return usize::try_from(value).map_err(|_| {
                CuError::from("Iceoryx2Bridge: max_payload_bytes does not fit in usize")
            });
        }
        Ok(64 * 1024)
    }

    fn parse_node_name(config: Option<&ComponentConfig>) -> CuResult<Option<NodeName>> {
        if let Some(config) = config
            && let Some(raw) = config.get::<String>("node_name")?
        {
            let node_name = NodeName::new(raw.as_str())
                .map_err(|e| CuError::new_with_cause("Iceoryx2Bridge: Invalid node_name", e))?;
            return Ok(Some(node_name));
        }
        Ok(None)
    }

    fn channel_route<Id: Copy + core::fmt::Debug>(
        channel: &BridgeChannelConfig<Id>,
    ) -> CuResult<String> {
        channel
            .effective_route()
            .map(|route| route.into_owned())
            .ok_or_else(|| {
                let id = channel.channel.id;
                CuError::from(format!(
                    "Iceoryx2Bridge: Missing service name for channel {:?}",
                    id
                ))
            })
    }

    fn channel_max_payload<Id: Copy>(
        channel: &BridgeChannelConfig<Id>,
        default: usize,
    ) -> CuResult<usize> {
        if let Some(config) = channel.config.as_ref()
            && let Some(value) = config.get::<u64>("max_payload_bytes")?
        {
            return usize::try_from(value).map_err(|_| {
                CuError::from("Iceoryx2Bridge: max_payload_bytes does not fit in usize")
            });
        }
        Ok(default)
    }

    fn find_tx_config(&self, id: Tx::Id) -> Option<&IceoryxChannelConfig<Tx::Id>> {
        self.tx_channels.iter().find(|channel| channel.id == id)
    }

    fn find_rx_config(&self, id: Rx::Id) -> Option<&IceoryxChannelConfig<Rx::Id>> {
        self.rx_channels.iter().find(|channel| channel.id == id)
    }

    fn find_tx_channel_mut<Payload: CuMsgPayload + 'static>(
        channels: &mut [IceoryxTxChannelEntry<Tx::Id>],
        id: Tx::Id,
    ) -> CuResult<Option<&mut IceoryxTxChannel<Payload>>> {
        let entry = channels.iter_mut().find(|channel| channel.id == id);
        if let Some(entry) = entry {
            return entry
                .channel
                .downcast_mut::<IceoryxTxChannel<Payload>>()
                .ok_or_else(|| CuError::from("Iceoryx2Bridge: Tx channel payload mismatch"))
                .map(Some);
        }
        Ok(None)
    }

    fn find_rx_channel_mut<Payload: CuMsgPayload + 'static>(
        channels: &mut [IceoryxRxChannelEntry<Rx::Id>],
        id: Rx::Id,
    ) -> CuResult<Option<&mut IceoryxRxChannel<Payload>>> {
        let entry = channels.iter_mut().find(|channel| channel.id == id);
        if let Some(entry) = entry {
            return entry
                .channel
                .downcast_mut::<IceoryxRxChannel<Payload>>()
                .ok_or_else(|| CuError::from("Iceoryx2Bridge: Rx channel payload mismatch"))
                .map(Some);
        }
        Ok(None)
    }
}

impl<Payload> IceoryxTxChannel<Payload>
where
    Payload: CuMsgPayload + 'static,
{
    fn new(
        node: &mut Node<IceoryxService>,
        service_str: &str,
        max_payload_bytes: usize,
    ) -> CuResult<Self> {
        let service_name = ServiceName::new(service_str).map_err(|e| {
            CuError::new_with_cause("Iceoryx2Bridge: Failed to create service name", e)
        })?;

        let service = node
            .service_builder(&service_name)
            .publish_subscribe::<[u8]>()
            .open_or_create()
            .map_err(|e| {
                CuError::new_with_cause(
                    format!(
                        "Iceoryx2Bridge({}): Failed to create service",
                        service_name.as_str()
                    )
                    .as_str(),
                    e,
                )
            })?;

        let publisher = service
            .publisher_builder()
            .initial_max_slice_len(max_payload_bytes)
            .create()
            .map_err(|e| {
                CuError::new_with_cause(
                    format!(
                        "Iceoryx2Bridge({}): Failed to create publisher",
                        service_name.as_str()
                    )
                    .as_str(),
                    e,
                )
            })?;

        Ok(Self {
            service_name,
            publisher,
            max_payload_bytes,
            _payload: PhantomData,
        })
    }

    fn send(&mut self, msg: &CuMsg<Payload>) -> CuResult<()> {
        let encoded = encode_message(msg)?;
        if encoded.len() > self.max_payload_bytes {
            return Err(CuError::from(format!(
                "Iceoryx2Bridge({}): payload size {} exceeds max_payload_bytes {}",
                self.service_name,
                encoded.len(),
                self.max_payload_bytes
            )));
        }

        let sample = self
            .publisher
            .loan_slice_uninit(encoded.len())
            .map_err(|e| {
                CuError::new_with_cause(
                    format!(
                        "Iceoryx2Bridge({}): Failed to loan sample",
                        self.service_name
                    )
                    .as_str(),
                    e,
                )
            })?;
        let sample = sample.write_from_fn(|idx| encoded[idx]);
        sample.send().map_err(|e| {
            CuError::new_with_cause(
                format!(
                    "Iceoryx2Bridge({}): Failed to send sample",
                    self.service_name
                )
                .as_str(),
                e,
            )
        })?;
        Ok(())
    }
}

impl<Payload> IceoryxRxChannel<Payload>
where
    Payload: CuMsgPayload + 'static,
{
    fn new(node: &mut Node<IceoryxService>, service_str: &str) -> CuResult<Self> {
        let service_name = ServiceName::new(service_str).map_err(|e| {
            CuError::new_with_cause("Iceoryx2Bridge: Failed to create service name", e)
        })?;

        let service = node
            .service_builder(&service_name)
            .publish_subscribe::<[u8]>()
            .open_or_create()
            .map_err(|e| {
                CuError::new_with_cause(
                    format!(
                        "Iceoryx2Bridge({}): Failed to create service",
                        service_name.as_str()
                    )
                    .as_str(),
                    e,
                )
            })?;

        let subscriber = service.subscriber_builder().create().map_err(|e| {
            CuError::new_with_cause(
                format!(
                    "Iceoryx2Bridge({}): Failed to create subscriber",
                    service_name.as_str()
                )
                .as_str(),
                e,
            )
        })?;

        Ok(Self {
            service_name,
            subscriber,
            _payload: PhantomData,
        })
    }

    fn receive(&mut self, clock: &RobotClock, msg: &mut CuMsg<Payload>) -> CuResult<()> {
        msg.tov = Tov::Time(clock.now());
        let sample = self.subscriber.receive().map_err(|e| {
            CuError::new_with_cause(
                format!("Iceoryx2Bridge({}): Receive failed", self.service_name).as_str(),
                e,
            )
        })?;

        if let Some(sample) = sample {
            let payload = sample.payload();
            let decoded = decode_message(payload)?;
            *msg = decoded;
        } else {
            msg.clear_payload();
        }
        Ok(())
    }
}

impl<Tx, Rx> CuBridge for Iceoryx2Bridge<Tx, Rx>
where
    Tx: BridgeChannelSet + 'static,
    Rx: BridgeChannelSet + 'static,
    Tx::Id: core::fmt::Debug + Send + Sync + 'static,
    Rx::Id: core::fmt::Debug + Send + Sync + 'static,
{
    type Tx = Tx;
    type Rx = Rx;
    type Resources<'r> = ();

    fn new(
        config: Option<&ComponentConfig>,
        tx_channels: &[BridgeChannelConfig<<Self::Tx as BridgeChannelSet>::Id>],
        rx_channels: &[BridgeChannelConfig<<Self::Rx as BridgeChannelSet>::Id>],
        _resources: Self::Resources<'_>,
    ) -> CuResult<Self>
    where
        Self: Sized,
    {
        let node_name = Self::parse_node_name(config)?;
        let default_max_payload = Self::parse_default_max_payload(config)?;

        let mut tx_cfgs = Vec::with_capacity(tx_channels.len());
        for channel in tx_channels {
            let service = Self::channel_route(channel)?;
            let max_payload = Self::channel_max_payload(channel, default_max_payload)?;
            tx_cfgs.push(IceoryxChannelConfig {
                id: channel.channel.id,
                service,
                max_payload_bytes: max_payload,
            });
        }

        let mut rx_cfgs = Vec::with_capacity(rx_channels.len());
        for channel in rx_channels {
            let service = Self::channel_route(channel)?;
            let max_payload = Self::channel_max_payload(channel, default_max_payload)?;
            rx_cfgs.push(IceoryxChannelConfig {
                id: channel.channel.id,
                service,
                max_payload_bytes: max_payload,
            });
        }

        Ok(Self {
            node_name,
            tx_channels: tx_cfgs,
            rx_channels: rx_cfgs,
            ctx: None,
        })
    }

    fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
        let mut builder = NodeBuilder::new();
        if let Some(name) = &self.node_name {
            builder = builder.name(name);
        }
        let node = builder
            .create::<IceoryxService>()
            .map_err(|e| CuError::new_with_cause("Iceoryx2Bridge: Failed to create node", e))?;

        let ctx = Box::new(IceoryxContext::<Tx::Id, Rx::Id> {
            node,
            tx_channels: Vec::new(),
            rx_channels: Vec::new(),
        });
        self.ctx = Some(Box::into_raw(ctx) as usize);
        Ok(())
    }

    fn send<'a, Payload>(
        &mut self,
        _clock: &RobotClock,
        channel: &'static BridgeChannel<<Self::Tx as BridgeChannelSet>::Id, Payload>,
        msg: &CuMsg<Payload>,
    ) -> CuResult<()>
    where
        Payload: CuMsgPayload + 'a + 'static,
    {
        let cfg = self.find_tx_config(channel.id()).ok_or_else(|| {
            CuError::from(format!(
                "Iceoryx2Bridge: Unknown Tx channel {:?}",
                channel.id()
            ))
        })?;
        let service = cfg.service.clone();
        let max_payload_bytes = cfg.max_payload_bytes;

        let ctx = self.ctx_mut()?;

        if let Some(tx_channel) =
            Self::find_tx_channel_mut::<Payload>(&mut ctx.tx_channels, channel.id())?
        {
            return tx_channel.send(msg);
        }

        let mut new_channel =
            IceoryxTxChannel::<Payload>::new(&mut ctx.node, &service, max_payload_bytes)?;
        new_channel.send(msg)?;
        ctx.tx_channels.push(IceoryxTxChannelEntry {
            id: channel.id(),
            channel: Box::new(new_channel),
        });
        Ok(())
    }

    fn receive<'a, Payload>(
        &mut self,
        clock: &RobotClock,
        channel: &'static BridgeChannel<<Self::Rx as BridgeChannelSet>::Id, Payload>,
        msg: &mut CuMsg<Payload>,
    ) -> CuResult<()>
    where
        Payload: CuMsgPayload + 'a + 'static,
    {
        let cfg = self.find_rx_config(channel.id()).ok_or_else(|| {
            CuError::from(format!(
                "Iceoryx2Bridge: Unknown Rx channel {:?}",
                channel.id()
            ))
        })?;
        let service = cfg.service.clone();

        let ctx = self.ctx_mut()?;

        if let Some(rx_channel) =
            Self::find_rx_channel_mut::<Payload>(&mut ctx.rx_channels, channel.id())?
        {
            return rx_channel.receive(clock, msg);
        }

        let mut new_channel = IceoryxRxChannel::<Payload>::new(&mut ctx.node, &service)?;
        new_channel.receive(clock, msg)?;
        ctx.rx_channels.push(IceoryxRxChannelEntry {
            id: channel.id(),
            channel: Box::new(new_channel),
        });
        Ok(())
    }

    fn stop(&mut self, _clock: &RobotClock) -> CuResult<()> {
        if let Some(raw) = self.ctx.take() {
            let ptr = raw as *mut IceoryxContext<Tx::Id, Rx::Id>;
            // SAFETY: pointer was created by `Box::into_raw` in `start()`.
            unsafe { drop(Box::from_raw(ptr)) };
        }
        Ok(())
    }
}

impl<Tx, Rx> Drop for Iceoryx2Bridge<Tx, Rx>
where
    Tx: BridgeChannelSet + 'static,
    Rx: BridgeChannelSet + 'static,
    Tx::Id: Send + Sync + 'static,
    Rx::Id: Send + Sync + 'static,
{
    fn drop(&mut self) {
        if let Some(raw) = self.ctx.take() {
            let ptr = raw as *mut IceoryxContext<Tx::Id, Rx::Id>;
            // SAFETY: pointer was created by `Box::into_raw` in `start()`.
            unsafe { drop(Box::from_raw(ptr)) };
        }
    }
}
