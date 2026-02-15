mod attachment;
mod error;
mod keyexpr;
mod liveliness;
mod node;
mod topic;

use attachment::encode_attachment;
use cdr::{CdrBe, Infinite};
use cu_ros2_payloads::RosBridgeAdapter;
use cu29::clock::RobotClock;
use cu29::cubridge::{BridgeChannel, BridgeChannelConfig, BridgeChannelSet, CuBridge};
use cu29::prelude::*;
use liveliness::{node_liveliness, publisher_liveliness};
use node::Node;
use topic::Topic;
use zenoh::bytes::Encoding;
use zenoh::{Config, Error as ZenohError};

use std::any::{Any, TypeId, type_name};
use std::collections::HashMap;
use std::sync::{OnceLock, RwLock};

// One node per bridge session.
const NODE_ID: u32 = 0;

#[derive(Clone, Copy)]
struct RosPayloadCodec {
    namespace: &'static str,
    type_name: &'static str,
    type_hash: &'static str,
    encode_payload: fn(&dyn Any) -> CuResult<Vec<u8>>,
    decode_payload: fn(&[u8]) -> CuResult<Box<dyn Any>>,
}

struct RosPayloadRegistry {
    codecs: RwLock<HashMap<TypeId, RosPayloadCodec>>,
}

impl RosPayloadRegistry {
    fn new() -> Self {
        Self {
            codecs: RwLock::new(HashMap::new()),
        }
    }

    fn register<Payload>(&self)
    where
        Payload: CuMsgPayload + RosBridgeAdapter + 'static,
    {
        let codec = RosPayloadCodec {
            namespace: <Payload as RosBridgeAdapter>::namespace(),
            type_name: <Payload as RosBridgeAdapter>::type_name(),
            type_hash: <Payload as RosBridgeAdapter>::type_hash(),
            encode_payload: encode_payload_with::<Payload>,
            decode_payload: decode_payload_with::<Payload>,
        };
        self.codecs
            .write()
            .expect("RosPayloadRegistry lock poisoned")
            .insert(TypeId::of::<Payload>(), codec);
    }

    fn codec_for<Payload>(&self) -> CuResult<RosPayloadCodec>
    where
        Payload: CuMsgPayload + 'static,
    {
        self.codecs
            .read()
            .expect("RosPayloadRegistry lock poisoned")
            .get(&TypeId::of::<Payload>())
            .copied()
            .ok_or_else(|| {
                CuError::from(format!(
                    "Ros2Bridge: No ROS codec registered for payload type {}",
                    type_name::<Payload>()
                ))
            })
    }
}

fn encode_payload_with<Payload>(payload_any: &dyn Any) -> CuResult<Vec<u8>>
where
    Payload: CuMsgPayload + RosBridgeAdapter + 'static,
{
    let payload = payload_any.downcast_ref::<Payload>().ok_or_else(|| {
        CuError::from(format!(
            "Ros2Bridge: Registry encode payload mismatch for {}",
            type_name::<Payload>()
        ))
    })?;
    let ros_payload = payload.to_ros_message();
    cdr::serialize::<_, _, CdrBe>(&ros_payload, Infinite)
        .map_err(|e| CuError::new_with_cause("Ros2Bridge: Failed to serialize payload", e))
}

fn decode_payload_with<Payload>(bytes: &[u8]) -> CuResult<Box<dyn Any>>
where
    Payload: CuMsgPayload + RosBridgeAdapter + 'static,
{
    let ros_payload: <Payload as RosBridgeAdapter>::RosMessage = cdr::deserialize(bytes)
        .map_err(|e| CuError::new_with_cause("Ros2Bridge: Failed to deserialize payload", e))?;
    let payload = Payload::from_ros_message(ros_payload).map_err(CuError::from)?;
    Ok(Box::new(payload))
}

fn payload_registry() -> &'static RosPayloadRegistry {
    static REGISTRY: OnceLock<RosPayloadRegistry> = OnceLock::new();
    REGISTRY.get_or_init(|| {
        let registry = RosPayloadRegistry::new();
        registry.register::<bool>();
        registry.register::<i8>();
        registry.register::<i16>();
        registry.register::<i32>();
        registry.register::<i64>();
        registry.register::<u8>();
        registry.register::<u16>();
        registry.register::<u32>();
        registry.register::<u64>();
        registry.register::<f32>();
        registry.register::<f64>();
        registry.register::<String>();
        registry
    })
}

/// Register a payload codec for ROS2 bridge transport.
///
/// Call this once at application startup for custom payload types that implement
/// [`cu_ros2_payloads::RosBridgeAdapter`].
pub fn register_ros2_payload<Payload>()
where
    Payload: CuMsgPayload + RosBridgeAdapter + 'static,
{
    payload_registry().register::<Payload>();
}

#[derive(Debug, Clone)]
struct Ros2ChannelConfig<Id: Copy> {
    id: Id,
    route: String,
}

type Ros2Subscriber =
    zenoh::pubsub::Subscriber<zenoh::handlers::FifoChannelHandler<zenoh::sample::Sample>>;

struct Ros2TxChannel<Id: Copy> {
    id: Id,
    route: String,
    entity_id: u32,
    sequence_number: u64,
    publisher: Option<zenoh::pubsub::Publisher<'static>>,
    publisher_token: Option<zenoh::liveliness::LivelinessToken>,
}

struct Ros2RxChannel<Id: Copy> {
    id: Id,
    route: String,
    subscriber: Option<Ros2Subscriber>,
}

struct Ros2Context<TxId: Copy, RxId: Copy> {
    session: zenoh::Session,
    #[allow(dead_code)]
    node_token: zenoh::liveliness::LivelinessToken,
    tx_channels: Vec<Ros2TxChannel<TxId>>,
    rx_channels: Vec<Ros2RxChannel<RxId>>,
}

#[derive(Reflect)]
#[reflect(from_reflect = false, no_field_bounds, type_path = false)]
pub struct Ros2Bridge<Tx, Rx>
where
    Tx: BridgeChannelSet + 'static,
    Rx: BridgeChannelSet + 'static,
    Tx::Id: Send + Sync + 'static,
    Rx::Id: Send + Sync + 'static,
{
    #[reflect(ignore)]
    session_config: Config,
    domain_id: u32,
    namespace: String,
    node: String,
    #[reflect(ignore)]
    tx_channels: Vec<Ros2ChannelConfig<Tx::Id>>,
    #[reflect(ignore)]
    rx_channels: Vec<Ros2ChannelConfig<Rx::Id>>,
    #[reflect(ignore)]
    ctx: Option<Ros2Context<Tx::Id, Rx::Id>>,
}

impl<Tx, Rx> Freezable for Ros2Bridge<Tx, Rx>
where
    Tx: BridgeChannelSet + 'static,
    Rx: BridgeChannelSet + 'static,
    Tx::Id: Send + Sync + 'static,
    Rx::Id: Send + Sync + 'static,
{
}

impl<Tx, Rx> cu29::reflect::TypePath for Ros2Bridge<Tx, Rx>
where
    Tx: BridgeChannelSet + 'static,
    Rx: BridgeChannelSet + 'static,
    Tx::Id: Send + Sync + 'static,
    Rx::Id: Send + Sync + 'static,
{
    fn type_path() -> &'static str {
        "cu_ros2_bridge::Ros2Bridge"
    }

    fn short_type_path() -> &'static str {
        "Ros2Bridge"
    }

    fn type_ident() -> Option<&'static str> {
        Some("Ros2Bridge")
    }

    fn crate_name() -> Option<&'static str> {
        Some("cu_ros2_bridge")
    }

    fn module_path() -> Option<&'static str> {
        Some("cu_ros2_bridge")
    }
}

impl<Tx, Rx> Ros2Bridge<Tx, Rx>
where
    Tx: BridgeChannelSet + 'static,
    Rx: BridgeChannelSet + 'static,
    Tx::Id: Send + Sync + 'static,
    Rx::Id: Send + Sync + 'static,
{
    fn parse_session_config(config: &ComponentConfig) -> CuResult<Config> {
        if let Some(path) = config.get::<String>("zenoh_config_file")? {
            return Config::from_file(&path).map_err(|e| {
                CuError::from(format!("Ros2Bridge: Failed to read config file: {e}"))
            });
        }
        if let Some(json) = config.get::<String>("zenoh_config_json")? {
            return Config::from_json5(&json).map_err(|e| {
                CuError::from(format!("Ros2Bridge: Failed to parse config json: {e}"))
            });
        }
        Ok(Config::default())
    }

    fn parse_domain_id(config: &ComponentConfig) -> CuResult<u32> {
        Ok(config.get::<u32>("domain_id")?.unwrap_or(0))
    }

    fn parse_namespace(config: &ComponentConfig) -> CuResult<String> {
        Ok(config
            .get::<String>("namespace")?
            .unwrap_or_else(|| "copper".to_string()))
    }

    fn parse_node_name(config: &ComponentConfig) -> CuResult<String> {
        Ok(config
            .get::<String>("node")?
            .unwrap_or_else(|| "node".to_string()))
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
                    "Ros2Bridge: Missing route/topic for channel {:?}",
                    id
                ))
            })
    }

    fn make_node<'a>(
        domain_id: u32,
        namespace: &'a str,
        node_name: &'a str,
        session: &'a zenoh::Session,
    ) -> Node<'a> {
        Node {
            domain_id,
            zid: session.zid(),
            id: NODE_ID,
            namespace,
            name: node_name,
        }
    }

    fn find_tx_channel_index(channels: &[Ros2TxChannel<Tx::Id>], id: Tx::Id) -> Option<usize> {
        channels.iter().position(|channel| channel.id == id)
    }

    fn find_rx_channel_index(channels: &[Ros2RxChannel<Rx::Id>], id: Rx::Id) -> Option<usize> {
        channels.iter().position(|channel| channel.id == id)
    }

    fn codec_for_payload<Payload>() -> CuResult<RosPayloadCodec>
    where
        Payload: CuMsgPayload + 'static,
    {
        payload_registry().codec_for::<Payload>()
    }

    fn topic_for_codec<'a>(route: &'a str, codec: RosPayloadCodec) -> Topic<'a> {
        Topic::from_ros_type(route, codec.namespace, codec.type_name, codec.type_hash)
    }

    fn encode_payload<Payload>(msg: &CuMsg<Payload>, codec: RosPayloadCodec) -> CuResult<Vec<u8>>
    where
        Payload: CuMsgPayload + 'static,
    {
        let payload = msg
            .payload()
            .ok_or_else(|| CuError::from("Ros2Bridge: Cannot send empty payload through bridge"))?;
        (codec.encode_payload)(payload as &dyn Any)
    }

    fn decode_payload_into<Payload>(
        bytes: &[u8],
        msg: &mut CuMsg<Payload>,
        codec: RosPayloadCodec,
    ) -> CuResult<()>
    where
        Payload: CuMsgPayload + 'static,
    {
        let payload_any = (codec.decode_payload)(bytes)?;
        let payload = payload_any.downcast::<Payload>().map_err(|_| {
            CuError::from(format!(
                "Ros2Bridge: Codec produced wrong payload type for {}",
                type_name::<Payload>()
            ))
        })?;
        msg.set_payload(*payload);
        Ok(())
    }

    fn init_tx_channel(
        domain_id: u32,
        namespace: &str,
        node_name: &str,
        ctx: &mut Ros2Context<Tx::Id, Rx::Id>,
        tx_idx: usize,
        codec: RosPayloadCodec,
    ) -> CuResult<()> {
        if ctx.tx_channels[tx_idx].publisher.is_some() {
            return Ok(());
        }

        let route = ctx.tx_channels[tx_idx].route.clone();
        let entity_id = ctx.tx_channels[tx_idx].entity_id;
        let topic = Self::topic_for_codec(route.as_str(), codec);
        let node = Self::make_node(domain_id, namespace, node_name, &ctx.session);

        let publisher_token = zenoh::Wait::wait(
            ctx.session
                .liveliness()
                .declare_token(publisher_liveliness(&node, &topic, entity_id)?),
        )
        .map_err(cu_error_map(
            "Ros2Bridge: Failed to declare topic liveliness token",
        ))?;

        let publisher =
            zenoh::Wait::wait(ctx.session.declare_publisher(topic.pubsub_keyexpr(&node)?))
                .map_err(cu_error_map("Ros2Bridge: Failed to create publisher"))?;

        ctx.tx_channels[tx_idx].publisher = Some(publisher);
        ctx.tx_channels[tx_idx].publisher_token = Some(publisher_token);

        Ok(())
    }

    fn init_rx_channel(
        domain_id: u32,
        namespace: &str,
        node_name: &str,
        ctx: &mut Ros2Context<Tx::Id, Rx::Id>,
        rx_idx: usize,
        codec: RosPayloadCodec,
    ) -> CuResult<()> {
        if ctx.rx_channels[rx_idx].subscriber.is_some() {
            return Ok(());
        }

        let route = ctx.rx_channels[rx_idx].route.clone();
        let topic = Self::topic_for_codec(route.as_str(), codec);
        let node = Self::make_node(domain_id, namespace, node_name, &ctx.session);

        let keyexpr = topic.pubsub_keyexpr(&node)?;
        let subscriber = zenoh::Wait::wait(ctx.session.declare_subscriber(keyexpr))
            .map_err(cu_error_map("Ros2Bridge: Failed to declare subscriber"))?;

        ctx.rx_channels[rx_idx].subscriber = Some(subscriber);
        Ok(())
    }

    fn ctx_mut(&mut self) -> CuResult<&mut Ros2Context<Tx::Id, Rx::Id>> {
        self.ctx
            .as_mut()
            .ok_or_else(|| CuError::from("Ros2Bridge: Context not initialized"))
    }
}

impl<Tx, Rx> CuBridge for Ros2Bridge<Tx, Rx>
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
        let default_config = ComponentConfig::default();
        let config = config.unwrap_or(&default_config);

        let session_config = Self::parse_session_config(config)?;
        let domain_id = Self::parse_domain_id(config)?;
        let namespace = Self::parse_namespace(config)?;
        let node = Self::parse_node_name(config)?;

        let mut tx_cfgs = Vec::with_capacity(tx_channels.len());
        for channel in tx_channels {
            let route = Self::channel_route(channel)?;
            tx_cfgs.push(Ros2ChannelConfig {
                id: channel.channel.id,
                route,
            });
        }

        let mut rx_cfgs = Vec::with_capacity(rx_channels.len());
        for channel in rx_channels {
            let route = Self::channel_route(channel)?;
            rx_cfgs.push(Ros2ChannelConfig {
                id: channel.channel.id,
                route,
            });
        }

        Ok(Self {
            session_config,
            domain_id,
            namespace,
            node,
            tx_channels: tx_cfgs,
            rx_channels: rx_cfgs,
            ctx: None,
        })
    }

    fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
        let session = zenoh::Wait::wait(zenoh::open(self.session_config.clone()))
            .map_err(cu_error_map("Ros2Bridge: Failed to open session"))?;

        let node = Self::make_node(self.domain_id, &self.namespace, &self.node, &session);
        let node_token =
            zenoh::Wait::wait(session.liveliness().declare_token(node_liveliness(&node)?))
                .map_err(cu_error_map(
                    "Ros2Bridge: Failed to declare node liveliness token",
                ))?;

        let tx_channels = self
            .tx_channels
            .iter()
            .enumerate()
            .map(|(index, channel)| Ros2TxChannel {
                id: channel.id,
                route: channel.route.clone(),
                entity_id: (index + 1) as u32,
                sequence_number: 0,
                publisher: None,
                publisher_token: None,
            })
            .collect();

        let rx_channels = self
            .rx_channels
            .iter()
            .map(|channel| Ros2RxChannel {
                id: channel.id,
                route: channel.route.clone(),
                subscriber: None,
            })
            .collect();

        self.ctx = Some(Ros2Context {
            session,
            node_token,
            tx_channels,
            rx_channels,
        });

        Ok(())
    }

    fn send<'a, Payload>(
        &mut self,
        clock: &RobotClock,
        channel: &'static BridgeChannel<<Self::Tx as BridgeChannelSet>::Id, Payload>,
        msg: &CuMsg<Payload>,
    ) -> CuResult<()>
    where
        Payload: CuMsgPayload + 'a,
    {
        let codec = Self::codec_for_payload::<Payload>()?;
        let domain_id = self.domain_id;
        let namespace = self.namespace.clone();
        let node_name = self.node.clone();
        let channel_id = channel.id();
        let ctx = self.ctx_mut()?;

        let tx_idx =
            Self::find_tx_channel_index(&ctx.tx_channels, channel_id).ok_or_else(|| {
                CuError::from(format!("Ros2Bridge: Unknown Tx channel {:?}", channel_id))
            })?;

        Self::init_tx_channel(domain_id, &namespace, &node_name, ctx, tx_idx, codec)?;

        let encoded = Self::encode_payload(msg, codec)?;
        let session_zid = ctx.session.zid();

        let tx_channel = &mut ctx.tx_channels[tx_idx];
        let publisher = tx_channel
            .publisher
            .as_mut()
            .ok_or_else(|| CuError::from("Ros2Bridge: Tx publisher not initialized"))?;

        let attachment = encode_attachment(tx_channel.sequence_number, clock, &session_zid);
        tx_channel.sequence_number += 1;

        zenoh::Wait::wait(
            publisher
                .put(encoded)
                .encoding(Encoding::APPLICATION_CDR)
                .attachment(attachment),
        )
        .map_err(cu_error_map("Ros2Bridge: Failed to put value"))?;

        Ok(())
    }

    fn receive<'a, Payload>(
        &mut self,
        clock: &RobotClock,
        channel: &'static BridgeChannel<<Self::Rx as BridgeChannelSet>::Id, Payload>,
        msg: &mut CuMsg<Payload>,
    ) -> CuResult<()>
    where
        Payload: CuMsgPayload + 'a,
    {
        let codec = Self::codec_for_payload::<Payload>()?;
        let domain_id = self.domain_id;
        let namespace = self.namespace.clone();
        let node_name = self.node.clone();
        let channel_id = channel.id();
        let ctx = self.ctx_mut()?;

        let rx_idx =
            Self::find_rx_channel_index(&ctx.rx_channels, channel_id).ok_or_else(|| {
                CuError::from(format!("Ros2Bridge: Unknown Rx channel {:?}", channel_id))
            })?;

        Self::init_rx_channel(domain_id, &namespace, &node_name, ctx, rx_idx, codec)?;

        msg.tov = Tov::Time(clock.now());

        let sample = {
            let subscriber = ctx.rx_channels[rx_idx]
                .subscriber
                .as_mut()
                .ok_or_else(|| CuError::from("Ros2Bridge: Rx subscriber not initialized"))?;
            subscriber
                .try_recv()
                .map_err(|e| CuError::from(format!("Ros2Bridge: receive failed: {e}")))?
        };

        if let Some(sample) = sample {
            let payload = sample.payload().to_bytes();
            Self::decode_payload_into(payload.as_ref(), msg, codec)?;
        } else {
            msg.clear_payload();
        }

        Ok(())
    }

    fn stop(&mut self, _clock: &RobotClock) -> CuResult<()> {
        if let Some(Ros2Context {
            session,
            node_token: _,
            tx_channels,
            rx_channels,
        }) = self.ctx.take()
        {
            for channel in tx_channels {
                if let Some(publisher) = channel.publisher {
                    zenoh::Wait::wait(publisher.undeclare())
                        .map_err(cu_error_map("Ros2Bridge: Failed to undeclare publisher"))?;
                }
            }

            for channel in rx_channels {
                if let Some(subscriber) = channel.subscriber {
                    zenoh::Wait::wait(subscriber.undeclare())
                        .map_err(cu_error_map("Ros2Bridge: Failed to undeclare subscriber"))?;
                }
            }

            zenoh::Wait::wait(session.close())
                .map_err(cu_error_map("Ros2Bridge: Failed to close session"))?;
        }

        Ok(())
    }
}

fn cu_error_map(msg: &str) -> impl FnOnce(ZenohError) -> CuError + '_ {
    move |e| CuError::from(format!("{msg}: {e}"))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn i8_payload_cdr_roundtrip() {
        let mut src = CuMsg::<i8>::default();
        src.set_payload(42);

        let codec =
            Ros2Bridge::<crate::tests::DummyTx, crate::tests::DummyRx>::codec_for_payload::<i8>()
                .expect("codec should be registered");
        let bytes =
            Ros2Bridge::<crate::tests::DummyTx, crate::tests::DummyRx>::encode_payload(&src, codec)
                .expect("encode should succeed");

        let mut dst = CuMsg::<i8>::default();
        Ros2Bridge::<crate::tests::DummyTx, crate::tests::DummyRx>::decode_payload_into(
            bytes.as_slice(),
            &mut dst,
            codec,
        )
        .expect("decode should succeed");

        assert_eq!(dst.payload(), Some(&42));
    }

    #[test]
    fn default_scalar_codecs_registered() {
        Ros2Bridge::<crate::tests::DummyTx, crate::tests::DummyRx>::codec_for_payload::<bool>()
            .expect("bool codec");
        Ros2Bridge::<crate::tests::DummyTx, crate::tests::DummyRx>::codec_for_payload::<i8>()
            .expect("i8 codec");
        Ros2Bridge::<crate::tests::DummyTx, crate::tests::DummyRx>::codec_for_payload::<i16>()
            .expect("i16 codec");
        Ros2Bridge::<crate::tests::DummyTx, crate::tests::DummyRx>::codec_for_payload::<i32>()
            .expect("i32 codec");
        Ros2Bridge::<crate::tests::DummyTx, crate::tests::DummyRx>::codec_for_payload::<i64>()
            .expect("i64 codec");
        Ros2Bridge::<crate::tests::DummyTx, crate::tests::DummyRx>::codec_for_payload::<u8>()
            .expect("u8 codec");
        Ros2Bridge::<crate::tests::DummyTx, crate::tests::DummyRx>::codec_for_payload::<u16>()
            .expect("u16 codec");
        Ros2Bridge::<crate::tests::DummyTx, crate::tests::DummyRx>::codec_for_payload::<u32>()
            .expect("u32 codec");
        Ros2Bridge::<crate::tests::DummyTx, crate::tests::DummyRx>::codec_for_payload::<u64>()
            .expect("u64 codec");
        Ros2Bridge::<crate::tests::DummyTx, crate::tests::DummyRx>::codec_for_payload::<f32>()
            .expect("f32 codec");
        Ros2Bridge::<crate::tests::DummyTx, crate::tests::DummyRx>::codec_for_payload::<f64>()
            .expect("f64 codec");
        Ros2Bridge::<crate::tests::DummyTx, crate::tests::DummyRx>::codec_for_payload::<String>()
            .expect("string codec");
    }

    tx_channels! {
        struct DummyTx : DummyTxId {
            out => i8,
        }
    }

    rx_channels! {
        struct DummyRx : DummyRxId {
            input => i8,
        }
    }
}
