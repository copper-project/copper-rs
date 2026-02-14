mod attachment;
mod error;
mod keyexpr;
mod liveliness;
mod node;
mod topic;

use attachment::encode_attachment;
use cdr::{CdrBe, Infinite};
use cu_ros_payloads::RosMsgAdapter;
use cu_ros_payloads::std_msgs::Int8;
use cu29::clock::RobotClock;
use cu29::cubridge::{BridgeChannel, BridgeChannelConfig, BridgeChannelSet, CuBridge};
use cu29::prelude::*;
use liveliness::{node_liveliness, publisher_liveliness};
use node::Node;
use topic::Topic;
use zenoh::bytes::Encoding;
use zenoh::{Config, Error as ZenohError};

use std::any::type_name;

// One node per bridge session.
const NODE_ID: u32 = 0;

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
    #[reflect(ignore)]
    domain_id: u32,
    #[reflect(ignore)]
    namespace: String,
    #[reflect(ignore)]
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
    fn parse_session_config(config: Option<&ComponentConfig>) -> CuResult<Config> {
        if let Some(config) = config {
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
        }
        Ok(Config::default())
    }

    fn parse_domain_id(config: Option<&ComponentConfig>) -> CuResult<u32> {
        if let Some(config) = config {
            return Ok(config.get::<u32>("domain_id")?.unwrap_or(0));
        }
        Ok(0)
    }

    fn parse_namespace(config: Option<&ComponentConfig>) -> CuResult<String> {
        if let Some(config) = config {
            return Ok(config
                .get::<String>("namespace")?
                .unwrap_or_else(|| "copper".to_string()));
        }
        Ok("copper".to_string())
    }

    fn parse_node_name(config: Option<&ComponentConfig>) -> CuResult<String> {
        if let Some(config) = config {
            return Ok(config
                .get::<String>("node")?
                .unwrap_or_else(|| "node".to_string()));
        }
        Ok("node".to_string())
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

    fn tx_topic_for_message<'a, Payload>(
        msg: &CuMsg<Payload>,
        route: &'a str,
    ) -> CuResult<Topic<'a>>
    where
        Payload: CuMsgPayload + 'a + 'static,
    {
        if msg.downcast_ref::<i8>().is_ok() {
            return Ok(Topic::new::<i8>(route));
        }

        Err(CuError::from(format!(
            "Ros2Bridge: Unsupported Tx payload type {}",
            type_name::<Payload>()
        )))
    }

    fn rx_topic_for_message<'a, Payload>(
        msg: &mut CuMsg<Payload>,
        route: &'a str,
    ) -> CuResult<Topic<'a>>
    where
        Payload: CuMsgPayload + 'a + 'static,
    {
        if msg.downcast_mut::<i8>().is_ok() {
            return Ok(Topic::new::<i8>(route));
        }

        Err(CuError::from(format!(
            "Ros2Bridge: Unsupported Rx payload type {}",
            type_name::<Payload>()
        )))
    }

    fn encode_payload<Payload>(msg: &CuMsg<Payload>) -> CuResult<Vec<u8>>
    where
        Payload: CuMsgPayload + 'static,
    {
        if let Ok(cu_msg) = msg.downcast_ref::<i8>() {
            let payload = cu_msg.payload().ok_or_else(|| {
                CuError::from("Ros2Bridge: Cannot send empty payload through ROS2 bridge")
            })?;
            let ros_payload = payload.convert();
            return cdr::serialize::<_, _, CdrBe>(&ros_payload, Infinite).map_err(|e| {
                CuError::new_with_cause("Ros2Bridge: Failed to serialize payload", e)
            });
        }

        Err(CuError::from(format!(
            "Ros2Bridge: Unsupported Tx payload type {}",
            type_name::<Payload>()
        )))
    }

    fn decode_payload_into<Payload>(bytes: &[u8], msg: &mut CuMsg<Payload>) -> CuResult<()>
    where
        Payload: CuMsgPayload + 'static,
    {
        if let Ok(cu_msg) = msg.downcast_mut::<i8>() {
            let (ros_msg, _hash): (Int8, String) = cdr::deserialize(bytes).map_err(|e| {
                CuError::new_with_cause("Ros2Bridge: Failed to deserialize payload", e)
            })?;
            cu_msg.set_payload(ros_msg.data);
            return Ok(());
        }

        Err(CuError::from(format!(
            "Ros2Bridge: Unsupported Rx payload type {}",
            type_name::<Payload>()
        )))
    }

    fn init_tx_channel<Payload>(
        domain_id: u32,
        namespace: &str,
        node_name: &str,
        ctx: &mut Ros2Context<Tx::Id, Rx::Id>,
        tx_idx: usize,
        msg: &CuMsg<Payload>,
    ) -> CuResult<()>
    where
        Payload: CuMsgPayload + 'static,
    {
        if ctx.tx_channels[tx_idx].publisher.is_some() {
            return Ok(());
        }

        let route = ctx.tx_channels[tx_idx].route.clone();
        let entity_id = ctx.tx_channels[tx_idx].entity_id;
        let topic = Self::tx_topic_for_message(msg, route.as_str())?;
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

    fn init_rx_channel<Payload>(
        domain_id: u32,
        namespace: &str,
        node_name: &str,
        ctx: &mut Ros2Context<Tx::Id, Rx::Id>,
        rx_idx: usize,
        msg: &mut CuMsg<Payload>,
    ) -> CuResult<()>
    where
        Payload: CuMsgPayload + 'static,
    {
        if ctx.rx_channels[rx_idx].subscriber.is_some() {
            return Ok(());
        }

        let route = ctx.rx_channels[rx_idx].route.clone();
        let topic = Self::rx_topic_for_message(msg, route.as_str())?;
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
        let domain_id = self.domain_id;
        let namespace = self.namespace.clone();
        let node_name = self.node.clone();
        let channel_id = channel.id();
        let ctx = self.ctx_mut()?;

        let tx_idx =
            Self::find_tx_channel_index(&ctx.tx_channels, channel_id).ok_or_else(|| {
                CuError::from(format!("Ros2Bridge: Unknown Tx channel {:?}", channel_id))
            })?;

        Self::init_tx_channel(domain_id, &namespace, &node_name, ctx, tx_idx, msg)?;

        let encoded = Self::encode_payload(msg)?;
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
        let domain_id = self.domain_id;
        let namespace = self.namespace.clone();
        let node_name = self.node.clone();
        let channel_id = channel.id();
        let ctx = self.ctx_mut()?;

        let rx_idx =
            Self::find_rx_channel_index(&ctx.rx_channels, channel_id).ok_or_else(|| {
                CuError::from(format!("Ros2Bridge: Unknown Rx channel {:?}", channel_id))
            })?;

        Self::init_rx_channel(domain_id, &namespace, &node_name, ctx, rx_idx, msg)?;

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
            Self::decode_payload_into(payload.as_ref(), msg)?;
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

fn cu_error(msg: &str, error: ZenohError) -> CuError {
    CuError::from(format!("{msg}: {error}"))
}

fn cu_error_map(msg: &str) -> impl FnOnce(ZenohError) -> CuError + '_ {
    move |e| cu_error(msg, e)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn i8_payload_cdr_roundtrip() {
        let mut src = CuMsg::<i8>::default();
        src.set_payload(42);

        let bytes =
            Ros2Bridge::<crate::tests::DummyTx, crate::tests::DummyRx>::encode_payload(&src)
                .expect("encode should succeed");

        let mut dst = CuMsg::<i8>::default();
        Ros2Bridge::<crate::tests::DummyTx, crate::tests::DummyRx>::decode_payload_into(
            bytes.as_slice(),
            &mut dst,
        )
        .expect("decode should succeed");

        assert_eq!(dst.payload(), Some(&42));
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
