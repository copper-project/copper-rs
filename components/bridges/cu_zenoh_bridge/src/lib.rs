use cu29::cubridge::{
    BridgeChannel, BridgeChannelConfig, BridgeChannelInfo, BridgeChannelSet, CuBridge,
};
use cu29::prelude::*;
use serde::{Deserialize, Serialize};
use zenoh::bytes::Encoding;
use zenoh::key_expr::KeyExpr;
use zenoh::{Config, Error as ZenohError};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum WireFormat {
    Bincode,
    Json,
    Cbor,
}

impl WireFormat {
    fn parse(value: &str) -> Option<Self> {
        match value.trim().to_ascii_lowercase().as_str() {
            "bincode" | "bin" | "binary" => Some(Self::Bincode),
            "json" => Some(Self::Json),
            "cbor" => Some(Self::Cbor),
            _ => None,
        }
    }

    fn encoding(self) -> Encoding {
        match self {
            Self::Bincode => Encoding::APPLICATION_OCTET_STREAM,
            Self::Json => Encoding::APPLICATION_JSON,
            Self::Cbor => Encoding::APPLICATION_CBOR,
        }
    }
}

#[derive(Debug, Clone)]
struct ZenohChannelConfig<Id: Copy> {
    id: Id,
    route: String,
    wire_format: WireFormat,
}

type ZenohSubscriber =
    zenoh::pubsub::Subscriber<zenoh::handlers::FifoChannelHandler<zenoh::sample::Sample>>;

struct ZenohTxChannel<Id: Copy> {
    id: Id,
    publisher: zenoh::pubsub::Publisher<'static>,
    wire_format: WireFormat,
}

struct ZenohRxChannel<Id: Copy> {
    id: Id,
    subscriber: ZenohSubscriber,
    wire_format: WireFormat,
}

struct ZenohContext<TxId: Copy, RxId: Copy> {
    session: zenoh::Session,
    tx_channels: Vec<ZenohTxChannel<TxId>>,
    rx_channels: Vec<ZenohRxChannel<RxId>>,
}

pub struct ZenohBridge<Tx, Rx>
where
    Tx: BridgeChannelSet,
    Rx: BridgeChannelSet,
{
    session_config: Config,
    tx_channels: Vec<ZenohChannelConfig<Tx::Id>>,
    rx_channels: Vec<ZenohChannelConfig<Rx::Id>>,
    ctx: Option<ZenohContext<Tx::Id, Rx::Id>>,
}

impl<Tx, Rx> Freezable for ZenohBridge<Tx, Rx>
where
    Tx: BridgeChannelSet,
    Rx: BridgeChannelSet,
{
}

impl<Tx, Rx> ZenohBridge<Tx, Rx>
where
    Tx: BridgeChannelSet,
    Rx: BridgeChannelSet,
{
    fn parse_session_config(config: Option<&ComponentConfig>) -> CuResult<Config> {
        if let Some(config) = config {
            if let Some(path) = config.get::<String>("zenoh_config_file") {
                return Config::from_file(&path).map_err(|e| {
                    CuError::from(format!("ZenohBridge: Failed to read config file: {e}"))
                });
            }
            if let Some(json) = config.get::<String>("zenoh_config_json") {
                return Config::from_json5(&json).map_err(|e| {
                    CuError::from(format!("ZenohBridge: Failed to parse config json: {e}"))
                });
            }
        }
        Ok(Config::default())
    }

    fn parse_default_wire_format(config: Option<&ComponentConfig>) -> CuResult<WireFormat> {
        if let Some(config) = config
            && let Some(raw) = config.get::<String>("wire_format")
        {
            return WireFormat::parse(&raw).ok_or_else(|| {
                CuError::from(format!(
                    "ZenohBridge: Unsupported wire_format '{raw}', expected bincode/json/cbor"
                ))
            });
        }
        Ok(WireFormat::Bincode)
    }

    fn channel_route<Id: Copy + core::fmt::Debug>(
        channel: &BridgeChannelConfig<Id>,
    ) -> CuResult<String> {
        channel
            .effective_route()
            .map(|route| route.into_owned())
            .ok_or_else(|| {
                let id = channel.channel.id;
                CuError::from(format!("ZenohBridge: Missing route for channel {:?}", id))
            })
    }

    fn channel_wire_format<Id: Copy>(
        channel: &BridgeChannelConfig<Id>,
        default: WireFormat,
    ) -> CuResult<WireFormat> {
        if let Some(config) = channel.config.as_ref()
            && let Some(raw) = config.get::<String>("wire_format")
        {
            return WireFormat::parse(&raw).ok_or_else(|| {
                CuError::from(format!(
                    "ZenohBridge: Unsupported wire_format '{raw}', expected bincode/json/cbor"
                ))
            });
        }
        Ok(default)
    }

    fn encode_message<Payload: CuMsgPayload>(
        wire_format: WireFormat,
        msg: &CuMsg<Payload>,
    ) -> CuResult<Vec<u8>> {
        match wire_format {
            WireFormat::Bincode => bincode::encode_to_vec(msg, bincode::config::standard())
                .map_err(|e| CuError::new_with_cause("ZenohBridge: bincode encode failed", e)),
            WireFormat::Json => serde_json::to_vec(msg)
                .map_err(|e| CuError::new_with_cause("ZenohBridge: json encode failed", e)),
            WireFormat::Cbor => minicbor_serde::to_vec(msg)
                .map_err(|e| CuError::new_with_cause("ZenohBridge: cbor encode failed", e)),
        }
    }

    fn decode_message<Payload: CuMsgPayload>(
        wire_format: WireFormat,
        bytes: &[u8],
    ) -> CuResult<CuMsg<Payload>> {
        match wire_format {
            WireFormat::Bincode => {
                let (decoded, _): (CuMsg<Payload>, usize) =
                    bincode::decode_from_slice(bytes, bincode::config::standard()).map_err(
                        |e| CuError::new_with_cause("ZenohBridge: bincode decode failed", e),
                    )?;
                Ok(decoded)
            }
            WireFormat::Json => serde_json::from_slice(bytes)
                .map_err(|e| CuError::new_with_cause("ZenohBridge: json decode failed", e)),
            WireFormat::Cbor => minicbor_serde::from_slice(bytes)
                .map_err(|e| CuError::new_with_cause("ZenohBridge: cbor decode failed", e)),
        }
    }

    fn find_tx_channel_mut(
        channels: &mut [ZenohTxChannel<Tx::Id>],
        id: Tx::Id,
    ) -> Option<&mut ZenohTxChannel<Tx::Id>> {
        channels.iter_mut().find(|channel| channel.id == id)
    }

    fn find_rx_channel_mut(
        channels: &mut [ZenohRxChannel<Rx::Id>],
        id: Rx::Id,
    ) -> Option<&mut ZenohRxChannel<Rx::Id>> {
        channels.iter_mut().find(|channel| channel.id == id)
    }
}

impl<Tx, Rx> CuBridge for ZenohBridge<Tx, Rx>
where
    Tx: BridgeChannelSet,
    Rx: BridgeChannelSet,
    Tx::Id: core::fmt::Debug,
    Rx::Id: core::fmt::Debug,
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
        let default_wire_format = Self::parse_default_wire_format(config)?;

        let mut tx_cfgs = Vec::with_capacity(tx_channels.len());
        for channel in tx_channels {
            let route = Self::channel_route(channel)?;
            let wire_format = Self::channel_wire_format(channel, default_wire_format)?;
            tx_cfgs.push(ZenohChannelConfig {
                id: channel.channel.id,
                route,
                wire_format,
            });
        }

        let mut rx_cfgs = Vec::with_capacity(rx_channels.len());
        for channel in rx_channels {
            let route = Self::channel_route(channel)?;
            let wire_format = Self::channel_wire_format(channel, default_wire_format)?;
            rx_cfgs.push(ZenohChannelConfig {
                id: channel.channel.id,
                route,
                wire_format,
            });
        }

        Ok(Self {
            session_config,
            tx_channels: tx_cfgs,
            rx_channels: rx_cfgs,
            ctx: None,
        })
    }

    fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
        let session = zenoh::Wait::wait(zenoh::open(self.session_config.clone()))
            .map_err(cu_error_map("ZenohBridge: Failed to open session"))?;

        let mut tx_channels = Vec::with_capacity(self.tx_channels.len());
        for channel in &self.tx_channels {
            let key_expr = KeyExpr::<'static>::new(channel.route.clone())
                .map_err(cu_error_map("ZenohBridge: Invalid Tx key expression"))?;
            let publisher = zenoh::Wait::wait(session.declare_publisher(key_expr))
                .map_err(cu_error_map("ZenohBridge: Failed to declare publisher"))?;
            tx_channels.push(ZenohTxChannel {
                id: channel.id,
                publisher,
                wire_format: channel.wire_format,
            });
        }

        let mut rx_channels = Vec::with_capacity(self.rx_channels.len());
        for channel in &self.rx_channels {
            let key_expr = KeyExpr::<'static>::new(channel.route.clone())
                .map_err(cu_error_map("ZenohBridge: Invalid Rx key expression"))?;
            let subscriber = zenoh::Wait::wait(session.declare_subscriber(key_expr))
                .map_err(cu_error_map("ZenohBridge: Failed to declare subscriber"))?;
            rx_channels.push(ZenohRxChannel {
                id: channel.id,
                subscriber,
                wire_format: channel.wire_format,
            });
        }

        self.ctx = Some(ZenohContext {
            session,
            tx_channels,
            rx_channels,
        });
        Ok(())
    }

    fn send<'a, Payload>(
        &mut self,
        _clock: &RobotClock,
        channel: &'static BridgeChannel<<Self::Tx as BridgeChannelSet>::Id, Payload>,
        msg: &CuMsg<Payload>,
    ) -> CuResult<()>
    where
        Payload: CuMsgPayload + 'a,
    {
        let ctx = self
            .ctx
            .as_mut()
            .ok_or_else(|| CuError::from("ZenohBridge: Context not initialized"))?;
        let tx_channel =
            Self::find_tx_channel_mut(&mut ctx.tx_channels, channel.id()).ok_or_else(|| {
                CuError::from(format!(
                    "ZenohBridge: Unknown Tx channel {:?}",
                    channel.id()
                ))
            })?;

        let encoded = Self::encode_message(tx_channel.wire_format, msg)?;
        zenoh::Wait::wait(
            tx_channel
                .publisher
                .put(encoded)
                .encoding(tx_channel.wire_format.encoding()),
        )
        .map_err(cu_error_map("ZenohBridge: Failed to publish"))?;
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
        let ctx = self
            .ctx
            .as_mut()
            .ok_or_else(|| CuError::from("ZenohBridge: Context not initialized"))?;
        let rx_channel =
            Self::find_rx_channel_mut(&mut ctx.rx_channels, channel.id()).ok_or_else(|| {
                CuError::from(format!(
                    "ZenohBridge: Unknown Rx channel {:?}",
                    channel.id()
                ))
            })?;

        msg.tov = Tov::Time(clock.now());

        let sample = rx_channel
            .subscriber
            .try_recv()
            .map_err(|e| CuError::from(format!("ZenohBridge: receive failed: {e}")))?;
        if let Some(sample) = sample {
            let payload = sample.payload().to_bytes();
            let decoded = Self::decode_message(rx_channel.wire_format, payload.as_ref())?;
            *msg = decoded;
        } else {
            msg.clear_payload();
        }
        Ok(())
    }

    fn stop(&mut self, _clock: &RobotClock) -> CuResult<()> {
        if let Some(ZenohContext {
            session,
            tx_channels,
            rx_channels,
        }) = self.ctx.take()
        {
            for channel in tx_channels {
                zenoh::Wait::wait(channel.publisher.undeclare())
                    .map_err(cu_error_map("ZenohBridge: Failed to undeclare publisher"))?;
            }
            for channel in rx_channels {
                zenoh::Wait::wait(channel.subscriber.undeclare())
                    .map_err(cu_error_map("ZenohBridge: Failed to undeclare subscriber"))?;
            }
            zenoh::Wait::wait(session.close())
                .map_err(cu_error_map("ZenohBridge: Failed to close session"))?;
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
