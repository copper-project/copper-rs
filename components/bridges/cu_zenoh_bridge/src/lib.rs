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

/// Per-channel Rx queue strategy, configured via `queue_mode` in the channel config.
///
/// - `"fifo"` (default): zenoh's default handler. Ordered, lossless, and bounded at
///   `API_DATA_RECEPTION_CHANNEL_SIZE` (256) — but a bridge consumes at most ONE sample per
///   `receive` call, so a channel whose publisher is faster than the consuming graph's rate
///   accumulates an unbounded backlog and the consumer reads ever-older samples.
/// - `"ring"`: drops the OLDEST sample when full, so `receive` always yields the newest one
///   available. `ring_size` sets the depth (default 1, i.e. latest-wins).
///
/// Measured on a loopback link, 724 B at 200 Hz against a consumer draining 100/s: with `fifo`
/// the consumer received sequence numbers 0..2490 contiguously while the publisher was at 5908 —
/// 12 s behind and growing linearly, with nothing dropped and no error on either side. The
/// publisher was NOT slowed (197 Hz sustained), so this is a staleness problem rather than a
/// back-pressure one, and it is invisible to a consumer that only checks whether samples arrive.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum RxQueueConfig {
    Fifo,
    Ring { size: usize },
}

impl RxQueueConfig {
    fn from_config(config: Option<&ComponentConfig>) -> CuResult<Self> {
        let Some(cfg) = config else {
            return Ok(Self::Fifo);
        };
        match cfg.get::<String>("queue_mode")?.as_deref() {
            Some("ring") => {
                // Zero would be rejected by `RingChannel::new` at declare time, which is a
                // confusing place for a config error to surface.
                let size = cfg.get::<u32>("ring_size")?.unwrap_or(1) as usize;
                if size == 0 {
                    return Err(CuError::from(
                        "ZenohBridge: ring_size must be at least 1".to_string(),
                    ));
                }
                Ok(Self::Ring { size })
            }
            Some("fifo") | None => Ok(Self::Fifo),
            Some(other) => Err(CuError::from(format!(
                "ZenohBridge: unknown queue_mode '{other}', expected fifo/ring"
            ))),
        }
    }
}

#[derive(Debug, Clone)]
struct ZenohChannelConfig<Id: Copy> {
    id: Id,
    route: String,
    wire_format: WireFormat,
    /// Meaningful on Rx only; `new` rejects `queue_mode` on a Tx channel rather than ignoring it.
    queue: RxQueueConfig,
}

/// A subscriber under either handler. The two are distinct types, so the choice cannot be made
/// behind a `dyn` and the enum is what carries it to `receive`.
enum ZenohSubscriber {
    Fifo(zenoh::pubsub::Subscriber<zenoh::handlers::FifoChannelHandler<zenoh::sample::Sample>>),
    Ring(zenoh::pubsub::Subscriber<zenoh::handlers::RingChannelHandler<zenoh::sample::Sample>>),
}

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
    local_subsystem_code: u16,
    local_instance_id: u32,
    tx_channels: Vec<ZenohTxChannel<TxId>>,
    rx_channels: Vec<ZenohRxChannel<RxId>>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, bincode::Encode, bincode::Decode)]
struct CopperBridgeAttachment {
    subsystem_code: u16,
    instance_id: u32,
    cl_id: u64,
}

#[derive(Reflect)]
#[reflect(from_reflect = false, no_field_bounds, type_path = false)]
pub struct ZenohBridge<Tx, Rx>
where
    Tx: BridgeChannelSet + 'static,
    Rx: BridgeChannelSet + 'static,
    Tx::Id: Send + Sync + 'static,
    Rx::Id: Send + Sync + 'static,
{
    #[reflect(ignore)]
    session_config: Config,
    #[reflect(ignore)]
    tx_channels: Vec<ZenohChannelConfig<Tx::Id>>,
    #[reflect(ignore)]
    rx_channels: Vec<ZenohChannelConfig<Rx::Id>>,
    #[reflect(ignore)]
    ctx: Option<ZenohContext<Tx::Id, Rx::Id>>,
}

impl<Tx, Rx> Freezable for ZenohBridge<Tx, Rx>
where
    Tx: BridgeChannelSet + 'static,
    Rx: BridgeChannelSet + 'static,
    Tx::Id: Send + Sync + 'static,
    Rx::Id: Send + Sync + 'static,
{
}

impl<Tx, Rx> cu29::reflect::TypePath for ZenohBridge<Tx, Rx>
where
    Tx: BridgeChannelSet + 'static,
    Rx: BridgeChannelSet + 'static,
    Tx::Id: Send + Sync + 'static,
    Rx::Id: Send + Sync + 'static,
{
    fn type_path() -> &'static str {
        "cu_zenoh_bridge::ZenohBridge"
    }

    fn short_type_path() -> &'static str {
        "ZenohBridge"
    }

    fn type_ident() -> Option<&'static str> {
        Some("ZenohBridge")
    }

    fn crate_name() -> Option<&'static str> {
        Some("cu_zenoh_bridge")
    }

    fn module_path() -> Option<&'static str> {
        Some("cu_zenoh_bridge")
    }
}

impl<Tx, Rx> ZenohBridge<Tx, Rx>
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
                    CuError::from(format!("ZenohBridge: Failed to read config file: {e}"))
                });
            }
            if let Some(json) = config.get::<String>("zenoh_config_json")? {
                return Config::from_json5(&json).map_err(|e| {
                    CuError::from(format!("ZenohBridge: Failed to parse config json: {e}"))
                });
            }
        }
        Ok(Config::default())
    }

    fn parse_default_wire_format(config: Option<&ComponentConfig>) -> CuResult<WireFormat> {
        if let Some(config) = config
            && let Some(raw) = config.get::<String>("wire_format")?
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
            && let Some(raw) = config.get::<String>("wire_format")?
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

    fn encode_attachment(ctx: &CuContext) -> CuResult<Vec<u8>> {
        bincode::encode_to_vec(
            CopperBridgeAttachment {
                subsystem_code: ctx.subsystem_code(),
                instance_id: ctx.instance_id(),
                cl_id: ctx.cl_id(),
            },
            bincode::config::standard(),
        )
        .map_err(|e| CuError::new_with_cause("ZenohBridge: attachment encode failed", e))
    }

    fn decode_attachment(sample: &zenoh::sample::Sample) -> CuResult<Option<CuMsgOrigin>> {
        let Some(attachment) = sample.attachment() else {
            return Ok(None);
        };
        let attachment_bytes = attachment.to_bytes();
        let (decoded, _): (CopperBridgeAttachment, usize) =
            bincode::decode_from_slice(attachment_bytes.as_ref(), bincode::config::standard())
                .map_err(|e| CuError::new_with_cause("ZenohBridge: attachment decode failed", e))?;
        Ok(Some(CuMsgOrigin {
            subsystem_code: decoded.subsystem_code,
            instance_id: decoded.instance_id,
            cl_id: decoded.cl_id,
        }))
    }
}

impl<Tx, Rx> CuBridge for ZenohBridge<Tx, Rx>
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
        let default_wire_format = Self::parse_default_wire_format(config)?;

        let mut tx_cfgs = Vec::with_capacity(tx_channels.len());
        for channel in tx_channels {
            let route = Self::channel_route(channel)?;
            let wire_format = Self::channel_wire_format(channel, default_wire_format)?;
            // REFUSED on Tx rather than ignored: `queue_mode` on the sending side reads like it
            // bounds the publisher and does nothing at all, which is the kind of config that
            // looks applied for months.
            if let Some(config) = channel.config.as_ref()
                && config.get::<String>("queue_mode")?.is_some()
            {
                return Err(CuError::from(
                    "ZenohBridge: queue_mode is an Rx-only setting, but it was set on a Tx channel"
                        .to_string(),
                ));
            }
            tx_cfgs.push(ZenohChannelConfig {
                id: channel.channel.id,
                route,
                wire_format,
                queue: RxQueueConfig::Fifo,
            });
        }

        let mut rx_cfgs = Vec::with_capacity(rx_channels.len());
        for channel in rx_channels {
            let route = Self::channel_route(channel)?;
            let wire_format = Self::channel_wire_format(channel, default_wire_format)?;
            let queue = RxQueueConfig::from_config(channel.config.as_ref())?;
            rx_cfgs.push(ZenohChannelConfig {
                id: channel.channel.id,
                route,
                wire_format,
                queue,
            });
        }

        Ok(Self {
            session_config,
            tx_channels: tx_cfgs,
            rx_channels: rx_cfgs,
            ctx: None,
        })
    }

    fn start(&mut self, ctx: &CuContext) -> CuResult<()> {
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
            let subscriber = match channel.queue {
                RxQueueConfig::Fifo => ZenohSubscriber::Fifo(
                    zenoh::Wait::wait(session.declare_subscriber(key_expr))
                        .map_err(cu_error_map("ZenohBridge: Failed to declare subscriber"))?,
                ),
                RxQueueConfig::Ring { size } => ZenohSubscriber::Ring(
                    zenoh::Wait::wait(
                        session
                            .declare_subscriber(key_expr)
                            .with(zenoh::handlers::RingChannel::new(size)),
                    )
                    .map_err(cu_error_map("ZenohBridge: Failed to declare subscriber"))?,
                ),
            };
            rx_channels.push(ZenohRxChannel {
                id: channel.id,
                subscriber,
                wire_format: channel.wire_format,
            });
        }

        self.ctx = Some(ZenohContext {
            session,
            local_subsystem_code: ctx.subsystem_code(),
            local_instance_id: ctx.instance_id(),
            tx_channels,
            rx_channels,
        });
        Ok(())
    }

    fn send<'a, Payload>(
        &mut self,
        ctx: &CuContext,
        channel: &'static BridgeChannel<<Self::Tx as BridgeChannelSet>::Id, Payload>,
        msg: &CuMsg<Payload>,
    ) -> CuResult<()>
    where
        Payload: CuMsgPayload + 'a,
    {
        let runtime_ctx = self
            .ctx
            .as_mut()
            .ok_or_else(|| CuError::from("ZenohBridge: Context not initialized"))?;
        debug_assert_eq!(runtime_ctx.local_subsystem_code, ctx.subsystem_code());
        debug_assert_eq!(runtime_ctx.local_instance_id, ctx.instance_id());
        let tx_channel = Self::find_tx_channel_mut(&mut runtime_ctx.tx_channels, channel.id())
            .ok_or_else(|| {
                CuError::from(format!(
                    "ZenohBridge: Unknown Tx channel {:?}",
                    channel.id()
                ))
            })?;

        let encoded = Self::encode_message(tx_channel.wire_format, msg)?;
        let attachment = Self::encode_attachment(ctx)?;
        zenoh::Wait::wait(
            tx_channel
                .publisher
                .put(encoded)
                .encoding(tx_channel.wire_format.encoding())
                .attachment(attachment),
        )
        .map_err(cu_error_map("ZenohBridge: Failed to publish"))?;
        Ok(())
    }

    fn receive<'a, Payload>(
        &mut self,
        ctx: &CuContext,
        channel: &'static BridgeChannel<<Self::Rx as BridgeChannelSet>::Id, Payload>,
        msg: &mut CuMsg<Payload>,
    ) -> CuResult<()>
    where
        Payload: CuMsgPayload + 'a,
    {
        let runtime_ctx = self
            .ctx
            .as_mut()
            .ok_or_else(|| CuError::from("ZenohBridge: Context not initialized"))?;
        debug_assert_eq!(runtime_ctx.local_subsystem_code, ctx.subsystem_code());
        debug_assert_eq!(runtime_ctx.local_instance_id, ctx.instance_id());
        let rx_channel = Self::find_rx_channel_mut(&mut runtime_ctx.rx_channels, channel.id())
            .ok_or_else(|| {
                CuError::from(format!(
                    "ZenohBridge: Unknown Rx channel {:?}",
                    channel.id()
                ))
            })?;

        msg.tov = Tov::Time(ctx.now());

        let sample = match &rx_channel.subscriber {
            ZenohSubscriber::Fifo(s) => s.try_recv(),
            ZenohSubscriber::Ring(s) => s.try_recv(),
        }
        .map_err(|e| CuError::from(format!("ZenohBridge: receive failed: {e}")))?;
        if let Some(sample) = sample {
            let origin = Self::decode_attachment(&sample)?;
            let payload = sample.payload().to_bytes();
            let decoded = Self::decode_message(rx_channel.wire_format, payload.as_ref())?;
            *msg = decoded;
            if let Some(origin) = origin {
                msg.metadata.set_origin(origin);
            } else {
                msg.metadata.clear_origin();
            }
        } else {
            msg.clear_payload();
            msg.metadata.clear_origin();
        }
        Ok(())
    }

    fn stop(&mut self, _ctx: &CuContext) -> CuResult<()> {
        if let Some(ZenohContext {
            session,
            tx_channels,
            rx_channels,
            ..
        }) = self.ctx.take()
        {
            for channel in tx_channels {
                zenoh::Wait::wait(channel.publisher.undeclare())
                    .map_err(cu_error_map("ZenohBridge: Failed to undeclare publisher"))?;
            }
            for channel in rx_channels {
                match channel.subscriber {
                    ZenohSubscriber::Fifo(s) => zenoh::Wait::wait(s.undeclare())
                        .map_err(cu_error_map("ZenohBridge: Failed to undeclare subscriber"))?,
                    ZenohSubscriber::Ring(s) => zenoh::Wait::wait(s.undeclare())
                        .map_err(cu_error_map("ZenohBridge: Failed to undeclare subscriber"))?,
                }
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

#[cfg(test)]
mod tests {
    use super::*;

    fn config_with(pairs: &[(&str, &str)]) -> ComponentConfig {
        let mut cfg = ComponentConfig::default();
        for (k, v) in pairs {
            cfg.set(k, v.to_string());
        }
        cfg
    }

    #[test]
    fn an_absent_config_is_fifo() {
        assert_eq!(RxQueueConfig::from_config(None).unwrap(), RxQueueConfig::Fifo);
    }

    #[test]
    fn a_config_without_queue_mode_is_fifo() {
        let cfg = config_with(&[("wire_format", "json")]);
        assert_eq!(
            RxQueueConfig::from_config(Some(&cfg)).unwrap(),
            RxQueueConfig::Fifo
        );
    }

    #[test]
    fn fifo_is_spelled_out_as_well_as_defaulted() {
        let cfg = config_with(&[("queue_mode", "fifo")]);
        assert_eq!(
            RxQueueConfig::from_config(Some(&cfg)).unwrap(),
            RxQueueConfig::Fifo
        );
    }

    /// The default depth is 1 — latest-wins — because that is what a sensor stream wants and
    /// because any larger default would reintroduce the staleness this option exists to remove.
    #[test]
    fn ring_defaults_to_a_depth_of_one() {
        let cfg = config_with(&[("queue_mode", "ring")]);
        assert_eq!(
            RxQueueConfig::from_config(Some(&cfg)).unwrap(),
            RxQueueConfig::Ring { size: 1 }
        );
    }

    #[test]
    fn ring_size_is_honoured() {
        let mut cfg = ComponentConfig::default();
        cfg.set("queue_mode", "ring".to_string());
        cfg.set("ring_size", 8u32);
        assert_eq!(
            RxQueueConfig::from_config(Some(&cfg)).unwrap(),
            RxQueueConfig::Ring { size: 8 }
        );
    }

    /// `RingChannel::new(0)` panics inside zenoh at declare time, which is a bewildering place
    /// for a typo in a RON to surface.
    #[test]
    fn a_zero_ring_size_is_refused_at_config_time() {
        let mut cfg = ComponentConfig::default();
        cfg.set("queue_mode", "ring".to_string());
        cfg.set("ring_size", 0u32);
        let err = RxQueueConfig::from_config(Some(&cfg)).unwrap_err();
        assert!(
            err.to_string().contains("ring_size"),
            "the error must name the key the operator got wrong, got: {err}"
        );
    }

    /// A misspelling must not silently fall back to `fifo`: that is the exact failure this
    /// option exists to make impossible, arriving through the config instead of the default.
    #[test]
    fn an_unknown_queue_mode_is_an_error_not_a_fallback() {
        let cfg = config_with(&[("queue_mode", "rng")]);
        let err = RxQueueConfig::from_config(Some(&cfg)).unwrap_err();
        assert!(
            err.to_string().contains("rng"),
            "the error must quote the value it rejected, got: {err}"
        );
    }
}
