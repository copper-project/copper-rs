//! `no_std` Zenoh bridge support.
//!
//! The bridge is deliberately synchronous from Copper's perspective.  The
//! resource supplied to it owns the Embassy/Zenoh task and exposes bounded,
//! nonblocking frame operations; therefore Copper never awaits UART I/O.

use alloc::{format, string::String, sync::Arc, vec::Vec};
use core::fmt::Debug;

use cu29::cubridge::{BridgeChannel, BridgeChannelConfig, BridgeChannelSet, CuBridge};
use cu29::prelude::*;
use spin::Mutex;

/// Re-export the in-tree Zenoh protocol fork used by the embedded runtime
/// resource implementation.
pub use zenoh_nostd as protocol;

const ATTACHMENT_BYTES: usize = 32;

/// The outcome of a nonblocking receive operation supplied by the embedded
/// Zenoh runtime resource.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct ZenohNostdSample {
    pub payload_len: usize,
    pub attachment_len: usize,
}

/// Queue-facing half of a `zenoh-nostd` session.
///
/// Implement this in the board layer that owns the Embassy task, UART and
/// session.  `try_publish` and `try_receive` must be bounded and must not
/// allocate or wait for I/O.  Returning `Ok(false)` from `try_publish` records
/// a dropped newest frame and preserves Copper's real-time schedule.
pub trait ZenohNostdRuntime: Send + Sync {
    fn try_publish(
        &mut self,
        route: &'static str,
        payload: &[u8],
        attachment: &[u8],
    ) -> CuResult<bool>;

    fn try_receive(
        &mut self,
        route: &'static str,
        payload: &mut [u8],
        attachment: &mut [u8],
    ) -> CuResult<Option<ZenohNostdSample>>;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, bincode::Encode, bincode::Decode)]
struct CopperBridgeAttachment {
    subsystem_code: u16,
    instance_id: u32,
    cl_id: u64,
}

#[derive(Clone, Copy)]
struct Channel<Id: Copy> {
    id: Id,
    route: &'static str,
}

/// Zenoh bridge backed by a no-std runtime resource.
///
/// `FRAME_BYTES` is a compile-time bound for both an encoded Copper message
/// and an incoming Zenoh payload.  Existing two-parameter aliases retain the
/// 1024-byte default.
#[derive(Reflect)]
#[reflect(from_reflect = false, no_field_bounds, type_path = false)]
pub struct ZenohBridge<Tx, Rx, const FRAME_BYTES: usize = 1024>
where
    Tx: BridgeChannelSet + 'static,
    Rx: BridgeChannelSet + 'static,
{
    #[reflect(ignore)]
    runtime: Arc<Mutex<dyn ZenohNostdRuntime>>,
    #[reflect(ignore)]
    tx_channels: Vec<Channel<Tx::Id>>,
    #[reflect(ignore)]
    rx_channels: Vec<Channel<Rx::Id>>,
    #[reflect(ignore)]
    dropped_newest: u64,
}

impl<Tx, Rx, const FRAME_BYTES: usize> Freezable for ZenohBridge<Tx, Rx, FRAME_BYTES>
where
    Tx: BridgeChannelSet + 'static,
    Rx: BridgeChannelSet + 'static,
{
}

impl<Tx, Rx, const FRAME_BYTES: usize> cu29::reflect::TypePath for ZenohBridge<Tx, Rx, FRAME_BYTES>
where
    Tx: BridgeChannelSet + 'static,
    Rx: BridgeChannelSet + 'static,
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

impl<Tx, Rx, const FRAME_BYTES: usize> ZenohBridge<Tx, Rx, FRAME_BYTES>
where
    Tx: BridgeChannelSet + 'static,
    Rx: BridgeChannelSet + 'static,
    Tx::Id: Copy + Eq,
    Rx::Id: Copy + Eq,
{
    /// Number of outgoing frames dropped because the runtime queue was full.
    pub const fn dropped_newest(&self) -> u64 {
        self.dropped_newest
    }

    fn channels<Id: Copy + Debug>(
        configs: &[BridgeChannelConfig<Id>],
    ) -> CuResult<Vec<Channel<Id>>> {
        let mut result = Vec::with_capacity(configs.len());
        for config in configs {
            if config.route.is_some() {
                return Err(CuError::from(
                    "ZenohBridge(no_std): channel route overrides are unsupported; declare a static route in tx_channels!/rx_channels!",
                ));
            }
            if let Some(channel_config) = config.config.as_ref()
                && channel_config.get::<String>("wire_format")?.is_some()
            {
                return Err(CuError::from(
                    "ZenohBridge(no_std): bincode is the only supported wire format",
                ));
            }
            let route = config.channel.default_route.ok_or_else(|| {
                CuError::from(format!(
                    "ZenohBridge(no_std): missing static route for channel {:?}",
                    config.channel.id
                ))
            })?;
            result.push(Channel {
                id: config.channel.id,
                route,
            });
        }
        Ok(result)
    }

    fn find<Id: Copy + Eq>(channels: &[Channel<Id>], id: Id) -> Option<&Channel<Id>> {
        channels.iter().find(|channel| channel.id == id)
    }

    fn encode_attachment(ctx: &CuContext, out: &mut [u8]) -> CuResult<usize> {
        bincode::encode_into_slice(
            CopperBridgeAttachment {
                subsystem_code: ctx.subsystem_code(),
                instance_id: ctx.instance_id(),
                cl_id: ctx.cl_id(),
            },
            out,
            bincode::config::standard(),
        )
        .map_err(|e| {
            CuError::from(format!(
                "ZenohBridge(no_std): attachment encode failed: {e}"
            ))
        })
    }
}

impl<Tx, Rx, const FRAME_BYTES: usize> CuBridge for ZenohBridge<Tx, Rx, FRAME_BYTES>
where
    Tx: BridgeChannelSet + 'static,
    Rx: BridgeChannelSet + 'static,
    Tx::Id: Copy + Eq + Debug + Send + Sync + 'static,
    Rx::Id: Copy + Eq + Debug + Send + Sync + 'static,
{
    type Tx = Tx;
    type Rx = Rx;
    type Resources<'r> = Arc<Mutex<dyn ZenohNostdRuntime>>;

    fn new(
        config: Option<&ComponentConfig>,
        tx_channels: &[BridgeChannelConfig<Tx::Id>],
        rx_channels: &[BridgeChannelConfig<Rx::Id>],
        runtime: Self::Resources<'_>,
    ) -> CuResult<Self> {
        if FRAME_BYTES == 0 {
            return Err(CuError::from(
                "ZenohBridge(no_std): FRAME_BYTES must be non-zero",
            ));
        }
        if let Some(config) = config {
            for key in ["zenoh_config_file", "zenoh_config_json", "wire_format"] {
                if config.get::<String>(key)?.is_some() {
                    return Err(CuError::from(format!(
                        "ZenohBridge(no_std): bridge config '{key}' is unsupported"
                    )));
                }
            }
        }
        Ok(Self {
            runtime,
            tx_channels: Self::channels(tx_channels)?,
            rx_channels: Self::channels(rx_channels)?,
            dropped_newest: 0,
        })
    }

    fn send<'a, Payload>(
        &mut self,
        ctx: &CuContext,
        channel: &'static BridgeChannel<Tx::Id, Payload>,
        msg: &CuMsg<Payload>,
    ) -> CuResult<()>
    where
        Payload: CuMsgPayload + 'a,
    {
        let route = Self::find(&self.tx_channels, channel.id())
            .ok_or_else(|| CuError::from("ZenohBridge(no_std): unknown Tx channel"))?
            .route;
        let mut payload = [0_u8; FRAME_BYTES];
        let payload_len =
            bincode::encode_into_slice(msg, &mut payload, bincode::config::standard()).map_err(
                |e| CuError::from(format!("ZenohBridge(no_std): bincode encode failed: {e}")),
            )?;
        let mut attachment = [0_u8; ATTACHMENT_BYTES];
        let attachment_len = Self::encode_attachment(ctx, &mut attachment)?;
        if !self.runtime.lock().try_publish(
            route,
            &payload[..payload_len],
            &attachment[..attachment_len],
        )? {
            self.dropped_newest = self.dropped_newest.saturating_add(1);
        }
        Ok(())
    }

    fn receive<'a, Payload>(
        &mut self,
        ctx: &CuContext,
        channel: &'static BridgeChannel<Rx::Id, Payload>,
        msg: &mut CuMsg<Payload>,
    ) -> CuResult<()>
    where
        Payload: CuMsgPayload + 'a,
    {
        let route = Self::find(&self.rx_channels, channel.id())
            .ok_or_else(|| CuError::from("ZenohBridge(no_std): unknown Rx channel"))?
            .route;
        let mut payload = [0_u8; FRAME_BYTES];
        let mut attachment = [0_u8; ATTACHMENT_BYTES];
        let Some(sample) = self
            .runtime
            .lock()
            .try_receive(route, &mut payload, &mut attachment)?
        else {
            msg.clear_payload();
            msg.metadata.clear_origin();
            msg.tov = Tov::Time(ctx.now());
            return Ok(());
        };
        if sample.payload_len > FRAME_BYTES || sample.attachment_len > ATTACHMENT_BYTES {
            return Err(CuError::from(
                "ZenohBridge(no_std): runtime returned an invalid frame length",
            ));
        }
        let (decoded, _): (CuMsg<Payload>, usize) =
            bincode::decode_from_slice(&payload[..sample.payload_len], bincode::config::standard())
                .map_err(|e| {
                    CuError::from(format!("ZenohBridge(no_std): bincode decode failed: {e}"))
                })?;
        *msg = decoded;
        if sample.attachment_len == 0 {
            msg.metadata.clear_origin();
        } else {
            let (origin, _): (CopperBridgeAttachment, usize) = bincode::decode_from_slice(
                &attachment[..sample.attachment_len],
                bincode::config::standard(),
            )
            .map_err(|e| {
                CuError::from(format!(
                    "ZenohBridge(no_std): attachment decode failed: {e}"
                ))
            })?;
            msg.metadata.set_origin(CuMsgOrigin {
                subsystem_code: origin.subsystem_code,
                instance_id: origin.instance_id,
                cl_id: origin.cl_id,
            });
        }
        Ok(())
    }
}
