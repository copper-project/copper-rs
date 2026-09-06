#![cfg(feature = "logstream")]

use bincode::{Decode, Encode};
use cu29::logstream::{
    CuStreamTx, CuStreamTxError, FecScheme, FiniteObjectLimits, SessionEvent, SessionRouter,
    SessionRouterLimits, WirePacket,
};
use cu29::prelude::*;
use serde::{Deserialize, Serialize};
use std::sync::{Arc, OnceLock};

#[derive(Clone, Debug, Default, PartialEq, Eq, Encode, Decode, Serialize, Deserialize, Reflect)]
struct ConfiguredStreamMsg(u64);

#[derive(Default, Reflect)]
struct ConfiguredStreamSource {
    next: u64,
}

impl Freezable for ConfiguredStreamSource {}

impl CuSrcTask for ConfiguredStreamSource {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(ConfiguredStreamMsg);

    fn new(_config: Option<&ComponentConfig>, _resources: ()) -> CuResult<Self> {
        Ok(Self::default())
    }

    fn process(&mut self, _ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        output.set_payload(ConfiguredStreamMsg(self.next));
        self.next += 1;
        Ok(())
    }
}

const MAX_PACKET_BYTES: usize = 1_200;
const PACKET_COUNT: usize = 128;
type CapturedPacket = heapless::Vec<u8, MAX_PACKET_BYTES>;
type CapturedPackets = heapless::mpmc::Queue<CapturedPacket, PACKET_COUNT>;

static CONFIGURED_PACKETS: OnceLock<Arc<CapturedPackets>> = OnceLock::new();

fn configured_packets() -> Arc<CapturedPackets> {
    CONFIGURED_PACKETS
        .get_or_init(|| Arc::new(CapturedPackets::default()))
        .clone()
}

#[derive(Clone)]
struct ConfiguredTx {
    packets: Arc<CapturedPackets>,
}

impl core::fmt::Debug for ConfiguredTx {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        formatter.write_str("ConfiguredTx")
    }
}

impl CuStreamTx for ConfiguredTx {
    fn try_send(&mut self, packet: &[u8]) -> Result<(), CuStreamTxError> {
        let mut captured = CapturedPacket::new();
        captured
            .extend_from_slice(packet)
            .map_err(|_| CuStreamTxError::Failed("configured test packet exceeds capacity"))?;
        self.packets
            .enqueue(captured)
            .map_err(|_| CuStreamTxError::WouldBlock)
    }
}

struct ConfiguredStreamResources;

bundle_resources!(ConfiguredStreamResources: Tx);

impl ResourceBundle for ConfiguredStreamResources {
    fn build(
        bundle: BundleContext<Self>,
        _config: Option<&ComponentConfig>,
        manager: &mut ResourceManager,
    ) -> CuResult<()> {
        manager.add_owned(
            bundle.key(ConfiguredStreamResourcesId::Tx),
            ConfiguredTx {
                packets: configured_packets(),
            },
        )
    }
}

#[copper_runtime(config = "tests/logstream_configured_runtime.ron")]
struct ConfiguredLogstreamApp {}

#[test]
fn generated_runtime_binds_configured_transport_and_emits_manifest() -> CuResult<()> {
    const ITERATIONS: usize = 4;
    let packets = configured_packets();
    while packets.dequeue().is_some() {}

    let app = ConfiguredLogstreamApp::builder()
        .with_instance_id(41)
        .build()?;
    let mut running = app.start()?;
    for _ in 0..ITERATIONS {
        running.run_one_iteration()?;
    }
    drop(running.stop()?);

    let mut control_packets = Vec::new();
    let mut continuous_packets = Vec::new();
    while let Some(packet) = packets.dequeue() {
        let bytes = packet.as_slice().to_vec();
        let decoded =
            WirePacket::decode(&bytes).map_err(|error| CuError::from(error.to_string()))?;
        if decoded.header.fec_scheme == FecScheme::RaptorQ {
            control_packets.push(bytes);
        } else {
            continuous_packets.push(bytes);
        }
    }
    assert!(!control_packets.is_empty());
    assert!(!continuous_packets.is_empty());

    let mut router = SessionRouter::<1128, 64, 64>::new(SessionRouterLimits {
        max_startup_packets: 64,
        max_recovery_records: 8,
        max_sessions: 1,
        max_pending_events: 32,
        max_record_bytes: 65_536,
        max_buffered_records: 64,
        equation_capacity: 64,
        finite_objects: FiniteObjectLimits::new(4_194_304, 1128, 4),
    })
    .map_err(|error| CuError::from(error.to_string()))?;
    let mut events = Vec::new();
    for packet in control_packets.into_iter().chain(continuous_packets) {
        router
            .receive_datagram(&packet, |event| {
                events.push(event.to_owned());
                Ok::<(), core::convert::Infallible>(())
            })
            .map_err(|error| CuError::from(format!("{error:?}")))?;
    }

    let manifest = events
        .iter()
        .find_map(|event| match event {
            SessionEvent::Manifest(manifest) => Some(manifest),
            _ => None,
        })
        .expect("configured sender must emit a decodable manifest");
    assert_eq!(manifest.identity.sender_id, 41);
    assert_eq!(manifest.plan.destination_id, "ground");
    assert_eq!(manifest.plan.symbol_size, 1128);
    assert_eq!(manifest.application_schema.outputs.len(), 1);
    assert_eq!(
        manifest.application_schema.outputs[0].payload_type,
        core::any::type_name::<ConfiguredStreamMsg>()
    );
    assert_eq!(
        events
            .iter()
            .filter(|event| matches!(event, SessionEvent::ContinuousRecord { .. }))
            .count(),
        ITERATIONS
    );
    Ok(())
}
