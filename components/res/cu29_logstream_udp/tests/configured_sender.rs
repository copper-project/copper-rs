use bincode::{Decode, Encode};
use cu29::prelude::*;
use cu29::resource::{BundleContext, BundleIndex, ResourceBundle, ResourceManager};
use cu29_logstream::{
    CuStreamRx, FiniteObjectLimits, RecordKind, SessionEvent, SessionRouter, SessionRouterLimits,
    decode_copperlist,
};
use cu29_logstream_udp::{CuUdpLogStreamResources, CuUdpLogStreamResourcesId, CuUdpLogStreamRx};
use serde::{Deserialize, Serialize};
use std::time::{Duration, Instant};

#[derive(Clone, Debug, Default, PartialEq, Eq, Encode, Decode, Serialize, Deserialize, Reflect)]
struct UdpMessage(u64);

#[derive(Default, Reflect)]
struct UdpSource {
    next: u64,
}

impl Freezable for UdpSource {}

impl CuSrcTask for UdpSource {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(UdpMessage);

    fn new(_config: Option<&ComponentConfig>, _resources: ()) -> CuResult<Self> {
        Ok(Self::default())
    }

    fn process(&mut self, _ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        output.set_payload(UdpMessage(self.next));
        self.next += 1;
        Ok(())
    }
}

#[copper_runtime(config = "tests/configured_sender.ron")]
struct UdpApp {}

#[test]
fn configured_runtime_bootstraps_over_udp_in_actual_arrival_order() -> CuResult<()> {
    // Construct the receiver from the same public RON resource contract, before
    // starting the sender. Port zero keeps parallel tests independent.
    let receiver_config = CuConfig::deserialize_ron(
        r#"(
        resources: [(
            id: "listen",
            provider: "cu29_logstream_udp::CuUdpLogStreamResources",
            config: {"bind_addr": "127.0.0.1:0", "recv_buffer_bytes": 262144},
        )],
        tasks: [], cnx: [],
    )"#,
    )?;
    let mut resources = ResourceManager::new(&[2]);
    CuUdpLogStreamResources::build(
        BundleContext::new(BundleIndex::new(0), "listen"),
        receiver_config.resources[0].config.as_ref(),
        &mut resources,
    )?;
    let context = BundleContext::<CuUdpLogStreamResources>::new(BundleIndex::new(0), "listen");
    let mut rx: CuUdpLogStreamRx = resources
        .take(context.key(CuUdpLogStreamResourcesId::Rx))?
        .0;
    let mut sender_config = CuConfig::deserialize_ron(include_str!("configured_sender.ron"))?;
    sender_config.resources[0]
        .config
        .as_mut()
        .unwrap()
        .set("remote_addr", rx.local_addr().unwrap().to_string());

    // The receiver knows hard capacity limits, but no sender identity or FEC plan.
    let mut router = SessionRouter::<1128, 64, 64>::new(SessionRouterLimits {
        max_sessions: 1,
        // A startup gap can release a full window of records plus the gap event.
        max_pending_events: 65,
        max_record_bytes: 4096,
        max_buffered_records: 64,
        equation_capacity: 64,
        finite_objects: FiniteObjectLimits::new(65536, 1128, 4),
    })
    .unwrap();
    let mut ids = Vec::new();
    let mut manifest_seen = false;
    let mut keyframe_seen = false;
    let mut anchor_seen = false;
    let mut gaps = Vec::new();
    let mut emit = |event| {
        match event {
            SessionEvent::Manifest(manifest) => {
                assert_eq!(manifest.identity.sender_id, 41);
                assert_eq!(manifest.plan.destination_id, "ground");
                assert_eq!(manifest.plan.symbol_size, 1128);
                assert_eq!(manifest.application_schema.outputs.len(), 1);
                assert_eq!(
                    manifest.application_schema.outputs[0].payload_type,
                    core::any::type_name::<UdpMessage>()
                );
                manifest_seen = true;
            }
            SessionEvent::ContinuousRecord { identity, record } => {
                assert!(manifest_seen);
                assert_eq!(identity.sender_id, 41);
                let record = record.decoded().unwrap();
                let copperlist: default::CuList = decode_copperlist(record.payload).unwrap();
                assert_eq!(record.object_id, copperlist.id);
                assert_eq!(
                    copperlist.msgs.0.0.payload(),
                    Some(&UdpMessage(copperlist.id))
                );
                assert!(ids.last().is_none_or(|&id| copperlist.id > id));
                ids.push(copperlist.id);
            }
            SessionEvent::Object { record, .. } => match record.decoded().unwrap().kind {
                RecordKind::KeyFrame => keyframe_seen = true,
                RecordKind::Anchor => anchor_seen = true,
                _ => {}
            },
            SessionEvent::Gap { gap, .. } => gaps.push(gap),
        }
        Ok::<(), core::convert::Infallible>(())
    };

    let logs = tempfile::tempdir().unwrap();
    let app = UdpApp::builder()
        .with_instance_id(41)
        .with_config(sender_config)
        .with_log_path(logs.path().join("sender.copper"), Some(1024 * 1024))?
        .build()?;
    let mut running = app.start()?;
    let mut packet = [0; 1200];
    const ITERATIONS: u64 = 128;
    for id in 0..ITERATIONS {
        running.run_one_iteration()?;
        // Drive the clean fixture by observable wire progress, not assumptions
        // about debug-build FEC speed or an unenforced link budget. This wait is
        // entirely in the test harness; the production sender remains one-way.
        let deadline = Instant::now() + Duration::from_secs(2);
        let mut source_received = false;
        while !source_received && Instant::now() < deadline {
            if let Some(len) = rx.try_recv(&mut packet).unwrap() {
                let header = cu29_logstream::WirePacket::decode(&packet[..len])
                    .unwrap()
                    .header;
                source_received = header.record_kind == RecordKind::CopperList
                    && header.symbol_kind == cu29_logstream::FecSymbolKind::Source
                    && header.object_id == id;
                router.receive_datagram(&packet[..len], &mut emit).unwrap();
            } else {
                std::thread::sleep(Duration::from_millis(1));
            }
        }
        assert!(source_received, "sender did not transmit CopperList {id}");
    }
    drop(running.stop()?);
    let deadline = Instant::now() + Duration::from_millis(100);
    while Instant::now() < deadline {
        if let Some(len) = rx.try_recv(&mut packet).unwrap() {
            router.receive_datagram(&packet[..len], &mut emit).unwrap();
        } else {
            std::thread::sleep(Duration::from_millis(1));
        }
    }

    assert!(manifest_seen && keyframe_seen && anchor_seen);
    assert_eq!(router.stats().sessions_discovered, 1);
    assert_eq!(router.stats().malformed_datagrams, 0);
    // Startup data can precede the manifest and be discarded by today's router.
    // Full history and automatic anchor resume are the next PR's contract. Here
    // require a substantial ordered, correctly typed suffix without sorting traffic.
    assert!(
        ids.len() >= 32,
        "too few recovered records: {}; gaps: {gaps:?}",
        ids.len()
    );
    assert_eq!(ids.last(), Some(&(ITERATIONS - 1)));
    Ok(())
}
