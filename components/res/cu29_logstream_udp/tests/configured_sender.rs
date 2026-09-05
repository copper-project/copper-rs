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
        max_startup_packets: 64,
        max_recovery_records: 8,
        max_sessions: 1,
        // A startup gap can release a full window of records plus the gap event.
        max_pending_events: 65,
        max_record_bytes: 4096,
        max_buffered_records: 64,
        equation_capacity: 64,
        finite_objects: FiniteObjectLimits::new(65536, 1128, 4),
    })
    .unwrap();
    let logs = tempfile::tempdir().unwrap();
    let archive_path = logs.path().join("received.copper");
    let expected_schema = cu29_logstream::ApplicationSchema::from_output_specs(
        default::CuStampedDataSet::get_output_specs(),
    );
    let mut archive: Option<cu29_logstream::NativeArchive<default::CuStampedDataSet>> = None;
    let mut expected_payloads = Vec::new();
    let mut verified_anchor_seen = false;
    let mut ids = Vec::new();
    let mut manifest_seen = false;
    let mut keyframe_seen = false;
    let mut anchor_seen = false;
    let mut gaps = Vec::new();
    let mut emit = |event| {
        if let SessionEvent::Manifest(manifest) = &event {
            let mut wrong_schema = manifest.clone();
            wrong_schema.application_schema.outputs[0]
                .payload_type
                .push_str("::Wrong");
            let rejected_path = logs.path().join("rejected.copper");
            assert!(
                cu29_logstream::NativeArchive::<default::CuStampedDataSet>::new(
                    &rejected_path,
                    wrong_schema,
                    1024 * 1024,
                    4096
                )
                .is_err()
            );
            assert!(!rejected_path.exists());
            archive = Some(
                cu29_logstream::NativeArchive::new(
                    &archive_path,
                    manifest.clone(),
                    1024 * 1024,
                    4096,
                )
                .unwrap(),
            );
        }
        if let Some(writer) = archive.as_mut() {
            writer.accept(&event).unwrap();
        }
        match event {
            SessionEvent::Manifest(manifest) => {
                assert_eq!(manifest.identity.sender_id, 41);
                assert_eq!(manifest.plan.destination_id, "ground");
                assert_eq!(manifest.plan.symbol_size, 1128);
                assert_eq!(manifest.application_schema, expected_schema);
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
                expected_payloads.push(record.payload.to_vec());
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
            SessionEvent::VerifiedAnchor { .. } => verified_anchor_seen = true,
        }
        Ok::<(), core::convert::Infallible>(())
    };

    let app = UdpApp::builder()
        .with_instance_id(41)
        .with_config(sender_config)
        .with_log_path(logs.path().join("sender.copper"), Some(1024 * 1024))?
        .build()?;
    let mut running = app.start()?;
    let mut packet = [0; 1200];
    let mut traffic = Vec::new();
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
                traffic.push(packet[..len].to_vec());
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
            traffic.push(packet[..len].to_vec());
            router.receive_datagram(&packet[..len], &mut emit).unwrap();
        } else {
            std::thread::sleep(Duration::from_millis(1));
        }
    }

    assert!(manifest_seen && keyframe_seen && anchor_seen);
    assert_eq!(router.stats().sessions_discovered, 1);
    assert_eq!(router.stats().malformed_datagrams, 0);
    assert!(verified_anchor_seen);
    assert!(gaps.is_empty(), "unexpected clean-link gaps: {gaps:?}");
    assert_eq!(ids, (0..ITERATIONS).collect::<Vec<_>>());
    archive.take().unwrap().finish().unwrap();
    let reader = UnifiedLoggerRead::new(&archive_path).unwrap();
    let mut reader = UnifiedLoggerIOReader::new(reader, UnifiedLogType::CopperList);
    let received: Vec<_> =
        cu29_export::copperlists_reader::<default::CuStampedDataSet>(&mut reader).collect();
    assert_eq!(received.len(), ITERATIONS as usize);
    for (record, expected) in received.iter().zip(&expected_payloads) {
        assert_eq!(
            &bincode::encode_to_vec(record, bincode::config::standard()).unwrap(),
            expected
        );
    }
    let reader = UnifiedLoggerRead::new(&archive_path).unwrap();
    let reader = UnifiedLoggerIOReader::new(reader, UnifiedLogType::FrozenTasks);
    assert!(cu29_export::keyframes_reader(reader).next().is_some());
    let reader = UnifiedLoggerRead::new(&archive_path).unwrap();
    let reader = UnifiedLoggerIOReader::new(reader, UnifiedLogType::StreamContinuity);
    let continuity: Vec<_> = cu29_export::stream_continuity_reader(reader).collect();
    assert!(matches!(
        continuity.first(),
        Some(cu29::continuity::StreamContinuityRecord::Manifest { .. })
    ));
    assert!(matches!(
        continuity.last(),
        Some(cu29::continuity::StreamContinuityRecord::Finished {
            next_copperlist_id: ITERATIONS
        })
    ));
    let boundary = |id| {
        traffic
            .iter()
            .position(|packet| {
                let header = cu29_logstream::WirePacket::decode(packet).unwrap().header;
                header.record_kind == RecordKind::CopperList && header.object_id == id
            })
            .unwrap()
    };
    let late = boundary(64);
    validate_impaired_archive(
        &logs.path().join("late.copper"),
        &traffic[late..],
        &expected_payloads,
        true,
    );
    let outage_start = boundary(16);
    let outage_end = boundary(96);
    let outage: Vec<_> = traffic[..outage_start]
        .iter()
        .chain(&traffic[outage_end..])
        .cloned()
        .collect();
    validate_impaired_archive(
        &logs.path().join("outage.copper"),
        &outage,
        &expected_payloads,
        false,
    );
    Ok(())
}

// Reuse actual UDP arrival order; only remove a prefix or one contiguous outage.
fn validate_impaired_archive(
    path: &std::path::Path,
    packets: &[Vec<u8>],
    expected: &[Vec<u8>],
    late: bool,
) {
    let mut router = SessionRouter::<1128, 64, 64>::new(SessionRouterLimits {
        max_startup_packets: 64,
        max_recovery_records: 8,
        max_sessions: 1,
        max_pending_events: 8,
        max_record_bytes: 4096,
        max_buffered_records: 64,
        equation_capacity: 64,
        finite_objects: FiniteObjectLimits::new(65536, 1128, 4),
    })
    .unwrap();
    let mut archive: Option<cu29_logstream::NativeArchive<default::CuStampedDataSet>> = None;
    for packet in packets {
        router
            .receive_datagram(packet, |event| {
                if let SessionEvent::Manifest(manifest) = &event {
                    archive = Some(
                        cu29_logstream::NativeArchive::new(
                            path,
                            manifest.clone(),
                            1024 * 1024,
                            4096,
                        )
                        .unwrap(),
                    );
                }
                if let Some(archive) = archive.as_mut() {
                    archive.accept(&event).unwrap();
                }
                Ok::<(), ()>(())
            })
            .unwrap();
    }
    archive.unwrap().finish().unwrap();
    let lists: Vec<_> =
        cu29_export::copperlists_reader::<default::CuStampedDataSet>(UnifiedLoggerIOReader::new(
            UnifiedLoggerRead::new(path).unwrap(),
            UnifiedLogType::CopperList,
        ))
        .collect();
    assert!(!lists.is_empty());
    assert_eq!(lists.last().unwrap().id, 127);
    for record in &lists {
        assert_eq!(
            bincode::encode_to_vec(record, bincode::config::standard()).unwrap(),
            expected[record.id as usize]
        );
    }
    let continuity: Vec<_> = cu29_export::stream_continuity_reader(UnifiedLoggerIOReader::new(
        UnifiedLoggerRead::new(path).unwrap(),
        UnifiedLogType::StreamContinuity,
    ))
    .collect();
    assert!(continuity.iter().any(|entry| matches!(entry,
        cu29::continuity::StreamContinuityRecord::Gap { first_id, reason, .. }
        if if late { *first_id == 0 && *reason == cu29::continuity::SourceGapReason::LateJoin } else { *first_id > 0 })));
    let keyframes: Vec<_> = cu29_export::keyframes_reader(UnifiedLoggerIOReader::new(
        UnifiedLoggerRead::new(path).unwrap(),
        UnifiedLogType::FrozenTasks,
    ))
    .collect();
    assert!(keyframes.iter().any(|frame| frame.culistid >= 64));
}
