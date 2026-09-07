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
    let router_limits = SessionRouterLimits {
        max_startup_packets: 64,
        max_recovery_records: 8,
        max_sessions: 1,
        // A startup gap can release a full window of records plus the gap event.
        max_pending_events: 65,
        max_record_bytes: 4096,
        max_buffered_records: 64,
        equation_capacity: 64,
        finite_objects: FiniteObjectLimits::new(65536, 1128, 4),
    };
    let mut router = SessionRouter::<1128, 64, 64>::new(router_limits).unwrap();
    let logs = tempfile::tempdir().unwrap();
    let archive_path = logs.path().join("received.copper");
    let expected_schema = cu29_logstream::ApplicationSchema::from_output_specs(
        default::CuStampedDataSet::get_output_specs(),
    );
    let mut archive: Option<cu29_logstream::NativeArchive<default::CuStampedDataSet>> = None;
    let mut expected_payloads = Vec::new();
    let mut verified_recovery_point_seen = false;
    let mut ids = Vec::new();
    let mut manifest_seen = false;
    let mut keyframe_seen = false;
    let mut recovery_point_seen = false;
    let mut gaps = Vec::new();
    let mut emit = |event: cu29_logstream::SessionEventRef<'_>| {
        if let SessionEvent::Manifest(manifest) = &event {
            let mut wrong_schema = manifest.manifest().clone();
            wrong_schema.application_schema.outputs[0]
                .payload_type
                .push_str("::Wrong");
            let rejected_path = logs.path().join("rejected.copper");
            assert!(
                cu29_logstream::NativeArchive::<default::CuStampedDataSet>::new(
                    &rejected_path,
                    &cu29_logstream::ReceivedManifest::decode_record(
                        wrong_schema.encode_record().unwrap()
                    )
                    .unwrap(),
                    1024 * 1024,
                    4096
                )
                .is_err()
            );
            assert!(!rejected_path.exists());
            archive = Some(
                cu29_logstream::NativeArchive::new(&archive_path, manifest, 1024 * 1024, 4096)
                    .unwrap(),
            );
        }
        if let Some(writer) = archive.as_mut() {
            writer.accept(&event).unwrap();
        }
        match event {
            SessionEvent::Manifest(manifest) => {
                assert_eq!(manifest.manifest().identity.sender_id, 41);
                assert_eq!(manifest.manifest().plan.destination_id, "ground");
                assert_eq!(manifest.manifest().plan.symbol_size, 1128);
                assert_eq!(manifest.manifest().application_schema, expected_schema);
                assert_eq!(manifest.manifest().application_schema.outputs.len(), 1);
                assert_eq!(
                    manifest.manifest().application_schema.outputs[0].payload_type,
                    core::any::type_name::<UdpMessage>()
                );
                manifest_seen = true;
            }
            SessionEvent::ContinuousRecord { identity, record } => {
                assert!(manifest_seen);
                assert_eq!(identity.sender_id, 41);
                let record = record.decoded();
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
            SessionEvent::Object { record, .. } => match record.decoded().kind {
                RecordKind::KeyFrame => keyframe_seen = true,
                RecordKind::RecoveryPoint => recovery_point_seen = true,
                _ => {}
            },
            SessionEvent::Gap { gap, .. } => gaps.push(gap),
            SessionEvent::VerifiedRecoveryPoint { .. } => verified_recovery_point_seen = true,
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
        // At the late-join and outage boundaries, also wait for a fresh receiver
        // to bootstrap. Manifest repetition is periodic, so source progress alone
        // does not guarantee the impaired capture contains a recovery bundle.
        let mut bootstrap = matches!(id, 64 | 96)
            .then(|| SessionRouter::<1128, 64, 64>::new(router_limits).unwrap());
        let mut recovery_received = bootstrap.is_none();
        while !(source_received && recovery_received) && Instant::now() < deadline {
            if let Some(len) = rx.try_recv(&mut packet).unwrap() {
                let header = cu29_logstream::WirePacket::decode(&packet[..len])
                    .unwrap()
                    .header;
                source_received |= header.record_kind == RecordKind::CopperList
                    && header.symbol_kind == cu29_logstream::FecSymbolKind::Source
                    && header.object_id == id;
                traffic.push(packet[..len].to_vec());
                router.receive_datagram(&packet[..len], &mut emit).unwrap();
                // The impaired capture starts at this source packet; earlier
                // manifests must not satisfy its bootstrap requirement.
                if source_received && let Some(bootstrap) = bootstrap.as_mut() {
                    bootstrap
                        .receive_datagram(&packet[..len], |event| {
                            if let SessionEvent::VerifiedRecoveryPoint { recovery_point, .. } =
                                event
                            {
                                recovery_received |= recovery_point.copperlist_id >= id;
                            }
                            Ok::<(), core::convert::Infallible>(())
                        })
                        .unwrap();
                }
            } else {
                std::thread::sleep(Duration::from_millis(1));
            }
        }
        assert!(source_received, "sender did not transmit CopperList {id}");
        assert!(
            recovery_received,
            "sender did not repeat a complete recovery bundle at CopperList {id}"
        );
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

    assert!(manifest_seen && keyframe_seen && recovery_point_seen);
    assert_eq!(router.stats().sessions_discovered, 1);
    assert_eq!(router.stats().malformed_datagrams, 0);
    assert!(verified_recovery_point_seen);
    let source_ids: std::collections::BTreeSet<_> = traffic
        .iter()
        .filter_map(|packet| {
            let header = cu29_logstream::WirePacket::decode(packet).unwrap().header;
            (header.record_kind == RecordKind::CopperList
                && header.symbol_kind == cu29_logstream::FecSymbolKind::Source)
                .then_some(header.object_id)
        })
        .collect();
    assert_eq!(
        source_ids,
        (0..ITERATIONS).collect(),
        "clean fixture must admit every source record"
    );
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
                        cu29_logstream::NativeArchive::new(path, manifest, 1024 * 1024, 4096)
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
    archive
        .expect("impaired capture must include a decodable session manifest")
        .finish()
        .unwrap();
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

mod feedback_runtime {
    use super::{UdpMessage, UdpSource};
    use cu29::prelude::*;
    use cu29_logstream::{
        CuStreamRx, FiniteObjectLimits, RecordKind, SessionEvent, SessionRouter,
        SessionRouterLimits,
    };
    use std::time::{Duration, Instant};
    #[copper_runtime(config = "tests/configured_feedback.ron")]
    struct FeedbackApp {}

    #[test]
    fn configured_feedback_resource_is_owned_and_advertised() -> CuResult<()> {
        use cu29_logstream::feedback::{FEEDBACK_BUFFER_BYTES, FeedbackReporter};
        use cu29_logstream::{CuFeedbackTx, FecSymbolKind, WirePacket};
        use cu29_logstream_udp::CuUdpLogStreamConfig;
        let sender_socket = std::net::UdpSocket::bind("127.0.0.1:0").unwrap();
        let sender_address = sender_socket.local_addr().unwrap();
        drop(sender_socket);
        let mut receiver = CuUdpLogStreamConfig::new("127.0.0.1:0".parse().unwrap());
        receiver.remote_addr = Some(sender_address);
        let (tx, mut rx) = receiver.open()?;
        let mut tx = tx.unwrap();
        let mut config = CuConfig::deserialize_ron(include_str!("configured_feedback.ron"))?;
        let resource = config.resources[0].config.as_mut().unwrap();
        resource.set("bind_addr", sender_address.to_string());
        resource.set("remote_addr", rx.local_addr().unwrap().to_string());
        let logs = tempfile::tempdir().unwrap();
        let app = FeedbackApp::builder()
            .with_config(config)
            .with_log_path(logs.path().join("sender.copper"), Some(1024 * 1024))?
            .build()?;
        let mut running = app.start()?;
        let mut router = SessionRouter::<1128, 64, 64>::new(SessionRouterLimits {
            max_sessions: 1,
            max_startup_packets: 64,
            max_recovery_records: 8,
            max_pending_events: 65,
            max_record_bytes: 4096,
            max_buffered_records: 64,
            equation_capacity: 64,
            finite_objects: FiniteObjectLimits::new(65536, 1128, 4),
        })
        .unwrap();
        let clock = RobotClock::new();
        let mut reporter = None;
        let mut identity = None;
        let mut packet = [0; 1200];
        let mut feedback_packet = [0; FEEDBACK_BUFFER_BYTES];
        let mut reports_sent = 0;
        let deadline = Instant::now() + Duration::from_secs(5);
        let mut sent = 0;
        let mut received = None;
        while Instant::now() < deadline && (sent < 100 || reports_sent < 4) {
            if sent < 100 && (sent == 0 || received == Some(sent - 1)) {
                running.run_one_iteration()?;
                sent += 1;
            }
            if let Some(len) = rx.try_recv(&mut packet).unwrap() {
                let header = WirePacket::decode(&packet[..len]).unwrap().header;
                if header.record_kind == RecordKind::CopperList
                    && header.symbol_kind == FecSymbolKind::Source
                {
                    received =
                        Some(received.map_or(header.object_id, |id: u64| id.max(header.object_id)));
                }
                router
                    .receive_datagram(&packet[..len], |event| {
                        if let SessionEvent::Manifest(manifest) = event {
                            identity = Some(manifest.manifest().identity);
                            reporter =
                                FeedbackReporter::new(manifest.manifest(), [9; 16], clock.now());
                            assert!(reporter.is_some());
                        }
                        Ok::<_, ()>(())
                    })
                    .unwrap();
            }
            if let (Some(reporter), Some(identity)) = (&mut reporter, identity)
                && let Some(counters) = router.feedback_counters(identity)
                && let Some(report) = reporter.report(clock.now(), counters)
            {
                let len = report.encode_into(&mut feedback_packet).unwrap();
                tx.try_send_feedback(&feedback_packet[..len]).unwrap();
                reports_sent += 1;
            }
            std::thread::sleep(Duration::from_millis(1));
        }
        drop(running.stop()?);
        assert_eq!(sent, 100);
        assert!(reports_sent >= 4);
        Ok(())
    }
}

#[test]
fn udp_feedback_worker_adapts_and_survives_a_lost_return_channel() -> CuResult<()> {
    use cu29_logstream::feedback::*;
    use cu29_logstream::{
        ApplicationSchema, CuFeedbackTx, LogStreamPlan, SeparateFeedback, StreamIdentity,
        scheduled_feedback_sinks,
    };
    use cu29_logstream_udp::CuUdpLogStreamConfig;
    let sender_socket = CuUdpLogStreamConfig::new("127.0.0.1:0".parse().unwrap());
    let (_, sender_rx) = sender_socket.open()?;
    let mut ground = CuUdpLogStreamConfig::new("127.0.0.1:0".parse().unwrap());
    ground.remote_addr = Some(sender_rx.local_addr().unwrap());
    let (ground_tx, _ground_rx) = ground.open()?;
    let mut ground_tx = ground_tx.unwrap();
    let mut data = CuUdpLogStreamConfig::new("127.0.0.1:0".parse().unwrap());
    data.remote_addr = Some(ground_tx.local_addr().unwrap());
    let (sender_tx, _) = data.open()?;
    let config = CuConfig::deserialize_ron(include_str!("configured_feedback.ron"))?;
    let plan = LogStreamPlan::resolve(&config.log_streaming.unwrap().destinations[0]).unwrap();
    let identity = StreamIdentity {
        session_id: [11; 16],
        sender_id: 3,
    };
    let config = plan
        .sender_config(
            identity,
            ApplicationSchema {
                outputs: vec![],
                reconstruction: vec![],
            },
        )
        .unwrap();
    let (lists, frames, monitor) = scheduled_feedback_sinks::<default::CuStampedDataSet, _>(
        SeparateFeedback {
            tx: sender_tx.unwrap(),
            feedback_rx: sender_rx,
        },
        config,
        RobotClock::new(),
    )?;
    let mut packet = [0; FEEDBACK_BUFFER_BYTES];
    for sequence in 1..=10 {
        let report = ReceiverReport {
            session_id: identity.session_id,
            sender_id: identity.sender_id,
            receiver_id: [12; 16],
            destination: destination_key("ground"),
            sequence,
            elapsed_us: sequence * 50_000,
            received_bytes: sequence * 1000,
            received_packets: sequence,
            sources: SourceOutcomes {
                first_esi: 0,
                finalized: sequence * 100,
                received: sequence * 100,
                recovered: 0,
                missing: 0,
            },
            record_capacity: 64,
            ..Default::default()
        };
        let len = report.encode_into(&mut packet).unwrap();
        ground_tx.try_send_feedback(&packet[..len]).unwrap();
        std::thread::sleep(Duration::from_millis(60));
    }
    let snapshot = monitor.snapshot();
    let feedback = snapshot.feedback.unwrap();
    assert!(feedback.accepted_reports >= 7, "{snapshot:?}");
    assert!(
        feedback.effective_repair_every_source_symbols > 4,
        "{snapshot:?}"
    );
    let sent = snapshot.stats.packets_sent;
    drop(ground_tx);
    std::thread::sleep(Duration::from_millis(600));
    let snapshot = monitor.snapshot();
    assert_eq!(snapshot.feedback.unwrap().state, FeedbackState::Stale);
    assert_eq!(
        snapshot
            .feedback
            .unwrap()
            .effective_repair_every_source_symbols,
        4
    );
    assert!(snapshot.stats.packets_sent > sent);
    assert!(!snapshot.failed);
    drop((lists, frames));
    assert!(monitor.snapshot().stopped);
    Ok(())
}
