use cu29_logstream::{
    ApplicationSchema, ContinuousEncoder, EncodingSymbolId, FiniteObjectEncoder,
    FiniteObjectLimits, LogStreamPlan, RecordKind, SessionEvent, SessionRouter,
    SessionRouterLimits, StreamIdentity, encode_record,
};
use cu29_runtime::config::{
    LogStreamContentConfig, LogStreamContinuousFecConfig, LogStreamDestinationConfig,
    LogStreamFecConfig, LogStreamLinkConfig, LogStreamObjectFecConfig, LogStreamRepairDensity,
    LogStreamRlcField, LogStreamTransportConfig,
};

fn destination() -> LogStreamDestinationConfig {
    LogStreamDestinationConfig {
        id: "ground".into(),
        transport: LogStreamTransportConfig {
            type_: "test::Tx".into(),
            resource: "network.tx".into(),
        },
        link: LogStreamLinkConfig {
            mtu_bytes: 1200,
            bitrate_bps: 1_000_000,
            memory_budget_kib: 512,
            max_latency_ms: 250,
            burst_packets: 8,
        },
        fec: LogStreamFecConfig {
            continuous: LogStreamContinuousFecConfig {
                field: LogStreamRlcField::Gf256,
                window_symbols: 64,
                repair_every_source_symbols: 4,
                repair_density: LogStreamRepairDensity::Full,
            },
            objects: LogStreamObjectFecConfig {
                max_object_bytes: 4_194_304,
                repair_symbols_per_block: 8,
            },
        },
        content: LogStreamContentConfig {
            archive: true,
            live_viz: true,
            anchor_interval: 100,
        },
        max_record_bytes: 65_536,
    }
}

#[test]
fn manifest_bootstraps_continuous_decoder_without_out_of_band_fec_config() {
    let identity = StreamIdentity {
        session_id: *b"router-session01",
        sender_id: 23,
    };
    let plan = LogStreamPlan::resolve(&destination()).unwrap();
    let sender = plan
        .sender_config(identity, ApplicationSchema { outputs: vec![] })
        .unwrap();

    let mut finite = FiniteObjectEncoder::new(sender.recovery.finite).unwrap();
    let mut manifest_packets = Vec::new();
    finite
        .push_record(&sender.recovery.manifest_record, &mut manifest_packets)
        .unwrap();

    let record = encode_record(RecordKind::CopperList, 0, b"semantic-copperlist").unwrap();
    let mut continuous = ContinuousEncoder::<1128, 64>::new(
        identity,
        sender.continuous.first_packet_sequence,
        sender.continuous.lane,
        sender.continuous.fec,
        sender.continuous.max_record_bytes,
        EncodingSymbolId::new(0),
    )
    .unwrap();
    let mut continuous_packets = Vec::new();
    continuous
        .push_record(&record, &mut continuous_packets)
        .unwrap();

    let mut router = SessionRouter::<1128, 64, 64>::new(SessionRouterLimits {
        max_startup_packets: 64,
        max_recovery_records: 8,
        max_sessions: 2,
        max_pending_events: 16,
        max_record_bytes: 65_536,
        max_buffered_records: 64,
        equation_capacity: 64,
        finite_objects: FiniteObjectLimits::new(4_194_304, 1128, 4),
    })
    .unwrap();
    let mut events = Vec::new();
    for packet in manifest_packets {
        router
            .receive_datagram(&packet, |event| {
                events.push(event);
                Ok::<(), ()>(())
            })
            .unwrap();
    }
    assert!(matches!(events.as_slice(), [SessionEvent::Manifest(_)]));
    assert_eq!(router.session_manifest(identity).unwrap().plan, plan);

    for packet in continuous_packets {
        router
            .receive_datagram(&packet, |event| {
                events.push(event);
                Ok::<(), ()>(())
            })
            .unwrap();
    }
    assert!(matches!(
        events.last(),
        Some(SessionEvent::ContinuousRecord {
            identity: event_identity,
            record: recovered,
        }) if *event_identity == identity && recovered.bytes() == record
    ));
}

fn setup() -> (
    cu29_logstream::LogStreamSenderConfig,
    SessionRouter<1128, 64, 64>,
) {
    let identity = StreamIdentity {
        session_id: *b"router-session01",
        sender_id: 23,
    };
    let sender = LogStreamPlan::resolve(&destination())
        .unwrap()
        .sender_config(identity, ApplicationSchema { outputs: vec![] })
        .unwrap();
    let router = SessionRouter::new(SessionRouterLimits {
        max_sessions: 1,
        max_startup_packets: 8,
        max_recovery_records: 4,
        max_pending_events: 3,
        max_record_bytes: 65_536,
        max_buffered_records: 64,
        equation_capacity: 64,
        finite_objects: FiniteObjectLimits::new(4_194_304, 1128, 4),
    })
    .unwrap();
    (sender, router)
}

fn objects(sender: &cu29_logstream::LogStreamSenderConfig, boundary: u64) -> [Vec<Vec<u8>>; 3] {
    let manifest = &sender.recovery.manifest_record;
    let decoded = cu29_logstream::decode_record(manifest).unwrap();
    let keyframe = cu29_runtime::curuntime::KeyFrame {
        culistid: boundary,
        timestamp: Default::default(),
        serialized_tasks: vec![1, 2, 3],
    };
    let (keyframe, anchor) =
        cu29_logstream::encode_keyframe_and_anchor(&keyframe, decoded.object_id, decoded.digest)
            .unwrap();
    [manifest, &keyframe, &anchor].map(|record| {
        let mut packets = Vec::new();
        FiniteObjectEncoder::new(sender.recovery.finite)
            .unwrap()
            .push_record(record, &mut packets)
            .unwrap();
        packets
    })
}

fn continuous(
    sender: &cu29_logstream::LogStreamSenderConfig,
    ids: core::ops::Range<u64>,
) -> Vec<Vec<u8>> {
    let mut encoder = ContinuousEncoder::<1128, 64>::new(
        sender.continuous.identity,
        0,
        sender.continuous.lane,
        sender.continuous.fec,
        sender.continuous.max_record_bytes,
        EncodingSymbolId::new(0),
    )
    .unwrap();
    let mut packets = Vec::new();
    for id in ids {
        encoder
            .push_record(
                &encode_record(RecordKind::CopperList, id, b"record").unwrap(),
                &mut packets,
            )
            .unwrap();
    }
    packets
}

fn receive(
    router: &mut SessionRouter<1128, 64, 64>,
    packets: &[Vec<u8>],
    events: &mut Vec<SessionEvent>,
) {
    for packet in packets {
        router
            .receive_datagram(packet, |event| {
                events.push(event);
                Ok::<(), ()>(())
            })
            .unwrap();
    }
}

fn ids(events: &[SessionEvent]) -> Vec<u64> {
    events
        .iter()
        .filter_map(|event| match event {
            SessionEvent::ContinuousRecord { record, .. } => {
                Some(record.decoded().unwrap().object_id)
            }
            _ => None,
        })
        .collect()
}

#[test]
fn startup_packets_keep_arrival_order_even_with_a_small_event_queue() {
    let (sender, mut router) = setup();
    let mut events = Vec::new();
    receive(&mut router, &continuous(&sender, 0..8), &mut events);
    assert!(events.is_empty());
    receive(&mut router, &objects(&sender, 0)[0], &mut events);
    assert_eq!(ids(&events), (0..8).collect::<Vec<_>>());
    assert_eq!(router.stats().startup_packets_dropped, 0);
    assert!(
        !events
            .iter()
            .any(|event| matches!(event, SessionEvent::Gap { .. }))
    );
}

#[test]
fn late_join_verifies_all_control_object_arrival_orders_and_releases_buffered_boundary() {
    for order in [
        [0, 1, 2],
        [0, 2, 1],
        [1, 0, 2],
        [1, 2, 0],
        [2, 0, 1],
        [2, 1, 0],
    ] {
        let (sender, mut router) = setup();
        let control = objects(&sender, 100);
        let mut events = Vec::new();
        receive(&mut router, &continuous(&sender, 100..103), &mut events);
        for index in order {
            receive(&mut router, &control[index], &mut events);
        }
        let gaps: Vec<_> = events
            .iter()
            .filter_map(|event| match event {
                SessionEvent::Gap { gap, .. } => Some(*gap),
                _ => None,
            })
            .collect();
        assert_eq!(
            gaps,
            [cu29_logstream::CopperListGap {
                first_id: 0,
                last_id: 99,
                reason: cu29_logstream::GapReason::LateJoin
            }],
            "{order:?}"
        );
        assert_eq!(ids(&events), [100, 101, 102], "{order:?}");
        assert_eq!(
            events
                .iter()
                .filter(|event| matches!(event, SessionEvent::VerifiedAnchor { .. }))
                .count(),
            1
        );
        // Repeated objects cannot rewind or re-archive a boundary.
        for packets in control {
            receive(&mut router, &packets, &mut events);
        }
        assert_eq!(ids(&events), [100, 101, 102]);
    }
}

#[test]
fn wrong_digest_or_keyframe_boundary_never_authorizes_a_restart() {
    for wrong_boundary in [false, true] {
        let (sender, mut router) = setup();
        let mut events = Vec::new();
        receive(&mut router, &objects(&sender, 100)[0], &mut events);
        receive(&mut router, &continuous(&sender, 100..101), &mut events);
        let manifest = cu29_logstream::decode_record(&sender.recovery.manifest_record).unwrap();
        let keyframe = cu29_runtime::curuntime::KeyFrame {
            culistid: if wrong_boundary { 101 } else { 100 },
            timestamp: Default::default(),
            serialized_tasks: vec![],
        };
        let keyframe_record = encode_record(
            RecordKind::KeyFrame,
            100,
            &cu29_logstream::encode_keyframe(&keyframe).unwrap(),
        )
        .unwrap();
        let mut anchor = cu29_logstream::Anchor {
            manifest_object_id: manifest.object_id,
            manifest_record_digest: manifest.digest,
            copperlist_id: 100,
            keyframe_object_id: 100,
            keyframe_record_digest: cu29_logstream::decode_record(&keyframe_record)
                .unwrap()
                .digest,
        };
        if !wrong_boundary {
            anchor.keyframe_record_digest[0] ^= 1;
        }
        let anchor_record = encode_record(
            RecordKind::Anchor,
            100,
            &cu29_logstream::encode_anchor(&anchor).unwrap(),
        )
        .unwrap();
        for record in [keyframe_record, anchor_record] {
            let mut packets = Vec::new();
            FiniteObjectEncoder::new(sender.recovery.finite)
                .unwrap()
                .push_record(&record, &mut packets)
                .unwrap();
            receive(&mut router, &packets, &mut events);
        }
        assert!(ids(&events).is_empty());
        assert!(
            !events
                .iter()
                .any(|event| matches!(event, SessionEvent::VerifiedAnchor { .. }))
        );
    }
}

#[test]
fn outage_recovery_and_terminal_gap_are_explicit_and_consumer_failure_is_retryable() {
    let (sender, mut router) = setup();
    let mut events = Vec::new();
    receive(&mut router, &objects(&sender, 0)[0], &mut events);
    receive(&mut router, &continuous(&sender, 0..1), &mut events);
    let control = objects(&sender, 100);
    receive(&mut router, &control[1], &mut events);
    let mut failed = false;
    for packet in &control[2] {
        let result = router.receive_datagram(packet, |event| {
            if matches!(event, SessionEvent::VerifiedAnchor { .. }) {
                return Err("archive unavailable");
            }
            events.push(event);
            Ok(())
        });
        if result.is_err() {
            failed = true;
            break;
        }
    }
    assert!(failed);
    router
        .drain_events(&mut |event| {
            events.push(event);
            Ok::<(), ()>(())
        })
        .unwrap();
    assert!(events.iter().any(|event| matches!(event, SessionEvent::Gap { gap, .. } if gap.first_id == 1 && gap.last_id == 99 && gap.reason == cu29_logstream::GapReason::AnchorRecovery)));
    router
        .finish_through(sender.continuous.identity, 102, |event| {
            events.push(event);
            Ok::<(), ()>(())
        })
        .unwrap();
    assert!(
        matches!(events.last(), Some(SessionEvent::Gap { gap, .. }) if gap.first_id == 100 && gap.last_id == 102 && gap.reason == cu29_logstream::GapReason::SessionEnded)
    );
}

#[test]
fn startup_overflow_is_bounded_and_never_claims_a_complete_session() {
    let (sender, mut router) = setup();
    let mut events = Vec::new();
    receive(&mut router, &continuous(&sender, 0..12), &mut events);
    assert_eq!(router.stats().startup_packets_dropped, 4);
    receive(&mut router, &objects(&sender, 0)[0], &mut events);
    assert_eq!(ids(&events), (0..8).collect::<Vec<_>>());
    router
        .finish_through(sender.continuous.identity, 11, |event| {
            events.push(event);
            Ok::<(), ()>(())
        })
        .unwrap();
    assert!(
        matches!(events.last(), Some(SessionEvent::Gap { gap, .. }) if gap.first_id == 8 && gap.last_id == 11)
    );
}
