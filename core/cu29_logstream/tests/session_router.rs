use cu29_logstream::{
    ApplicationSchema, ContinuousEncoder, EncodingSymbolId, FiniteObjectEncoder,
    FiniteObjectLimits, LogStreamPlan, RecordKind, SessionEvent, SessionRouter,
    SessionRouterLimits, StreamIdentity, encode_record,
};
use cu29_runtime::config::{
    LogStreamContinuousFecConfig, LogStreamDestinationConfig, LogStreamFecConfig,
    LogStreamLinkConfig, LogStreamObjectFecConfig, LogStreamRepairDensity, LogStreamRlcField,
    LogStreamTransportConfig,
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
        recovery_interval: 100,
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
        .sender_config(
            identity,
            ApplicationSchema {
                outputs: vec![],
                reconstruction: vec![],
            },
        )
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
                events.push(event.to_owned());
                Ok::<(), ()>(())
            })
            .unwrap();
    }
    assert!(matches!(events.as_slice(), [SessionEvent::Manifest(_)]));
    assert_eq!(router.session_manifest(identity).unwrap().plan, plan);

    for packet in continuous_packets {
        router
            .receive_datagram(&packet, |event| {
                events.push(event.to_owned());
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
        .sender_config(
            identity,
            ApplicationSchema {
                outputs: vec![],
                reconstruction: vec![],
            },
        )
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
    let (keyframe, recovery_point) = cu29_logstream::encode_keyframe_and_recovery_point(
        &keyframe,
        decoded.object_id,
        decoded.digest,
    )
    .unwrap();
    [manifest, &keyframe, &recovery_point].map(|record| {
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
                events.push(event.to_owned());
                Ok::<(), ()>(())
            })
            .unwrap();
    }
}

fn ids(events: &[SessionEvent]) -> Vec<u64> {
    events
        .iter()
        .filter_map(|event| match event {
            SessionEvent::ContinuousRecord { record, .. } => Some(record.decoded().object_id),
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
                .filter(|event| matches!(event, SessionEvent::VerifiedRecoveryPoint { .. }))
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
        let mut recovery_point = cu29_logstream::RecoveryPoint {
            manifest_object_id: manifest.object_id,
            manifest_record_digest: manifest.digest,
            copperlist_id: 100,
            keyframe_object_id: 100,
            keyframe_record_digest: cu29_logstream::decode_record(&keyframe_record)
                .unwrap()
                .digest,
        };
        if !wrong_boundary {
            recovery_point.keyframe_record_digest[0] ^= 1;
        }
        let recovery_point_record = encode_record(
            RecordKind::RecoveryPoint,
            100,
            &cu29_logstream::encode_recovery_point(&recovery_point).unwrap(),
        )
        .unwrap();
        for record in [keyframe_record, recovery_point_record] {
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
                .any(|event| matches!(event, SessionEvent::VerifiedRecoveryPoint { .. }))
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
            if matches!(event, SessionEvent::VerifiedRecoveryPoint { .. }) {
                return Err("archive unavailable");
            }
            events.push(event.to_owned());
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
            events.push(event.to_owned());
            Ok::<(), ()>(())
        })
        .unwrap();
    assert!(events.iter().any(|event| matches!(event, SessionEvent::Gap { gap, .. } if gap.first_id == 1 && gap.last_id == 99 && gap.reason == cu29_logstream::GapReason::RecoveryPoint)));
    router
        .finish_through(sender.continuous.identity, 102, |event| {
            events.push(event.to_owned());
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
            events.push(event.to_owned());
            Ok::<(), ()>(())
        })
        .unwrap();
    assert!(
        matches!(events.last(), Some(SessionEvent::Gap { gap, .. }) if gap.first_id == 8 && gap.last_id == 11)
    );
}

struct CountingAllocator;
thread_local! {
    static ALLOCATIONS: std::cell::Cell<Option<usize>> = const { std::cell::Cell::new(None) };
}
unsafe impl std::alloc::GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: std::alloc::Layout) -> *mut u8 {
        let _ = ALLOCATIONS.try_with(|count| {
            if let Some(value) = count.get() {
                count.set(Some(value + 1));
            }
        });
        unsafe { std::alloc::System.alloc(layout) }
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: std::alloc::Layout) {
        unsafe { std::alloc::System.dealloc(ptr, layout) }
    }
    unsafe fn realloc(&self, ptr: *mut u8, layout: std::alloc::Layout, size: usize) -> *mut u8 {
        let _ = ALLOCATIONS.try_with(|count| {
            if let Some(value) = count.get() {
                count.set(Some(value + 1));
            }
        });
        unsafe { std::alloc::System.realloc(ptr, layout, size) }
    }
}
#[global_allocator]
static ALLOCATOR: CountingAllocator = CountingAllocator;

fn count_allocations(run: impl FnOnce()) -> usize {
    ALLOCATIONS.with(|count| count.set(Some(0)));
    run();
    ALLOCATIONS.with(|count| count.replace(None).unwrap())
}

#[test]
fn warmed_router_reuses_record_buffers_with_reordering_and_consumer_retries() {
    let (sender, mut router) = setup();
    receive(&mut router, &objects(&sender, 0)[0], &mut Vec::new());
    let mut encoder = ContinuousEncoder::<1128, 64>::new(
        sender.continuous.identity,
        0,
        sender.continuous.lane,
        sender.continuous.fec,
        sender.continuous.max_record_bytes,
        EncodingSymbolId::new(0),
    )
    .unwrap();
    let packets: Vec<Vec<Vec<u8>>> = (0..200)
        .map(|id| {
            // Warm both buffers at the maximum size, then vary lengths and fragment counts.
            let size = if id < 2 {
                4096
            } else {
                [80, 2048, 4096][id as usize % 3]
            };
            let record = encode_record(RecordKind::CopperList, id, &vec![id as u8; size]).unwrap();
            let mut packets = Vec::new();
            encoder.push_record(&record, &mut packets).unwrap();
            packets
        })
        .collect();
    let mut next = 0;
    let mut retry_pointer = None;
    let mut deliver_pair = |pair: &[Vec<Vec<u8>>]| {
        // Complete the later record first, forcing two concurrent assembly buffers.
        for packet in pair[1].iter().chain(&pair[0]) {
            let result = router.receive_datagram(packet, |event| {
                if let SessionEvent::ContinuousRecord { record, .. } = event {
                    assert_eq!(record.decoded().object_id, next);
                    retry_pointer = Some(record.bytes().as_ptr());
                    return Err("retry");
                }
                Ok(())
            });
            if let Err(cu29_logstream::ReceiveError::Consumer("retry")) = result {
                router
                    .drain_events(&mut |event| {
                        if let SessionEvent::ContinuousRecord { record, .. } = event {
                            if let Some(pointer) = retry_pointer.take() {
                                assert_eq!(record.bytes().as_ptr(), pointer);
                            }
                            let decoded = record.decoded();
                            assert_eq!(decoded.object_id, next);
                            assert!(decoded.payload.iter().all(|byte| *byte == next as u8));
                            next += 1;
                        }
                        Ok::<(), ()>(())
                    })
                    .unwrap();
            } else {
                result.unwrap();
            }
        }
    };
    deliver_pair(&packets[..2]);
    let allocations = count_allocations(|| {
        for pair in packets[2..].as_chunks::<2>().0 {
            deliver_pair(pair);
        }
    });
    assert_eq!(next, 200);
    assert_eq!(allocations, 0, "steady-state receive and retry allocated");
}

#[test]
fn recovery_delivery_retries_borrow_the_same_verified_control_records() {
    let (sender, mut router) = setup();
    let control = objects(&sender, 100);
    receive(&mut router, &control[0], &mut Vec::new());
    receive(&mut router, &control[1], &mut Vec::new());
    let mut pointers = None;
    for packet in &control[2] {
        let result = router.receive_datagram(packet, |event| {
            if let SessionEvent::VerifiedRecoveryPoint {
                recovery_record,
                keyframe,
                ..
            } = event
            {
                pointers = Some((recovery_record.bytes().as_ptr(), keyframe.bytes().as_ptr()));
                return Err("retry");
            }
            Ok(())
        });
        if result.is_err() {
            break;
        }
    }
    let pointers = pointers.expect("recovery event");
    let mut delivered = false;
    let allocations = count_allocations(|| {
        router
            .drain_events(&mut |event| {
                if let SessionEvent::VerifiedRecoveryPoint {
                    recovery_record,
                    keyframe,
                    ..
                } = event
                {
                    assert_eq!(
                        (recovery_record.bytes().as_ptr(), keyframe.bytes().as_ptr()),
                        pointers
                    );
                    assert_eq!(keyframe.keyframe_id(), Some(100));
                    delivered = true;
                }
                Ok::<(), ()>(())
            })
            .unwrap();
    });
    assert!(delivered);
    assert_eq!(allocations, 0);
}

#[derive(Debug, Default, bincode::Encode, bincode::Decode, serde::Serialize)]
struct EmptyDataSet;
impl cu29_traits::ErasedCuStampedDataSet for EmptyDataSet {
    fn cumsgs(&self) -> Vec<&dyn cu29_traits::ErasedCuStampedData> {
        Vec::new()
    }
}
impl cu29_traits::MatchingTasks for EmptyDataSet {
    fn get_all_task_ids() -> &'static [&'static str] {
        &[]
    }
}

#[cfg(feature = "std")]
#[test]
fn archive_writes_recovery_bytes_without_allocating() {
    let (sender, mut router) = setup();
    let logs = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../examples/cu_logstream_demo/logs");
    std::fs::create_dir_all(&logs).unwrap();
    let path = logs.join(format!("receiver-buffers-{}.copper", std::process::id()));
    let mut archive = None;
    let mut recovery_written = false;
    for packets in objects(&sender, 0) {
        for packet in packets {
            router
                .receive_datagram(&packet, |event| {
                    if let SessionEvent::Manifest(manifest) = &event {
                        archive = Some(
                            cu29_logstream::NativeArchive::<EmptyDataSet>::new(
                                &path,
                                manifest,
                                1024 * 1024,
                                64 * 1024,
                            )
                            .unwrap(),
                        );
                    }
                    if matches!(event, SessionEvent::VerifiedRecoveryPoint { .. }) {
                        let allocations = count_allocations(|| {
                            archive.as_mut().unwrap().accept(&event).unwrap();
                        });
                        assert_eq!(
                            allocations, 0,
                            "archive re-encoded recovery bytes or copied task state"
                        );
                        recovery_written = true;
                    }
                    Ok::<(), ()>(())
                })
                .unwrap();
        }
    }
    assert!(recovery_written);
    archive.unwrap().finish().unwrap();
}

#[test]
fn continuity_records_support_borrowed_writes_and_owned_reads() {
    use cu29_runtime::continuity::StreamContinuityRecord;
    let payload = [42; 400];
    let records = [
        StreamContinuityRecord::Manifest {
            record: payload.as_slice(),
        },
        StreamContinuityRecord::RecoveryPoint {
            copperlist_id: 100,
            record: payload.as_slice(),
        },
    ];
    for record in records {
        let mut bytes = [0; 512];
        let mut len = 0;
        let allocations = count_allocations(|| {
            len = bincode::encode_into_slice(&record, &mut bytes, bincode::config::standard())
                .unwrap();
            let (decoded, used): (StreamContinuityRecord<&[u8]>, _) =
                bincode::borrow_decode_from_slice(&bytes[..len], bincode::config::standard())
                    .unwrap();
            assert_eq!(used, len);
            assert_eq!(decoded, record);
        });
        assert_eq!(allocations, 0);
        let (owned, used): (StreamContinuityRecord, _) =
            bincode::decode_from_slice(&bytes[..len], bincode::config::standard()).unwrap();
        assert_eq!(used, len);
        let owned_bytes = bincode::encode_to_vec(owned, bincode::config::standard()).unwrap();
        assert_eq!(owned_bytes, bytes[..len]);
    }
}

#[test]
fn drain_processes_symbols_accepted_before_a_consumer_failure() {
    use cu29_logstream::{ContinuousDecoder, ContinuousReceiveEvent, ReceiveError, ReceiverLimits};
    let (sender, _) = setup();
    let packets = continuous(&sender, 0..2);
    let mut decoder = ContinuousDecoder::<1128, 64, 64>::new(
        sender.continuous.identity,
        sender.continuous.lane,
        sender.continuous.fec,
        64,
        0,
        ReceiverLimits::new(65_536, 64),
    )
    .unwrap();
    for packet in packets {
        assert_eq!(
            decoder.receive_datagram(&packet, |_| Err("retry")),
            Err(ReceiveError::Consumer("retry"))
        );
    }
    // The second packet reached FEC, but delivery of record 0 interrupted its assembly.
    // No further datagram is needed to assemble and deliver record 1.
    let mut next = 0;
    decoder
        .drain_events(|event| {
            let ContinuousReceiveEvent::Record(record) = event else {
                panic!("unexpected gap")
            };
            assert_eq!(record.decoded().object_id, next);
            next += 1;
            Ok::<(), ()>(())
        })
        .unwrap();
    assert_eq!(next, 2);
}
