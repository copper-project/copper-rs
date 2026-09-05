use cu29_clock::{CuDuration, CuTime, RobotClock};
use cu29_logstream::*;
use cu29_runtime::curuntime::KeyFrame;

fn config() -> LogStreamSenderConfig {
    let plan = LogStreamPlan {
        destination_id: "ground".into(),
        mtu_bytes: 1200,
        symbol_size: 1128,
        bitrate_bps: 1_000_000,
        memory_budget_kib: 512,
        max_latency_ms: 250,
        burst_packets: 4,
        continuous: ResolvedContinuousFec {
            field: ResolvedRlcField::Gf256,
            window_symbols: 64,
            repair_every_source_symbols: 4,
            repair_density: DensityThreshold::FULL.get(),
        },
        objects: ResolvedObjectFec {
            max_object_bytes: 4096,
            repair_symbols_per_block: 2,
        },
        content: ResolvedContentPolicy {
            archive: true,
            live_viz: false,
            anchor_interval: 1,
        },
        max_record_bytes: 4096,
    };
    plan.sender_config(
        StreamIdentity {
            session_id: *b"paced-session001",
            sender_id: 1,
        },
        ApplicationSchema { outputs: vec![] },
    )
    .unwrap()
}

fn cl(id: u64) -> Vec<u8> {
    encode_record(RecordKind::CopperList, id, &[42; 100]).unwrap()
}
fn kf(id: u64) -> Vec<u8> {
    encode_record(
        RecordKind::KeyFrame,
        id,
        &encode_keyframe(&KeyFrame {
            culistid: id,
            timestamp: CuTime::default(),
            serialized_tasks: vec![7; 100],
        })
        .unwrap(),
    )
    .unwrap()
}

#[derive(Debug, Default)]
struct Capture {
    now: CuTime,
    packets: Vec<(CuTime, Vec<u8>)>,
    block: bool,
}
impl CuStreamTx for Capture {
    fn try_send(&mut self, packet: &[u8]) -> std::result::Result<(), CuStreamTxError> {
        self.packets.push((self.now, packet.to_vec()));
        if self.block {
            Err(CuStreamTxError::WouldBlock)
        } else {
            Ok(())
        }
    }
}

fn advance(
    core: &mut SenderCore,
    tx: &mut Capture,
    clock: &RobotClock,
    mock: &cu29_clock::RobotClockMock,
    from: u64,
    through: u64,
) {
    for ms in from..=through {
        mock.set_value(ms * 1_000_000);
        tx.now = clock.now();
        core.poll(clock.now(), tx).unwrap();
    }
}

#[test]
fn shared_budget_burst_and_queue_pressure_are_bounded() {
    let (clock, mock) = RobotClock::mock();
    let config = config();
    let mut core = SenderCore::new(config.clone(), clock.now(), 0).unwrap();
    for id in 0..2000 {
        core.accept_record(&cl(id), clock.now()).unwrap();
    }
    core.accept_record(&kf(1999), clock.now()).unwrap();
    let mut tx = Capture::default();
    advance(&mut core, &mut tx, &clock, &mock, 0, 1000);
    assert!(core.stats().queue_drops > 0);
    assert!(core.stats().expired_packets > 0);
    assert!(core.stats().queue_peak * 1200 < config.pacing.memory_budget_bytes);
    // Check every observed time interval, including same-timestamp bursts and
    // mixed source, repair, manifest, and anchor packets.
    for first in 0..tx.packets.len() {
        let mut bytes = 0_u128;
        for last in first..tx.packets.len() {
            bytes += tx.packets[last].1.len() as u128;
            let elapsed = (tx.packets[last].0 - tx.packets[first].0).as_nanos();
            assert!(
                bytes * 8 * 1_000_000_000
                    <= u128::from(config.pacing.burst_packets) * 1200 * 8 * 1_000_000_000
                        + u128::from(elapsed) * u128::from(config.pacing.bitrate_bps)
            );
        }
    }
    let before = tx.packets.len();
    mock.set_value(60_000_000_000);
    tx.now = clock.now();
    core.poll(clock.now(), &mut tx).unwrap();
    assert!(tx.packets.len() - before <= config.pacing.burst_packets as usize);
}

#[test]
fn missed_bootstrap_recovers_during_idle_without_new_captures() {
    let (clock, mock) = RobotClock::mock();
    let mut core = SenderCore::new(config(), clock.now(), 0).unwrap();
    core.accept_record(&kf(32), clock.now()).unwrap();
    core.accept_record(&cl(32), clock.now()).unwrap();
    let mut tx = Capture::default();
    advance(&mut core, &mut tx, &clock, &mock, 0, 200);
    tx.packets.clear(); // Lose the entire initial transmission, including CL32.
    advance(&mut core, &mut tx, &clock, &mock, 201, 600);
    let mut router = SessionRouter::<1128, 64, 64>::new(SessionRouterLimits {
        max_sessions: 1,
        max_startup_packets: 64,
        max_recovery_records: 8,
        max_pending_events: 32,
        max_record_bytes: 4096,
        max_buffered_records: 64,
        equation_capacity: 64,
        finite_objects: FiniteObjectLimits::new(4096, 1128, 4),
    })
    .unwrap();
    let mut events = Vec::new();
    for (_, packet) in tx.packets {
        router
            .receive_datagram(&packet, |event| {
                events.push(event);
                Ok::<_, ()>(())
            })
            .unwrap();
    }
    assert!(
        events
            .iter()
            .any(|e| matches!(e, SessionEvent::Manifest(_)))
    );
    assert!(events.iter().any(
        |e| matches!(e, SessionEvent::VerifiedAnchor { anchor, .. } if anchor.copperlist_id == 32)
    ));
    assert!(events.iter().any(
        |e| matches!(e, SessionEvent::Gap { gap, .. } if gap.first_id == 0 && gap.last_id == 31)
    ));
    assert_eq!(
        events
            .iter()
            .filter(|e| matches!(e, SessionEvent::ContinuousRecord { .. }))
            .count(),
        1
    );
}

#[test]
fn independently_ordered_workers_pair_bounded_recovery_records() {
    for keyframes_first in [false, true] {
        let (clock, mock) = RobotClock::mock();
        let mut core = SenderCore::new(config(), clock.now(), 0).unwrap();
        if keyframes_first {
            for id in 0..2 {
                core.accept_record(&kf(id), clock.now()).unwrap();
            }
            for id in 0..4 {
                core.accept_record(&cl(id), clock.now()).unwrap();
            }
        } else {
            for id in 0..4 {
                core.accept_record(&cl(id), clock.now()).unwrap();
            }
            for id in 0..2 {
                core.accept_record(&kf(id), clock.now()).unwrap();
            }
        }
        let mut tx = Capture::default();
        advance(&mut core, &mut tx, &clock, &mock, 0, 200);
        for id in 0..2 {
            assert!(tx.packets.iter().any(|(_, p)| {
                let h = WirePacket::decode(p).unwrap().header;
                h.record_kind == RecordKind::Anchor && h.object_id == id
            }));
        }
    }
}

#[test]
fn feedback_requests_coalesce_and_backpressure_does_not_retry() {
    let (clock, mock) = RobotClock::mock();
    let mut baseline = SenderCore::new(config(), clock.now(), 0).unwrap();
    let mut requested = SenderCore::new(config(), clock.now(), 0).unwrap();
    for core in [&mut baseline, &mut requested] {
        core.accept_record(&cl(0), clock.now()).unwrap();
        core.accept_record(&kf(0), clock.now()).unwrap();
    }
    let mut a = Capture {
        block: true,
        ..Capture::default()
    };
    let mut b = Capture {
        block: true,
        ..Capture::default()
    };
    for ms in 0..100 {
        mock.set_value(ms * 1_000_000);
        a.now = clock.now();
        b.now = clock.now();
        for _ in 0..100 {
            requested.request_recovery();
        }
        baseline.poll(clock.now(), &mut a).unwrap();
        requested.poll(clock.now(), &mut b).unwrap();
        // While bootstrap remains in flight, requests must not reset its cursor.
        if !baseline.is_idle() {
            assert_eq!(a.packets, b.packets);
        }
    }
    assert_eq!(requested.stats().packets_sent, 0);
    assert_eq!(requested.stats().transport_drops as usize, b.packets.len());
    requested.begin_shutdown();
    advance(&mut requested, &mut b, &clock, &mock, 100, 300);
    let count = b.packets.len();
    requested.request_recovery();
    advance(&mut requested, &mut b, &clock, &mock, 301, 1000);
    assert_eq!(count, b.packets.len());
}

#[test]
fn invalid_bounds_and_backward_clock_fail_explicitly() {
    let mut invalid = config();
    invalid.pacing.memory_budget_bytes = 1;
    assert!(SenderCore::new(invalid, CuTime::default(), 0).is_err());
    let mut core = SenderCore::new(config(), CuTime::from_millis(10), 0).unwrap();
    assert!(
        core.poll(CuTime::default(), &mut Capture::default())
            .is_err()
    );
    let (clock, mock) = RobotClock::mock();
    assert!(clock.is_mock());
    mock.increment(CuDuration::from_millis(1));
    assert_eq!(clock.clone().now(), clock.now());
}

#[test]
fn recovery_never_overtakes_older_queued_source_records() {
    let (clock, mock) = RobotClock::mock();
    let mut cfg = config();
    cfg.pacing.max_latency = CuDuration::from_millis(2000);
    let mut core = SenderCore::new(cfg, clock.now(), 0).unwrap();
    for id in 0..33 {
        core.accept_record(&cl(id), clock.now()).unwrap();
    }
    core.accept_record(&kf(32), clock.now()).unwrap();
    let mut tx = Capture::default();
    advance(&mut core, &mut tx, &clock, &mock, 0, 600);
    let anchor = tx
        .packets
        .iter()
        .position(|(_, p)| WirePacket::decode(p).unwrap().header.record_kind == RecordKind::Anchor)
        .unwrap();
    for id in 0..32 {
        assert!(tx.packets[..anchor].iter().any(|(_, p)| {
            let h = WirePacket::decode(p).unwrap().header;
            h.record_kind == RecordKind::CopperList
                && h.symbol_kind == FecSymbolKind::Source
                && h.object_id == id
        }));
    }
    assert_eq!(core.stats().expired_packets, 0);
}
