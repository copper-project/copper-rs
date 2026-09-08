use cu29_clock::{CuDuration, CuTime};
use cu29_logstream::feedback::*;
use cu29_logstream::*;

fn time(ms: u64) -> CuTime {
    CuTime::from_nanos(ms * 1_000_000)
}
fn policy(adaptive: bool) -> FeedbackPolicy {
    FeedbackPolicy {
        report_interval_ms: 500,
        timeout_ms: 2000,
        adaptation: adaptive.then_some(AdaptationBounds {
            min_repair_every_source_symbols: 1,
            max_repair_every_source_symbols: 16,
        }),
    }
}
fn report(sequence: u64, n: u64, received: u64, recovered: u64) -> ReceiverReport {
    ReceiverReport {
        session_id: [1; 16],
        sender_id: 7,
        destination: destination_key("ground"),
        receiver_id: [2; 16],
        sequence,
        elapsed_us: sequence * 500_000,
        received_bytes: sequence * 1000,
        received_packets: sequence * 10,
        sources: SourceOutcomes {
            first_esi: 0,
            finalized: n,
            received,
            recovered,
            missing: n - received - recovered,
        },
        record_capacity: 64,
        ..Default::default()
    }
}
fn controller(adaptive: bool) -> FeedbackController {
    FeedbackController::new(
        policy(adaptive),
        StreamIdentity {
            session_id: [1; 16],
            sender_id: 7,
        },
        destination_key("ground"),
        4,
    )
    .unwrap()
}
#[test]
fn bounded_protocol_rejects_truncation_corruption_and_invalid_counts() {
    let report = report(1, 100, 80, 15);
    let mut buffer = [0; FEEDBACK_BUFFER_BYTES];
    let len = report.encode_into(&mut buffer).unwrap();
    assert_eq!(ReceiverReport::decode(&buffer[..len]).unwrap(), report);
    for end in 0..len {
        assert!(ReceiverReport::decode(&buffer[..end]).is_err());
    }
    for index in 0..len {
        buffer[index] ^= 1;
        assert!(ReceiverReport::decode(&buffer[..len]).is_err());
        buffer[index] ^= 1;
    }
    let mut invalid = report;
    invalid.sources.missing += 1;
    assert!(invalid.encode_into(&mut buffer).is_err());
    assert!(ReceiverReport::decode(&[0; FEEDBACK_BUFFER_BYTES + 1]).is_err());
}
#[test]
fn adaptive_fec_moves_both_directions_and_stale_returns_to_baseline() {
    let mut c = controller(true);
    for i in 1..=40 {
        c.receive(report(i, i * 100, i * 100, 0), time(i * 500));
    }
    assert_eq!(c.snapshot().effective_repair_every_source_symbols, 16);
    assert_eq!(c.snapshot().receiver_bytes_per_second, 2000);
    assert_eq!(c.snapshot().receiver_packets_per_second, 20);
    c.receive(report(41, 4100, 4050, 20), time(20_500));
    assert!(c.snapshot().effective_repair_every_source_symbols < 16);
    c.tick(time(22_500));
    assert_eq!(c.snapshot().state, FeedbackState::Stale);
    for ms in (23_000..=32_000).step_by(500) {
        c.tick(time(ms));
    }
    assert_eq!(c.snapshot().effective_repair_every_source_symbols, 4);
    // Same receiver resumes with cumulative counters, even when reports were lost.
    c.receive(report(60, 6000, 5000, 500), time(32_500));
    assert_eq!(c.snapshot().state, FeedbackState::Active);
    assert!((1..=16).contains(&c.snapshot().effective_repair_every_source_symbols));
}
#[test]
fn stale_duplicate_wrong_session_and_competing_receivers_do_not_refresh_health() {
    let mut c = controller(true);
    let first = report(1, 100, 100, 0);
    c.receive(first, time(500));
    c.receive(first, time(1000));
    let mut foreign = report(2, 200, 200, 0);
    foreign.destination = destination_key("other");
    c.receive(foreign, time(1000));
    foreign = report(2, 200, 200, 0);
    foreign.receiver_id = [3; 16];
    c.receive(foreign, time(1500));
    assert_eq!(c.snapshot().accepted_reports, 1);
    assert_eq!(c.snapshot().rejected_reports, 3);
    c.tick(time(2500));
    assert_eq!(c.snapshot().state, FeedbackState::Stale);
    c.receive(foreign, time(2600));
    assert_eq!(c.snapshot().accepted_reports, 2);
    assert_eq!(c.snapshot().report.unwrap().receiver_id, [3; 16]);
    assert!(!c.snapshot().receiver_rates_available);
    assert!(!c.snapshot().source_metrics_available);
}
#[test]
fn reporting_only_idle_and_cadence_never_reduce_protection() {
    let mut c = controller(false);
    for i in 1..=8 {
        c.receive(report(i, i * 100, i * 50, 0), time(i * 500));
    }
    assert_eq!(c.snapshot().effective_repair_every_source_symbols, 4);
    let mut c = controller(true);
    for i in 1..=20 {
        c.receive(report(i, 0, 0, 0), time(i * 500));
    }
    assert_eq!(c.snapshot().effective_repair_every_source_symbols, 4);
    let mut request = report(21, 0, 0, 0);
    request.request_recovery = true;
    assert!(c.receive(request, time(10_500)));
    assert!(!c.receive(report(22, 100, 100, 0), time(10_501)));
}
#[test]
fn invalid_policy_and_baseline_are_rejected() {
    let mut p = policy(true);
    assert!(p.validate(0).is_err());
    assert!(p.validate(17).is_err());
    p.timeout_ms = p.report_interval_ms;
    assert!(p.validate(4).is_err());
}

#[test]
fn real_decoder_finalizes_loss_after_window_expiry_and_handles_reordering() {
    const SYMBOL: usize = 128;
    const WINDOW: usize = 8;
    let identity = StreamIdentity {
        session_id: [1; 16],
        sender_id: 7,
    };
    let config = RlcConfig::new(SYMBOL, WINDOW, Field::Gf256).unwrap();
    let mut encoder = ContinuousEncoder::<SYMBOL, WINDOW>::new(
        identity,
        Lane::ReplayCritical,
        config,
        256,
        EncodingSymbolId::new(0),
    )
    .unwrap();
    let mut decoder = ContinuousDecoder::<SYMBOL, WINDOW, WINDOW>::new(
        identity,
        Lane::ReplayCritical,
        config,
        WINDOW,
        0,
        ReceiverLimits::new(256, WINDOW),
    )
    .unwrap();
    decoder.enable_feedback();
    let mut packet = [0; SYMBOL + PACKET_HEADER_LEN];
    let mut packets = Vec::new();
    for id in 0..24 {
        encoder
            .push_record_with(
                &encode_record(RecordKind::CopperList, id, &[42]).unwrap(),
                &mut packet,
                |p| {
                    packets.push(p.to_vec());
                    Ok(())
                },
            )
            .unwrap();
    }
    // Reordering entirely inside the coding window is not loss.
    for i in [0, 2, 1, 3, 4, 5, 6, 7] {
        decoder
            .receive_datagram(&packets[i], |_| Ok::<_, ()>(()))
            .unwrap();
    }
    assert_eq!(decoder.source_outcomes().finalized, 0);
    // ESI 9 is never delivered; all other finalized symbols arrive intact.
    for (i, packet) in packets.iter().enumerate().skip(8) {
        if i != 9 {
            decoder
                .receive_datagram(packet, |_| Ok::<_, ()>(()))
                .unwrap();
        }
    }
    let outcomes = decoder.source_outcomes();
    assert_eq!(outcomes.finalized, 16);
    assert_eq!(outcomes.received, 15);
    assert_eq!(outcomes.missing, 1);
    decoder
        .receive_datagram(&packets[9], |_| Ok::<_, ()>(()))
        .unwrap();
    assert_eq!(decoder.source_outcomes(), outcomes);
}

#[test]
fn reporter_uses_local_cadence_and_advertised_capability() {
    let plan = LogStreamPlan {
        feedback: Some(policy(true)),
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
            repair_density: 15,
        },
        objects: ResolvedObjectFec {
            max_object_bytes: 4096,
            repair_symbols_per_block: 2,
        },
        recovery_interval: 1,
        max_record_bytes: 4096,
    };
    let manifest = SessionManifest::new(
        StreamIdentity {
            session_id: [1; 16],
            sender_id: 7,
        },
        plan.receiver_requirements(),
        ApplicationSchema {
            outputs: vec![],
            reconstruction: vec![],
        },
    );
    let decoded = SessionManifest::decode_record(&manifest.encode_record().unwrap()).unwrap();
    assert_eq!(decoded, manifest);
    let mut reporter = FeedbackReporter::new(&manifest, [2; 16], time(1000)).unwrap();
    assert!(reporter.report(time(1499), report(1, 0, 0, 0)).is_none());
    let first = reporter.report(time(1500), report(1, 0, 0, 0)).unwrap();
    assert_eq!(
        first.elapsed_us,
        CuDuration::from_millis(500).as_nanos() / 1000
    );
    assert_eq!(first.sequence, 1);
    assert!(reporter.report(time(1500), report(1, 0, 0, 0)).is_none());
}
