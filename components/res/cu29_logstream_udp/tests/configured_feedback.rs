#[path = "support/tasks.rs"]
mod tasks;
use tasks::{UdpMessage, UdpSource};

use cu29::prelude::*;
use cu29_logstream::{
    CuStreamRx, FiniteObjectLimits, RecordKind, SessionEvent, SessionRouter, SessionRouterLimits,
};
use std::time::{Duration, Instant};
thread_local! {
    static STREAMS: std::cell::RefCell<Option<std::sync::Arc<[cu29::monitoring::LogStreamMonitor]>>> = const { std::cell::RefCell::new(None) };
}
struct StreamProbe;
impl cu29::monitoring::CuMonitor for StreamProbe {
    fn new(
        _: cu29::monitoring::CuMonitoringMetadata,
        runtime: cu29::monitoring::CuMonitoringRuntime,
    ) -> CuResult<Self> {
        STREAMS.with(|streams| *streams.borrow_mut() = runtime.log_streams());
        Ok(Self)
    }
    fn process_copperlist(
        &self,
        _: &CuContext,
        _: cu29::monitoring::CopperListView<'_>,
    ) -> CuResult<()> {
        Ok(())
    }
    fn process_error(
        &self,
        _: cu29::monitoring::ComponentId,
        _: cu29::monitoring::CuComponentState,
        _: &CuError,
    ) -> cu29::monitoring::Decision {
        cu29::monitoring::Decision::Shutdown
    }
}
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
                        reporter = FeedbackReporter::new(manifest.manifest(), [9; 16], clock.now());
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
    STREAMS.with(|streams| {
        let streams = streams.borrow();
        let monitors = streams.as_ref().expect("generated monitor handles");
        assert_eq!(monitors.len(), 1);
        assert_eq!(monitors[0].destination, "ground");
        let snapshot = monitors[0].snapshot();
        assert!(snapshot.packets_sent > 0);
        assert!(snapshot.feedback.unwrap().reports > 0);
        assert!(snapshot.stopped);
    });
    Ok(())
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
    // Reports and monitor snapshots are asynchronous. Keep reporting until the
    // worker publishes adaptation instead of assuming a fixed sleep is enough.
    let deadline = Instant::now() + Duration::from_secs(5);
    let mut sequence = 0;
    while Instant::now() < deadline {
        sequence += 1;
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
        if monitor.snapshot().feedback.is_some_and(|feedback| {
            feedback.accepted_reports >= 7 && feedback.effective_repair_every_source_symbols > 4
        }) {
            break;
        }
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
    let deadline = Instant::now() + Duration::from_secs(5);
    let snapshot = loop {
        let snapshot = monitor.snapshot();
        if snapshot.feedback.is_some_and(|feedback| {
            feedback.state == FeedbackState::Stale
                && feedback.effective_repair_every_source_symbols == 4
        }) && snapshot.stats.packets_sent > sent
            || Instant::now() >= deadline
        {
            break snapshot;
        }
        std::thread::sleep(Duration::from_millis(10));
    };
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
