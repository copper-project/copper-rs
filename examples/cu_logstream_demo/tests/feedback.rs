#![cfg(feature = "feedback")]

use cu29::monitoring::{LogStreamFeedbackState, LogStreamMonitor, LogStreamStats};
use cu29::prelude::*;
use cu29_logstream::{CuFeedbackTx, CuStreamTxError};
use cu29_logstream_udp::{CuUdpLogStreamConfig, CuUdpLogStreamTx};
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};
use std::time::{Duration, Instant};

thread_local! {
    static STREAMS: std::cell::RefCell<Option<Arc<[LogStreamMonitor]>>> = const { std::cell::RefCell::new(None) };
}

struct StreamProbe;
impl CuMonitor for StreamProbe {
    fn new(_: CuMonitoringMetadata, runtime: CuMonitoringRuntime) -> CuResult<Self> {
        STREAMS.with(|streams| *streams.borrow_mut() = runtime.log_streams());
        Ok(Self)
    }
    fn process_copperlist(&self, _: &CuContext, _: CopperListView<'_>) -> CuResult<()> {
        Ok(())
    }
    fn process_error(&self, _: ComponentId, _: CuComponentState, _: &CuError) -> Decision {
        Decision::Shutdown
    }
}

#[copper_runtime(config = "tests/feedbackconfig.ron")]
struct FeedbackDemo {}

#[derive(Debug)]
struct ReturnPath {
    tx: CuUdpLogStreamTx,
    enabled: Arc<AtomicBool>,
}
impl CuFeedbackTx for ReturnPath {
    fn try_send_feedback(&mut self, packet: &[u8]) -> Result<(), CuStreamTxError> {
        if self.enabled.load(Ordering::Relaxed) {
            self.tx.try_send_feedback(packet)
        } else {
            Err(CuStreamTxError::WouldBlock)
        }
    }
}

#[test]
fn real_receiver_feedback_changes_robot_stats_and_stale_feedback_restores_baseline() {
    let logs = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("logs");
    std::fs::create_dir_all(&logs).unwrap();
    let directory = tempfile::tempdir_in(logs).unwrap();
    let reservation = std::net::UdpSocket::bind("127.0.0.1:0").unwrap();
    let sender_address = reservation.local_addr().unwrap();
    let mut ground = CuUdpLogStreamConfig::new("127.0.0.1:0".parse().unwrap());
    ground.remote_addr = Some(sender_address);
    let (tx, rx) = ground.open().unwrap();
    let mut config = CuConfig::deserialize_ron(&FeedbackDemo::original_config()).unwrap();
    let resource = config.resources[0].config.as_mut().unwrap();
    resource.set("bind_addr", sender_address.to_string());
    resource.set("remote_addr", rx.local_addr().unwrap().to_string());
    drop(reservation);
    let app = FeedbackDemo::builder()
        .with_config(config)
        .with_log_path(
            directory.path().join("sender.copper"),
            Some(cu_logstream_demo::SLAB_BYTES),
        )
        .unwrap()
        .build()
        .unwrap();
    let monitor = STREAMS.with(|streams| streams.borrow().as_ref().unwrap()[0].clone());
    let return_path = Arc::new(AtomicBool::new(false));
    let (mut twin, mut reader) = cu_logstream_demo::twin::Twin::twin(rx)
        .with_feedback(ReturnPath {
            tx: tx.unwrap(),
            enabled: return_path.clone(),
        })
        .with_log_path(directory.path().join("received.copper"))
        .spawn()
        .unwrap();
    let mut running = app.start().unwrap();
    let mut until = |ready: &dyn Fn(LogStreamStats) -> bool| {
        let deadline = Instant::now() + Duration::from_secs(8);
        loop {
            running.run_one_iteration().unwrap();
            std::thread::sleep(Duration::from_millis(10));
            let snapshot = monitor.snapshot();
            if ready(snapshot) {
                break snapshot;
            }
            assert!(
                Instant::now() < deadline,
                "feedback transition timed out: {snapshot:?}"
            );
        }
    };
    let waiting = until(&|s| s.packets_sent > 0 && s.feedback.is_some());
    assert_eq!(
        waiting.feedback.unwrap().state,
        LogStreamFeedbackState::Waiting
    );
    assert_eq!(waiting.feedback.unwrap().reports, 0);
    return_path.store(true, Ordering::Relaxed);
    let active = until(&|s| {
        s.feedback.is_some_and(|f| {
            f.state == LogStreamFeedbackState::Active
                && f.rates_available
                && f.source_metrics_available
                && f.effective_repair_every_source_symbols > 4
        })
    });
    assert!(active.feedback.unwrap().bytes_per_second > 0);
    assert!(reader.status().feedback_reports_sent > 0);
    let archived = reader.status().archived;
    return_path.store(false, Ordering::Relaxed);
    let stale = until(&|s| {
        s.feedback
            .is_some_and(|f| f.state == LogStreamFeedbackState::Stale)
    });
    assert_eq!(
        stale
            .feedback
            .unwrap()
            .effective_repair_every_source_symbols,
        4
    );
    assert!(stale.packets_sent > active.packets_sent);
    assert!(reader.status().archived > archived);
    return_path.store(true, Ordering::Relaxed);
    until(&|s| {
        s.feedback
            .is_some_and(|f| f.state == LogStreamFeedbackState::Active)
    });
    drop(running.stop().unwrap());
    let status = twin.stop().unwrap();
    assert!(!status.feedback_failed);
    assert!(status.archived > 0);
}
