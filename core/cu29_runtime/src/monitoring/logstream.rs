//! Read-only handles for statically wired sender workers. No stream protocol dependency.
use compact_str::CompactString;
use cu29_clock::{CuDuration, CuTime};
use std::sync::Arc;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum LogStreamFeedbackState {
    #[default]
    Waiting,
    Active,
    Stale,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct LogStreamFeedbackStats {
    pub state: LogStreamFeedbackState,
    pub age: Option<CuDuration>,
    pub failed: bool,
    pub reports: u64,
    pub rejected_reports: u64,
    pub invalid_reports: u64,
    pub rates_available: bool,
    pub source_metrics_available: bool,
    pub bytes_per_second: u64,
    pub packets_per_second: u64,
    pub finalized_symbols: u64,
    pub loss_basis_points: u16,
    pub recovery_basis_points: u16,
    pub buffered_records: u32,
    pub record_capacity: u32,
    pub latest_copperlist: Option<u64>,
    pub effective_repair_every_source_symbols: u16,
    pub invalid_packets: u64,
    pub duplicate_packets: u64,
    pub expired_records: u64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct LogStreamStats {
    pub sampled_at: CuTime,
    pub packets_sent: u64,
    pub bytes_sent: u64,
    pub queue_drops: u64,
    pub expired_packets: u64,
    pub transport_drops: u64,
    pub inbox_drops: u64,
    pub shutdown_drops: u64,
    pub recovery_rounds: u64,
    pub recovery_superseded: u64,
    pub queue_peak: usize,
    pub stopped: bool,
    pub failed: bool,
    pub feedback: Option<LogStreamFeedbackStats>,
}

/// Queried only by the monitor's presentation thread, never by task execution.
pub trait LogStreamStatsSource: core::fmt::Debug + Send + Sync {
    fn snapshot(&self) -> LogStreamStats;
}

/// Identity and immutable policy are bound once during generated app construction.
#[derive(Clone, Debug)]
pub struct LogStreamMonitor {
    pub destination: CompactString,
    pub bitrate_bps: u64,
    pub baseline_repair_every_source_symbols: usize,
    source: Arc<dyn LogStreamStatsSource>,
}
impl LogStreamMonitor {
    pub fn new(
        destination: &str,
        bitrate_bps: u64,
        baseline: usize,
        source: impl LogStreamStatsSource + 'static,
    ) -> Self {
        Self {
            destination: destination.into(),
            bitrate_bps,
            baseline_repair_every_source_symbols: baseline,
            source: Arc::new(source),
        }
    }
    pub fn snapshot(&self) -> LogStreamStats {
        self.source.snapshot()
    }
}
