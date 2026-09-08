use crate::{palette, ui::format_bytes};
use cu29::monitoring::{LogStreamFeedbackState, LogStreamMonitor, LogStreamStats};
use ratatui::{
    layout::Alignment,
    style::{Modifier, Style},
    text::Line,
    widgets::{Cell, Row},
};

/// UI-owned differences between worker snapshots; no per-CopperList callbacks.
#[derive(Default)]
pub(crate) struct StreamRates {
    previous: Option<LogStreamStats>,
    pub(crate) bytes: Option<f64>,
    pub(crate) packets: Option<f64>,
}
impl StreamRates {
    pub(crate) fn update(&mut self, snapshot: LogStreamStats) {
        if snapshot.stopped || snapshot.failed {
            self.bytes = Some(0.0);
            self.packets = Some(0.0);
        } else if let Some(previous) = self.previous {
            let now = snapshot.sampled_at.as_nanos();
            let before = previous.sampled_at.as_nanos();
            if now > before {
                let seconds = (now - before) as f64 / 1_000_000_000.0;
                self.bytes =
                    Some(snapshot.bytes_sent.saturating_sub(previous.bytes_sent) as f64 / seconds);
                self.packets = Some(
                    snapshot.packets_sent.saturating_sub(previous.packets_sent) as f64 / seconds,
                );
            } else if now < before {
                self.bytes = None;
                self.packets = None;
            }
        }
        self.previous = Some(snapshot);
    }
}

fn row(metric: &'static str, value: impl Into<String>) -> Row<'static> {
    Row::new([
        Cell::from(metric),
        Cell::from(Line::from(value.into()).alignment(Alignment::Right)),
    ])
}
fn count(metric: &'static str, value: u64) -> Row<'static> {
    let row = row(metric, value.to_string());
    if value == 0 {
        row
    } else {
        row.style(
            Style::default()
                .fg(palette::LIGHT_RED)
                .add_modifier(Modifier::BOLD),
        )
    }
}
fn bytes_rate(bytes: f64) -> String {
    format!("{}/s", format_bytes(bytes))
}

pub(crate) fn rows(
    monitor: &LogStreamMonitor,
    snapshot: LogStreamStats,
    rates: &StreamRates,
) -> Vec<Row<'static>> {
    let (feedback_state, feedback_color) = match snapshot.feedback {
        None => ("Disabled", palette::FOREGROUND),
        Some(feedback) if feedback.failed => ("Failed", palette::LIGHT_RED),
        Some(feedback) => match feedback.state {
            LogStreamFeedbackState::Waiting => ("Waiting", palette::YELLOW),
            LogStreamFeedbackState::Active => ("Active", palette::CYAN),
            LogStreamFeedbackState::Stale => ("Stale", palette::YELLOW),
        },
    };
    let mut rows = vec![
        row(
            "Mode",
            if snapshot.feedback.is_some() {
                "Two-way"
            } else {
                "One-way"
            },
        ),
        row(
            "Sender",
            if snapshot.failed {
                "Failed"
            } else if snapshot.stopped {
                "Stopped"
            } else {
                "Running"
            },
        )
        .style(Style::default().fg(if snapshot.failed {
            palette::LIGHT_RED
        } else {
            palette::CYAN
        })),
        row("Feedback", feedback_state).style(Style::default().fg(feedback_color)),
        row(
            "Configured budget",
            format!("{} bit/s", monitor.bitrate_bps),
        ),
        row(
            "TX BW (Copper packets)",
            rates.bytes.map_or_else(|| "n/a".into(), bytes_rate),
        ),
        row(
            "TX packet rate",
            rates
                .packets
                .map_or_else(|| "n/a".into(), |rate| format!("{rate:.1} pkt/s")),
        ),
        row("Sent packets", snapshot.packets_sent.to_string()),
        row("Sent bytes", format_bytes(snapshot.bytes_sent as f64)),
        count("Queue drops", snapshot.queue_drops),
        count("Expired packets", snapshot.expired_packets),
        count("Transport drops", snapshot.transport_drops),
        count("Inbox record drops", snapshot.inbox_drops),
        count("Shutdown drops", snapshot.shutdown_drops),
        row("Recovery rounds", snapshot.recovery_rounds.to_string()),
        count("Recovery replaced", snapshot.recovery_superseded),
        row("Queue peak (packets)", snapshot.queue_peak.to_string()),
        row(
            "FEC baseline interval",
            format!("{} sources", monitor.baseline_repair_every_source_symbols),
        ),
    ];
    if let Some(feedback) = snapshot.feedback {
        let active = feedback.state == LogStreamFeedbackState::Active && !feedback.failed;
        let current = |value: String| if active { value } else { "n/a".into() };
        rows.extend([
            row(
                "Report age",
                feedback.age.map_or_else(
                    || "n/a".into(),
                    |age| format!("{} ms", age.as_nanos() / 1_000_000),
                ),
            ),
            row("Feedback reports", feedback.reports.to_string()),
            count("Rejected reports", feedback.rejected_reports),
            count("Invalid reports", feedback.invalid_reports),
            row(
                "FEC effective interval",
                format!("{} sources", feedback.effective_repair_every_source_symbols),
            ),
            row(
                "RX BW",
                if feedback.rates_available {
                    current(bytes_rate(feedback.bytes_per_second as f64))
                } else {
                    "n/a".into()
                },
            ),
            row(
                "RX packet rate",
                if feedback.rates_available {
                    current(format!("{} pkt/s", feedback.packets_per_second))
                } else {
                    "n/a".into()
                },
            ),
            row(
                "Finalized symbols",
                current(feedback.finalized_symbols.to_string()),
            ),
            row(
                "Source loss",
                if feedback.source_metrics_available {
                    current(format!(
                        "{:.2}%",
                        f64::from(feedback.loss_basis_points) / 100.0
                    ))
                } else {
                    "n/a".into()
                },
            ),
            row(
                "Missing recovered (FEC)",
                if feedback.source_metrics_available {
                    current(format!(
                        "{:.2}%",
                        f64::from(feedback.recovery_basis_points) / 100.0
                    ))
                } else {
                    "n/a".into()
                },
            ),
            row(
                "RX records buffered",
                current(format!(
                    "{} / {}",
                    feedback.buffered_records, feedback.record_capacity
                )),
            ),
            row(
                "RX latest CopperList",
                current(
                    feedback
                        .latest_copperlist
                        .map_or_else(|| "n/a".into(), |id| id.to_string()),
                ),
            ),
            row(
                "RX invalid packets",
                current(feedback.invalid_packets.to_string()),
            ),
            row(
                "RX duplicate packets",
                current(feedback.duplicate_packets.to_string()),
            ),
            row(
                "RX expired records",
                current(feedback.expired_records.to_string()),
            ),
        ]);
    }
    rows
}

#[cfg(test)]
mod tests {
    use super::*;
    use cu29::clock::CuTime;
    #[test]
    fn rates_use_worker_time_and_do_not_double_count_repeated_snapshots() {
        let mut rates = StreamRates::default();
        let first = LogStreamStats {
            sampled_at: CuTime::from_nanos(1_000_000_000),
            bytes_sent: 1000,
            packets_sent: 10,
            ..Default::default()
        };
        rates.update(first);
        assert_eq!(rates.bytes, None);
        let second = LogStreamStats {
            sampled_at: CuTime::from_nanos(2_000_000_000),
            bytes_sent: 3000,
            packets_sent: 30,
            ..first
        };
        rates.update(second);
        assert_eq!(rates.bytes, Some(2000.0));
        assert_eq!(rates.packets, Some(20.0));
        rates.update(second);
        assert_eq!(rates.bytes, Some(2000.0));
        rates.update(LogStreamStats {
            stopped: true,
            ..second
        });
        assert_eq!(rates.bytes, Some(0.0));
        rates.update(first);
        assert_eq!(rates.bytes, None);
    }
}
