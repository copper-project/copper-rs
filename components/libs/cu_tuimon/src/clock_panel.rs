//! Live clock health, sampled and formatted only by the UI thread.

use crate::palette;
use crate::ui::MonitorUi;
use cu29::clock::CuDuration;
use cu29::clock::sync::SyncState;
use ratatui::Frame;
use ratatui::layout::Constraint;
use ratatui::layout::Rect;
use ratatui::style::Style;
use ratatui::widgets::Block;
use ratatui::widgets::BorderType;
use ratatui::widgets::Borders;
use ratatui::widgets::Cell;
use ratatui::widgets::Paragraph;
use ratatui::widgets::Row;
use ratatui::widgets::Table;

impl MonitorUi {
    pub(super) fn draw_clock(&self, frame: &mut Frame, area: Rect) {
        let block = Block::default()
            .borders(Borders::ALL)
            .border_type(BorderType::Rounded)
            .title(" RobotClock ");
        let Some(clock) = self.model().clock() else {
            frame.render_widget(Paragraph::new("No clock attached").block(block), area);
            return;
        };
        let now = clock.now().as_nanos();
        let row =
            |name: &'static str, value: String| Row::new([Cell::from(name), Cell::from(value)]);
        let mut rows = vec![row("Robot time (ns)", now.to_string())];
        if let Some(status) = clock.sync_status() {
            let (label, color) = match status.state {
                SyncState::Acquiring => ("ACQUIRING", palette::YELLOW),
                SyncState::Locked => ("LOCKED (within allowed uncertainty)", palette::GREEN),
                SyncState::Holdover => ("HOLDOVER (reference unavailable)", palette::YELLOW),
                SyncState::Expired => ("EXPIRED (limits exceeded; reacquire)", palette::LIGHT_RED),
            };
            rows.insert(
                0,
                row("State", label.into()).style(Style::default().fg(color)),
            );
            let identity = status
                .domain
                .identity
                .map(|byte| format!("{byte:02x}"))
                .join(":");
            rows.push(row("Origin", "External reference".into()));
            if status.estimated_error != CuDuration::MAX {
                let phase = status.phase_offset_ns.unsigned_abs();
                let direction = match status.phase_offset_ns.cmp(&0) {
                    std::cmp::Ordering::Greater => "behind",
                    std::cmp::Ordering::Less => "ahead",
                    std::cmp::Ordering::Equal => "aligned",
                };
                // Undo the combined bound using the same integer rounding as
                // ClockSnapshot::status. All calculations stay on the UI thread.
                let aging = (u128::from(status.sample_age.as_nanos())
                    * u128::from(status.drift_bound_ppb))
                .div_ceil(1_000_000_000)
                .min(u128::from(u64::MAX)) as u64;
                let reference = status
                    .estimated_error
                    .as_nanos()
                    .saturating_sub(phase)
                    .saturating_sub(aging);
                rows.extend([
                    row(
                        "Robot vs reference",
                        format!("{:.3} µs {direction} (estimate)", phase as f64 / 1000.0),
                    ),
                    row(
                        "Uncertainty bound",
                        format!(
                            "≤ {:.3} ms (conservative total)\nIncludes allowances below; not measured error",
                            status.estimated_error.as_nanos() as f64 / 1_000_000.0
                        ),
                    )
                    .height(2),
                    row(
                        "  Reference + capture",
                        format!("{:.3} ms (reported/configured bound)", reference as f64 / 1_000_000.0),
                    ),
                    row(
                        "  Sample aging",
                        format!(
                            "{:.3} µs ({:.0} ppm allowance)",
                            aging as f64 / 1000.0,
                            f64::from(status.drift_bound_ppb) / 1000.0
                        ),
                    ),
                    row(
                        "  Clock mismatch",
                        format!("{:.3} µs (offset magnitude)", phase as f64 / 1000.0),
                    ),
                ]);
            } else {
                rows.push(row("Robot vs reference", "Unavailable".into()));
                rows.push(row(
                    "Uncertainty bound",
                    "No usable reference sample".into(),
                ));
            }
            rows.extend([
                row(
                    "Allowed uncertainty",
                    format!(
                        "{:.3} ms (configured maximum)\nAbove limit: LOCKED/HOLDOVER expires",
                        status.max_error.as_nanos() as f64 / 1_000_000.0
                    ),
                )
                .height(2),
                row(
                    "Estimated drift",
                    format!("{:.3} ppm", status.drift_ppb as f64 / 1000.0),
                ),
                row(
                    "Sample age",
                    format!(
                        "{:.1} ms",
                        status.sample_age.as_nanos() as f64 / 1_000_000.0
                    ),
                ),
                row("Reference identity", identity),
                row(
                    "Domain / session",
                    format!("{} / {}", status.domain.id, status.domain.session),
                ),
            ]);
        } else {
            rows.insert(
                0,
                row("State", "LOCAL".into()).style(Style::default().fg(palette::GREEN)),
            );
            rows.push(row(
                "Origin",
                if clock.is_mock() {
                    "Mock counter"
                } else {
                    "Local monotonic counter"
                }
                .into(),
            ));
        }
        rows.push(row(
            "Last CopperList",
            self.model()
                .last_seen_copperlist_id()
                .map_or_else(|| "Waiting for first iteration".into(), |id| id.to_string()),
        ));
        rows.push(row("Task timestamps", "ctx.now() shares this clock".into()));
        let table = Table::new(rows, [Constraint::Length(24), Constraint::Min(0)])
            .header(Row::new(["Metric", "Live value"]).style(Style::default().fg(palette::YELLOW)))
            .column_spacing(2)
            .block(block);
        frame.render_widget(table, area);
    }
}
