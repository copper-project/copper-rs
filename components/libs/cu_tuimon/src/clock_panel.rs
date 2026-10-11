//! Live clock health, sampled and formatted only by the UI thread.

use crate::palette;
use crate::ui::MonitorUi;
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
                SyncState::Locked => ("LOCKED", palette::GREEN),
                SyncState::Holdover => ("HOLDOVER", palette::YELLOW),
                SyncState::Expired => ("EXPIRED", palette::LIGHT_RED),
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
            rows.extend([
                row("Origin", "External reference".into()),
                row(
                    "Error bound",
                    format!(
                        "{:.3} ms",
                        status.estimated_error.as_nanos() as f64 / 1_000_000.0
                    ),
                ),
                row(
                    "Error budget",
                    format!("{:.3} ms", status.max_error.as_nanos() as f64 / 1_000_000.0),
                ),
                row("Phase offset", format!("{} ns", status.phase_offset_ns)),
                row(
                    "Drift",
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
        let table = Table::new(rows, [Constraint::Length(22), Constraint::Min(0)])
            .header(Row::new(["Metric", "Live value"]).style(Style::default().fg(palette::YELLOW)))
            .column_spacing(2)
            .block(block);
        frame.render_widget(table, area);
    }
}
