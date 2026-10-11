//! Live clock health, sampled and formatted only by the UI thread.

use crate::palette;
use crate::ui::MonitorUi;
use cu29::clock::CuDuration;
use cu29::clock::sync::SyncState;
use ratatui::Frame;
use ratatui::layout::{Constraint, Layout, Position, Rect, Size};
use ratatui::style::Style;
use ratatui::widgets::Block;
use ratatui::widgets::BorderType;
use ratatui::widgets::Borders;
use ratatui::widgets::Cell;
use ratatui::widgets::Paragraph;
use ratatui::widgets::Row;
use ratatui::widgets::Table;
use ratatui::widgets::{StatefulWidget, Wrap};
use tui_widgets::scrollview::ScrollView;

impl MonitorUi {
    pub(super) fn draw_clock(&mut self, frame: &mut Frame, area: Rect) {
        // Keep both topics side by side, with the same scrolling behavior as BW.
        let size = Size::new(area.width.max(128), area.height.max(18));
        let offset = self.clock_scroll_state.offset();
        self.clock_scroll_state.set_offset(Position::new(
            offset.x.min(size.width.saturating_sub(area.width)),
            offset.y.min(size.height.saturating_sub(area.height)),
        ));
        let mut scroll = ScrollView::new(size);
        scroll.render_widget(
            Block::default().style(Style::default().bg(palette::BACKGROUND)),
            Rect::new(0, 0, size.width, size.height),
        );
        let panels = Layout::horizontal([Constraint::Fill(1); 2]).split(Rect::new(
            0,
            0,
            size.width,
            size.height,
        ));
        let block = |title| {
            Block::default()
                .borders(Borders::ALL)
                .border_type(BorderType::Rounded)
                .title(title)
        };
        let clock_block = block(" Clock / reference ");
        let uncertainty_block = block(" Timestamp uncertainty ");
        let uncertainty_area = uncertainty_block.inner(panels[1]);
        scroll.render_widget(uncertainty_block, panels[1]);
        let Some(clock) = self.model().clock() else {
            scroll.render_widget(
                Paragraph::new("No clock attached").block(clock_block),
                panels[0],
            );
            scroll.render_widget(Paragraph::new("No synchronization data"), uncertainty_area);
            scroll.render(area, frame.buffer_mut(), &mut self.clock_scroll_state);
            return;
        };
        let now = clock.now().as_nanos();
        let row =
            |name: &'static str, value: String| Row::new([Cell::from(name), Cell::from(value)]);
        let mut rows = vec![row("Robot time (ns)", now.to_string())];
        let mut uncertainty_rows = Vec::new();
        if let Some(status) = clock.sync_status() {
            let (label, color) = match status.state {
                SyncState::Acquiring => ("ACQUIRING", palette::YELLOW),
                SyncState::Locked => ("LOCKED", palette::GREEN),
                SyncState::Holdover => ("HOLDOVER (reference unavailable)", palette::YELLOW),
                SyncState::Expired => ("EXPIRED (reacquire reference)", palette::LIGHT_RED),
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
                rows.push(row(
                    "Robot vs reference",
                    format!("{:+.3} µs", -(status.phase_offset_ns as f64) / 1000.0),
                ));
                uncertainty_rows.extend([
                    row(
                        "Uncertainty bound",
                        format!(
                            "≤ {:.3} ms (conservative total)",
                            status.estimated_error.as_nanos() as f64 / 1_000_000.0
                        ),
                    ),
                    row(
                        "Reference + capture",
                        format!(
                            "{:.3} ms (reported/configured bound)",
                            reference as f64 / 1_000_000.0
                        ),
                    ),
                    row(
                        "Sample aging",
                        format!(
                            "{:.3} µs ({:.0} ppm allowance)",
                            aging as f64 / 1000.0,
                            f64::from(status.drift_bound_ppb) / 1000.0
                        ),
                    ),
                    row(
                        "Clock mismatch",
                        format!("{:.3} µs (offset magnitude)", phase as f64 / 1000.0),
                    ),
                ]);
            } else {
                rows.push(row("Robot vs reference", "Unavailable".into()));
                uncertainty_rows.push(row(
                    "Uncertainty bound",
                    "No usable reference sample".into(),
                ));
            }
            uncertainty_rows.push(row("", "".into()));
            uncertainty_rows.push(row(
                "Allowed uncertainty",
                format!(
                    "{:.3} ms (configured maximum)",
                    status.max_error.as_nanos() as f64 / 1_000_000.0
                ),
            ));
            rows.extend([
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
        rows.push(row("Task timestamps", "ctx.now() shares this clock".into()));
        let table = |rows, value_label| {
            Table::new(rows, [Constraint::Length(22), Constraint::Min(0)])
                .header(
                    Row::new(["Metric", value_label])
                        .style(Style::default().fg(palette::YELLOW))
                        .bottom_margin(1),
                )
                .column_spacing(2)
        };
        scroll.render_widget(table(rows, "Live value").block(clock_block), panels[0]);
        if !uncertainty_rows.is_empty() {
            let sections = Layout::vertical([Constraint::Length(9), Constraint::Min(0)])
                .split(uncertainty_area);
            scroll.render_widget(table(uncertainty_rows, "Bound / limit"), sections[0]);
            scroll.render_widget(
                Paragraph::new("Allowed uncertainty is a configured maximum.\nAbove limit: LOCKED/HOLDOVER expires.\nSample-age limits also apply.")
                    .wrap(Wrap { trim: false }),
                sections[1],
            );
        } else {
            scroll.render_widget(
                Paragraph::new("Local clock.\nNo external synchronization configured.")
                    .wrap(Wrap { trim: false }),
                uncertainty_area,
            );
        }
        scroll.render(area, frame.buffer_mut(), &mut self.clock_scroll_state);
    }
}
