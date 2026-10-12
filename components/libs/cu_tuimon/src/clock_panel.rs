//! Live clock health, sampled and formatted only by the UI thread.

use crate::palette;
use crate::ui::MonitorUi;
use chrono::DateTime;
use chrono::Local;
use chrono::SecondsFormat;
use chrono::Utc;
use cu29::clock::ClockEpoch;
use cu29::clock::ClockSource;
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
use ratatui::widgets::StatefulWidget;
use ratatui::widgets::Table;
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
        let origin = clock.origin();
        let mut rows = vec![
            row("Robot time", format!("{now} ns")),
            row("Source", source_label(origin.source)),
            row("Epoch", epoch_label(origin.epoch)),
        ];
        if let ClockEpoch::Tai {
            tai_minus_utc_seconds,
        } = origin.epoch
        {
            rows.push(row(
                "TAI − UTC",
                tai_minus_utc_seconds
                    .map_or_else(|| "Unavailable".into(), |seconds| format!("{seconds:+} s")),
            ));
            rows.push(row(
                "Local time",
                tai_local_time(now, origin.epoch).unwrap_or_else(|| "Unavailable".into()),
            ));
        }
        let mut uncertainty_rows = Vec::new();
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
                    "Offset from reference",
                    format!("{:+.3} µs", -(status.phase_offset_ns as f64) / 1000.0),
                ));
                uncertainty_rows.extend([
                    row(
                        "Uncertainty bound",
                        format!(
                            "≤ {:.3} ms",
                            status.estimated_error.as_nanos() as f64 / 1_000_000.0
                        ),
                    ),
                    row(
                        "Reference uncertainty",
                        format!("{:.3} ms", reference as f64 / 1_000_000.0),
                    ),
                    row(
                        "Aging uncertainty",
                        format!("{:.3} µs", aging as f64 / 1000.0),
                    ),
                    row(
                        "Offset magnitude",
                        format!("{:.3} µs", phase as f64 / 1000.0),
                    ),
                ]);
            } else {
                rows.push(row("Offset from reference", "Unavailable".into()));
                uncertainty_rows.push(row("Uncertainty bound", "Unavailable".into()));
            }
            uncertainty_rows.push(row(
                "Drift bound",
                format!("{:.3} ppm", f64::from(status.drift_bound_ppb) / 1000.0),
            ));
            uncertainty_rows.push(row("", "".into()));
            uncertainty_rows.push(row(
                "Uncertainty limit",
                format!("{:.3} ms", status.max_error.as_nanos() as f64 / 1_000_000.0),
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
        }
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
            scroll.render_widget(table(uncertainty_rows, "Bound / limit"), uncertainty_area);
        } else {
            scroll.render_widget(Paragraph::new("Unavailable"), uncertainty_area);
        }
        scroll.render(area, frame.buffer_mut(), &mut self.clock_scroll_state);
    }
}

// Only draw_clock calls this conversion; other tabs and task execution never do.
fn tai_local_time(nanos: u64, epoch: ClockEpoch) -> Option<String> {
    let ClockEpoch::Tai {
        tai_minus_utc_seconds: Some(offset),
    } = epoch
    else {
        return None;
    };
    let utc_seconds = i64::try_from(nanos / 1_000_000_000)
        .ok()?
        .checked_sub(i64::from(offset))?;
    let utc = DateTime::<Utc>::from_timestamp(utc_seconds, 0)?;
    Some(
        utc.with_timezone(&Local)
            .to_rfc3339_opts(SecondsFormat::Secs, false),
    )
}

fn source_label(source: ClockSource) -> String {
    match source {
        ClockSource::LocalMonotonic => "Local monotonic counter".into(),
        ClockSource::Mock => "Mock counter".into(),
        ClockSource::LocalRtc => "Local RTC".into(),
        ClockSource::Gnss => "GNSS".into(),
        ClockSource::AtomicClock => "Atomic clock".into(),
        ClockSource::TerrestrialRadio => "Terrestrial radio".into(),
        ClockSource::Ptp => "PTP".into(),
        ClockSource::Ntp => "NTP".into(),
        ClockSource::HandSet => "Hand set".into(),
        ClockSource::InternalOscillator => "Internal oscillator".into(),
        ClockSource::Other(code) => format!("Other 0x{code:02x}"),
        ClockSource::Unknown => "Unknown".into(),
    }
}

fn epoch_label(epoch: ClockEpoch) -> String {
    match epoch {
        ClockEpoch::Local => "Local".into(),
        ClockEpoch::Tai { .. } => "TAI".into(),
        ClockEpoch::Gps => "GPS".into(),
        ClockEpoch::Utc => "UTC".into(),
        ClockEpoch::Other(id) => format!("Other {id}"),
        ClockEpoch::Unknown => "Unknown".into(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_tai_date_uses_reference_offset_and_local_timezone_to_seconds() {
        let epoch = ClockEpoch::Tai {
            tai_minus_utc_seconds: Some(37),
        };
        let tai = 1_800_000_037_987_654_321;
        let displayed = tai_local_time(tai, epoch).unwrap();
        let parsed = DateTime::parse_from_rfc3339(&displayed).unwrap();
        assert_eq!(parsed.timestamp(), 1_800_000_000);
        assert_eq!(parsed.timestamp_subsec_nanos(), 0);
        let expected = DateTime::<Utc>::from_timestamp(1_800_000_000, 0)
            .unwrap()
            .with_timezone(&Local);
        assert_eq!(
            parsed.offset().local_minus_utc(),
            expected.offset().local_minus_utc()
        );
        assert_eq!(displayed.len(), 25);
        assert!(!displayed.contains('.'));
    }

    #[test]
    fn test_calendar_conversion_requires_tai_with_a_known_offset() {
        for epoch in [
            ClockEpoch::Local,
            ClockEpoch::Gps,
            ClockEpoch::Utc,
            ClockEpoch::Unknown,
            ClockEpoch::Other(42),
            ClockEpoch::Tai {
                tai_minus_utc_seconds: None,
            },
        ] {
            assert_eq!(tai_local_time(1_800_000_000_000_000_000, epoch), None);
        }
        assert_eq!(
            tai_local_time(
                0,
                ClockEpoch::Tai {
                    tai_minus_utc_seconds: Some(37)
                }
            )
            .unwrap(),
            DateTime::<Utc>::from_timestamp(-37, 0)
                .unwrap()
                .with_timezone(&Local)
                .to_rfc3339_opts(SecondsFormat::Secs, false)
        );
    }
}
