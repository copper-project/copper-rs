//! A caller-owned UI: the receiver only publishes frames and wakes the reader.
use crate::{
    Result,
    receiver::{self, ReceiverOptions},
};
use cu_logstream_demo::telemetry::{Frame, RecordingState, Status};
use cu29_logstream::telemetry::TelemetryReader;
use ratatui::{
    crossterm::event::{self, Event, KeyCode, KeyEventKind, KeyModifiers},
    layout::{Constraint, Layout},
    style::{Color, Style},
    text::Line,
    widgets::{Block, Paragraph, Sparkline},
};
use std::{
    collections::VecDeque,
    io::IsTerminal,
    time::{Duration, Instant},
};

const BUFFER_CAPACITY: usize = 64;
const CHART_CAPACITY: usize = 120;
const UI_TICK: Duration = Duration::from_millis(50);

#[derive(Default)]
struct View {
    paused: bool,
    counter: Option<u64>,
    sum: Option<u64>,
    derived: Option<u64>,
    displayed: Option<u64>,
    missed: u64,
    frame_age: Option<Instant>,
    session: Option<cu29_logstream::StreamIdentity>,
    history: VecDeque<u64>,
}

impl View {
    fn consume(&mut self, reader: &mut TelemetryReader<Frame, Status>) {
        if self.paused {
            return;
        }
        // Bound work per redraw even while the publisher is active.
        for _ in 0..BUFFER_CAPACITY {
            let Some(update) = reader.try_read() else {
                break;
            };
            let frame = update.frame;
            self.missed += update.missed;
            if self.session != Some(frame.identity) {
                self.history.clear();
                self.session = Some(frame.identity);
            }
            self.displayed = Some(frame.copperlist.id);
            self.frame_age = Some(frame.received_at);
            self.counter = frame
                .copperlist
                .msgs
                .get_counter_output()
                .payload()
                .map(|sample| sample.0);
            self.sum = frame
                .copperlist
                .msgs
                .get_sum_output()
                .payload()
                .map(|sample| sample.0);
            self.derived = frame
                .copperlist
                .msgs
                .get_derived_output()
                .payload()
                .map(|sample| sample.0);
            // This history policy belongs to this widget, not the backend.
            if let Some(counter) = self.counter {
                if self.history.len() == CHART_CAPACITY {
                    self.history.pop_front();
                }
                self.history.push_back(counter);
            }
        }
    }

    fn draw(&self, frame: &mut ratatui::Frame<'_>, status: Status, overwritten: u64, path: &str) {
        use cu29_logstream::twin::ReconstructionState;
        let reconstruction = match status.twin.state {
            ReconstructionState::Waiting => "Waiting for anchor",
            ReconstructionState::Recovering => "Recovering",
            ReconstructionState::Reconstructed => "Reconstructed locally",
            ReconstructionState::Verified => "Verified (developer checks)",
            ReconstructionState::Diverged => "DIVERGED",
        };
        let derived = if matches!(
            status.twin.state,
            ReconstructionState::Reconstructed | ReconstructionState::Verified
        ) {
            number(self.derived)
        } else {
            "—".into()
        };
        if frame.area().height < 20 || frame.area().width < 50 {
            let [header, body] =
                Layout::vertical([Constraint::Length(1), Constraint::Min(0)]).areas(frame.area());
            let [title, mascot] =
                Layout::horizontal([Constraint::Min(0), Constraint::Length(4)]).areas(header);
            frame.render_widget(
                Line::from(if self.paused {
                    "VIEW PAUSED"
                } else {
                    "LIVE VIEW"
                }),
                title,
            );
            frame.render_widget(Line::from(" 😼 ").right_aligned(), mascot);
            frame.render_widget(Paragraph::new(format!(
                "Counter: {}   Sum: {}\nDerived: {derived} (local)\nPayload NOT transmitted\n{reconstruction}\nArchived: {}   Gaps: {}\nUI missed: {}\nSpace: pause/resume   q: quit",
                number(self.counter), number(self.sum), status.archived, status.gaps, self.missed,
            )), body);
            return;
        }
        let [header, values, chart, health, footer] = Layout::vertical([
            Constraint::Length(3),
            Constraint::Length(6),
            Constraint::Min(3),
            Constraint::Length(7),
            Constraint::Length(3),
        ])
        .areas(frame.area());
        let recording = match status.state {
            RecordingState::Waiting => "Waiting for robot",
            RecordingState::Recording => "Recording",
            RecordingState::Closed => "Archive closed",
            RecordingState::Failed => "RECEIVER ERROR",
        };
        frame.render_widget(
            Paragraph::new(format!(
                "{recording}  |  {}",
                if self.paused {
                    "VIEW PAUSED"
                } else {
                    "LIVE VIEW"
                }
            ))
            .block(
                Block::bordered()
                    .title("Copper · UDP ground station")
                    .title(Line::from(" 😼 ").right_aligned()),
            )
            .style(Style::default().fg(if self.paused {
                Color::Yellow
            } else {
                Color::Cyan
            })),
            header,
        );
        frame.render_widget(
            Paragraph::new(format!(
                "Counter: {}     Sum: {}\nDerived: {derived} — reconstructed locally; payload not transmitted\n{reconstruction}\nDisplayed CopperList: {}     Frame age: {}",
                number(self.counter),
                number(self.sum),
                number(self.displayed),
                age(self.frame_age)
            ))
            .block(Block::bordered().title("Captured inputs + Copper twin output")),
            values,
        );
        let history: Vec<_> = self.history.iter().copied().collect();
        frame.render_widget(
            Sparkline::default()
                .data(&history)
                .block(Block::bordered().title("Counter · last 120 consumed samples"))
                .style(Style::default().fg(Color::Green)),
            chart,
        );
        frame.render_widget(
            Paragraph::new(vec![
                Line::from(format!(
                    "Packets: {}    Last packet: {}",
                    status.packets,
                    age(status.last_packet)
                )),
                Line::from(format!(
                    "Archived: {}    Latest CL: {}    Verified anchor: {}",
                    status.archived,
                    number(status.latest),
                    number(status.anchor)
                )),
                Line::from(format!(
                    "Source gaps: {}    Replay queue drops: {}",
                    status.gaps, status.twin.queue_overflows
                )),
                Line::from(format!(
                    "UI missed: {}    Buffer overwrites: {}    Capacity: {BUFFER_CAPACITY}",
                    self.missed, overwritten
                )),
                Line::from(format!("Archive: {path}")),
            ])
            .block(Block::bordered().title("Recording continues while the view is paused")),
            health,
        );
        frame.render_widget(Paragraph::new("Space: pause/resume view   q: close receiver\nReceived history may have gaps; an unobserved sender tail is unknown.")
            .block(Block::bordered()), footer);
    }
}

fn number(value: Option<u64>) -> String {
    value.map_or_else(|| "—".into(), |v| v.to_string())
}
fn age(value: Option<Instant>) -> String {
    value.map_or_else(
        || "—".into(),
        |v| format!("{:.1}s", v.elapsed().as_secs_f32()),
    )
}

pub fn run(options: ReceiverOptions) -> Result<()> {
    if !std::io::stdin().is_terminal() || !std::io::stdout().is_terminal() {
        return Err(
            "Dashboard needs a terminal; use the receiver command for headless recording".into(),
        );
    }
    let (mut twin, mut reader, _) = receiver::start(&options)?;
    let ui_result: std::io::Result<()> = ratatui::run(|terminal| {
        let mut view = View {
            history: VecDeque::with_capacity(CHART_CAPACITY),
            ..Default::default()
        };
        let mut redraw = Instant::now();
        loop {
            // Paused/closed views only service keyboard/age timers. Live views
            // wait for backend notification, with the same keyboard deadline.
            if view.paused || reader.is_closed() {
                event::poll(UI_TICK)?;
            } else {
                reader.wait_timeout(UI_TICK);
            }
            while event::poll(Duration::ZERO)? {
                if let Event::Key(key) = event::read()?
                    && key.kind == KeyEventKind::Press
                {
                    match key.code {
                        KeyCode::Char('q') | KeyCode::Esc => return Ok(()),
                        KeyCode::Char('c') if key.modifiers.contains(KeyModifiers::CONTROL) => {
                            return Ok(());
                        }
                        KeyCode::Char(' ') => {
                            view.paused = !view.paused;
                            redraw = Instant::now();
                        }
                        _ => {}
                    }
                }
            }
            view.consume(&mut reader);
            let status = reader.status();
            if Instant::now() >= redraw {
                terminal.draw(|frame| {
                    view.draw(
                        frame,
                        status,
                        reader.overwritten(),
                        &options.log_base.display().to_string(),
                    )
                })?;
                redraw = Instant::now() + UI_TICK;
            }
            if reader.is_closed() && status.state == RecordingState::Failed {
                return Ok(());
            }
        }
    });
    let receiver_result = twin.stop();
    ui_result?;
    receiver_result?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use ratatui::{Terminal, backend::TestBackend};

    #[test]
    fn pause_and_recording_status_are_visible_even_on_small_terminals() {
        for (width, height) in [(100, 26), (30, 8)] {
            let mut terminal = Terminal::new(TestBackend::new(width, height)).unwrap();
            let view = View {
                paused: true,
                missed: 73,
                ..Default::default()
            };
            let status = Status {
                state: RecordingState::Recording,
                archived: 200,
                ..Default::default()
            };
            terminal
                .draw(|frame| view.draw(frame, status, 73, "logs/test.copper"))
                .unwrap();
            let screen: String = terminal
                .backend()
                .buffer()
                .content
                .iter()
                .map(|cell| cell.symbol())
                .collect();
            assert!(screen.contains("VIEW PAUSED"));
            assert!(screen.contains(if width == 100 {
                "payload not transmitted"
            } else {
                "Payload NOT transmitted"
            }));
            if width == 100 {
                assert!(screen.contains("Archived: 200"));
                assert!(screen.contains("UI missed: 73"));
            }
        }
    }
}
