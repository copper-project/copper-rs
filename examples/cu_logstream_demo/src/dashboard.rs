//! A caller-owned UI: the receiver only publishes frames and wakes the reader.
use crate::{
    Result,
    receiver::{self, ReceiverOptions},
};
use cu_logstream_demo::tasks::{ArmPose, FULL_TURN, JointAngles};
use cu_logstream_demo::telemetry::{Frame, RecordingState, Status};
use cu29_logstream::telemetry::TelemetryReader;
use ratatui::{
    crossterm::event::{self, Event, KeyCode, KeyEventKind, KeyModifiers},
    layout::{Constraint, Layout, Rect},
    style::{Color, Modifier, Style},
    symbols::Marker,
    text::{Line, Span},
    widgets::{
        Block, BorderType, Paragraph, Sparkline,
        canvas::{Canvas, Circle, Line as CanvasLine, Points},
    },
};
use std::{
    collections::VecDeque,
    io::IsTerminal,
    time::{Duration, Instant},
};

const BUFFER_CAPACITY: usize = 64;
const CHART_CAPACITY: usize = 120;
const TRAIL_CAPACITY: usize = 1200; // One 12-second loop at 100 Hz, UI storage only.
const UI_TICK: Duration = Duration::from_millis(50);

// Match cu_tuimon's explicit RGB palette and tab/command chrome.
const BG: Color = Color::Rgb(0, 0, 0);
const FG: Color = Color::Rgb(221, 221, 221);
const MUTED: Color = Color::Rgb(118, 118, 118);
const GREEN: Color = Color::Rgb(25, 203, 0);
const CYAN: Color = Color::Rgb(13, 205, 205);
const YELLOW: Color = Color::Rgb(255, 208, 128);
const RED: Color = Color::Rgb(242, 32, 31);
const BAR: Color = Color::Rgb(16, 18, 20);
const ACTIVE: Color = Color::Rgb(56, 110, 120);
const INACTIVE: Color = Color::Rgb(40, 44, 52);

#[derive(Default)]
struct View {
    paused: bool,
    health_tab: bool,
    health_scroll: (u16, u16),
    angles: Option<JointAngles>,
    pose: Option<ArmPose>,
    displayed: Option<u64>,
    missed: u64,
    frame_age: Option<Instant>,
    session: Option<cu29_logstream::StreamIdentity>,
    history: VecDeque<JointAngles>,
    trail: VecDeque<[f64; 2]>,
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
            self.accept(frame);
        }
    }

    fn accept(&mut self, frame: &Frame) {
        if self.session != Some(frame.identity) {
            self.history.clear();
            self.trail.clear();
            self.session = Some(frame.identity);
        } else if self.displayed.and_then(|id| id.checked_add(1)) != Some(frame.copperlist.id) {
            // Never draw an invented trajectory across dropped/missed frames.
            self.trail.clear();
        }
        self.displayed = Some(frame.copperlist.id);
        self.frame_age = Some(frame.received_at);
        self.angles = frame
            .copperlist
            .msgs
            .get_encoders_output()
            .payload()
            .copied();
        self.pose = frame
            .copperlist
            .msgs
            .get_kinematics_output()
            .payload()
            .copied();
        if let Some(angles) = self.angles {
            if self.history.len() == CHART_CAPACITY {
                self.history.pop_front();
            }
            self.history.push_back(angles);
        }
        if let Some(pose) = self.pose {
            if self.trail.len() == TRAIL_CAPACITY {
                self.trail.pop_front();
            }
            self.trail.push_back(pose.tip);
        }
    }

    fn key(&mut self, code: KeyCode) -> bool {
        match code {
            KeyCode::Char('1') => self.health_tab = false,
            KeyCode::Char('2') => self.health_tab = true,
            KeyCode::Tab | KeyCode::BackTab => self.health_tab = !self.health_tab,
            KeyCode::Char(' ') => self.paused = !self.paused,
            KeyCode::Char('j') | KeyCode::Down if self.health_tab => {
                self.health_scroll.0 = self.health_scroll.0.saturating_add(1);
            }
            KeyCode::Char('k') | KeyCode::Up if self.health_tab => {
                self.health_scroll.0 = self.health_scroll.0.saturating_sub(1);
            }
            KeyCode::Char('h') | KeyCode::Left if self.health_tab => {
                self.health_scroll.1 = self.health_scroll.1.saturating_sub(5);
            }
            KeyCode::Char('l') | KeyCode::Right if self.health_tab => {
                self.health_scroll.1 = self.health_scroll.1.saturating_add(5);
            }
            KeyCode::Char('q') | KeyCode::Esc => return true,
            _ => {}
        }
        false
    }

    fn draw(
        &mut self,
        frame: &mut ratatui::Frame<'_>,
        status: Status,
        overwritten: u64,
        path: &str,
    ) {
        use cu29_logstream::twin::ReconstructionState;
        frame.render_widget(
            Block::default().style(Style::default().fg(FG).bg(BG)),
            frame.area(),
        );
        let (reconstruction, twin_color) = match status.twin.state {
            ReconstructionState::Waiting => ("Waiting for recovery point", YELLOW),
            ReconstructionState::Recovering => ("Recovering", YELLOW),
            ReconstructionState::Reconstructed => ("Reconstructed locally", CYAN),
            ReconstructionState::Verified => ("Verified (developer checks)", GREEN),
            ReconstructionState::Diverged => ("DIVERGED", RED),
        };
        let tip = if matches!(
            status.twin.state,
            ReconstructionState::Reconstructed | ReconstructionState::Verified
        ) {
            position(self.pose.map(|pose| pose.tip))
        } else {
            "—".into()
        };
        let (recording, recording_color) = match status.state {
            RecordingState::Waiting => ("Waiting for robot", YELLOW),
            RecordingState::Recording => ("Recording", GREEN),
            RecordingState::Closed => ("Archive closed", MUTED),
            RecordingState::Failed => ("RECEIVER ERROR", RED),
        };
        let view_state = if self.paused {
            "VIEW PAUSED"
        } else {
            "LIVE VIEW"
        };
        let view_color = if self.paused { YELLOW } else { CYAN };
        let [header, body, footer] = Layout::vertical([
            Constraint::Length(1),
            Constraint::Min(0),
            Constraint::Length(1),
        ])
        .areas(frame.area());
        let compact = frame.area().height < 20 || frame.area().width < 70;
        let [tabs, mascot] =
            Layout::horizontal([Constraint::Min(0), Constraint::Length(4)]).areas(header);
        let mut tab_spans = vec![Span::raw(" ")];
        tab_spans.extend(badge(
            "1",
            "LIVE",
            if self.health_tab { INACTIVE } else { ACTIVE },
        ));
        tab_spans.extend(badge(
            "2",
            "HEALTH",
            if self.health_tab { ACTIVE } else { INACTIVE },
        ));
        if !compact {
            tab_spans.push(Span::styled(
                " Copper · UDP ground station",
                Style::default().fg(FG),
            ));
        }
        frame.render_widget(
            Paragraph::new(Line::from(tab_spans)).style(Style::default().bg(BAR)),
            tabs,
        );
        frame.render_widget(
            Line::from(" 😼 ")
                .right_aligned()
                .style(Style::default().bg(BAR)),
            mascot,
        );
        let mut commands = vec![Span::raw(" ")];
        if !compact {
            commands.extend(badge("1-2", "Tabs", Color::Rgb(86, 114, 98)));
        }
        commands.extend(badge(
            "Space",
            if self.paused { "Resume" } else { "Pause" },
            Color::Rgb(92, 102, 150),
        ));
        if self.health_tab && !compact {
            commands.extend(badge("hjkl/↑↓←→", "Scroll", Color::Rgb(92, 102, 150)));
        }
        commands.extend(badge("q", "Quit", Color::Rgb(124, 118, 76)));
        frame.render_widget(
            Paragraph::new(Line::from(commands)).style(Style::default().bg(BAR)),
            footer,
        );

        if self.health_tab {
            let [state, details] =
                Layout::vertical([Constraint::Length(2), Constraint::Min(0)]).areas(body);
            frame.render_widget(
                Paragraph::new(vec![
                    Line::from(vec![
                        value(recording, recording_color),
                        Span::raw("  |  "),
                        value(view_state, view_color),
                    ]),
                    Line::from(value(reconstruction, twin_color)),
                ]),
                state,
            );
            let lines = self.health_lines(status, overwritten, path);
            let block = panel("Stream health · recording continues while paused", CYAN);
            let inner = block.inner(details);
            self.health_scroll.0 = self
                .health_scroll
                .0
                .min(lines.len().saturating_sub(inner.height as usize) as u16);
            self.health_scroll.1 = self.health_scroll.1.min(
                lines
                    .iter()
                    .map(Line::width)
                    .max()
                    .unwrap_or(0)
                    .saturating_sub(inner.width as usize)
                    .min(u16::MAX as usize) as u16,
            );
            frame.render_widget(
                Paragraph::new(lines)
                    .scroll(self.health_scroll)
                    .block(block),
                details,
            );
            return;
        }
        if compact {
            frame.render_widget(
                Paragraph::new(vec![
                    Line::from(vec![
                        value(view_state, view_color),
                        Span::raw(" · "),
                        value(recording, recording_color),
                    ]),
                    Line::from(
                        [
                            metric("Shoulder", angle(self.angles.map(|a| a.shoulder)), GREEN),
                            metric("Elbow", angle(self.angles.map(|a| a.elbow)), GREEN),
                        ]
                        .concat(),
                    ),
                    Line::from(
                        [
                            metric("Tip", tip, CYAN),
                            vec![Span::styled("(local)", Style::default().fg(CYAN))],
                        ]
                        .concat(),
                    ),
                    Line::from(value("Payload NOT transmitted", CYAN)),
                    Line::from(
                        [
                            metric("Archived", status.archived, GREEN),
                            metric("Gaps", status.gaps, warning(status.gaps > 0)),
                        ]
                        .concat(),
                    ),
                    Line::from(value(reconstruction, twin_color)),
                ]),
                body,
            );
            return;
        }
        let [values, charts, health] = Layout::vertical([
            Constraint::Length(6),
            Constraint::Min(3),
            Constraint::Length(4),
        ])
        .areas(body);
        frame.render_widget(
            Paragraph::new(vec![
                Line::from(vec![
                    value(recording, recording_color),
                    Span::raw("  |  "),
                    value(view_state, view_color),
                ]),
                Line::from(
                    [
                        metric("Shoulder", angle(self.angles.map(|a| a.shoulder)), GREEN),
                        metric("Elbow", angle(self.angles.map(|a| a.elbow)), GREEN),
                        metric("Tip", &tip, CYAN),
                    ]
                    .concat(),
                ),
                Line::from(vec![
                    value(reconstruction, twin_color),
                    Span::styled(" · pose payload not transmitted", Style::default().fg(CYAN)),
                ]),
                Line::from(
                    [
                        metric("Displayed CL", number(self.displayed), FG),
                        metric("Frame age", age(self.frame_age), FG),
                    ]
                    .concat(),
                ),
            ])
            .block(panel("Captured inputs + Copper twin output", CYAN)),
            values,
        );
        let [encoder_charts, arm] =
            Layout::horizontal([Constraint::Percentage(40), Constraint::Percentage(60)])
                .areas(charts);
        let [shoulder_chart, elbow_chart] =
            Layout::vertical([Constraint::Percentage(50), Constraint::Percentage(50)])
                .areas(encoder_charts);
        let visible = self.history.iter().skip(
            self.history
                .len()
                .saturating_sub(encoder_charts.width.saturating_sub(2) as usize),
        );
        let shoulder: Vec<_> = visible.clone().map(|a| u64::from(a.shoulder)).collect();
        let elbow: Vec<_> = visible.map(|a| u64::from(a.elbow)).collect();
        for (area, title, samples, reading) in [
            (
                shoulder_chart,
                "Shoulder · received",
                &shoulder,
                self.angles.map(|a| a.shoulder),
            ),
            (
                elbow_chart,
                "Elbow · received",
                &elbow,
                self.angles.map(|a| a.elbow),
            ),
        ] {
            frame.render_widget(
                Sparkline::default()
                    .data(samples)
                    .max(u64::from(FULL_TURN))
                    .block(panel(title, GREEN).title_bottom(format!("Angle: {}", angle(reading))))
                    .style(Style::default().fg(GREEN)),
                area,
            );
        }
        self.draw_arm(
            frame,
            arm,
            if self.paused { YELLOW } else { twin_color },
            &tip,
        );
        frame.render_widget(
            Paragraph::new(vec![
                Line::from(
                    [
                        metric("Archived", status.archived, GREEN),
                        metric("Source gaps", status.gaps, warning(status.gaps > 0)),
                        metric(
                            "Replay drops",
                            status.twin.queue_overflows,
                            warning(status.twin.queue_overflows > 0),
                        ),
                    ]
                    .concat(),
                ),
                Line::from(
                    [
                        metric("UI missed", self.missed, warning(self.missed > 0)),
                        metric("Last packet", age(status.last_packet), FG),
                        vec![Span::styled(
                            "2: stream details",
                            Style::default().fg(YELLOW),
                        )],
                    ]
                    .concat(),
                ),
            ])
            .block(panel("Recording health", MUTED)),
            health,
        );
    }

    fn draw_arm(&self, frame: &mut ratatui::Frame<'_>, area: Rect, color: Color, tip: &str) {
        let block = panel("Robot arm · reconstructed locally", color)
            .title_bottom(format!("Tip: {tip} · meters"));
        let inner = block.inner(area);
        // Terminal cells are approximately twice as tall as they are wide.
        let aspect = f64::from(inner.width.max(1)) / (2.0 * f64::from(inner.height.max(1)));
        let x = 1.85 * aspect.max(1.0);
        let y = 1.85 / aspect.min(1.0);
        let trail: Vec<_> = self.trail.iter().map(|p| (p[0], p[1])).collect();
        frame.render_widget(
            Canvas::default()
                .block(block)
                .background_color(BG)
                .marker(Marker::Braille)
                .x_bounds([-x, x])
                .y_bounds([-y, y])
                .paint(|ctx| {
                    // Fade old recorded points. No interpolation or forward kinematics in the UI.
                    let chunk_size = trail.len().div_ceil(3).max(1);
                    for (index, points) in trail.chunks(chunk_size).enumerate() {
                        ctx.draw(&Points {
                            coords: points,
                            color: [Color::Rgb(0, 65, 65), Color::Rgb(0, 115, 115), CYAN][index],
                        });
                    }
                    ctx.layer();
                    if let Some(pose) = self.pose {
                        ctx.draw(&CanvasLine {
                            x1: 0.0,
                            y1: 0.0,
                            x2: pose.elbow[0],
                            y2: pose.elbow[1],
                            color,
                        });
                        ctx.draw(&CanvasLine {
                            x1: pose.elbow[0],
                            y1: pose.elbow[1],
                            x2: pose.tip[0],
                            y2: pose.tip[1],
                            color,
                        });
                        for point in [[0.0, 0.0], pose.elbow, pose.tip] {
                            ctx.draw(&Circle {
                                x: point[0],
                                y: point[1],
                                radius: 0.04,
                                color: FG,
                            });
                        }
                    }
                    ctx.print(
                        -x + 0.1,
                        y - 0.2,
                        Span::styled("Only joint angles transmitted", Style::default().fg(CYAN)),
                    );
                    if tip == "—" {
                        ctx.print(
                            -x + 0.1,
                            -y + 0.2,
                            Span::styled("Waiting · last pose held", Style::default().fg(color)),
                        );
                    }
                }),
            area,
        );
    }

    fn health_lines(&self, status: Status, overwritten: u64, path: &str) -> Vec<Line<'static>> {
        vec![
            Line::from(metric("Packets", status.packets, CYAN)),
            Line::from(metric("Last packet", age(status.last_packet), FG)),
            Line::from(metric("Archived", status.archived, GREEN)),
            Line::from(metric("Latest CL", number(status.latest), GREEN)),
            Line::from(metric(
                "Verified recovery point",
                number(status.recovery_point),
                CYAN,
            )),
            Line::from(metric("Source gaps", status.gaps, warning(status.gaps > 0))),
            Line::from(metric(
                "Reconstructed frames",
                status.twin.reconstructed,
                CYAN,
            )),
            Line::from(metric("Verified frames", status.twin.verified, GREEN)),
            Line::from(metric(
                "Divergences",
                status.twin.divergences,
                if status.twin.divergences > 0 {
                    RED
                } else {
                    GREEN
                },
            )),
            Line::from(metric(
                "Replay queue drops",
                status.twin.queue_overflows,
                warning(status.twin.queue_overflows > 0),
            )),
            Line::from(metric("UI missed", self.missed, warning(self.missed > 0))),
            Line::from(metric(
                "Buffer overwrites",
                overwritten,
                warning(overwritten > 0),
            )),
            Line::from(metric("Buffer capacity", BUFFER_CAPACITY, FG)),
            Line::from(metric("Archive", path, FG)),
            Line::from(""),
            Line::from(Span::styled(
                "Received history may have gaps; an unobserved sender tail is unknown.",
                Style::default().fg(MUTED),
            )),
        ]
    }
}

fn panel(title: &str, color: Color) -> Block<'_> {
    Block::bordered()
        .border_type(BorderType::Rounded)
        .border_style(Style::default().fg(MUTED))
        .title(Span::styled(
            title,
            Style::default().fg(color).add_modifier(Modifier::BOLD),
        ))
}

fn value(text: impl ToString, color: Color) -> Span<'static> {
    Span::styled(
        text.to_string(),
        Style::default().fg(color).add_modifier(Modifier::BOLD),
    )
}

fn metric(label: &str, data: impl ToString, color: Color) -> Vec<Span<'static>> {
    vec![
        Span::styled(format!("{label}: "), Style::default().fg(FG)),
        value(data, color),
        Span::raw("   "),
    ]
}

fn warning(present: bool) -> Color {
    if present { YELLOW } else { GREEN }
}

fn badge(key: &str, label: &str, bg: Color) -> Vec<Span<'static>> {
    vec![
        Span::styled("", Style::default().fg(bg).bg(BAR)),
        Span::styled(
            format!(" {key} "),
            Style::default()
                .fg(YELLOW)
                .bg(bg)
                .add_modifier(Modifier::BOLD),
        ),
        Span::styled(format!("{label} "), Style::default().fg(FG).bg(bg)),
        Span::styled("", Style::default().fg(bg).bg(BAR)),
        Span::styled(" ", Style::default().bg(BAR)),
    ]
}

fn angle(value: Option<u16>) -> String {
    value.map_or_else(|| "—".into(), |v| format!("{:.1}°", f64::from(v) / 100.0))
}
fn position(value: Option<[f64; 2]>) -> String {
    value.map_or_else(|| "—".into(), |p| format!("({:+.2}, {:+.2})", p[0], p[1]))
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
            trail: VecDeque::with_capacity(TRAIL_CAPACITY),
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
                    if key.code == KeyCode::Char('c')
                        && key.modifiers.contains(KeyModifiers::CONTROL)
                        || view.key(key.code)
                    {
                        return Ok(());
                    }
                    redraw = Instant::now();
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
    fn trail_uses_received_poses_and_breaks_at_gaps_and_new_sessions() {
        let mut frame = Frame {
            identity: cu29_logstream::StreamIdentity {
                session_id: [1; 16],
                sender_id: 41,
            },
            received_at: Instant::now(),
            copperlist: cu_logstream_demo::List::default(),
        };
        // Deliberately unrelated angles/positions: the UI must draw the task output.
        frame
            .copperlist
            .msgs
            .0
            .0
            .set_payload(JointAngles::default());
        frame.copperlist.msgs.0.1.set_payload(ArmPose {
            elbow: [0.0, 1.0],
            tip: [0.5, 1.0],
        });
        let mut view = View::default();
        view.accept(&frame);
        frame.copperlist.id = 1;
        view.accept(&frame);
        assert_eq!(view.trail, [[0.5, 1.0]; 2]);
        frame.copperlist.id = 3;
        view.accept(&frame);
        assert_eq!(view.trail.len(), 1);
        frame.identity.session_id = [2; 16];
        view.accept(&frame);
        assert_eq!(view.trail.len(), 1);
        assert_eq!(view.history.len(), 1);
        for id in 4..1300 {
            frame.copperlist.id = id;
            view.accept(&frame);
        }
        assert_eq!(view.trail.len(), TRAIL_CAPACITY);
        assert_eq!(view.history.len(), CHART_CAPACITY);
    }

    #[test]
    fn tabs_scroll_and_pause_preserve_the_live_view() {
        let mut view = View {
            pose: Some(ArmPose {
                elbow: [1.0, 0.0],
                tip: [1.65, 0.0],
            }),
            trail: VecDeque::from([[1.65, 0.0]]),
            ..Default::default()
        };
        assert!(!view.key(KeyCode::Char('2')));
        assert!(view.health_tab);
        view.key(KeyCode::Char(' '));
        view.key(KeyCode::Char('j'));
        view.key(KeyCode::Right);
        assert!(view.paused);
        assert_eq!(view.health_scroll, (1, 5));
        view.key(KeyCode::Up);
        view.key(KeyCode::Char('h'));
        assert_eq!(view.health_scroll, (0, 0));
        view.key(KeyCode::Tab);
        assert!(!view.health_tab);
        assert_eq!(view.pose.unwrap().tip, [1.65, 0.0]);
        assert_eq!(view.trail, [[1.65, 0.0]]);
        view.key(KeyCode::BackTab);
        view.key(KeyCode::Char('1'));
        assert!(!view.health_tab);
        assert!(view.key(KeyCode::Char('q')));
    }

    #[test]
    fn health_colors_errors_and_scrolls_to_archive_on_small_terminals() {
        let mut terminal = Terminal::new(TestBackend::new(50, 12)).unwrap();
        let mut view = View {
            health_tab: true,
            ..Default::default()
        };
        let mut status = Status {
            state: RecordingState::Failed,
            gaps: 3,
            ..Default::default()
        };
        status.twin.state = cu29_logstream::twin::ReconstructionState::Diverged;
        terminal
            .draw(|frame| view.draw(frame, status, 0, "logs/test.copper"))
            .unwrap();
        let buffer = terminal.backend().buffer();
        assert_eq!(buffer[(0, 1)].fg, RED);
        assert_eq!(buffer[(0, 2)].fg, RED);
        assert_eq!(buffer[(49, 5)].bg, BG);
        for _ in 0..20 {
            view.key(KeyCode::Down);
        }
        terminal
            .draw(|frame| view.draw(frame, status, 0, "logs/test.copper"))
            .unwrap();
        let screen: String = terminal
            .backend()
            .buffer()
            .content
            .iter()
            .map(|cell| cell.symbol())
            .collect();
        assert!(screen.contains("Archive: logs/test.copper"));
        assert!(screen.contains("Quit"));
    }

    #[test]
    fn reconstructed_arm_is_drawn_on_the_right() {
        let mut terminal = Terminal::new(TestBackend::new(100, 26)).unwrap();
        let mut view = View {
            pose: Some(ArmPose {
                elbow: [1.0, 0.0],
                tip: [1.0, 0.65],
            }),
            trail: VecDeque::from([[1.1, 0.5], [1.0, 0.65]]),
            ..Default::default()
        };
        let mut status = Status {
            state: RecordingState::Recording,
            ..Default::default()
        };
        status.twin.state = cu29_logstream::twin::ReconstructionState::Reconstructed;
        terminal
            .draw(|frame| view.draw(frame, status, 0, "logs/test.copper"))
            .unwrap();
        let buffer = terminal.backend().buffer();
        let right: String = (7..21)
            .flat_map(|y| (40..100).map(move |x| buffer[(x, y)].symbol()))
            .collect();
        assert!(right.contains("Robot arm · reconstructed locally"));
        assert!(right.contains("Tip: (+1.00, +0.65)"));
        let left: String = (7..21)
            .flat_map(|y| (0..40).map(move |x| buffer[(x, y)].symbol()))
            .collect();
        assert!(left.contains("Shoulder · received"));
        assert!(left.contains("Angle: —"));
        assert!(
            right
                .chars()
                .any(|c| ('\u{2801}'..='\u{28ff}').contains(&c))
        );
        assert!(right.contains("Only joint angles transmitted"));
    }

    #[test]
    fn pause_and_recording_status_are_visible_even_on_small_terminals() {
        for (width, height) in [(100, 26), (30, 8)] {
            let mut terminal = Terminal::new(TestBackend::new(width, height)).unwrap();
            let mut view = View {
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
