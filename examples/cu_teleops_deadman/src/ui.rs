//! Terminal controls for a single deadman measurement.
use cu29::clock::{CuDuration, CuInstant};
use ratatui::crossterm::{
    event::{
        self, DisableMouseCapture, EnableMouseCapture, Event, KeyCode, KeyEventKind, KeyModifiers,
        MouseButton, MouseEventKind,
    },
    execute,
};
use ratatui::prelude::*;
use ratatui::widgets::Paragraph;
use std::io;

pub enum Action {
    None,
    Trip,
    Resume,
    Quit,
}

pub struct Screen {
    terminal: ratatui::DefaultTerminal,
    button: Rect,
    resume: bool,
    last_draw: Option<CuInstant>,
}

// The cap rows use partial blocks against the terminal background. The full
// middle rows use the same fill behind the lettering, leaving no outline seams.
fn draw_button(frame: &mut Frame<'_>, area: Rect, resume: bool) {
    let fill = if resume {
        Color::Rgb(18, 130, 79)
    } else {
        Color::Rgb(197, 32, 51)
    };
    let rounded = area.width >= 6 && area.height >= 3;
    for row in area.rows() {
        // Quarter discs meet the quarter-height caps and half-width sides.
        let edges = if !rounded {
            None
        } else if row.top() == area.top() {
            Some(('🯮', '▂', '🯭'))
        } else if row.bottom() == area.bottom() {
            Some(('🯬', '🮂', '🯯'))
        } else {
            Some(('▐', ' ', '▌'))
        };
        for col in row.columns() {
            let cell = &mut frame.buffer_mut()[(col.x, row.y)];
            let glyph = match edges {
                Some((left, _, _)) if col.x == area.left() => left,
                Some((_, _, right)) if col.right() == area.right() => right,
                Some((_, middle, _)) => middle,
                None => ' ',
            };
            if glyph == ' ' {
                cell.set_bg(fill);
            } else {
                cell.set_char(glyph).set_fg(fill);
            }
        }
    }

    let face = area.inner(Margin::new(u16::from(area.width >= 6) * 2, 0));
    let (text, lines) = if resume && face.width >= 25 && area.height >= 5 {
        (
            "█▀█ █▀▀ █▀▀ █ █ █▄ ▄█ █▀▀\n█▀▄ █▀  ▀▀█ █ █ █ ▀ █ █▀ \n█ █ █▄▄ ▄▄█ █▄█ █   █ █▄▄",
            3,
        )
    } else if resume {
        ("RESUME", 1)
    } else if face.width >= 24 && area.height >= 5 {
        (
            "█▀▀▀  ▀▀█▀▀  █▀▀█  █▀▀█\n▀▀▀█    █    █  █  █▀▀▀\n▄▄▄█    █    █▄▄█  █   ",
            3,
        )
    } else {
        ("STOP", 1)
    };
    let label = Rect::new(
        face.x,
        face.y + face.height.saturating_sub(lines) / 2,
        face.width,
        area.height.min(lines),
    );
    frame.render_widget(Paragraph::new(text).white().centered(), label);
}

impl Screen {
    pub fn open() -> io::Result<Self> {
        let terminal = ratatui::try_init()?;
        let screen = Self {
            terminal,
            button: Rect::default(),
            resume: false,
            last_draw: None,
        };
        execute!(io::stdout(), EnableMouseCapture)?;
        Ok(screen)
    }

    pub fn draw(
        &mut self,
        now: CuInstant,
        twinned_cl: Option<u64>,
        link: crate::latency::LinkStatus,
        status: &str,
        resume: bool,
        result: Option<crate::latency::Latency>,
    ) -> io::Result<()> {
        if self.resume == resume
            && self
                .last_draw
                .is_some_and(|last| now - last < CuDuration::from_millis(40))
        {
            return Ok(());
        }
        self.resume = resume;
        self.terminal.draw(|frame| {
            let area = frame.area();
            let height = area.height.min(16);
            let width = area.width.min(56);
            let panel = Rect::new(
                (area.width - width) / 2,
                (area.height - height) / 2,
                width,
                height,
            );
            let [title, state, _above, slot, _below, timing, keys] = Layout::vertical([
                Constraint::Length(1),
                Constraint::Length(1),
                Constraint::Length(1),
                Constraint::Fill(1),
                Constraint::Length(1),
                Constraint::Length(2),
                Constraint::Length(1),
            ])
            .areas(panel);
            let width = slot.width.saturating_sub(2).min(31);
            self.button = Rect::new(
                slot.x + (slot.width - width) / 2,
                slot.y,
                width,
                slot.height,
            );
            let columns = Layout::horizontal([Constraint::Fill(1), Constraint::Length(18)]);
            let [identity, rtt] = columns.areas(title);
            let mut header = Vec::new();
            if identity.width >= 30 {
                header.extend(["DEADMAN".bold(), "  ·  ".into()]);
            }
            header.push("Twin CL ".into());
            header.push(
                twinned_cl
                    .map_or_else(|| "—".into(), |id| id.to_string())
                    .into(),
            );
            let reading = match link.rtt {
                Some(value) => format!("{:8.3}", value.as_nanos() as f64 / 1_000_000.0),
                None => format!("{:>8}", "—"),
            };
            frame.render_widget(Paragraph::new(Line::from(header)), identity);
            frame.render_widget(Paragraph::new(format!("· RTT {reading} ms")), rtt);
            let [robot, clock] = columns.areas(state);
            frame.render_widget(Paragraph::new(status), robot);
            let clock_status = if link.synced {
                "Clock synced".green()
            } else {
                "Clock acquiring".yellow()
            };
            frame.render_widget(
                Paragraph::new(Line::from(vec!["· ".into(), clock_status])),
                clock,
            );
            draw_button(frame, self.button, resume);
            if let Some(elapsed) = result {
                frame.render_widget(
                    Paragraph::new(elapsed.to_string()).bold().centered(),
                    timing,
                );
            }
            frame.render_widget(
                Paragraph::new(if resume {
                    "Click or Enter to resume    Q to quit"
                } else {
                    "Click or Enter to stop    Q to quit"
                })
                .centered(),
                keys,
            );
        })?;
        self.last_draw = Some(now);
        Ok(())
    }

    pub fn input(&mut self) -> io::Result<Action> {
        if !event::poll(CuDuration::MIN.into())? {
            return Ok(Action::None);
        }
        let activate = if self.resume {
            Action::Resume
        } else {
            Action::Trip
        };
        Ok(match event::read()? {
            Event::Key(key) if key.kind == KeyEventKind::Press => match key.code {
                KeyCode::Char('q') | KeyCode::Esc => Action::Quit,
                KeyCode::Char('c') if key.modifiers.contains(KeyModifiers::CONTROL) => Action::Quit,
                KeyCode::Enter | KeyCode::Char(' ') => activate,
                _ => Action::None,
            },
            Event::Mouse(mouse)
                if mouse.kind == MouseEventKind::Down(MouseButton::Left)
                    && self.button.contains(Position::new(mouse.column, mouse.row)) =>
            {
                activate
            }
            Event::Resize(_, _) => {
                self.last_draw = None;
                Action::None
            }
            _ => Action::None,
        })
    }
}

impl Drop for Screen {
    fn drop(&mut self) {
        let _ = execute!(io::stdout(), DisableMouseCapture);
        ratatui::restore();
    }
}
