pub mod sysinfo;

use cu29::clock::RobotClock;
use cu29::config::ComponentConfig;
use cu29::cutask::CuMsgMetadata;
use cu29::monitoring::{CuMonitor, CuTaskState, Decision};
use cu29::{CuError, CuResult};
use ratatui::crossterm::event::{DisableMouseCapture, EnableMouseCapture, Event, KeyCode};
use ratatui::crossterm::terminal::{
    disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen,
};
use ratatui::crossterm::{event, execute};
use ratatui::prelude::*;

use ratatui::widgets::{Block, Borders, Paragraph};
use ratatui::Terminal;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use std::{io, thread};

enum Screen {
    Neofetch,
    Dag,
    Latency,
}

fn run_app<B: Backend>(
    terminal: &mut Terminal<B>,
    active_screen: Arc<Mutex<Screen>>,
) -> io::Result<()> {
    loop {
        terminal.draw(|f| {
            let screen = active_screen.lock().unwrap();
            draw_ui(f, &*screen);
        })?;

        if event::poll(Duration::from_millis(10))? {
            if let Event::Key(key) = event::read()? {
                let mut screen = active_screen.lock().unwrap();
                match key.code {
                    KeyCode::Char('1') => *screen = Screen::Neofetch,
                    KeyCode::Char('2') => *screen = Screen::Dag,
                    KeyCode::Char('3') => *screen = Screen::Latency,
                    KeyCode::Char('q') => return Ok(()), // Quit
                    _ => {}
                }
            }
        }
    }
}

fn draw_ui(f: &mut Frame, active_screen: &Screen) {
    let layout = Layout::default()
        .direction(Direction::Vertical)
        .constraints(
            [
                Constraint::Length(3), // For the top menu bar
                Constraint::Min(0),    // For the main content
            ]
            .as_ref(),
        )
        .split(f.area());

    let menu = Paragraph::new("[1] Neofetch  [2] DAG  [3] Latency  [q] Quit")
        .style(
            Style::default()
                .fg(Color::Yellow)
                .add_modifier(Modifier::BOLD),
        )
        .block(Block::default().borders(Borders::BOTTOM));
    f.render_widget(menu, layout[0]);

    let content = match active_screen {
        Screen::Neofetch => Paragraph::new("Neofetch-like system info")
            .block(Block::default().title("System Info").borders(Borders::ALL)),
        Screen::Dag => Paragraph::new("DAG of Copper")
            .block(Block::default().title("Copper DAG").borders(Borders::ALL)),
        Screen::Latency => Paragraph::new("Node Latency Stats: Min, Max, Percentiles...")
            .block(Block::default().title("Latency").borders(Borders::ALL)),
    };

    f.render_widget(content, layout[1]);
}

/// A TUI based realtime console for Copper.
pub struct CuConsoleMon {
    taskids: &'static [&'static str],
    active_screen: Arc<Mutex<Screen>>,
}

impl CuMonitor for CuConsoleMon {
    fn new(config: Option<&ComponentConfig>, taskids: &'static [&'static str]) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {
            taskids,
            active_screen: Arc::new(Mutex::new(Screen::Neofetch)),
        })
    }

    fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
        enable_raw_mode()
            .map_err(|e| CuError::new_with_cause("Could not enable console raw mode", e))?;
        execute!(io::stdout(), EnterAlternateScreen, EnableMouseCapture)
            .map_err(|e| CuError::new_with_cause("Could not execute crossterm", e))?;

        let backend = CrosstermBackend::new(io::stdout());
        let mut terminal = Terminal::new(backend)
            .map_err(|e| CuError::new_with_cause("Failed to initialize terminal backend", e))?;
        let active_screen = Arc::clone(&self.active_screen);
        // Start the main UI loop
        thread::spawn(move || {
            run_app(&mut terminal, active_screen).expect("Failed to run app");
        });
        Ok(())
    }

    fn process_copperlist(&self, msgs: &[&CuMsgMetadata]) -> CuResult<()> {
        Ok(())
    }

    fn process_error(&self, taskid: usize, step: CuTaskState, error: &CuError) -> Decision {
        Decision::Ignore
    }

    fn stop(&mut self, _clock: &RobotClock) -> CuResult<()> {
        disable_raw_mode()
            .map_err(|e| CuError::new_with_cause("Could not disable console raw mode", e))?;
        execute!(io::stdout(), LeaveAlternateScreen, DisableMouseCapture).map_err(|e| {
            CuError::new_with_cause(
                "Could not leave alternate screen or disable mouse capture",
                e,
            )
        })?;
        Ok(())
    }
}
