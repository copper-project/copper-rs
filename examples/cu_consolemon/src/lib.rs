pub mod sysinfo;

use ansi_to_tui::IntoText;
use cu29::clock::{CuDuration, RobotClock};
use cu29::config::{ComponentConfig, CuConfig};
use cu29::cutask::CuMsgMetadata;
use cu29::monitoring::{CuDurationStatistics, CuMonitor, CuTaskState, Decision};
use cu29::{read_configuration, CuError, CuResult};
use nix::sys::signal;
use nix::sys::signal::Signal;
use ratatui::backend::CrosstermBackend;
use ratatui::crossterm::event::{DisableMouseCapture, EnableMouseCapture, Event, KeyCode};
use ratatui::crossterm::terminal::{
    disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen,
};
use ratatui::crossterm::{event, execute};
use ratatui::layout::{Alignment, Constraint, Direction, Layout};
use ratatui::prelude::Stylize;
use ratatui::prelude::{Backend, Rect};
use ratatui::style::{Color, Modifier, Style};
use ratatui::text::{Line, Text};
use ratatui::widgets::{Block, Borders, Cell, Paragraph, Row, Table};
use ratatui::{Frame, Terminal};
use std::process;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use std::{io, thread};

enum Screen {
    Neofetch,
    Dag,
    Latency,
}

struct TaskStats {
    stats: Vec<CuDurationStatistics>,
    end2end: CuDurationStatistics,
}

impl TaskStats {
    fn new(num_tasks: usize, max_duration: CuDuration) -> Self {
        let stats = vec![CuDurationStatistics::new(max_duration); num_tasks];
        TaskStats {
            stats,
            end2end: CuDurationStatistics::new(max_duration),
        }
    }

    fn update(&mut self, msgs: &[&CuMsgMetadata]) {
        for (i, &msg) in msgs.iter().enumerate() {
            let (before, after) = (msg.before_process.unwrap(), msg.after_process.unwrap());
            self.stats[i].record(after - before);
        }
        self.end2end.record(compute_end_to_end_latency(msgs));
    }
}

fn dag(config: &CuConfig) -> String {
    "test".to_string()
}

/// A TUI based realtime console for Copper.
pub struct CuConsoleMon {
    config: CuConfig,
    taskids: &'static [&'static str],
    task_stats: Arc<Mutex<TaskStats>>,
}

struct UI {
    config: CuConfig,
    task_ids: &'static [&'static str],
    active_screen: Screen,
    sysinfo: String,
    task_stats: Arc<Mutex<TaskStats>>,
}

fn compute_end_to_end_latency(msgs: &[&CuMsgMetadata]) -> CuDuration {
    msgs.last().unwrap().after_process.unwrap() - msgs.first().unwrap().before_process.unwrap()
}

impl UI {
    fn new(
        config: CuConfig,
        task_ids: &'static [&'static str],
        task_stats: Arc<Mutex<TaskStats>>,
    ) -> UI {
        let num_tasks = task_ids.len();
        Self {
            config,
            task_ids,
            active_screen: Screen::Neofetch,
            sysinfo: sysinfo::pfetch_info(),
            task_stats,
        }
    }

    fn draw_latency_table(&self, f: &mut Frame, area: Rect) {
        let header_cells = [
            "🛠 Task",
            "⬇ Min",
            "⬆ Max",
            "∅ Mean",
            "σ Stddev",
            "⧖∅ Jitter",
            "⧗⬆ Jitter",
        ]
        .iter()
        .map(|h| {
            Cell::from(Line::from(*h).alignment(Alignment::Right)).style(
                Style::default()
                    .fg(Color::Yellow)
                    .add_modifier(Modifier::BOLD),
            )
        });

        let header = Row::new(header_cells)
            .style(Style::default().fg(Color::Yellow))
            .bottom_margin(1)
            .top_margin(1);

        let task_stats = self.task_stats.lock().unwrap(); // Acquire lock to read task_stats
        let mut rows = task_stats
            .stats
            .iter()
            .enumerate()
            .map(|(i, stat)| {
                let cells = vec![
                    Cell::from(Line::from(self.task_ids[i]).alignment(Alignment::Right))
                        .light_blue(),
                    Cell::from(Line::from(stat.min().to_string()).alignment(Alignment::Right))
                        .style(Style::default()),
                    Cell::from(Line::from(stat.max().to_string()).alignment(Alignment::Right))
                        .style(Style::default()),
                    Cell::from(Line::from(stat.mean().to_string()).alignment(Alignment::Right))
                        .style(Style::default()),
                    Cell::from(Line::from(stat.stddev().to_string()).alignment(Alignment::Right))
                        .style(Style::default()),
                    Cell::from(
                        Line::from(stat.jitter_mean().to_string()).alignment(Alignment::Right),
                    )
                    .style(Style::default()),
                    Cell::from(
                        Line::from(stat.jitter_max().to_string()).alignment(Alignment::Right),
                    )
                    .style(Style::default()),
                ];
                Row::new(cells)
            })
            .collect::<Vec<Row>>();

        let cells = vec![
            Cell::from(
                Line::from("End2End")
                    .light_red()
                    .alignment(Alignment::Right),
            ),
            Cell::from(
                Line::from(task_stats.end2end.min().to_string())
                    .light_red()
                    .alignment(Alignment::Right),
            )
            .style(Style::default()),
            Cell::from(
                Line::from(task_stats.end2end.max().to_string())
                    .light_red()
                    .alignment(Alignment::Right),
            )
            .style(Style::default()),
            Cell::from(
                Line::from(task_stats.end2end.mean().to_string())
                    .light_red()
                    .alignment(Alignment::Right),
            )
            .style(Style::default()),
            Cell::from(
                Line::from(task_stats.end2end.stddev().to_string())
                    .light_red()
                    .alignment(Alignment::Right),
            )
            .style(Style::default()),
            Cell::from(
                Line::from(task_stats.end2end.jitter_mean().to_string())
                    .light_red()
                    .alignment(Alignment::Right),
            )
            .style(Style::default()),
            Cell::from(
                Line::from(task_stats.end2end.jitter_max().to_string())
                    .light_red()
                    .alignment(Alignment::Right),
            )
            .style(Style::default()),
        ];
        rows.push(Row::new(cells).top_margin(1));

        let table = Table::new(
            rows,
            &[
                Constraint::Length(10),
                Constraint::Length(10),
                Constraint::Length(12),
                Constraint::Length(12),
                Constraint::Length(10),
                Constraint::Length(12),
                Constraint::Length(13),
            ],
        )
        .header(header)
        .block(Block::default().borders(Borders::ALL).title(" Latencies "));

        f.render_widget(table, area);
    }

    fn draw(&self, f: &mut Frame) {
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

        let menu = Paragraph::new("   [1] SysInfo  [2] DAG  [3] Latencies  [q] Quit")
            .style(
                Style::default()
                    .fg(Color::Yellow)
                    .add_modifier(Modifier::ITALIC),
            )
            .block(Block::default().borders(Borders::BOTTOM));
        f.render_widget(menu, layout[0]);

        match self.active_screen {
            Screen::Neofetch => {
                let text: Text = format!("\n{}", self.sysinfo).into_text().unwrap();
                let p = Paragraph::new::<Text>(text).block(
                    Block::default()
                        .title(" System Info ")
                        .borders(Borders::ALL),
                );
                f.render_widget(p, layout[1]);
            }
            Screen::Dag => {
                let p = Paragraph::new(dag(&self.config))
                    .block(Block::default().title(" Tasks DAG ").borders(Borders::ALL));
                f.render_widget(p, layout[1]);
            }
            Screen::Latency => self.draw_latency_table(f, layout[1]),
        };
    }

    fn run_app<B: Backend>(&mut self, terminal: &mut Terminal<B>) -> io::Result<()> {
        loop {
            terminal.draw(|f| {
                self.draw(f);
            })?;

            if event::poll(Duration::from_millis(10))? {
                if let Event::Key(key) = event::read()? {
                    match key.code {
                        KeyCode::Char('1') => self.active_screen = Screen::Neofetch,
                        KeyCode::Char('2') => self.active_screen = Screen::Dag,
                        KeyCode::Char('3') => self.active_screen = Screen::Latency,
                        KeyCode::Char('q') => {
                            // Basically triggers the clean ctrlc logic of copper.
                            signal::kill(
                                nix::unistd::Pid::from_raw(process::id() as i32),
                                Signal::SIGINT,
                            )
                            .expect("Error sending SIGINT");
                        }
                        _ => {}
                    }
                }
            }
        }
    }
}

impl CuMonitor for CuConsoleMon {
    fn new(config: Option<&ComponentConfig>, taskids: &'static [&'static str]) -> CuResult<Self>
    where
        Self: Sized,
    {
        let config_file: String = if let Some(config) = config {
            config
                .0
                .get("file")
                .expect("You need to passs the path to the copper config file to use")
                .to_string()
        } else {
            panic!("You need to passs the path to the copper config file to use");
        };

        let config =
            read_configuration(config_file.as_str()).expect("Could not read configuration");
        let task_stats = Arc::new(Mutex::new(TaskStats::new(
            taskids.len(),
            CuDuration::from(Duration::from_secs(1)),
        )));

        Ok(Self {
            config,
            taskids,
            task_stats,
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

        let config_dup = self.config.clone();
        let taskids = self.taskids.clone();

        let task_stats_ui = self.task_stats.clone();

        // Start the main UI loop
        thread::spawn(move || {
            let mut ui = UI::new(config_dup, taskids, task_stats_ui);
            ui.run_app(&mut terminal).expect("Failed to run app");
        });

        Ok(())
    }

    fn process_copperlist(&self, msgs: &[&CuMsgMetadata]) -> CuResult<()> {
        let mut task_stats = self.task_stats.lock().unwrap();
        task_stats.update(msgs);
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

        self.task_stats
            .lock()
            .unwrap()
            .stats
            .iter_mut()
            .for_each(|s| s.reset());
        Ok(())
    }
}
