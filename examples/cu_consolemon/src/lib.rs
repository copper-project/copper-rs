pub mod sysinfo;

use ansi_to_tui::IntoText;
use color_eyre::config::HookBuilder;
use cu29::clock::{CuDuration, RobotClock};
use cu29::config::{ComponentConfig, CuConfig, Node};
use cu29::cutask::CuMsgMetadata;
use cu29::monitoring::{CuDurationStatistics, CuMonitor, CuTaskState, Decision};
use cu29::{read_configuration, CuError, CuResult};
use nix::sys::signal;
use nix::sys::signal::Signal;
use ratatui::backend::CrosstermBackend;
use ratatui::buffer::Buffer;
use ratatui::crossterm::event::{DisableMouseCapture, EnableMouseCapture, Event, KeyCode};
use ratatui::crossterm::terminal::{
    disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen,
};
use ratatui::crossterm::ExecutableCommand;
use ratatui::crossterm::{event, execute};
use ratatui::layout::{Alignment, Constraint, Direction, Layout, Size};
use ratatui::prelude::{Backend, Rect};
use ratatui::prelude::{Stylize, Widget};
use ratatui::style::{Color, Modifier, Style};
use ratatui::text::{Line, Text};
use ratatui::widgets::{Block, Borders, Cell, Paragraph, Row, StatefulWidget, Table};
use ratatui::{Frame, Terminal};
use std::io::stdout;
use std::marker::PhantomData;
use std::process;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use std::{io, thread};
use tui_nodes::{Connection, NodeGraph, NodeLayout};
use tui_widgets::scrollview::{ScrollView, ScrollViewState};

#[derive(PartialEq)]
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

    fn reset(&mut self) {
        for s in &mut self.stats {
            s.reset();
        }
        self.end2end.reset();
    }
}

fn compute_end_to_end_latency(msgs: &[&CuMsgMetadata]) -> CuDuration {
    msgs.last().unwrap().after_process.unwrap() - msgs.first().unwrap().before_process.unwrap()
}

struct NodesScrollableWidgetState {
    config_nodes: Vec<Node>,
    connections: Vec<Connection>,
    nodes_scrollable_state: ScrollViewState,
    errors: Arc<Mutex<Vec<Option<CuError>>>>,
}

impl NodesScrollableWidgetState {
    fn new(config: &CuConfig, errors: Arc<Mutex<Vec<Option<CuError>>>>) -> Self {
        let mut config_nodes: Vec<Node> = Vec::new();
        for node in config.get_all_nodes() {
            config_nodes.push(node.clone());
        }
        let mut connections: Vec<Connection> = Vec::with_capacity(config.graph.edge_count());

        // Keep track if we already used a port.
        for (dst_index, dst_node) in config_nodes.iter().enumerate() {
            let node_incoming_edges = config.get_dst_edges(dst_index as u32);
            for (dst_port, edge_id) in node_incoming_edges.iter().enumerate() {
                if let Some((src_index, dst_index)) =
                    config.graph.edge_endpoints((*edge_id as u32).into())
                {
                    connections.push(Connection::new(
                        src_index.index(),
                        0, // There is only one output per task today
                        dst_index.index(),
                        dst_port,
                    ));
                } else {
                    panic!("Can't find back srcs for {}", dst_node.get_id());
                }
            }
        }
        NodesScrollableWidgetState {
            config_nodes,
            connections,
            nodes_scrollable_state: ScrollViewState::default(),
            errors,
        }
    }
}

struct NodesScrollableWidget<'a> {
    _marker: PhantomData<&'a ()>,
}

struct GraphWrapper<'a> {
    inner: NodeGraph<'a>,
}

impl Widget for GraphWrapper<'_> {
    fn render(self, area: Rect, buf: &mut Buffer)
    where
        Self: Sized,
    {
        self.inner.render(area, buf, &mut ())
    }
}

const NODE_WIDTH: u16 = 26;
const NODE_WIDTH_CONTENT: u16 = NODE_WIDTH - 2;

const NODE_HEIGHT: u16 = 3;
const NODE_HEIGHT_CONTENT: u16 = NODE_HEIGHT - 2;

impl StatefulWidget for NodesScrollableWidget<'_> {
    type State = NodesScrollableWidgetState;

    fn render(self, area: Rect, buf: &mut Buffer, state: &mut Self::State) {
        let node_ids: Vec<String> = state
            .config_nodes
            .iter()
            .map(|node| format!(" {} ", node.get_id()))
            .collect();
        let node_layouts = state
            .config_nodes
            .iter()
            .zip(node_ids.iter())
            .map(|(_, node_id)| {
                NodeLayout::new((NODE_WIDTH, NODE_HEIGHT)).with_title(node_id.as_str())
            })
            .collect();

        let content_size = Size::new(300, 100);
        let mut scroll_view = ScrollView::new(content_size);
        let mut graph = NodeGraph::new(
            node_layouts,
            state.connections.clone(),
            content_size.width as usize,
            content_size.height as usize,
        );
        graph.calculate();
        let zones = graph.split(scroll_view.area());
        for (idx, ea_zone) in zones.into_iter().enumerate() {
            let s = state.config_nodes[idx].get_type();
            let paragraph = Paragraph::new(format!(
                " {} ",
                &s[s.len().saturating_sub(NODE_WIDTH_CONTENT as usize - 2)..]
            ));
            let paragraph = if state.errors.lock().unwrap()[idx].is_some() {
                paragraph.red()
            } else {
                paragraph.green()
            };
            scroll_view.render_widget(paragraph, ea_zone);
        }

        scroll_view.render_widget(
            GraphWrapper { inner: graph },
            Rect {
                x: 0,
                y: 0,
                width: content_size.width,
                height: content_size.height,
            },
        );
        scroll_view.render(area, buf, &mut state.nodes_scrollable_state);
    }
}

/// A TUI based realtime console for Copper.
pub struct CuConsoleMon {
    config: CuConfig,
    taskids: &'static [&'static str],
    task_stats: Arc<Mutex<TaskStats>>,
    error_states: Arc<Mutex<Vec<Option<CuError>>>>,
}

struct UI {
    task_ids: &'static [&'static str],
    active_screen: Screen,
    sysinfo: String,
    task_stats: Arc<Mutex<TaskStats>>,
    nodes_scrollable_widget_state: NodesScrollableWidgetState,
}

impl UI {
    fn new(
        config: CuConfig,
        task_ids: &'static [&'static str],
        task_stats: Arc<Mutex<TaskStats>>,
        error_states: Arc<Mutex<Vec<Option<CuError>>>>,
    ) -> UI {
        init_error_hooks();
        let nodes_scrollable_widget_state =
            NodesScrollableWidgetState::new(&config, error_states.clone());
        Self {
            task_ids,
            active_screen: Screen::Neofetch,
            sysinfo: sysinfo::pfetch_info(),
            task_stats,
            nodes_scrollable_widget_state,
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

    fn draw_nodes(&mut self, f: &mut Frame, space: Rect) {
        NodesScrollableWidget {
            _marker: Default::default(),
        }
        .render(
            space,
            f.buffer_mut(),
            &mut self.nodes_scrollable_widget_state,
        )
    }

    fn draw(&mut self, f: &mut Frame) {
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
                const VERSION: &str = env!("CARGO_PKG_VERSION");
                let text: Text = format!("\n   -> Copper v{}\n\n{}\n\n ", VERSION, self.sysinfo)
                    .into_text()
                    .unwrap();
                let p = Paragraph::new::<Text>(text).block(
                    Block::default()
                        .title(" System Info ")
                        .borders(Borders::ALL),
                );
                f.render_widget(p, layout[1]);
            }
            Screen::Dag => {
                self.draw_nodes(f, layout[1]);
            }
            Screen::Latency => self.draw_latency_table(f, layout[1]),
        };
    }

    fn run_app<B: Backend>(&mut self, terminal: &mut Terminal<B>) -> io::Result<()> {
        loop {
            terminal.draw(|f| {
                self.draw(f);
            })?;

            if event::poll(Duration::from_millis(50))? {
                if let Event::Key(key) = event::read()? {
                    match key.code {
                        KeyCode::Char('1') => self.active_screen = Screen::Neofetch,
                        KeyCode::Char('2') => self.active_screen = Screen::Dag,
                        KeyCode::Char('3') => self.active_screen = Screen::Latency,
                        KeyCode::Char('r') => {
                            if self.active_screen == Screen::Latency {
                                self.task_stats.lock().unwrap().reset()
                            }
                        }
                        KeyCode::Char('j') => {
                            if self.active_screen == Screen::Dag {
                                for _ in 0..1 {
                                    self.nodes_scrollable_widget_state
                                        .nodes_scrollable_state
                                        .scroll_down();
                                }
                            }
                        }
                        KeyCode::Char('k') => {
                            if self.active_screen == Screen::Dag {
                                for _ in 0..1 {
                                    self.nodes_scrollable_widget_state
                                        .nodes_scrollable_state
                                        .scroll_up();
                                }
                            }
                        }
                        KeyCode::Char('h') => {
                            if self.active_screen == Screen::Dag {
                                for _ in 0..5 {
                                    self.nodes_scrollable_widget_state
                                        .nodes_scrollable_state
                                        .scroll_left();
                                }
                            }
                        }
                        KeyCode::Char('l') => {
                            if self.active_screen == Screen::Dag {
                                for _ in 0..5 {
                                    self.nodes_scrollable_widget_state
                                        .nodes_scrollable_state
                                        .scroll_right();
                                }
                            }
                        }
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
            error_states: Arc::new(Mutex::new(vec![None; taskids.len()])),
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
        let taskids = self.taskids;

        let task_stats_ui = self.task_stats.clone();
        let error_states = self.error_states.clone();

        // Start the main UI loop
        thread::spawn(move || {
            let mut ui = UI::new(config_dup, taskids, task_stats_ui, error_states);
            ui.run_app(&mut terminal).expect("Failed to run app");
        });

        Ok(())
    }

    fn process_copperlist(&self, msgs: &[&CuMsgMetadata]) -> CuResult<()> {
        let mut task_stats = self.task_stats.lock().unwrap();
        task_stats.update(msgs);
        Ok(())
    }

    fn process_error(&self, taskid: usize, _step: CuTaskState, error: &CuError) -> Decision {
        self.error_states.lock().unwrap()[taskid] = Some(error.clone());
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

fn init_error_hooks() {
    let (panic, error) = HookBuilder::default().into_hooks();
    let panic = panic.into_panic_hook();
    let error = error.into_eyre_hook();
    color_eyre::eyre::set_hook(Box::new(move |e| {
        let _ = restore_terminal();
        error(e)
    }))
    .unwrap();
    std::panic::set_hook(Box::new(move |info| {
        let _ = restore_terminal();
        panic(info)
    }));
}

fn restore_terminal() {
    disable_raw_mode().unwrap();
    stdout().execute(LeaveAlternateScreen).unwrap();
}
