pub mod sysinfo;

use ansi_to_tui::IntoText;
use color_eyre::config::HookBuilder;
use compact_str::{CompactString, ToCompactString};
use cu29::clock::{CuDuration, RobotClock};
use cu29::config::{ComponentConfig, CuConfig, Node};
use cu29::cutask::CuMsgMetadata;
use cu29::monitoring::{CuDurationStatistics, CuMonitor, CuTaskState, Decision};
use cu29::{read_configuration, CuError, CuResult};
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
use std::fmt::{Display, Formatter};
use std::io::stdout;
use std::marker::PhantomData;
use std::sync::atomic::{AtomicBool, Ordering};
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

// This is kind of terrible.
#[derive(Copy, Clone)]
enum NodeType {
    Unknown,
    Source,
    Sink,
    Task,
}

impl Display for NodeType {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Unknown => write!(f, "?"),
            Self::Source => write!(f, "⏩"),
            Self::Task => write!(f, "⚡"),
            Self::Sink => write!(f, "🏁"),
        }
    }
}

impl NodeType {
    fn add_incoming(self) -> NodeType {
        match self {
            Self::Unknown => Self::Sink,
            Self::Source => Self::Task,
            Self::Sink => Self::Sink,
            Self::Task => Self::Task,
        }
    }

    fn add_outgoing(self) -> NodeType {
        match self {
            Self::Unknown => Self::Source,
            Self::Source => Self::Source,
            Self::Sink => Self::Task,
            Self::Task => Self::Task,
        }
    }
}

#[derive(Default, Clone)]
struct TaskStatus {
    is_error: bool,
    status_txt: CompactString,
    error: CompactString,
}

struct NodesScrollableWidgetState {
    config_nodes: Vec<Node>,
    node_types: Vec<NodeType>,
    connections: Vec<Connection>,
    statuses: Arc<Mutex<Vec<TaskStatus>>>,
    nodes_scrollable_state: ScrollViewState,
}

impl NodesScrollableWidgetState {
    fn new(config: &CuConfig, errors: Arc<Mutex<Vec<TaskStatus>>>) -> Self {
        let mut config_nodes: Vec<Node> = Vec::new();
        let mut node_types: Vec<NodeType> = Vec::new();
        for node in config.get_all_nodes() {
            config_nodes.push(node.clone());
            node_types.push(NodeType::Unknown);
        }
        let mut connections: Vec<Connection> = Vec::with_capacity(config.graph.edge_count());

        // Keep track if we already used a port.
        for (dst_index, dst_node) in config_nodes.iter().enumerate() {
            let node_incoming_edges = config.get_dst_edges(dst_index as u32);
            for (dst_port, edge_id) in node_incoming_edges.iter().enumerate() {
                if let Some((src_index, dst_index)) =
                    config.graph.edge_endpoints((*edge_id as u32).into())
                {
                    let (src_index, dst_index) = (src_index.index(), dst_index.index());
                    connections.push(Connection::new(
                        src_index, 0, // There is only one output per task today
                        dst_index, dst_port,
                    ));
                    node_types[dst_index] = node_types[dst_index].add_incoming(); // 🤮
                    node_types[src_index] = node_types[src_index].add_outgoing();
                } else {
                    panic!("Can't find back srcs for {}", dst_node.get_id());
                }
            }
        }
        NodesScrollableWidgetState {
            config_nodes,
            node_types,
            connections,
            nodes_scrollable_state: ScrollViewState::default(),
            statuses: errors,
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

const NODE_WIDTH: u16 = 29;
const NODE_WIDTH_CONTENT: u16 = NODE_WIDTH - 2;

const NODE_HEIGHT: u16 = 5;
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

        let content_size = Size::new(200, 100);
        let mut scroll_view = ScrollView::new(content_size);
        let mut graph = NodeGraph::new(
            node_layouts,
            state.connections.clone(),
            content_size.width as usize,
            content_size.height as usize,
        );
        graph.calculate();
        let zones = graph.split(scroll_view.area());

        {
            let mut statuses = state.statuses.lock().unwrap();
            for (idx, ea_zone) in zones.into_iter().enumerate() {
                let s = state.config_nodes[idx].get_type();
                let status = &mut statuses[idx];
                let status_line = if status.is_error {
                    format!("❌ {}", status.error)
                } else {
                    format!("✓ {}", status.status_txt)
                };

                let txt = format!(
                    " {}\n {}\n {}",
                    state.node_types[idx],
                    &s[s.len().saturating_sub(NODE_WIDTH_CONTENT as usize - 2)..],
                    status_line,
                );
                let paragraph = Paragraph::new(txt);
                let paragraph = if status.is_error {
                    paragraph.red()
                } else {
                    paragraph.green()
                };
                status.is_error = false; // reset if it was displayed
                scroll_view.render_widget(paragraph, ea_zone);
            }
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
    task_statuses: Arc<Mutex<Vec<TaskStatus>>>,
    quitting: Arc<AtomicBool>,
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
        task_statuses: Arc<Mutex<Vec<TaskStatus>>>,
    ) -> UI {
        init_error_hooks();
        let nodes_scrollable_widget_state =
            NodesScrollableWidgetState::new(&config, task_statuses.clone());
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
                            break;
                        }
                        _ => {}
                    }
                }
            }
        }
        restore_terminal();
        println!("Terminal restored.");
        Ok(())
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
            CuDuration::from(Duration::from_secs(5)),
        )));

        Ok(Self {
            config,
            taskids,
            task_stats,
            task_statuses: Arc::new(Mutex::new(vec![TaskStatus::default(); taskids.len()])),
            quitting: Arc::new(AtomicBool::new(false)),
        })
    }

    fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
        enable_raw_mode()
            .map_err(|e| CuError::new_with_cause("Could not enable console raw mode", e))?;
        execute!(stdout(), EnterAlternateScreen, EnableMouseCapture)
            .map_err(|e| CuError::new_with_cause("Could not execute crossterm", e))?;

        let backend = CrosstermBackend::new(stdout());
        let mut terminal = Terminal::new(backend)
            .map_err(|e| CuError::new_with_cause("Failed to initialize terminal backend", e))?;

        let config_dup = self.config.clone();
        let taskids = self.taskids;

        let task_stats_ui = self.task_stats.clone();
        let error_states = self.task_statuses.clone();
        let quitting = self.quitting.clone();

        // Start the main UI loop
        thread::spawn(move || {
            let mut ui = UI::new(config_dup, taskids, task_stats_ui, error_states);
            ui.run_app(&mut terminal).expect("Failed to run app");
            quitting.store(true, Ordering::SeqCst);
        });

        Ok(())
    }

    fn process_copperlist(&self, msgs: &[&CuMsgMetadata]) -> CuResult<()> {
        {
            let mut task_stats = self.task_stats.lock().unwrap();
            task_stats.update(msgs);
        }
        {
            let mut task_statuses = self.task_statuses.lock().unwrap();
            for (i, msg) in msgs.iter().enumerate() {
                task_statuses[i].status_txt = msg.status_txt.0.clone();
                if task_statuses[i].status_txt.as_bytes()[0] == 0 {
                    task_statuses[i].status_txt = "".to_compact_string();
                }
            }
        }

        if self.quitting.load(Ordering::SeqCst) {
            return Err("Exiting...".into());
        }
        Ok(())
    }

    fn process_error(&self, taskid: usize, step: CuTaskState, error: &CuError) -> Decision {
        {
            let status = &mut self.task_statuses.lock().unwrap()[taskid];
            status.is_error = true;
            status.error = error.to_compact_string();
        }
        match step {
            CuTaskState::Start => Decision::Shutdown,
            CuTaskState::Preprocess => Decision::Abort,
            CuTaskState::Process => Decision::Ignore,
            CuTaskState::Postprocess => Decision::Ignore,
            CuTaskState::Stop => Decision::Shutdown,
        }
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
