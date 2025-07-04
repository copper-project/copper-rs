#[cfg(feature = "debug_pane")]
mod debug_pane;
pub mod sysinfo;

use ansi_to_tui::IntoText;
use color_eyre::config::HookBuilder;
use compact_str::{CompactString, ToCompactString};
use cu29::clock::{CuDuration, RobotClock};
use cu29::config::{CuConfig, Node};
use cu29::cutask::CuMsgMetadata;
use cu29::monitoring::{CuDurationStatistics, CuMonitor, CuTaskState, Decision};
use cu29::prelude::{pool, CuCompactString};
use cu29::{CuError, CuResult};
#[cfg(feature = "debug_pane")]
use debug_pane::UIExt;
use ratatui::backend::CrosstermBackend;
use ratatui::buffer::Buffer;
use ratatui::crossterm::event::{DisableMouseCapture, EnableMouseCapture, Event, KeyCode};
use ratatui::crossterm::terminal::{
    disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen,
};
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
use std::time::{Duration, Instant};
use std::{io, thread};
use tui_nodes::{Connection, NodeGraph, NodeLayout};
use tui_widgets::scrollview::{ScrollView, ScrollViewState};

#[cfg(feature = "debug_pane")]
const MENU_CONTENT: &str =
    "   [1] SysInfo  [2] DAG  [3] Latencies  [4] Memory Pools [5] Debug Output  [q] Quit | Scroll: hjkl or ↑↓←→   ";
#[cfg(not(feature = "debug_pane"))]
const MENU_CONTENT: &str =
    "   [1] SysInfo  [2] DAG  [3] Latencies  [4] Memory Pools [q] Quit | Scroll: hjkl or ↑↓←→   ";

#[derive(PartialEq)]
enum Screen {
    Neofetch,
    Dag,
    Latency,
    #[cfg(feature = "debug_pane")]
    DebugOutput,
    MemoryPools,
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
            let (before, after) = (
                msg.process_time.start.unwrap(),
                msg.process_time.end.unwrap(),
            );
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
struct PoolStats {
    id: CompactString,
    space_left: usize,
    total_size: usize,
    buffer_size: usize,
    handles_in_use: usize,
    handles_per_second: usize,
    last_update: Instant,
    prev_handles_in_use: usize,
}

impl PoolStats {
    fn new(
        id: impl ToCompactString,
        space_left: usize,
        total_size: usize,
        buffer_size: usize,
    ) -> Self {
        Self {
            id: id.to_compact_string(),
            space_left,
            total_size,
            buffer_size,
            handles_in_use: total_size - space_left,
            handles_per_second: 0,
            last_update: Instant::now(),
            prev_handles_in_use: 0,
        }
    }

    fn update(&mut self, space_left: usize, total_size: usize) {
        let now = Instant::now();
        let handles_in_use = total_size - space_left;
        let elapsed = now.duration_since(self.last_update).as_secs_f32();

        if elapsed >= 1.0 {
            self.handles_per_second =
                ((handles_in_use.abs_diff(self.handles_in_use)) as f32 / elapsed) as usize;
            self.prev_handles_in_use = self.handles_in_use;
            self.last_update = now;
        }

        self.handles_in_use = handles_in_use;
        self.space_left = space_left;
        self.total_size = total_size;
    }
}
fn compute_end_to_end_latency(msgs: &[&CuMsgMetadata]) -> CuDuration {
    msgs.last().unwrap().process_time.end.unwrap()
        - msgs.first().unwrap().process_time.start.unwrap()
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
    fn new(config: &CuConfig, errors: Arc<Mutex<Vec<TaskStatus>>>, mission: Option<&str>) -> Self {
        let mut config_nodes: Vec<Node> = Vec::new();
        let mut node_types: Vec<NodeType> = Vec::new();

        let graph = config
            .get_graph(mission)
            .expect("Only supported for simple config");

        for (_, node) in graph.get_all_nodes() {
            // FIXME(gbin): Multimission
            config_nodes.push(node.clone());
            node_types.push(NodeType::Unknown);
        }
        let mut connections: Vec<Connection> = Vec::with_capacity(graph.0.edge_count());

        // Keep track if we already used a port.
        for (dst_index, dst_node) in config_nodes.iter().enumerate() {
            let node_incoming_edges = graph
                .get_dst_edges(dst_index as u32)
                .expect("Error getting edges");
            for (dst_port, edge_id) in node_incoming_edges.iter().enumerate() {
                if let Some((src_index, dst_index)) =
                    graph.0.edge_endpoints((*edge_id as u32).into())
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

#[allow(dead_code)]
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
    pool_stats: Arc<Mutex<Vec<PoolStats>>>,
    quitting: Arc<AtomicBool>,
}

struct UI {
    task_ids: &'static [&'static str],
    active_screen: Screen,
    sysinfo: String,
    task_stats: Arc<Mutex<TaskStats>>,
    nodes_scrollable_widget_state: NodesScrollableWidgetState,
    #[cfg(feature = "debug_pane")]
    error_redirect: gag::BufferRedirect,
    #[cfg(feature = "debug_pane")]
    debug_output: Option<debug_pane::DebugLog>,
    pool_stats: Arc<Mutex<Vec<PoolStats>>>,
}

impl UI {
    #[cfg(feature = "debug_pane")]
    #[allow(clippy::too_many_arguments)]
    fn new(
        config: CuConfig,
        mission: Option<&str>,
        task_ids: &'static [&'static str],
        task_stats: Arc<Mutex<TaskStats>>,
        task_statuses: Arc<Mutex<Vec<TaskStatus>>>,
        error_redirect: gag::BufferRedirect,
        debug_output: Option<debug_pane::DebugLog>,
        pool_stats: Arc<Mutex<Vec<PoolStats>>>,
    ) -> UI {
        init_error_hooks();
        let nodes_scrollable_widget_state =
            NodesScrollableWidgetState::new(&config, task_statuses.clone(), mission);

        Self {
            task_ids,
            active_screen: Screen::Neofetch,
            sysinfo: sysinfo::pfetch_info(),
            task_stats,
            nodes_scrollable_widget_state,
            error_redirect,
            debug_output,
            pool_stats,
        }
    }

    #[cfg(not(feature = "debug_pane"))]
    fn new(
        config: CuConfig,
        task_ids: &'static [&'static str],
        task_stats: Arc<Mutex<TaskStats>>,
        task_statuses: Arc<Mutex<Vec<TaskStatus>>>,
        pool_stats: Arc<Mutex<Vec<PoolStats>>>,
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
            pool_stats,
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

    fn draw_memory_pools(&self, f: &mut Frame, area: Rect) {
        let header_cells = [
            "Pool ID",
            "Used/Total",
            "Buffer Size",
            "Handles in Use",
            "Handles/sec",
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
            .bottom_margin(1);

        let pool_stats = self.pool_stats.lock().unwrap();
        let rows = pool_stats
            .iter()
            .map(|stat| {
                let used = stat.total_size - stat.space_left;
                let percent = if stat.total_size > 0 {
                    100.0 * used as f64 / stat.total_size as f64
                } else {
                    0.0
                };
                let buffer_size = stat.buffer_size;
                let mb_unit = 1024.0 * 1024.0;

                let cells = vec![
                    Cell::from(Line::from(stat.id.to_string()).alignment(Alignment::Right))
                        .light_blue(),
                    Cell::from(
                        Line::from(format!(
                            "{:.2} MB / {:.2} MB ({:.1}%)",
                            used as f64 * buffer_size as f64 / mb_unit,
                            stat.total_size as f64 * buffer_size as f64 / mb_unit,
                            percent
                        ))
                        .alignment(Alignment::Right),
                    ),
                    Cell::from(
                        Line::from(format!("{} KB", stat.buffer_size / 1024))
                            .alignment(Alignment::Right),
                    ),
                    Cell::from(
                        Line::from(format!("{}", stat.handles_in_use)).alignment(Alignment::Right),
                    ),
                    Cell::from(
                        Line::from(format!("{}/s", stat.handles_per_second))
                            .alignment(Alignment::Right),
                    ),
                ];
                Row::new(cells)
            })
            .collect::<Vec<Row>>();

        let table = Table::new(
            rows,
            &[
                Constraint::Percentage(30),
                Constraint::Percentage(20),
                Constraint::Percentage(15),
                Constraint::Percentage(15),
                Constraint::Percentage(20),
            ],
        )
        .header(header)
        .block(
            Block::default()
                .borders(Borders::ALL)
                .title(" Memory Pools "),
        );

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

        let menu = Paragraph::new(MENU_CONTENT)
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
            Screen::MemoryPools => self.draw_memory_pools(f, layout[1]),
            #[cfg(feature = "debug_pane")]
            Screen::DebugOutput => self.draw_debug_output(f, layout[1]),
        };
    }

    fn run_app<B: Backend>(&mut self, terminal: &mut Terminal<B>) -> io::Result<()> {
        loop {
            #[cfg(feature = "debug_pane")]
            self.update_debug_output();

            terminal.draw(|f| {
                self.draw(f);
            })?;

            if event::poll(Duration::from_millis(50))? {
                let event = event::read()?;

                match event {
                    Event::Key(key) => match key.code {
                        KeyCode::Char('1') => self.active_screen = Screen::Neofetch,
                        KeyCode::Char('2') => self.active_screen = Screen::Dag,
                        KeyCode::Char('3') => self.active_screen = Screen::Latency,
                        KeyCode::Char('4') => self.active_screen = Screen::MemoryPools,
                        #[cfg(feature = "debug_pane")]
                        KeyCode::Char('5') => self.active_screen = Screen::DebugOutput,
                        KeyCode::Char('r') => {
                            if self.active_screen == Screen::Latency {
                                self.task_stats.lock().unwrap().reset()
                            }
                        }
                        KeyCode::Char('j') | KeyCode::Down => {
                            if self.active_screen == Screen::Dag {
                                for _ in 0..1 {
                                    self.nodes_scrollable_widget_state
                                        .nodes_scrollable_state
                                        .scroll_down();
                                }
                            }
                        }
                        KeyCode::Char('k') | KeyCode::Up => {
                            if self.active_screen == Screen::Dag {
                                for _ in 0..1 {
                                    self.nodes_scrollable_widget_state
                                        .nodes_scrollable_state
                                        .scroll_up();
                                }
                            }
                        }
                        KeyCode::Char('h') | KeyCode::Left => {
                            if self.active_screen == Screen::Dag {
                                for _ in 0..5 {
                                    self.nodes_scrollable_widget_state
                                        .nodes_scrollable_state
                                        .scroll_left();
                                }
                            }
                        }
                        KeyCode::Char('l') | KeyCode::Right => {
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
                    },

                    #[cfg(feature = "debug_pane")]
                    Event::Resize(_columns, rows) => {
                        if let Some(debug_output) = self.debug_output.as_mut() {
                            debug_output.max_rows.store(rows, Ordering::SeqCst)
                        }
                    }
                    _ => {}
                }
            }
        }
        Ok(())
    }
}

impl CuMonitor for CuConsoleMon {
    fn new(config: &CuConfig, taskids: &'static [&'static str]) -> CuResult<Self>
    where
        Self: Sized,
    {
        let task_stats = Arc::new(Mutex::new(TaskStats::new(
            taskids.len(),
            CuDuration::from(Duration::from_secs(5)),
        )));

        Ok(Self {
            config: config.clone(),
            taskids,
            task_stats,
            task_statuses: Arc::new(Mutex::new(vec![TaskStatus::default(); taskids.len()])),
            quitting: Arc::new(AtomicBool::new(false)),
            pool_stats: Arc::new(Mutex::new(Vec::new())),
        })
    }

    fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
        let config_dup = self.config.clone();
        let taskids = self.taskids;

        let task_stats_ui = self.task_stats.clone();
        let error_states = self.task_statuses.clone();
        let pool_stats_ui = self.pool_stats.clone();
        let quitting = self.quitting.clone();

        // Start the main UI loop
        thread::spawn(move || {
            let backend = CrosstermBackend::new(stdout());

            setup_terminal();

            let mut terminal =
                Terminal::new(backend).expect("Failed to initialize terminal backend");

            #[cfg(feature = "debug_pane")]
            {
                // redirect stderr, so it doesn't pop in the terminal
                let error_redirect = gag::BufferRedirect::stderr().unwrap();

                let mut ui = UI::new(
                    config_dup,
                    None, // FIXME(gbin): Allow somethere an API to get the current mission running
                    taskids,
                    task_stats_ui,
                    error_states,
                    error_redirect,
                    None,
                    pool_stats_ui,
                );

                // Override the cu29-log-runtime Log Subscriber
                #[cfg(debug_assertions)]
                if cu29_log_runtime::EXTRA_TEXT_LOGGER
                    .read()
                    .unwrap()
                    .is_some()
                {
                    let max_lines = terminal.size().unwrap().height - 5;
                    let (debug_log, tx) = debug_pane::DebugLog::new(max_lines);

                    let log_subscriber = debug_pane::LogSubscriber::new(tx);

                    *cu29_log_runtime::EXTRA_TEXT_LOGGER.write().unwrap() =
                        Some(Box::new(log_subscriber) as Box<dyn log::Log>);

                    // Set up the terminal again, as there might be some logs which in the console before updating `EXTRA_TEXT_LOGGER`
                    setup_terminal();

                    ui.debug_output = Some(debug_log);
                } else {
                    println!("EXTRA_TEXT_LOGGER is none");
                }
                ui.run_app(&mut terminal).expect("Failed to run app");
            }

            #[cfg(not(feature = "debug_pane"))]
            {
                let stderr_gag = gag::Gag::stderr().unwrap();

                let mut ui = UI::new(
                    config_dup,
                    taskids,
                    task_stats_ui,
                    error_states,
                    pool_stats_ui,
                );
                ui.run_app(&mut terminal).expect("Failed to run app");

                drop(stderr_gag);
            }

            quitting.store(true, Ordering::SeqCst);
            // restoring the terminal
            restore_terminal();
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
                let CuCompactString(status_txt) = &msg.status_txt;
                task_statuses[i].status_txt = status_txt.clone();
                if task_statuses[i].status_txt.as_bytes()[0] == 0 {
                    task_statuses[i].status_txt = "".to_compact_string();
                }
            }
        }

        // Update pool statistics
        {
            let pool_stats_data = pool::pools_statistics();
            let mut pool_stats = self.pool_stats.lock().unwrap();

            // Update existing pools or add new ones
            for (id, space_left, total_size, buffer_size) in pool_stats_data {
                let id_str = id.to_string();
                if let Some(existing) = pool_stats.iter_mut().find(|p| p.id == id_str) {
                    existing.update(space_left, total_size);
                } else {
                    pool_stats.push(PoolStats::new(id_str, space_left, total_size, buffer_size));
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
        restore_terminal();

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
        restore_terminal();
        error(e)
    }))
    .unwrap();
    std::panic::set_hook(Box::new(move |info| {
        restore_terminal();
        panic(info)
    }));
}

fn setup_terminal() {
    enable_raw_mode().expect("Could not enter raw mode: check terminal compatibility.");
    execute!(stdout(), EnterAlternateScreen, EnableMouseCapture)
        .expect("Could not enter alternateScreen: check terminal compatibility.");
}

fn restore_terminal() {
    execute!(stdout(), LeaveAlternateScreen, DisableMouseCapture)
        .expect("Could not leave alternate screen");
    disable_raw_mode().expect("Could not restore the terminal.");
}
