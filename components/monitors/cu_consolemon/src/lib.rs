#[cfg(feature = "debug_pane")]
mod debug_pane;
pub mod sysinfo;
mod tui_nodes;

use crate::tui_nodes::{Connection, NodeGraph, NodeLayout};
use ansi_to_tui::IntoText;
use color_eyre::config::HookBuilder;
use compact_str::{CompactString, ToCompactString};
use cu29::clock::{CuDuration, RobotClock};
use cu29::config::CuConfig;
use cu29::cutask::CuMsgMetadata;
use cu29::monitoring::{
    ComponentKind, CuDurationStatistics, CuMonitor, CuTaskState, Decision, MonitorTopology,
};
use cu29::prelude::{pool, CuCompactString, CuTime};
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
use ratatui::text::{Line, Span, Text};
use ratatui::widgets::{Block, Borders, Cell, Paragraph, Row, StatefulWidget, Table};
use ratatui::{Frame, Terminal};
use std::fmt::{Display, Formatter};
use std::io::stdout;
use std::marker::PhantomData;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, OnceLock};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};
use std::{collections::HashMap, io, panic, thread};
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
    let start = msgs.first().map(|m| m.process_time.start);
    let end = msgs.last().map(|m| m.process_time.end);

    match (start, end) {
        (Some(s), Some(e)) => match (Option::<CuTime>::from(s), Option::<CuTime>::from(e)) {
            (Some(s), Some(e)) if e >= s => e - s,
            (Some(_), Some(_)) => CuDuration::MIN,
            _ => CuDuration::MIN,
        },
        _ => CuDuration::MIN,
    }
}

// This is kind of terrible.
#[derive(Copy, Clone)]
enum NodeType {
    Unknown,
    Source,
    Sink,
    Task,
    Bridge,
}

impl Display for NodeType {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Unknown => write!(f, "?"),
            Self::Source => write!(f, "◈"),
            Self::Task => write!(f, "⚙"),
            Self::Sink => write!(f, "⭳"),
            Self::Bridge => write!(f, "⇆"),
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
            Self::Bridge => Self::Bridge,
        }
    }

    fn add_outgoing(self) -> NodeType {
        match self {
            Self::Unknown => Self::Source,
            Self::Source => Self::Source,
            Self::Sink => Self::Task,
            Self::Task => Self::Task,
            Self::Bridge => Self::Bridge,
        }
    }

    fn color(self) -> Color {
        match self {
            Self::Unknown => Color::Gray,
            Self::Source => Color::Rgb(255, 191, 0),
            Self::Sink => Color::Rgb(255, 102, 204),
            Self::Task => Color::White,
            Self::Bridge => Color::Rgb(204, 153, 255),
        }
    }
}

#[derive(Default, Clone)]
struct TaskStatus {
    is_error: bool,
    status_txt: CompactString,
    error: CompactString,
}

#[derive(Clone)]
struct DisplayNode {
    id: String,
    type_label: String,
    node_type: NodeType,
    inputs: Vec<String>,
    outputs: Vec<String>,
}

struct NodesScrollableWidgetState {
    display_nodes: Vec<DisplayNode>,
    connections: Vec<Connection>,
    statuses: Arc<Mutex<Vec<TaskStatus>>>,
    status_index_map: Vec<Option<usize>>,
    task_count: usize,
    nodes_scrollable_state: ScrollViewState,
}

impl NodesScrollableWidgetState {
    fn new(
        config: &CuConfig,
        errors: Arc<Mutex<Vec<TaskStatus>>>,
        mission: Option<&str>,
        task_ids: &'static [&'static str],
        topology: Option<MonitorTopology>,
    ) -> Self {
        let topology = topology
            .or_else(|| cu29::monitoring::build_monitor_topology(config, mission).ok())
            .unwrap_or_default();

        let mut display_nodes: Vec<DisplayNode> = Vec::new();
        let mut status_index_map = Vec::new();
        let mut node_lookup = HashMap::new();
        let task_index_lookup: HashMap<&str, usize> = task_ids
            .iter()
            .enumerate()
            .map(|(i, id)| (*id, i))
            .collect();

        for node in topology.nodes.iter() {
            let node_type = match node.kind {
                ComponentKind::Bridge => NodeType::Bridge,
                ComponentKind::Task => {
                    let mut role = NodeType::Unknown;
                    if !node.inputs.is_empty() {
                        role = role.add_incoming();
                    }
                    if !node.outputs.is_empty() {
                        role = role.add_outgoing();
                    }
                    role
                }
            };

            display_nodes.push(DisplayNode {
                id: node.id.clone(),
                type_label: node
                    .type_name
                    .clone()
                    .unwrap_or_else(|| "unknown".to_string()),
                node_type,
                inputs: node.inputs.clone(),
                outputs: node.outputs.clone(),
            });
            let idx = display_nodes.len() - 1;
            node_lookup.insert(node.id.clone(), idx);

            let status_idx = match node.kind {
                ComponentKind::Task => task_index_lookup.get(node.id.as_str()).cloned(),
                ComponentKind::Bridge => None,
            };
            status_index_map.push(status_idx);
        }

        let mut connections: Vec<Connection> = Vec::with_capacity(topology.connections.len());
        for cnx in topology.connections.iter() {
            let Some(&src_idx) = node_lookup.get(&cnx.src) else {
                continue;
            };
            let Some(&dst_idx) = node_lookup.get(&cnx.dst) else {
                continue;
            };
            let src_node = &display_nodes[src_idx];
            let dst_node = &display_nodes[dst_idx];
            let src_port = cnx
                .src_port
                .as_ref()
                .and_then(|p| src_node.outputs.iter().position(|name| name == p))
                .unwrap_or(0);
            let dst_port = cnx
                .dst_port
                .as_ref()
                .and_then(|p| dst_node.inputs.iter().position(|name| name == p))
                .unwrap_or(0);

            connections.push(Connection::new(
                src_idx,
                src_port + NODE_PORT_ROW_OFFSET,
                dst_idx,
                dst_port + NODE_PORT_ROW_OFFSET,
            ));
        }

        // tui-nodes drops all nodes when every node has an outgoing edge (no roots).
        // If that happens, drop the outgoing edges for the first node so at least one root exists.
        if !display_nodes.is_empty() {
            let mut from_set = std::collections::HashSet::new();
            for conn in &connections {
                from_set.insert(conn.from_node);
            }
            if from_set.len() == display_nodes.len() {
                let root_idx = 0;
                connections.retain(|c| c.from_node != root_idx);
            }
        }

        NodesScrollableWidgetState {
            display_nodes,
            connections,
            nodes_scrollable_state: ScrollViewState::default(),
            statuses: errors,
            status_index_map,
            task_count: task_ids.len(),
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
const NODE_META_LINES: usize = 2;
const NODE_PORT_ROW_OFFSET: usize = NODE_META_LINES;

fn clip_tail(value: &str, max_chars: usize) -> String {
    if max_chars == 0 {
        return String::new();
    }
    let char_count = value.chars().count();
    if char_count <= max_chars {
        return value.to_string();
    }
    let skip = char_count.saturating_sub(max_chars);
    let start = value
        .char_indices()
        .nth(skip)
        .map(|(idx, _)| idx)
        .unwrap_or(value.len());
    value[start..].to_string()
}

#[allow(dead_code)]
const NODE_HEIGHT_CONTENT: u16 = NODE_HEIGHT - 2;
const GRAPH_WIDTH_PADDING: u16 = NODE_WIDTH;
const GRAPH_HEIGHT_PADDING: u16 = NODE_HEIGHT;

impl StatefulWidget for NodesScrollableWidget<'_> {
    type State = NodesScrollableWidgetState;

    fn render(self, area: Rect, buf: &mut Buffer, state: &mut Self::State) {
        let build_node_layouts = || {
            state
                .display_nodes
                .iter()
                .map(|node| {
                    let ports = node.inputs.len().max(node.outputs.len());
                    let content_rows = ports + NODE_PORT_ROW_OFFSET;
                    let height = (content_rows as u16).saturating_add(2).max(NODE_HEIGHT);
                    let mut title_line = Line::default();
                    title_line.spans.push(Span::styled(
                        format!(" {}", node.node_type),
                        Style::default().fg(node.node_type.color()),
                    ));
                    title_line.spans.push(Span::styled(
                        format!(" {} ", node.id),
                        Style::default().fg(Color::White),
                    ));
                    NodeLayout::new((NODE_WIDTH, height)).with_title_line(title_line)
                })
                .collect::<Vec<_>>()
        };

        let node_count = state.display_nodes.len().max(1);
        let content_width = (node_count as u16)
            .saturating_mul(NODE_WIDTH + 12)
            .max(NODE_WIDTH);
        let max_ports = state
            .display_nodes
            .iter()
            .map(|node| node.inputs.len().max(node.outputs.len()))
            .max()
            .unwrap_or_default();
        let content_height = (((max_ports + NODE_PORT_ROW_OFFSET) as u16) * 4).max(NODE_HEIGHT);
        let connections = state.connections.clone();
        let build_graph = |width: u16, height: u16| {
            NodeGraph::new(
                build_node_layouts(),
                connections.clone(),
                width as usize,
                height as usize,
            )
        };
        let mut graph = build_graph(content_width, content_height);
        graph.calculate();
        let mut content_size = Size::new(content_width, content_height);
        if state.display_nodes.is_empty() {
            content_size = Size::new(area.width.max(NODE_WIDTH), area.height.max(NODE_HEIGHT));
            graph = build_graph(content_size.width, content_size.height);
            graph.calculate();
        } else {
            let bounds = graph.content_bounds();
            let desired_width = bounds
                .width
                .saturating_add(GRAPH_WIDTH_PADDING)
                .max(NODE_WIDTH);
            let desired_height = bounds
                .height
                .saturating_add(GRAPH_HEIGHT_PADDING)
                .max(NODE_HEIGHT);
            if desired_width != content_size.width || desired_height != content_size.height {
                content_size = Size::new(desired_width, desired_height);
                graph = build_graph(content_size.width, content_size.height);
                graph.calculate();
            }
        }
        let mut scroll_view = ScrollView::new(content_size);
        let zones = graph.split(scroll_view.area());

        {
            let mut statuses = state.statuses.lock().unwrap();
            if statuses.len() <= state.task_count {
                statuses.resize(state.task_count + 1, TaskStatus::default());
            }
            for (idx, ea_zone) in zones.into_iter().enumerate() {
                let fallback_idx = state.task_count;
                let status_idx = state
                    .status_index_map
                    .get(idx)
                    .and_then(|opt| *opt)
                    .unwrap_or(fallback_idx);
                let safe_index = if status_idx < statuses.len() {
                    status_idx
                } else {
                    statuses.len() - 1
                };
                let status = &mut statuses[safe_index];
                let s = &state.display_nodes[idx].type_label;
                let status_line = if status.is_error {
                    format!("❌ {}", status.error)
                } else {
                    format!("✓ {}", status.status_txt)
                };

                let label_width = (NODE_WIDTH_CONTENT as usize).saturating_sub(2);
                let type_label = clip_tail(s, label_width);
                let status_text = clip_tail(&status_line, label_width);
                let base_style = if status.is_error {
                    Style::default().fg(Color::Red)
                } else {
                    Style::default().fg(Color::Green)
                };
                let mut lines: Vec<Line> = Vec::new();
                lines.push(Line::styled(format!(" {}", type_label), base_style));
                lines.push(Line::styled(format!(" {}", status_text), base_style));

                let max_ports = state.display_nodes[idx]
                    .inputs
                    .len()
                    .max(state.display_nodes[idx].outputs.len());
                if max_ports > 0 {
                    let left_width = (NODE_WIDTH_CONTENT as usize - 2) / 2;
                    let right_width = NODE_WIDTH_CONTENT as usize - 2 - left_width;
                    let input_style = Style::default().fg(Color::Yellow);
                    let output_style = Style::default().fg(Color::Cyan);
                    let dotted_style = Style::default().fg(Color::DarkGray);
                    for port_idx in 0..max_ports {
                        let input = state.display_nodes[idx]
                            .inputs
                            .get(port_idx)
                            .map(|label| clip_tail(label, left_width))
                            .unwrap_or_default();
                        let output = state.display_nodes[idx]
                            .outputs
                            .get(port_idx)
                            .map(|label| clip_tail(label, right_width))
                            .unwrap_or_default();
                        let mut port_line = Line::default();
                        port_line.spans.push(Span::styled(
                            format!(" {:<left_width$}", input, left_width = left_width),
                            input_style,
                        ));
                        port_line.spans.push(Span::styled("┆", dotted_style));
                        port_line.spans.push(Span::styled(
                            format!("{:>right_width$}", output, right_width = right_width),
                            output_style,
                        ));
                        lines.push(port_line);
                    }
                }

                let txt = Text::from(lines);
                let paragraph = Paragraph::new(txt);
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
    ui_handle: Option<JoinHandle<()>>,
    pool_stats: Arc<Mutex<Vec<PoolStats>>>,
    quitting: Arc<AtomicBool>,
    topology: Option<MonitorTopology>,
}

impl Drop for CuConsoleMon {
    fn drop(&mut self) {
        self.quitting.store(true, Ordering::SeqCst);
        let _ = restore_terminal();
        if let Some(handle) = self.ui_handle.take() {
            let _ = handle.join();
        }
    }
}

struct UI {
    task_ids: &'static [&'static str],
    active_screen: Screen,
    sysinfo: String,
    task_stats: Arc<Mutex<TaskStats>>,
    quitting: Arc<AtomicBool>,
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
        quitting: Arc<AtomicBool>,
        error_redirect: gag::BufferRedirect,
        debug_output: Option<debug_pane::DebugLog>,
        pool_stats: Arc<Mutex<Vec<PoolStats>>>,
        topology: Option<MonitorTopology>,
    ) -> UI {
        init_error_hooks();
        let nodes_scrollable_widget_state = NodesScrollableWidgetState::new(
            &config,
            task_statuses.clone(),
            mission,
            task_ids,
            topology.clone(),
        );

        Self {
            task_ids,
            active_screen: Screen::Neofetch,
            sysinfo: sysinfo::pfetch_info(),
            task_stats,
            quitting,
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
        quitting: Arc<AtomicBool>,
        pool_stats: Arc<Mutex<Vec<PoolStats>>>,
        topology: Option<MonitorTopology>,
    ) -> UI {
        init_error_hooks();
        let nodes_scrollable_widget_state = NodesScrollableWidgetState::new(
            &config,
            task_statuses.clone(),
            None,
            task_ids,
            topology.clone(),
        );

        Self {
            task_ids,
            active_screen: Screen::Neofetch,
            sysinfo: sysinfo::pfetch_info(),
            task_stats,
            quitting,
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
            if self.quitting.load(Ordering::SeqCst) {
                break;
            }
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
            ui_handle: None,
            quitting: Arc::new(AtomicBool::new(false)),
            pool_stats: Arc::new(Mutex::new(Vec::new())),
            topology: None,
        })
    }
    fn set_topology(&mut self, topology: MonitorTopology) {
        self.topology = Some(topology);
    }

    fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
        let config_dup = self.config.clone();
        let taskids = self.taskids;

        let task_stats_ui = self.task_stats.clone();
        let error_states = self.task_statuses.clone();
        let pool_stats_ui = self.pool_stats.clone();
        let quitting = self.quitting.clone();
        let topology = self.topology.clone();

        // Start the main UI loop
        let handle = thread::spawn(move || {
            let backend = CrosstermBackend::new(stdout());
            let _terminal_guard = TerminalRestoreGuard;

            // Ensure the terminal is restored if a panic occurs while the TUI is active.
            let prev_hook = panic::take_hook();
            panic::set_hook(Box::new(move |info| {
                let _ = restore_terminal();
                prev_hook(info);
            }));

            install_signal_handlers(quitting.clone());

            if let Err(err) = setup_terminal() {
                eprintln!("Failed to prepare terminal UI: {err}");
                return;
            }

            let mut terminal = match Terminal::new(backend) {
                Ok(terminal) => terminal,
                Err(err) => {
                    eprintln!("Failed to initialize terminal backend: {err}");
                    return;
                }
            };

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
                    quitting.clone(),
                    error_redirect,
                    None,
                    pool_stats_ui,
                    topology.clone(),
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
                    if let Err(err) = setup_terminal() {
                        eprintln!("Failed to reinitialize terminal after log redirect: {err}");
                    }

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
                    quitting,
                    pool_stats_ui,
                    topology,
                );
                ui.run_app(&mut terminal).expect("Failed to run app");

                drop(stderr_gag);
            }

            quitting.store(true, Ordering::SeqCst);
            // restoring the terminal
            let _ = restore_terminal();
        });

        self.ui_handle = Some(handle);
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
        self.quitting.store(true, Ordering::SeqCst);
        let _ = restore_terminal();

        if let Some(handle) = self.ui_handle.take() {
            let _ = handle.join();
        }

        self.task_stats
            .lock()
            .unwrap()
            .stats
            .iter_mut()
            .for_each(|s| s.reset());
        Ok(())
    }
}

struct TerminalRestoreGuard;

impl Drop for TerminalRestoreGuard {
    fn drop(&mut self) {
        let _ = restore_terminal();
    }
}

fn install_signal_handlers(quitting: Arc<AtomicBool>) {
    static SIGNAL_HANDLER: OnceLock<()> = OnceLock::new();

    let _ = SIGNAL_HANDLER.get_or_init(|| {
        let quitting = quitting.clone();
        if let Err(err) = ctrlc::set_handler(move || {
            quitting.store(true, Ordering::SeqCst);
            let _ = restore_terminal();
            // Match the conventional 130 exit code for Ctrl-C.
            std::process::exit(130);
        }) {
            eprintln!("Failed to install Ctrl-C handler: {err}");
        }
    });
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

fn setup_terminal() -> io::Result<()> {
    enable_raw_mode()?;
    execute!(stdout(), EnterAlternateScreen, EnableMouseCapture)?;
    Ok(())
}

fn restore_terminal() -> io::Result<()> {
    execute!(stdout(), LeaveAlternateScreen, DisableMouseCapture)?;
    disable_raw_mode()
}
