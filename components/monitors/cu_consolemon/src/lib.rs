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
use cu29::config::Flavor;
use cu29::curuntime::{CuExecutionUnit, compute_runtime_plan};
use cu29::cutask::CuMsgMetadata;
use cu29::monitoring::{
    ComponentKind, CopperListInfo, CopperListIoStats, CuDurationStatistics, CuMonitor, CuTaskState,
    Decision, MonitorTopology,
};
use cu29::prelude::{CuCompactString, CuTime, pool};
use cu29::{CuError, CuResult};
use cu29_log::CuLogLevel;
#[cfg(feature = "debug_pane")]
use debug_pane::{StyledLine, StyledRun, UIExt};
use ratatui::backend::CrosstermBackend;
use ratatui::buffer::Buffer;
use ratatui::crossterm::event::{
    DisableMouseCapture, EnableMouseCapture, Event, KeyCode, MouseButton, MouseEventKind,
};
use ratatui::crossterm::terminal::{
    EnterAlternateScreen, LeaveAlternateScreen, disable_raw_mode, enable_raw_mode,
};
use ratatui::crossterm::tty::IsTty;
use ratatui::crossterm::{event, execute};
use ratatui::layout::{Alignment, Constraint, Direction, Layout, Position, Size};
use ratatui::prelude::Stylize;
use ratatui::prelude::{Backend, Rect};
use ratatui::style::{Color, Modifier, Style};
use ratatui::text::{Line, Span, Text};
use ratatui::widgets::{Block, BorderType, Borders, Cell, Paragraph, Row, StatefulWidget, Table};
use ratatui::{Frame, Terminal};
use std::backtrace::Backtrace;
use std::fmt::{Display, Formatter};
use std::io::{Write, stdin, stdout};
use std::marker::PhantomData;
use std::process;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, OnceLock};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};
use std::{collections::HashMap, io, thread};
use tui_widgets::scrollview::{ScrollView, ScrollViewState};

#[cfg(feature = "debug_pane")]
use arboard::Clipboard;

#[derive(Clone, Copy)]
struct TabDef {
    screen: Screen,
    label: &'static str,
    key: &'static str,
}

#[derive(Clone, Copy)]
struct TabHitbox {
    screen: Screen,
    x: u16,
    y: u16,
    width: u16,
    height: u16,
}

#[derive(Clone, Copy)]
enum HelpAction {
    ResetLatency,
    Quit,
}

#[derive(Clone, Copy)]
struct HelpHitbox {
    action: HelpAction,
    x: u16,
    y: u16,
    width: u16,
    height: u16,
}

const TAB_DEFS: &[TabDef] = &[
    TabDef {
        screen: Screen::Neofetch,
        label: "SYS",
        key: "1",
    },
    TabDef {
        screen: Screen::Dag,
        label: "DAG",
        key: "2",
    },
    TabDef {
        screen: Screen::Latency,
        label: "LAT",
        key: "3",
    },
    TabDef {
        screen: Screen::CopperList,
        label: "BW",
        key: "4",
    },
    TabDef {
        screen: Screen::MemoryPools,
        label: "MEM",
        key: "5",
    },
    #[cfg(feature = "debug_pane")]
    TabDef {
        screen: Screen::DebugOutput,
        label: "LOG",
        key: "6",
    },
];

const COPPERLIST_RATE_WINDOW: Duration = Duration::from_secs(1);

#[derive(Clone, Copy, PartialEq)]
enum Screen {
    Neofetch,
    Dag,
    Latency,
    MemoryPools,
    CopperList,
    #[cfg(feature = "debug_pane")]
    DebugOutput,
}

#[cfg(feature = "debug_pane")]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
struct SelectionPoint {
    row: usize,
    col: usize,
}

#[cfg(feature = "debug_pane")]
#[derive(Clone, Copy, Debug, Default)]
struct DebugSelection {
    anchor: Option<SelectionPoint>,
    cursor: Option<SelectionPoint>,
}

#[cfg(feature = "debug_pane")]
impl DebugSelection {
    fn clear(&mut self) {
        self.anchor = None;
        self.cursor = None;
    }

    fn start(&mut self, point: SelectionPoint) {
        self.anchor = Some(point);
        self.cursor = Some(point);
    }

    fn update(&mut self, point: SelectionPoint) {
        if self.anchor.is_some() {
            self.cursor = Some(point);
        }
    }

    fn range(&self) -> Option<(SelectionPoint, SelectionPoint)> {
        let anchor = self.anchor?;
        let cursor = self.cursor?;
        if (anchor.row, anchor.col) <= (cursor.row, cursor.col) {
            Some((anchor, cursor))
        } else {
            Some((cursor, anchor))
        }
    }
}

fn segment_width(key: &str, label: &str) -> u16 {
    (6 + key.chars().count() + label.chars().count()) as u16
}

fn mouse_inside(mouse: &event::MouseEvent, x: u16, y: u16, width: u16, height: u16) -> bool {
    mouse.column >= x && mouse.column < x + width && mouse.row >= y && mouse.row < y + height
}

#[cfg(feature = "debug_pane")]
fn char_to_byte_index(text: &str, char_idx: usize) -> usize {
    text.char_indices()
        .nth(char_idx)
        .map(|(idx, _)| idx)
        .unwrap_or(text.len())
}

#[cfg(feature = "debug_pane")]
fn slice_char_range(text: &str, start: usize, end: usize) -> (&str, &str, &str) {
    let start_idx = char_to_byte_index(text, start);
    let end_idx = char_to_byte_index(text, end);
    let start_idx = start_idx.min(text.len());
    let end_idx = end_idx.min(text.len());
    let (start_idx, end_idx) = if start_idx <= end_idx {
        (start_idx, end_idx)
    } else {
        (end_idx, start_idx)
    };
    (
        &text[..start_idx],
        &text[start_idx..end_idx],
        &text[end_idx..],
    )
}

#[cfg(feature = "debug_pane")]
fn line_selection_bounds(
    line_index: usize,
    line_len: usize,
    start: SelectionPoint,
    end: SelectionPoint,
) -> Option<(usize, usize)> {
    if line_index < start.row || line_index > end.row {
        return None;
    }

    let start_col = if line_index == start.row {
        start.col
    } else {
        0
    };
    let mut end_col = if line_index == end.row {
        end.col
    } else {
        line_len
    };
    if line_index == end.row {
        end_col = end_col.saturating_add(1).min(line_len);
    }
    let start_col = start_col.min(line_len);
    let end_col = end_col.min(line_len);
    if start_col >= end_col {
        return None;
    }

    Some((start_col, end_col))
}

#[cfg(feature = "debug_pane")]
fn slice_chars_owned(text: &str, start: usize, end: usize) -> String {
    let start_idx = char_to_byte_index(text, start);
    let end_idx = char_to_byte_index(text, end);
    text[start_idx.min(text.len())..end_idx.min(text.len())].to_string()
}

#[cfg(feature = "debug_pane")]
fn spans_from_runs(line: &StyledLine) -> Vec<Span<'static>> {
    if line.runs.is_empty() {
        return vec![Span::raw(line.text.clone())];
    }

    let mut spans = Vec::new();
    let mut cursor = 0usize;
    let total_chars = line.text.chars().count();
    let mut runs = line.runs.clone();
    runs.sort_by_key(|r| r.start);

    for run in runs {
        let start = run.start.min(total_chars);
        let end = run.end.min(total_chars);
        if start > cursor {
            let before = slice_chars_owned(&line.text, cursor, start);
            if !before.is_empty() {
                spans.push(Span::raw(before));
            }
        }
        if end > start {
            let segment = slice_chars_owned(&line.text, start, end);
            spans.push(Span::styled(segment, run.style));
        }
        cursor = cursor.max(end);
    }

    if cursor < total_chars {
        let tail = slice_chars_owned(&line.text, cursor, total_chars);
        if !tail.is_empty() {
            spans.push(Span::raw(tail));
        }
    }

    spans
}

#[cfg(feature = "debug_pane")]
fn color_for_level(level: CuLogLevel) -> Color {
    match level {
        CuLogLevel::Debug => Color::Green,
        CuLogLevel::Info => Color::Gray,
        CuLogLevel::Warning => Color::Yellow,
        CuLogLevel::Error => Color::Red,
        CuLogLevel::Critical => Color::Red,
    }
}

#[cfg(feature = "debug_pane")]
fn format_ts(time: CuTime) -> String {
    let nanos = time.as_nanos();
    let total_ms = nanos / 1_000_000;
    let millis = total_ms % 1000;
    let total_s = total_ms / 1000;
    let secs = total_s % 60;
    let mins = (total_s / 60) % 60;
    let hours = (total_s / 3600) % 24;
    format!("{hours:02}:{mins:02}:{secs:02}.{millis:03}")
}

#[cfg(feature = "debug_pane")]
fn build_message_with_runs(
    format_str: &str,
    params: &[String],
    named_params: &HashMap<String, String>,
) -> (String, Vec<(usize, usize)>) {
    let mut out = String::new();
    let mut param_spans = Vec::new();
    let mut anon_iter = params.iter();
    let mut idx = 0;
    let bytes = format_str.as_bytes();
    while idx < bytes.len() {
        if bytes[idx] == b'{' {
            if let Some(end) = format_str[idx + 1..].find('}') {
                let end_idx = idx + 1 + end;
                let placeholder = &format_str[idx + 1..end_idx];
                let replacement_opt = if placeholder.is_empty() {
                    anon_iter.next()
                } else {
                    named_params.get(placeholder)
                };
                if let Some(repl) = replacement_opt {
                    let start = out.chars().count();
                    out.push_str(repl);
                    let end = out.chars().count();
                    param_spans.push((start, end));
                    idx = end_idx + 1;
                    continue;
                }
            }
        }
        out.push(format_str[idx..idx + 1].chars().next().unwrap());
        idx += 1;
    }
    (out, param_spans)
}

#[cfg(feature = "debug_pane")]
fn styled_line_from_structured(
    time: CuTime,
    level: CuLogLevel,
    format_str: &str,
    params: &[String],
    named_params: &HashMap<String, String>,
) -> StyledLine {
    let ts = format_ts(time);
    let level_txt = format!("[{:?}]", level);

    let (msg_text, param_spans) = build_message_with_runs(format_str, params, named_params);
    let mut msg_runs = Vec::new();
    let mut cursor = 0usize;
    let param_spans_sorted = {
        let mut v = param_spans;
        v.sort_by_key(|p| p.0);
        v
    };
    for (start, end) in param_spans_sorted {
        if start > cursor {
            msg_runs.push(StyledRun {
                start: cursor,
                end: start,
                style: Style::default().fg(Color::Gray),
            });
        }
        msg_runs.push(StyledRun {
            start,
            end,
            style: Style::default().fg(Color::Magenta),
        });
        cursor = end;
    }
    if cursor < msg_text.chars().count() {
        msg_runs.push(StyledRun {
            start: cursor,
            end: msg_text.chars().count(),
            style: Style::default().fg(Color::Gray),
        });
    }

    let prefix = format!("{ts} {level_txt}: ");
    let prefix_len = prefix.chars().count();
    let line_text = format!("{prefix}{msg_text}");

    let mut runs = Vec::new();
    let ts_len = ts.chars().count();
    let level_start = ts_len + 1;
    let level_end = level_start + level_txt.chars().count();

    runs.push(StyledRun {
        start: 0,
        end: ts_len,
        style: Style::default().fg(Color::Blue),
    });
    runs.push(StyledRun {
        start: level_start,
        end: level_end,
        style: Style::default().fg(color_for_level(level)).bold(),
    });
    for run in msg_runs {
        runs.push(StyledRun {
            start: prefix_len + run.start,
            end: prefix_len + run.end,
            style: run.style,
        });
    }

    StyledLine {
        text: line_text,
        runs,
    }
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
        }
    }

    fn update(&mut self, space_left: usize, total_size: usize) {
        let now = Instant::now();
        let handles_in_use = total_size - space_left;
        let elapsed = now.duration_since(self.last_update).as_secs_f32();

        if elapsed >= 1.0 {
            self.handles_per_second =
                ((handles_in_use.abs_diff(self.handles_in_use)) as f32 / elapsed) as usize;
            self.last_update = now;
        }

        self.handles_in_use = handles_in_use;
        self.space_left = space_left;
        self.total_size = total_size;
    }
}

struct CopperListStats {
    size_bytes: usize,
    raw_culist_bytes: u64,
    handle_bytes: u64,
    encoded_bytes: u64,
    keyframe_bytes: u64,
    structured_total_bytes: u64,
    structured_bytes_per_cl: u64,
    total_copperlists: u64,
    window_copperlists: u64,
    last_rate_at: Instant,
    rate_hz: f64,
}

impl CopperListStats {
    fn new() -> Self {
        Self {
            size_bytes: 0,
            raw_culist_bytes: 0,
            handle_bytes: 0,
            encoded_bytes: 0,
            keyframe_bytes: 0,
            structured_total_bytes: 0,
            structured_bytes_per_cl: 0,
            total_copperlists: 0,
            window_copperlists: 0,
            last_rate_at: Instant::now(),
            rate_hz: 0.0,
        }
    }

    fn set_info(&mut self, info: CopperListInfo) {
        self.size_bytes = info.size_bytes;
    }

    fn update_io(&mut self, stats: cu29::monitoring::CopperListIoStats) {
        self.raw_culist_bytes = stats.raw_culist_bytes;
        self.handle_bytes = stats.handle_bytes;
        self.encoded_bytes = stats.encoded_culist_bytes;
        self.keyframe_bytes = stats.keyframe_bytes;
        let total = stats.structured_log_bytes_total;
        self.structured_bytes_per_cl = total.saturating_sub(self.structured_total_bytes);
        self.structured_total_bytes = total;
    }

    fn update_rate(&mut self) {
        self.total_copperlists = self.total_copperlists.saturating_add(1);
        self.window_copperlists = self.window_copperlists.saturating_add(1);

        let now = Instant::now();
        let elapsed = now.duration_since(self.last_rate_at);
        if elapsed >= COPPERLIST_RATE_WINDOW {
            let elapsed_secs = elapsed.as_secs_f64();
            self.rate_hz = if elapsed_secs > 0.0 {
                self.window_copperlists as f64 / elapsed_secs
            } else {
                0.0
            };
            self.window_copperlists = 0;
            self.last_rate_at = now;
        }
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

fn format_bytes(bytes: f64) -> String {
    const UNITS: [&str; 4] = ["B", "KiB", "MiB", "GiB"];
    let mut value = bytes;
    let mut unit_idx = 0;
    while value >= 1024.0 && unit_idx < UNITS.len() - 1 {
        value /= 1024.0;
        unit_idx += 1;
    }
    if unit_idx == 0 {
        format!("{:.0} {}", value, UNITS[unit_idx])
    } else {
        format!("{:.2} {}", value, UNITS[unit_idx])
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

#[derive(Clone, Copy, PartialEq, Eq)]
struct GraphCacheKey {
    area: Size,
    node_count: usize,
    connection_count: usize,
}

struct GraphCache {
    graph: Option<NodeGraph<'static>>,
    content_size: Size,
    key: Option<GraphCacheKey>,
    dirty: bool,
}

impl GraphCache {
    fn new() -> Self {
        Self {
            graph: None,
            content_size: Size::ZERO,
            key: None,
            dirty: true,
        }
    }
}

impl GraphCache {
    fn needs_rebuild(&self, key: GraphCacheKey) -> bool {
        self.dirty || self.graph.is_none() || self.key != Some(key)
    }
}

struct NodesScrollableWidgetState {
    display_nodes: Vec<DisplayNode>,
    connections: Vec<Connection>,
    statuses: Arc<Mutex<Vec<TaskStatus>>>,
    status_index_map: Vec<Option<usize>>,
    task_count: usize,
    nodes_scrollable_state: ScrollViewState,
    graph_cache: GraphCache,
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
                ComponentKind::Task => task_ids.iter().position(|id| *id == node.id.as_str()),
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
            statuses: errors,
            status_index_map,
            task_count: task_ids.len(),
            nodes_scrollable_state: ScrollViewState::default(),
            graph_cache: GraphCache::new(),
        }
    }

    fn mark_graph_dirty(&mut self) {
        self.graph_cache.dirty = true;
    }

    fn ensure_graph_cache(&mut self, area: Rect) -> Size {
        let key = self.graph_cache_key(area);
        if self.graph_cache.needs_rebuild(key) {
            self.rebuild_graph_cache(area, key);
        }
        self.graph_cache.content_size
    }

    fn graph(&self) -> &NodeGraph<'static> {
        self.graph_cache
            .graph
            .as_ref()
            .expect("graph cache must be initialized before render")
    }

    fn graph_cache_key(&self, area: Rect) -> GraphCacheKey {
        GraphCacheKey {
            area: area.into(),
            node_count: self.display_nodes.len(),
            connection_count: self.connections.len(),
        }
    }

    fn build_graph(&self, content_size: Size) -> NodeGraph<'static> {
        let mut graph = NodeGraph::new(
            self.build_node_layouts(),
            self.connections.clone(),
            content_size.width as usize,
            content_size.height as usize,
        );
        graph.calculate();
        graph
    }

    fn rebuild_graph_cache(&mut self, area: Rect, key: GraphCacheKey) {
        let content_size = if self.display_nodes.is_empty() {
            Size::new(area.width.max(NODE_WIDTH), area.height.max(NODE_HEIGHT))
        } else {
            let node_count = self.display_nodes.len();
            let content_width = (node_count as u16)
                .saturating_mul(NODE_WIDTH + 20)
                .max(NODE_WIDTH);
            let max_ports = self
                .display_nodes
                .iter()
                .map(|node| node.inputs.len().max(node.outputs.len()))
                .max()
                .unwrap_or_default();
            let content_height =
                (((max_ports + NODE_PORT_ROW_OFFSET) as u16) * 12).max(NODE_HEIGHT * 6);

            let initial_size = Size::new(content_width, content_height);
            let graph = self.build_graph(initial_size);
            let bounds = graph.content_bounds();
            let desired_width = bounds
                .width
                .saturating_add(GRAPH_WIDTH_PADDING)
                .max(NODE_WIDTH);
            let desired_height = bounds
                .height
                .saturating_add(GRAPH_HEIGHT_PADDING)
                .max(NODE_HEIGHT);
            Size::new(desired_width, desired_height)
        };

        self.graph_cache.graph = Some(self.build_graph(content_size));
        self.graph_cache.content_size = content_size;
        self.graph_cache.key = Some(key);
        self.graph_cache.dirty = false;

        self.clamp_scroll_offset(area, content_size);
    }

    fn build_node_layouts(&self) -> Vec<NodeLayout<'static>> {
        self.display_nodes
            .iter()
            .map(|node| {
                let ports = node.inputs.len().max(node.outputs.len());
                let content_rows = ports + NODE_PORT_ROW_OFFSET;
                let height = (content_rows as u16).saturating_add(2).max(NODE_HEIGHT);
                let title_line = Line::from(vec![
                    Span::styled(
                        format!(" {}", node.node_type),
                        Style::default().fg(node.node_type.color()),
                    ),
                    Span::styled(format!(" {} ", node.id), Style::default().fg(Color::White)),
                ]);
                NodeLayout::new((NODE_WIDTH, height)).with_title_line(title_line)
            })
            .collect()
    }

    fn clamp_scroll_offset(&mut self, area: Rect, content_size: Size) {
        let max_x = content_size
            .width
            .saturating_sub(area.width.saturating_sub(1));
        let max_y = content_size
            .height
            .saturating_sub(area.height.saturating_sub(1));
        let offset = self.nodes_scrollable_state.offset();
        let clamped = Position::new(offset.x.min(max_x), offset.y.min(max_y));
        self.nodes_scrollable_state.set_offset(clamped);
    }
}

struct NodesScrollableWidget<'a> {
    _marker: PhantomData<&'a ()>,
}

const NODE_WIDTH: u16 = 29;
const NODE_WIDTH_CONTENT: u16 = NODE_WIDTH - 2;

const NODE_HEIGHT: u16 = 5;
const NODE_META_LINES: usize = 2;
const NODE_PORT_ROW_OFFSET: usize = NODE_META_LINES;
const MAX_CULIST_MAP: usize = 512;

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
const GRAPH_WIDTH_PADDING: u16 = NODE_WIDTH * 2;
const GRAPH_HEIGHT_PADDING: u16 = NODE_HEIGHT * 4;

impl StatefulWidget for NodesScrollableWidget<'_> {
    type State = NodesScrollableWidgetState;

    fn render(self, area: Rect, buf: &mut Buffer, state: &mut Self::State) {
        let content_size = state.ensure_graph_cache(area);
        let mut scroll_view = ScrollView::new(content_size);

        {
            let graph = state.graph();
            let zones = graph.split(scroll_view.area());

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
                let safe_index = status_idx.min(statuses.len().saturating_sub(1));
                let status = &mut statuses[safe_index];
                let node = &state.display_nodes[idx];
                let status_line = if status.is_error {
                    format!("❌ {}", status.error)
                } else {
                    format!("✓ {}", status.status_txt)
                };

                let label_width = (NODE_WIDTH_CONTENT as usize).saturating_sub(2);
                let type_label = clip_tail(&node.type_label, label_width);
                let status_text = clip_tail(&status_line, label_width);
                let base_style = if status.is_error {
                    Style::default().fg(Color::Red)
                } else {
                    Style::default().fg(Color::Green)
                };
                let mut lines: Vec<Line> = Vec::new();
                lines.push(Line::styled(format!(" {}", type_label), base_style));
                lines.push(Line::styled(format!(" {}", status_text), base_style));

                let max_ports = node.inputs.len().max(node.outputs.len());
                if max_ports > 0 {
                    let left_width = (NODE_WIDTH_CONTENT as usize - 2) / 2;
                    let right_width = NODE_WIDTH_CONTENT as usize - 2 - left_width;
                    let input_style = Style::default().fg(Color::Yellow);
                    let output_style = Style::default().fg(Color::Cyan);
                    let dotted_style = Style::default().fg(Color::DarkGray);
                    for port_idx in 0..max_ports {
                        let input = node
                            .inputs
                            .get(port_idx)
                            .map(|label| clip_tail(label, left_width))
                            .unwrap_or_default();
                        let output = node
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

                let paragraph = Paragraph::new(Text::from(lines));
                status.is_error = false; // reset if it was displayed
                scroll_view.render_widget(paragraph, ea_zone);
            }

            let content_area = Rect::new(0, 0, content_size.width, content_size.height);
            scroll_view.render_widget(graph, content_area);
        }

        scroll_view.render(area, buf, &mut state.nodes_scrollable_state);
    }
}

/// A TUI based realtime console for Copper.
pub struct CuConsoleMon {
    config: CuConfig,
    taskids: &'static [&'static str],
    task_stats: Arc<Mutex<TaskStats>>,
    task_statuses: Arc<Mutex<Vec<TaskStatus>>>,
    culist_to_task: [usize; MAX_CULIST_MAP],
    ui_handle: Option<JoinHandle<()>>,
    pool_stats: Arc<Mutex<Vec<PoolStats>>>,
    copperlist_stats: Arc<Mutex<CopperListStats>>,
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
    tab_hitboxes: Vec<TabHitbox>,
    help_hitboxes: Vec<HelpHitbox>,
    nodes_scrollable_widget_state: NodesScrollableWidgetState,
    #[cfg(feature = "debug_pane")]
    error_redirect: Option<gag::BufferRedirect>,
    #[cfg(feature = "debug_pane")]
    debug_output: Option<debug_pane::DebugLog>,
    #[cfg(feature = "debug_pane")]
    debug_output_area: Option<Rect>,
    #[cfg(feature = "debug_pane")]
    debug_output_visible_offset: usize,
    #[cfg(feature = "debug_pane")]
    debug_output_lines: Vec<debug_pane::StyledLine>,
    #[cfg(feature = "debug_pane")]
    debug_selection: DebugSelection,
    #[cfg(feature = "debug_pane")]
    clipboard: Option<Clipboard>,
    pool_stats: Arc<Mutex<Vec<PoolStats>>>,
    copperlist_stats: Arc<Mutex<CopperListStats>>,
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
        error_redirect: Option<gag::BufferRedirect>,
        debug_output: Option<debug_pane::DebugLog>,
        pool_stats: Arc<Mutex<Vec<PoolStats>>>,
        copperlist_stats: Arc<Mutex<CopperListStats>>,
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
            tab_hitboxes: Vec::new(),
            help_hitboxes: Vec::new(),
            nodes_scrollable_widget_state,
            error_redirect,
            debug_output,
            debug_output_area: None,
            debug_output_visible_offset: 0,
            debug_output_lines: Vec::new(),
            debug_selection: DebugSelection::default(),
            clipboard: None,
            pool_stats,
            copperlist_stats,
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
        copperlist_stats: Arc<Mutex<CopperListStats>>,
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
            tab_hitboxes: Vec::new(),
            help_hitboxes: Vec::new(),
            nodes_scrollable_widget_state,
            pool_stats,
            copperlist_stats,
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
        .block(
            Block::default()
                .borders(Borders::ALL)
                .border_type(BorderType::Rounded)
                .title(" Latencies "),
        );

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
                .border_type(BorderType::Rounded)
                .title(" Memory Pools "),
        );

        f.render_widget(table, area);
    }

    fn draw_copperlist_stats(&self, f: &mut Frame, area: Rect) {
        let stats = self.copperlist_stats.lock().unwrap();
        let size_display = if stats.size_bytes > 0 {
            format_bytes(stats.size_bytes as f64)
        } else {
            "unknown".to_string()
        };
        let raw_total = stats.raw_culist_bytes.max(stats.size_bytes as u64);
        let handles_display = if stats.handle_bytes > 0 {
            format_bytes(stats.handle_bytes as f64)
        } else {
            "0 B".to_string()
        };
        let mem_total = raw_total
            .saturating_add(stats.keyframe_bytes)
            .saturating_add(stats.structured_bytes_per_cl);
        let mem_total_display = if mem_total > 0 {
            format_bytes(mem_total as f64)
        } else {
            "unknown".to_string()
        };
        let encoded_display = if stats.encoded_bytes > 0 {
            format_bytes(stats.encoded_bytes as f64)
        } else {
            "n/a".to_string()
        };
        let efficiency_display = if raw_total > 0 && stats.encoded_bytes > 0 {
            let ratio = (stats.encoded_bytes as f64) / (raw_total as f64);
            format!("{:.1}%", ratio * 100.0)
        } else {
            "n/a".to_string()
        };
        let rate_display = format!("{:.2} Hz", stats.rate_hz);
        let raw_bw = if mem_total > 0 {
            format!("{}/s", format_bytes((mem_total as f64) * stats.rate_hz))
        } else {
            "n/a".to_string()
        };
        let _encoded_bw = if stats.encoded_bytes > 0 {
            format!(
                "{}/s",
                format_bytes((stats.encoded_bytes as f64) * stats.rate_hz)
            )
        } else {
            "n/a".to_string()
        };
        let keyframe_display = if stats.keyframe_bytes > 0 {
            format_bytes(stats.keyframe_bytes as f64)
        } else {
            "0 B".to_string()
        };
        let structured_display = if stats.structured_bytes_per_cl > 0 {
            format_bytes(stats.structured_bytes_per_cl as f64)
        } else {
            "0 B".to_string()
        };
        let structured_bw = if stats.structured_bytes_per_cl > 0 {
            format!(
                "{}/s",
                format_bytes((stats.structured_bytes_per_cl as f64) * stats.rate_hz)
            )
        } else {
            "n/a".to_string()
        };
        let disk_total_bytes = stats
            .encoded_bytes
            .saturating_add(stats.keyframe_bytes)
            .saturating_add(stats.structured_bytes_per_cl);
        let disk_total_bw = if disk_total_bytes > 0 {
            format!(
                "{}/s",
                format_bytes((disk_total_bytes as f64) * stats.rate_hz)
            )
        } else {
            "n/a".to_string()
        };

        let header_cells = ["Metric", "Value"].iter().map(|h| {
            Cell::from(Line::from(*h)).style(
                Style::default()
                    .fg(Color::Yellow)
                    .add_modifier(Modifier::BOLD),
            )
        });

        let header = Row::new(header_cells).bottom_margin(1);

        let spacer = Row::new(vec![
            Cell::from(Line::from(" ")),
            Cell::from(Line::from(" ")),
        ]);

        let rate_style = Style::default().fg(Color::Cyan);
        let mem_rows = vec![
            Row::new(vec![
                Cell::from(Line::from("Observed rate")),
                Cell::from(Line::from(rate_display).alignment(Alignment::Right)),
            ])
            .style(rate_style),
            spacer.clone(),
            Row::new(vec![
                Cell::from(Line::from("CopperList size")),
                Cell::from(Line::from(size_display).alignment(Alignment::Right)),
            ]),
            Row::new(vec![
                Cell::from(Line::from("Pool memory used")),
                Cell::from(Line::from(handles_display).alignment(Alignment::Right)),
            ]),
            Row::new(vec![
                Cell::from(Line::from("Keyframe size")),
                Cell::from(Line::from(keyframe_display).alignment(Alignment::Right)),
            ]),
            Row::new(vec![
                Cell::from(Line::from("Mem total (CL+KF+SL)")),
                Cell::from(Line::from(mem_total_display).alignment(Alignment::Right)),
            ]),
            spacer.clone(),
            Row::new(vec![
                Cell::from(Line::from("RAM BW (raw)")),
                Cell::from(Line::from(raw_bw).alignment(Alignment::Right)),
            ]),
        ];

        let disk_rows = vec![
            Row::new(vec![
                Cell::from(Line::from("CL serialized size")),
                Cell::from(Line::from(encoded_display).alignment(Alignment::Right)),
            ]),
            Row::new(vec![
                Cell::from(Line::from("CL encoding efficiency")),
                Cell::from(Line::from(efficiency_display).alignment(Alignment::Right)),
            ]),
            Row::new(vec![
                Cell::from(Line::from("Structured log / CL")),
                Cell::from(Line::from(structured_display).alignment(Alignment::Right)),
            ]),
            Row::new(vec![
                Cell::from(Line::from("Structured BW")),
                Cell::from(Line::from(structured_bw).alignment(Alignment::Right)),
            ]),
            spacer.clone(),
            Row::new(vec![
                Cell::from(Line::from("Total disk BW")),
                Cell::from(Line::from(disk_total_bw).alignment(Alignment::Right)),
            ]),
        ];

        let mem_table = Table::new(mem_rows, &[Constraint::Length(24), Constraint::Length(12)])
            .header(header.clone())
            .block(
                Block::default()
                    .borders(Borders::ALL)
                    .border_type(BorderType::Rounded)
                    .title(" Memory BW "),
            );

        let disk_table = Table::new(disk_rows, &[Constraint::Length(24), Constraint::Length(12)])
            .header(header)
            .block(
                Block::default()
                    .borders(Borders::ALL)
                    .border_type(BorderType::Rounded)
                    .title(" Disk / Encoding "),
            );

        let layout = Layout::default()
            .direction(Direction::Horizontal)
            .constraints([Constraint::Length(42), Constraint::Length(42)].as_ref())
            .split(area);

        f.render_widget(mem_table, layout[0]);
        f.render_widget(disk_table, layout[1]);
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

    fn render_tabs(&mut self, f: &mut Frame, area: Rect) {
        let base_bg = Color::Rgb(16, 18, 20);
        let active_bg = Color::Rgb(56, 110, 120);
        let inactive_bg = Color::Rgb(40, 44, 52);
        let active_fg = Color::Rgb(245, 246, 247);
        let inactive_fg = Color::Rgb(198, 200, 204);
        let key_fg = Color::Rgb(255, 208, 128);

        let mut spans = Vec::new();
        self.tab_hitboxes.clear();
        let mut cursor_x = area.x;
        spans.push(Span::styled(" ", Style::default().bg(base_bg)));
        cursor_x = cursor_x.saturating_add(1);

        for tab in TAB_DEFS {
            let is_active = self.active_screen == tab.screen;
            let bg = if is_active { active_bg } else { inactive_bg };
            let fg = if is_active { active_fg } else { inactive_fg };
            let label_style = if is_active {
                Style::default().fg(fg).bg(bg).add_modifier(Modifier::BOLD)
            } else {
                Style::default().fg(fg).bg(bg)
            };
            let tab_width = segment_width(tab.key, tab.label);
            self.tab_hitboxes.push(TabHitbox {
                screen: tab.screen,
                x: cursor_x,
                y: area.y,
                width: tab_width,
                height: area.height,
            });
            cursor_x = cursor_x.saturating_add(tab_width);

            spans.push(Span::styled("", Style::default().fg(bg).bg(base_bg)));
            spans.push(Span::styled(" ", Style::default().bg(bg)));
            spans.push(Span::styled(
                tab.key,
                Style::default()
                    .fg(key_fg)
                    .bg(bg)
                    .add_modifier(Modifier::BOLD),
            ));
            spans.push(Span::styled(" ", Style::default().bg(bg)));
            spans.push(Span::styled(tab.label, label_style));
            spans.push(Span::styled(" ", Style::default().bg(bg)));
            spans.push(Span::styled("", Style::default().fg(bg).bg(base_bg)));
            spans.push(Span::styled(" ", Style::default().bg(base_bg)));
        }

        let tabs = Paragraph::new(Line::from(spans))
            .style(Style::default().bg(base_bg))
            .block(
                Block::default()
                    .borders(Borders::BOTTOM)
                    .style(Style::default().bg(base_bg)),
            );
        f.render_widget(tabs, area);
    }

    fn render_help(&mut self, f: &mut Frame, area: Rect) {
        let base_bg = Color::Rgb(18, 16, 22);
        let key_fg = Color::Rgb(248, 231, 176);
        let text_fg = Color::Rgb(236, 236, 236);

        let mut spans = Vec::new();
        self.help_hitboxes.clear();
        let mut cursor_x = area.x;
        spans.push(Span::styled(" ", Style::default().bg(base_bg)));
        cursor_x = cursor_x.saturating_add(1);

        let tab_hint = if cfg!(feature = "debug_pane") {
            "1-6"
        } else {
            "1-5"
        };

        let segments = [
            (tab_hint, "Tabs", Color::Rgb(86, 114, 98)),
            ("r", "Reset latency", Color::Rgb(136, 92, 78)),
            ("hjkl/←↑→↓", "Scroll", Color::Rgb(92, 102, 150)),
            ("q", "Quit", Color::Rgb(124, 118, 76)),
        ];

        for (key, label, bg) in segments {
            let segment_len = segment_width(key, label);
            let action = match key {
                "r" => Some(HelpAction::ResetLatency),
                "q" => Some(HelpAction::Quit),
                _ => None,
            };
            if let Some(action) = action {
                self.help_hitboxes.push(HelpHitbox {
                    action,
                    x: cursor_x,
                    y: area.y,
                    width: segment_len,
                    height: area.height,
                });
            }
            cursor_x = cursor_x.saturating_add(segment_len);

            spans.push(Span::styled("", Style::default().fg(bg).bg(base_bg)));
            spans.push(Span::styled(" ", Style::default().bg(bg)));
            spans.push(Span::styled(
                key,
                Style::default()
                    .fg(key_fg)
                    .bg(bg)
                    .add_modifier(Modifier::BOLD),
            ));
            spans.push(Span::styled(" ", Style::default().bg(bg)));
            spans.push(Span::styled(label, Style::default().fg(text_fg).bg(bg)));
            spans.push(Span::styled(" ", Style::default().bg(bg)));
            spans.push(Span::styled("", Style::default().fg(bg).bg(base_bg)));
            spans.push(Span::styled(" ", Style::default().bg(base_bg)));
        }

        let help = Paragraph::new(Line::from(spans))
            .style(Style::default().bg(base_bg))
            .block(
                Block::default()
                    .borders(Borders::TOP)
                    .style(Style::default().bg(base_bg)),
            );
        f.render_widget(help, area);
    }

    fn draw(&mut self, f: &mut Frame) {
        let layout = Layout::default()
            .direction(Direction::Vertical)
            .constraints(
                [
                    Constraint::Length(2), // Top tabs
                    Constraint::Min(0),    // Main content
                    Constraint::Length(2), // Bottom help bar
                ]
                .as_ref(),
            )
            .split(f.area());

        self.render_tabs(f, layout[0]);
        self.render_help(f, layout[2]);

        match self.active_screen {
            Screen::Neofetch => {
                const VERSION: &str = env!("CARGO_PKG_VERSION");
                let text: Text = format!("\n   -> Copper v{}\n\n{}\n\n ", VERSION, self.sysinfo)
                    .into_text()
                    .unwrap();
                let p = Paragraph::new::<Text>(text).block(
                    Block::default()
                        .title(" System Info ")
                        .borders(Borders::ALL)
                        .border_type(BorderType::Rounded),
                );
                f.render_widget(p, layout[1]);
            }
            Screen::Dag => {
                self.draw_nodes(f, layout[1]);
            }
            Screen::Latency => self.draw_latency_table(f, layout[1]),
            Screen::MemoryPools => self.draw_memory_pools(f, layout[1]),
            Screen::CopperList => self.draw_copperlist_stats(f, layout[1]),
            #[cfg(feature = "debug_pane")]
            Screen::DebugOutput => self.draw_debug_output(f, layout[1]),
        };
    }

    fn handle_tab_click(&mut self, mouse: event::MouseEvent) {
        if !matches!(mouse.kind, MouseEventKind::Down(MouseButton::Left)) {
            return;
        }

        for hitbox in &self.tab_hitboxes {
            if mouse_inside(&mouse, hitbox.x, hitbox.y, hitbox.width, hitbox.height) {
                self.active_screen = hitbox.screen;
                break;
            }
        }
    }

    fn handle_help_click(&mut self, mouse: event::MouseEvent) -> bool {
        if !matches!(mouse.kind, MouseEventKind::Down(MouseButton::Left)) {
            return false;
        }

        for hitbox in &self.help_hitboxes {
            if !mouse_inside(&mouse, hitbox.x, hitbox.y, hitbox.width, hitbox.height) {
                continue;
            }

            match hitbox.action {
                HelpAction::ResetLatency => {
                    if self.active_screen == Screen::Latency {
                        self.task_stats.lock().unwrap().reset();
                    }
                }
                HelpAction::Quit => {
                    self.quitting.store(true, Ordering::SeqCst);
                }
            }
            return true;
        }

        false
    }

    fn handle_scroll_mouse(&mut self, mouse: event::MouseEvent) {
        if self.active_screen != Screen::Dag {
            return;
        }

        match mouse.kind {
            MouseEventKind::ScrollDown => {
                self.nodes_scrollable_widget_state
                    .nodes_scrollable_state
                    .scroll_down();
            }
            MouseEventKind::ScrollUp => {
                self.nodes_scrollable_widget_state
                    .nodes_scrollable_state
                    .scroll_up();
            }
            MouseEventKind::ScrollRight => {
                for _ in 0..5 {
                    self.nodes_scrollable_widget_state
                        .nodes_scrollable_state
                        .scroll_right();
                }
            }
            MouseEventKind::ScrollLeft => {
                for _ in 0..5 {
                    self.nodes_scrollable_widget_state
                        .nodes_scrollable_state
                        .scroll_left();
                }
            }
            _ => {}
        }
    }

    #[cfg(feature = "debug_pane")]
    fn handle_mouse_event(&mut self, mouse: event::MouseEvent) {
        self.handle_tab_click(mouse);
        if self.handle_help_click(mouse) {
            return;
        }
        self.handle_scroll_mouse(mouse);

        if self.active_screen != Screen::DebugOutput {
            return;
        }

        let Some(area) = self.debug_output_area else {
            return;
        };

        if !mouse_inside(&mouse, area.x, area.y, area.width, area.height) {
            if matches!(mouse.kind, MouseEventKind::Down(MouseButton::Left)) {
                self.debug_selection.clear();
            }
            return;
        }

        let rel_row = (mouse.row - area.y) as usize;
        let rel_col = (mouse.column - area.x) as usize;
        let line_index = self.debug_output_visible_offset.saturating_add(rel_row);
        let Some(line) = self.debug_output_lines.get(line_index) else {
            return;
        };
        let line_len = line.text.chars().count();
        let point = SelectionPoint {
            row: line_index,
            col: rel_col.min(line_len),
        };

        match mouse.kind {
            MouseEventKind::Down(MouseButton::Left) => {
                self.debug_selection.start(point);
            }
            MouseEventKind::Drag(MouseButton::Left) => {
                self.debug_selection.update(point);
            }
            MouseEventKind::Up(MouseButton::Left) => {
                self.debug_selection.update(point);
                if let Some(text) = self.selected_debug_text() {
                    self.copy_debug_text(text);
                }
            }
            _ => {}
        }
    }

    #[cfg(not(feature = "debug_pane"))]
    fn handle_mouse_event(&mut self, mouse: event::MouseEvent) {
        self.handle_tab_click(mouse);
        let _ = self.handle_help_click(mouse);
        self.handle_scroll_mouse(mouse);
    }

    #[cfg(feature = "debug_pane")]
    fn selected_debug_text(&self) -> Option<String> {
        let (start, end) = self.debug_selection.range()?;
        if start == end {
            return None;
        }
        if self.debug_output_lines.is_empty() {
            return None;
        }
        if start.row >= self.debug_output_lines.len() || end.row >= self.debug_output_lines.len() {
            return None;
        }

        let mut selected = Vec::new();
        for row in start.row..=end.row {
            let line = &self.debug_output_lines[row];
            let line_len = line.text.chars().count();
            let Some((start_col, end_col)) = line_selection_bounds(row, line_len, start, end)
            else {
                selected.push(String::new());
                continue;
            };
            let (_, selected_part, _) = slice_char_range(&line.text, start_col, end_col);
            selected.push(selected_part.to_string());
        }
        Some(selected.join("\n"))
    }

    #[cfg(feature = "debug_pane")]
    fn copy_debug_text(&mut self, text: String) {
        if text.is_empty() {
            return;
        }

        if self.clipboard.is_none() {
            match Clipboard::new() {
                Ok(clipboard) => self.clipboard = Some(clipboard),
                Err(err) => {
                    eprintln!("CuConsoleMon clipboard init failed: {err}");
                    return;
                }
            }
        }

        if let Some(clipboard) = self.clipboard.as_mut()
            && let Err(err) = clipboard.set_text(text)
        {
            eprintln!("CuConsoleMon clipboard copy failed: {err}");
        }
    }

    #[cfg(feature = "debug_pane")]
    fn build_debug_output_text(&self, area: Rect) -> Text<'_> {
        let mut rendered_lines = Vec::new();
        let selection = self
            .debug_selection
            .range()
            .filter(|(start, end)| start != end);
        let selection_style = Style::default().bg(Color::Blue).fg(Color::Black);
        let visible = self
            .debug_output_lines
            .iter()
            .skip(self.debug_output_visible_offset)
            .take(area.height as usize);

        for (idx, line) in visible.enumerate() {
            let line_index = self.debug_output_visible_offset + idx;
            let spans = if let Some((start, end)) = selection {
                let line_len = line.text.chars().count();
                if let Some((start_col, end_col)) =
                    line_selection_bounds(line_index, line_len, start, end)
                {
                    let (before, selected, after) =
                        slice_char_range(&line.text, start_col, end_col);
                    let mut spans = Vec::new();
                    if !before.is_empty() {
                        spans.push(Span::raw(before.to_string()));
                    }
                    spans.push(Span::styled(selected.to_string(), selection_style));
                    if !after.is_empty() {
                        spans.push(Span::raw(after.to_string()));
                    }
                    spans
                } else {
                    spans_from_runs(line)
                }
            } else {
                spans_from_runs(line)
            };
            rendered_lines.push(Line::from(spans));
        }

        Text::from(rendered_lines)
    }

    fn run_app<B: Backend<Error = io::Error>>(
        &mut self,
        terminal: &mut Terminal<B>,
    ) -> io::Result<()> {
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
                        KeyCode::Char('4') => self.active_screen = Screen::CopperList,
                        KeyCode::Char('5') => self.active_screen = Screen::MemoryPools,
                        #[cfg(feature = "debug_pane")]
                        KeyCode::Char('6') => self.active_screen = Screen::DebugOutput,
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
                            self.quitting.store(true, Ordering::SeqCst);
                            break;
                        }
                        _ => {}
                    },

                    Event::Mouse(mouse) => {
                        self.handle_mouse_event(mouse);
                    }
                    Event::Resize(_columns, rows) => {
                        self.nodes_scrollable_widget_state.mark_graph_dirty();
                        #[cfg(not(feature = "debug_pane"))]
                        let _ = rows;
                        #[cfg(feature = "debug_pane")]
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
        let mut culist_to_task = [usize::MAX; MAX_CULIST_MAP];
        if let Ok(map) = build_culist_to_task_index(config, taskids) {
            culist_to_task = map;
        }
        let task_stats = Arc::new(Mutex::new(TaskStats::new(
            taskids.len(),
            CuDuration::from(Duration::from_secs(5)),
        )));

        Ok(Self {
            config: config.clone(),
            taskids,
            task_stats,
            task_statuses: Arc::new(Mutex::new(vec![TaskStatus::default(); taskids.len()])),
            culist_to_task,
            ui_handle: None,
            quitting: Arc::new(AtomicBool::new(false)),
            pool_stats: Arc::new(Mutex::new(Vec::new())),
            copperlist_stats: Arc::new(Mutex::new(CopperListStats::new())),
            topology: None,
        })
    }
    fn set_topology(&mut self, topology: MonitorTopology) {
        self.topology = Some(topology);
    }

    fn set_copperlist_info(&mut self, info: CopperListInfo) {
        let mut stats = self.copperlist_stats.lock().unwrap();
        stats.set_info(info);
    }

    fn observe_copperlist_io(&self, stats: CopperListIoStats) {
        let mut cl_stats = self.copperlist_stats.lock().unwrap();
        cl_stats.update_io(stats);
    }

    fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
        if !should_start_ui() {
            return Ok(());
        }

        let config_dup = self.config.clone();
        let taskids = self.taskids;

        let task_stats_ui = self.task_stats.clone();
        let error_states = self.task_statuses.clone();
        let pool_stats_ui = self.pool_stats.clone();
        let copperlist_stats_ui = self.copperlist_stats.clone();
        let quitting = self.quitting.clone();
        let topology = self.topology.clone();

        // Start the main UI loop
        let handle = thread::spawn(move || {
            let backend = CrosstermBackend::new(stdout());
            let _terminal_guard = TerminalRestoreGuard;

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
                let error_redirect = match gag::BufferRedirect::stderr() {
                    Ok(redirect) => Some(redirect),
                    Err(err) => {
                        eprintln!(
                            "Failed to redirect stderr for debug pane; continuing without redirect: {err}"
                        );
                        None
                    }
                };

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
                    copperlist_stats_ui,
                    topology.clone(),
                );

                #[cfg(debug_assertions)]
                {
                    let max_lines = terminal.size().unwrap().height - 5;
                    let (mut debug_log, tx) = debug_pane::DebugLog::new(max_lines);

                    cu29_log_runtime::register_live_log_listener(
                        move |entry, format_str, param_names| {
                            // Rebuild line from structured data, then push to bounded channel.
                            let params: Vec<String> =
                                entry.params.iter().map(|v| v.to_string()).collect();
                            let named_params: HashMap<String, String> = param_names
                                .iter()
                                .zip(params.iter())
                                .map(|(name, value)| (name.to_string(), value.clone()))
                                .collect();
                            let line = styled_line_from_structured(
                                entry.time,
                                entry.level,
                                format_str,
                                params.as_slice(),
                                &named_params,
                            );
                            // Non-blocking: drop log if the bounded channel is full to avoid stalling the runtime.
                            match tx.try_send(line) {
                                Ok(_) => {}
                                Err(std::sync::mpsc::TrySendError::Full(_)) => {}
                                Err(std::sync::mpsc::TrySendError::Disconnected(_)) => {
                                    // Receiver dropped; nothing else we can do.
                                }
                            }
                        },
                    );

                    // Drain any pending from the channel into the UI buffer once to size it.
                    debug_log.update_logs();
                    ui.debug_output = Some(debug_log);
                }
                if let Err(err) = ui.run_app(&mut terminal) {
                    let _ = restore_terminal();
                    eprintln!("CuConsoleMon UI exited with error: {err}");
                    cu29_log_runtime::unregister_live_log_listener();
                    return;
                }
                cu29_log_runtime::unregister_live_log_listener();
            }

            #[cfg(not(feature = "debug_pane"))]
            {
                let stderr_gag = gag::Gag::stderr().unwrap();

                let mut ui = UI::new(
                    config_dup,
                    taskids,
                    task_stats_ui,
                    error_states,
                    quitting.clone(),
                    pool_stats_ui,
                    copperlist_stats_ui,
                    topology,
                );
                if let Err(err) = ui.run_app(&mut terminal) {
                    let _ = restore_terminal();
                    eprintln!("CuConsoleMon UI exited with error: {err}");
                    return;
                }

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
            let mut copperlist_stats = self.copperlist_stats.lock().unwrap();
            copperlist_stats.update_rate();
        }
        {
            let mut task_statuses = self.task_statuses.lock().unwrap();
            for (i, msg) in msgs.iter().enumerate() {
                let CuCompactString(status_txt) = &msg.status_txt;
                if let Some(&task_idx) = self.culist_to_task.get(i)
                    && task_idx != usize::MAX
                    && task_idx < task_statuses.len()
                {
                    task_statuses[task_idx].status_txt = status_txt.clone();
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

fn build_culist_to_task_index(
    config: &CuConfig,
    task_ids: &'static [&'static str],
) -> CuResult<[usize; MAX_CULIST_MAP]> {
    let graph = config.get_graph(None)?;
    let plan = compute_runtime_plan(graph)?;

    let mut mapping = [usize::MAX; MAX_CULIST_MAP];

    for unit in &plan.steps {
        if let CuExecutionUnit::Step(step) = unit
            && let Some(output_pack) = &step.output_msg_pack
        {
            if step.node.get_flavor() != Flavor::Task {
                continue;
            }
            let node_id = step.node.get_id();
            let culist_idx = output_pack.culist_index as usize;
            if culist_idx >= MAX_CULIST_MAP {
                continue;
            }
            if let Some(task_idx) = task_ids.iter().position(|id| *id == node_id.as_str()) {
                mapping[culist_idx] = task_idx;
            }
        }
    }

    Ok(mapping)
}

fn init_error_hooks() {
    static ONCE: OnceLock<()> = OnceLock::new();
    if ONCE.get().is_some() {
        return;
    }

    let (_panic_hook, error) = HookBuilder::default().into_hooks();
    let error = error.into_eyre_hook();
    color_eyre::eyre::set_hook(Box::new(move |e| {
        let _ = restore_terminal();
        error(e)
    }))
    .unwrap();
    std::panic::set_hook(Box::new(move |info| {
        let _ = restore_terminal();
        let bt = Backtrace::force_capture();
        // stderr may be gagged; print to stdout so the panic is visible.
        println!("CuConsoleMon panic: {info}");
        println!("Backtrace:\n{bt}");
        let _ = stdout().flush();
        // Exit immediately so the process doesn't hang after the TUI restores.
        process::exit(1);
    }));

    let _ = ONCE.set(());
}

fn setup_terminal() -> io::Result<()> {
    enable_raw_mode()?;
    // Enable mouse capture for in-app log selection.
    execute!(stdout(), EnterAlternateScreen, EnableMouseCapture)?;
    Ok(())
}

fn restore_terminal() -> io::Result<()> {
    execute!(stdout(), LeaveAlternateScreen, DisableMouseCapture)?;
    disable_raw_mode()
}

fn should_start_ui() -> bool {
    if !stdout().is_tty() || !stdin().is_tty() {
        return false;
    }

    #[cfg(unix)]
    {
        use std::os::unix::io::AsRawFd;
        let stdin_fd = stdin().as_raw_fd();
        // SAFETY: tcgetpgrp only reads process group state for a valid fd.
        let fg_pgrp = unsafe { libc::tcgetpgrp(stdin_fd) };
        if fg_pgrp == -1 {
            return false;
        }
        // SAFETY: getpgrp has no safety requirements beyond being called in a process.
        let pgrp = unsafe { libc::getpgrp() };
        if fg_pgrp != pgrp {
            return false;
        }
    }

    true
}
