use crate::MonitorModel;
#[cfg(feature = "log_pane")]
use crate::logpane::StyledLine;
use crate::model::ComponentStatus;
use crate::palette;
use crate::system_info::{SystemInfo, default_system_info};
use crate::tui_nodes::{Connection, NodeGraph, NodeLayout};
#[cfg(not(all(target_family = "wasm", target_os = "unknown")))]
use ansi_to_tui::IntoText;
use ratatui::Frame;
use ratatui::buffer::Buffer;
use ratatui::layout::{Alignment, Constraint, Direction, Layout, Position, Rect, Size};
use ratatui::prelude::Stylize;
use ratatui::style::{Color, Modifier, Style};
use ratatui::text::{Line, Span, Text};
use ratatui::widgets::{Block, BorderType, Borders, Cell, Paragraph, Row, StatefulWidget, Table};
use std::collections::HashMap;
use std::marker::PhantomData;
use tui_widgets::scrollview::{ScrollView, ScrollViewState};

use cu29::monitoring::{ComponentId, ComponentType, MonitorComponentMetadata};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MonitorScreen {
    System,
    Dag,
    Latency,
    CopperList,
    MemoryPools,
    #[cfg(feature = "log_pane")]
    Logs,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ScrollDirection {
    Up,
    Down,
    Left,
    Right,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum MonitorUiAction {
    None,
    QuitRequested,
    #[cfg(feature = "log_pane")]
    CopyLogSelection(String),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MonitorUiKey {
    Char(char),
    Left,
    Right,
    Up,
    Down,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MonitorUiEvent {
    Key(MonitorUiKey),
    MouseDown {
        col: u16,
        row: u16,
    },
    #[cfg(feature = "log_pane")]
    MouseDrag {
        col: u16,
        row: u16,
    },
    #[cfg(feature = "log_pane")]
    MouseUp {
        col: u16,
        row: u16,
    },
    Scroll {
        direction: ScrollDirection,
        steps: usize,
    },
}

#[derive(Clone, Debug, Default)]
pub struct MonitorUiOptions {
    pub show_quit_hint: bool,
}

#[derive(Clone, Copy)]
struct TabDef {
    screen: MonitorScreen,
    label: &'static str,
    key: &'static str,
}

#[derive(Clone, Copy)]
struct TabHitbox {
    screen: MonitorScreen,
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
        screen: MonitorScreen::System,
        label: "SYS",
        key: "1",
    },
    TabDef {
        screen: MonitorScreen::Dag,
        label: "DAG",
        key: "2",
    },
    TabDef {
        screen: MonitorScreen::Latency,
        label: "LAT",
        key: "3",
    },
    TabDef {
        screen: MonitorScreen::CopperList,
        label: "BW",
        key: "4",
    },
    TabDef {
        screen: MonitorScreen::MemoryPools,
        label: "MEM",
        key: "5",
    },
    #[cfg(feature = "log_pane")]
    TabDef {
        screen: MonitorScreen::Logs,
        label: "LOG",
        key: "6",
    },
];

#[cfg(feature = "log_pane")]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
struct SelectionPoint {
    row: usize,
    col: usize,
}

#[cfg(feature = "log_pane")]
#[derive(Clone, Copy, Debug, Default)]
struct LogSelection {
    anchor: Option<SelectionPoint>,
    cursor: Option<SelectionPoint>,
}

#[cfg(feature = "log_pane")]
impl LogSelection {
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

pub struct MonitorUi {
    model: MonitorModel,
    runtime_node_col_width: u16,
    active_screen: MonitorScreen,
    system_info: SystemInfo,
    show_quit_hint: bool,
    tab_hitboxes: Vec<TabHitbox>,
    help_hitboxes: Vec<HelpHitbox>,
    nodes_scrollable_widget_state: NodesScrollableWidgetState,
    latency_scroll_state: ScrollViewState,
    #[cfg(feature = "log_pane")]
    log_area: Option<Rect>,
    #[cfg(feature = "log_pane")]
    log_lines: Vec<StyledLine>,
    #[cfg(feature = "log_pane")]
    log_selection: LogSelection,
    #[cfg(feature = "log_pane")]
    log_offset_from_bottom: usize,
}

impl MonitorUi {
    pub fn new(model: MonitorModel, options: MonitorUiOptions) -> Self {
        let runtime_node_col_width = Self::compute_runtime_node_col_width(model.components());
        let nodes_scrollable_widget_state = NodesScrollableWidgetState::new(model.clone());
        Self {
            model,
            runtime_node_col_width,
            active_screen: MonitorScreen::System,
            system_info: default_system_info(),
            show_quit_hint: options.show_quit_hint,
            tab_hitboxes: Vec::new(),
            help_hitboxes: Vec::new(),
            nodes_scrollable_widget_state,
            latency_scroll_state: ScrollViewState::default(),
            #[cfg(feature = "log_pane")]
            log_area: None,
            #[cfg(feature = "log_pane")]
            log_lines: Vec::new(),
            #[cfg(feature = "log_pane")]
            log_selection: LogSelection::default(),
            #[cfg(feature = "log_pane")]
            log_offset_from_bottom: 0,
        }
    }

    pub fn active_screen(&self) -> MonitorScreen {
        self.active_screen
    }

    pub fn model(&self) -> &MonitorModel {
        &self.model
    }

    pub fn set_active_screen(&mut self, screen: MonitorScreen) {
        self.active_screen = screen;
    }

    pub fn handle_event(&mut self, event: MonitorUiEvent) -> MonitorUiAction {
        match event {
            MonitorUiEvent::Key(key) => self.handle_key(key),
            MonitorUiEvent::MouseDown { col, row } => self.click(col, row),
            #[cfg(feature = "log_pane")]
            MonitorUiEvent::MouseDrag { col, row } => self.drag_log_selection(col, row),
            #[cfg(feature = "log_pane")]
            MonitorUiEvent::MouseUp { col, row } => self.finish_log_selection(col, row),
            MonitorUiEvent::Scroll { direction, steps } => {
                self.scroll(direction, steps);
                MonitorUiAction::None
            }
        }
    }

    pub fn handle_key(&mut self, key: MonitorUiKey) -> MonitorUiAction {
        match key {
            MonitorUiKey::Char(key) => {
                if let Some(screen) = screen_for_tab_key(key) {
                    self.active_screen = screen;
                } else {
                    match key {
                        'r' => {
                            if self.active_screen == MonitorScreen::Latency {
                                self.model.reset_latency();
                            }
                        }
                        'j' => self.scroll(ScrollDirection::Down, 1),
                        'k' => self.scroll(ScrollDirection::Up, 1),
                        'h' => self.scroll(ScrollDirection::Left, 5),
                        'l' => self.scroll(ScrollDirection::Right, 5),
                        'q' if self.show_quit_hint => return MonitorUiAction::QuitRequested,
                        _ => {}
                    }
                }
            }
            MonitorUiKey::Left => self.scroll(ScrollDirection::Left, 5),
            MonitorUiKey::Right => self.scroll(ScrollDirection::Right, 5),
            MonitorUiKey::Up => self.scroll(ScrollDirection::Up, 1),
            MonitorUiKey::Down => self.scroll(ScrollDirection::Down, 1),
        }

        MonitorUiAction::None
    }

    pub fn handle_char_key(&mut self, key: char) -> MonitorUiAction {
        self.handle_key(MonitorUiKey::Char(key))
    }

    pub fn scroll(&mut self, direction: ScrollDirection, steps: usize) {
        match (self.active_screen, direction) {
            (MonitorScreen::Dag, ScrollDirection::Down) => {
                self.nodes_scrollable_widget_state
                    .nodes_scrollable_state
                    .scroll_down();
            }
            (MonitorScreen::Dag, ScrollDirection::Up) => {
                self.nodes_scrollable_widget_state
                    .nodes_scrollable_state
                    .scroll_up();
            }
            (MonitorScreen::Latency, ScrollDirection::Down) => {
                self.latency_scroll_state.scroll_down();
            }
            (MonitorScreen::Latency, ScrollDirection::Up) => {
                self.latency_scroll_state.scroll_up();
            }
            (MonitorScreen::Dag, ScrollDirection::Right) => {
                for _ in 0..steps {
                    self.nodes_scrollable_widget_state
                        .nodes_scrollable_state
                        .scroll_right();
                }
            }
            (MonitorScreen::Dag, ScrollDirection::Left) => {
                for _ in 0..steps {
                    self.nodes_scrollable_widget_state
                        .nodes_scrollable_state
                        .scroll_left();
                }
            }
            (MonitorScreen::Latency, ScrollDirection::Right) => {
                for _ in 0..steps {
                    self.latency_scroll_state.scroll_right();
                }
            }
            (MonitorScreen::Latency, ScrollDirection::Left) => {
                for _ in 0..steps {
                    self.latency_scroll_state.scroll_left();
                }
            }
            #[cfg(feature = "log_pane")]
            (MonitorScreen::Logs, ScrollDirection::Up) => {
                self.log_offset_from_bottom = self.log_offset_from_bottom.saturating_add(steps);
            }
            #[cfg(feature = "log_pane")]
            (MonitorScreen::Logs, ScrollDirection::Down) => {
                self.log_offset_from_bottom = self.log_offset_from_bottom.saturating_sub(steps);
            }
            _ => {}
        }
    }

    pub fn click(&mut self, x: u16, y: u16) -> MonitorUiAction {
        for hitbox in &self.tab_hitboxes {
            if point_inside(x, y, hitbox.x, hitbox.y, hitbox.width, hitbox.height) {
                self.active_screen = hitbox.screen;
                return MonitorUiAction::None;
            }
        }

        for hitbox in &self.help_hitboxes {
            if !point_inside(x, y, hitbox.x, hitbox.y, hitbox.width, hitbox.height) {
                continue;
            }
            match hitbox.action {
                HelpAction::ResetLatency => {
                    if self.active_screen == MonitorScreen::Latency {
                        self.model.reset_latency();
                    }
                }
                HelpAction::Quit => return MonitorUiAction::QuitRequested,
            }
            return MonitorUiAction::None;
        }

        #[cfg(feature = "log_pane")]
        if self.active_screen == MonitorScreen::Logs {
            return self.start_log_selection(x, y);
        }

        MonitorUiAction::None
    }

    pub fn mark_graph_dirty(&mut self) {
        self.nodes_scrollable_widget_state.mark_graph_dirty();
    }

    pub fn draw(&mut self, f: &mut Frame) {
        let layout = Layout::default()
            .direction(Direction::Vertical)
            .constraints(
                [
                    Constraint::Length(1),
                    Constraint::Min(0),
                    Constraint::Length(1),
                ]
                .as_ref(),
            )
            .split(f.area());

        self.render_tabs(f, layout[0]);
        self.render_help(f, layout[2]);
        self.draw_content(f, layout[1]);
    }

    pub fn draw_content(&mut self, f: &mut Frame, area: Rect) {
        // Avoid backend-specific "reset" colors bleeding through the scrollable canvases.
        f.render_widget(
            Block::default().style(Style::default().bg(palette::BACKGROUND)),
            area,
        );

        match self.active_screen {
            MonitorScreen::System => self.draw_system_info(f, area),
            MonitorScreen::Dag => self.draw_nodes(f, area),
            MonitorScreen::Latency => self.draw_latency_table(f, area),
            MonitorScreen::CopperList => self.draw_copperlist_stats(f, area),
            MonitorScreen::MemoryPools => self.draw_memory_pools(f, area),
            #[cfg(feature = "log_pane")]
            MonitorScreen::Logs => self.draw_logs(f, area),
        }
    }

    fn compute_runtime_node_col_width(components: &'static [MonitorComponentMetadata]) -> u16 {
        const MIN_WIDTH: usize = 24;
        const MAX_WIDTH: usize = 56;

        let header_width = "Runtime Node".chars().count();
        let max_name_width = components
            .iter()
            .map(|component| component.id().chars().count())
            .max()
            .unwrap_or(0);
        let width = header_width.max(max_name_width).saturating_add(2);
        width.clamp(MIN_WIDTH, MAX_WIDTH) as u16
    }

    fn component_label(&self, component_id: ComponentId) -> &'static str {
        debug_assert!(component_id.index() < self.model.components().len());
        self.model.components()[component_id.index()].id()
    }

    fn draw_system_info(&self, f: &mut Frame, area: Rect) {
        const VERSION: &str = env!("CARGO_PKG_VERSION");
        let mut lines = vec![
            Line::raw(""),
            Line::raw(format!("   -> Copper v{VERSION}")),
            Line::raw(""),
        ];
        let mut body = match &self.system_info {
            #[cfg(not(all(target_family = "wasm", target_os = "unknown")))]
            SystemInfo::Ansi(raw) => raw
                .clone()
                .into_text()
                .map(|text| text.to_owned())
                .unwrap_or_else(|_| Text::from(raw.clone())),
            #[cfg(all(target_family = "wasm", target_os = "unknown"))]
            SystemInfo::Rich(text) => text.clone(),
        };
        palette::normalize_text_colors(&mut body, palette::FOREGROUND, palette::BACKGROUND);
        lines.append(&mut body.lines);
        lines.push(Line::raw(" "));
        let text = Text::from(lines);
        let paragraph = Paragraph::new(text).block(
            Block::default()
                .title(" System Info ")
                .borders(Borders::ALL)
                .border_type(BorderType::Rounded),
        );
        f.render_widget(paragraph, area);
    }

    fn draw_latency_table(&mut self, f: &mut Frame, area: Rect) {
        let header_cells = [
            "⌘ Runtime Node",
            "Kind",
            "⬇ Min",
            "⬆ Max",
            "∅ Mean",
            "σ Stddev",
            "⧖∅ Jitter",
            "⧗⬆ Jitter",
        ]
        .iter()
        .enumerate()
        .map(|(idx, header)| {
            let align = if idx <= 1 {
                Alignment::Left
            } else {
                Alignment::Right
            };
            Cell::from(Line::from(*header).alignment(align)).style(
                Style::default()
                    .fg(palette::YELLOW)
                    .add_modifier(Modifier::BOLD),
            )
        });

        let header = Row::new(header_cells)
            .style(Style::default().fg(palette::YELLOW))
            .bottom_margin(1)
            .top_margin(1);

        let component_stats = self.model.inner.component_stats.lock().unwrap();
        let mut rows = component_stats
            .stats
            .iter()
            .enumerate()
            .map(|(index, stat)| {
                let component_id = ComponentId::new(index);
                let kind_label = match self.model.components()[component_id.index()].kind() {
                    ComponentType::Source => "◈ Src",
                    ComponentType::Task => "⚙ Task",
                    ComponentType::Sink => "⭳ Sink",
                    ComponentType::Bridge => "⇆ Brg",
                    _ => "?",
                };
                let cells = vec![
                    Cell::from(
                        Line::from(self.component_label(component_id)).alignment(Alignment::Left),
                    )
                    .light_blue(),
                    Cell::from(Line::from(kind_label).alignment(Alignment::Left)),
                    Cell::from(Line::from(stat.min().to_string()).alignment(Alignment::Right)),
                    Cell::from(Line::from(stat.max().to_string()).alignment(Alignment::Right)),
                    Cell::from(Line::from(stat.mean().to_string()).alignment(Alignment::Right)),
                    Cell::from(Line::from(stat.stddev().to_string()).alignment(Alignment::Right)),
                    Cell::from(
                        Line::from(stat.jitter_mean().to_string()).alignment(Alignment::Right),
                    ),
                    Cell::from(
                        Line::from(stat.jitter_max().to_string()).alignment(Alignment::Right),
                    ),
                ];
                Row::new(cells)
            })
            .collect::<Vec<Row>>();

        let cells = vec![
            Cell::from(Line::from("End2End").light_red().alignment(Alignment::Left)),
            Cell::from(Line::from("All").light_red().alignment(Alignment::Left)),
            Cell::from(
                Line::from(component_stats.end2end.min().to_string())
                    .light_red()
                    .alignment(Alignment::Right),
            ),
            Cell::from(
                Line::from(component_stats.end2end.max().to_string())
                    .light_red()
                    .alignment(Alignment::Right),
            ),
            Cell::from(
                Line::from(component_stats.end2end.mean().to_string())
                    .light_red()
                    .alignment(Alignment::Right),
            ),
            Cell::from(
                Line::from(component_stats.end2end.stddev().to_string())
                    .light_red()
                    .alignment(Alignment::Right),
            ),
            Cell::from(
                Line::from(component_stats.end2end.jitter_mean().to_string())
                    .light_red()
                    .alignment(Alignment::Right),
            ),
            Cell::from(
                Line::from(component_stats.end2end.jitter_max().to_string())
                    .light_red()
                    .alignment(Alignment::Right),
            ),
        ];
        rows.push(Row::new(cells).top_margin(1));
        let row_count = rows.len();
        drop(component_stats);

        let table = Table::new(
            rows,
            &[
                Constraint::Length(self.runtime_node_col_width),
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

        let content_width = self
            .runtime_node_col_width
            .saturating_add(10)
            .saturating_add(10)
            .saturating_add(12)
            .saturating_add(12)
            .saturating_add(10)
            .saturating_add(12)
            .saturating_add(13)
            .saturating_add(24)
            .max(area.width);
        let content_height = (row_count as u16).saturating_add(6).max(area.height);
        let content_size = Size::new(content_width, content_height);
        self.clamp_latency_scroll_offset(area, content_size);
        let mut scroll_view = ScrollView::new(content_size);
        scroll_view.render_widget(
            Block::default().style(Style::default().bg(palette::BACKGROUND)),
            Rect::new(0, 0, content_size.width, content_size.height),
        );
        scroll_view.render_widget(
            table,
            Rect::new(0, 0, content_size.width, content_size.height),
        );
        scroll_view.render(area, f.buffer_mut(), &mut self.latency_scroll_state);
    }

    fn clamp_latency_scroll_offset(&mut self, area: Rect, content_size: Size) {
        let max_x = content_size.width.saturating_sub(area.width);
        let max_y = content_size.height.saturating_sub(area.height);
        let offset = self.latency_scroll_state.offset();
        let clamped = Position::new(offset.x.min(max_x), offset.y.min(max_y));
        self.latency_scroll_state.set_offset(clamped);
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
        .map(|header| {
            Cell::from(Line::from(*header).alignment(Alignment::Right)).style(
                Style::default()
                    .fg(palette::YELLOW)
                    .add_modifier(Modifier::BOLD),
            )
        });

        let header = Row::new(header_cells)
            .style(Style::default().fg(palette::YELLOW))
            .bottom_margin(1);

        let pool_stats = self.model.inner.pool_stats.lock().unwrap();
        let rows = pool_stats
            .iter()
            .map(|stat| {
                let used = stat.total_size.saturating_sub(stat.space_left);
                let percent = if stat.total_size > 0 {
                    100.0 * used as f64 / stat.total_size as f64
                } else {
                    0.0
                };
                let mb_unit = 1024.0 * 1024.0;

                Row::new(vec![
                    Cell::from(Line::from(stat.id.to_string()).alignment(Alignment::Right))
                        .light_blue(),
                    Cell::from(
                        Line::from(format!(
                            "{:.2} MB / {:.2} MB ({:.1}%)",
                            used as f64 * stat.buffer_size as f64 / mb_unit,
                            stat.total_size as f64 * stat.buffer_size as f64 / mb_unit,
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
                ])
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
        let stats = self.model.inner.copperlist_stats.lock().unwrap();
        let size_display = format_bytes_or(stats.size_bytes as u64, "unknown");
        let raw_total = stats.raw_culist_bytes.max(stats.size_bytes as u64);
        let handles_display = format_bytes_or(stats.handle_bytes, "0 B");
        let mem_total = raw_total
            .saturating_add(stats.keyframe_bytes)
            .saturating_add(stats.structured_bytes_per_cl);
        let mem_total_display = format_bytes_or(mem_total, "unknown");
        let encoded_display = format_bytes_or(stats.encoded_bytes, "n/a");
        let efficiency_display = if raw_total > 0 && stats.encoded_bytes > 0 {
            let ratio = (stats.encoded_bytes as f64) / (raw_total as f64);
            format!("{:.1}%", ratio * 100.0)
        } else {
            "n/a".to_string()
        };
        let rate_display = format!("{:.2} Hz", stats.rate_hz);
        let raw_bw = format_rate_bytes_or_na(mem_total, stats.rate_hz);
        let keyframe_display = format_bytes_or(stats.keyframe_bytes, "0 B");
        let structured_display = format_bytes_or(stats.structured_bytes_per_cl, "0 B");
        let structured_bw = format_rate_bytes_or_na(stats.structured_bytes_per_cl, stats.rate_hz);
        let disk_total_bytes = stats
            .encoded_bytes
            .saturating_add(stats.keyframe_bytes)
            .saturating_add(stats.structured_bytes_per_cl);
        let disk_total_bw = format_rate_bytes_or_na(disk_total_bytes, stats.rate_hz);

        let header_cells = ["Metric", "Value"].iter().map(|header| {
            Cell::from(Line::from(*header)).style(
                Style::default()
                    .fg(palette::YELLOW)
                    .add_modifier(Modifier::BOLD),
            )
        });

        let header = Row::new(header_cells).bottom_margin(1);
        let row = |metric: &'static str, value: String| {
            Row::new(vec![
                Cell::from(Line::from(metric)),
                Cell::from(Line::from(value).alignment(Alignment::Right)),
            ])
        };
        let spacer = row(" ", " ".to_string());

        let rate_style = Style::default().fg(palette::CYAN);
        let mem_rows = vec![
            row("Observed rate", rate_display).style(rate_style),
            spacer.clone(),
            row("CopperList size", size_display),
            row("Pool memory used", handles_display),
            row("Keyframe size", keyframe_display),
            row("Mem total (CL+KF+SL)", mem_total_display),
            spacer.clone(),
            row("RAM BW (raw)", raw_bw),
        ];

        let disk_rows = vec![
            row("CL serialized size", encoded_display),
            row("CL encoding efficiency", efficiency_display),
            row("Structured log / CL", structured_display),
            row("Structured BW", structured_bw),
            spacer.clone(),
            row("Total disk BW", disk_total_bw),
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

    fn draw_nodes(&mut self, f: &mut Frame, area: Rect) {
        NodesScrollableWidget {
            _marker: Default::default(),
        }
        .render(
            area,
            f.buffer_mut(),
            &mut self.nodes_scrollable_widget_state,
        );
    }

    #[cfg(feature = "log_pane")]
    fn start_log_selection(&mut self, col: u16, row: u16) -> MonitorUiAction {
        let Some(area) = self.log_area else {
            self.log_selection.clear();
            return MonitorUiAction::None;
        };
        if !point_inside(col, row, area.x, area.y, area.width, area.height) {
            self.log_selection.clear();
            return MonitorUiAction::None;
        }

        let Some(point) = self.log_selection_point(col, row) else {
            return MonitorUiAction::None;
        };
        self.log_selection.start(point);
        MonitorUiAction::None
    }

    #[cfg(feature = "log_pane")]
    fn drag_log_selection(&mut self, col: u16, row: u16) -> MonitorUiAction {
        let Some(point) = self.log_selection_point(col, row) else {
            return MonitorUiAction::None;
        };
        self.log_selection.update(point);
        MonitorUiAction::None
    }

    #[cfg(feature = "log_pane")]
    fn finish_log_selection(&mut self, col: u16, row: u16) -> MonitorUiAction {
        let Some(point) = self.log_selection_point(col, row) else {
            self.log_selection.clear();
            return MonitorUiAction::None;
        };
        self.log_selection.update(point);
        self.selected_log_text()
            .map(MonitorUiAction::CopyLogSelection)
            .unwrap_or(MonitorUiAction::None)
    }

    #[cfg(feature = "log_pane")]
    fn log_selection_point(&self, col: u16, row: u16) -> Option<SelectionPoint> {
        let area = self.log_area?;
        if !point_inside(col, row, area.x, area.y, area.width, area.height) {
            return None;
        }

        let rel_row = (row - area.y) as usize;
        let rel_col = (col - area.x) as usize;
        let line_index = self.visible_log_offset(area).saturating_add(rel_row);
        let line = self.log_lines.get(line_index)?;
        Some(SelectionPoint {
            row: line_index,
            col: rel_col.min(line.text.chars().count()),
        })
    }

    #[cfg(feature = "log_pane")]
    fn visible_log_offset(&self, area: Rect) -> usize {
        let visible_rows = area.height as usize;
        let total_lines = self.log_lines.len();
        let max_offset = total_lines.saturating_sub(visible_rows);
        let offset_from_bottom = self.log_offset_from_bottom.min(max_offset);
        total_lines.saturating_sub(visible_rows.saturating_add(offset_from_bottom))
    }

    #[cfg(feature = "log_pane")]
    fn draw_logs(&mut self, f: &mut Frame, area: Rect) {
        let block = Block::default()
            .title(" Debug Output ")
            .title_bottom(format!("{} log entries", self.model.log_line_count()))
            .borders(Borders::ALL)
            .border_type(BorderType::Rounded);
        let inner = block.inner(area);
        self.log_area = Some(inner);
        self.log_lines = self.model.log_lines();

        let visible_offset = self.visible_log_offset(inner);
        if let Some((start, end)) = self.log_selection.range()
            && (start.row >= self.log_lines.len() || end.row >= self.log_lines.len())
        {
            self.log_selection.clear();
        }

        let paragraph = Paragraph::new(self.build_log_text(inner, visible_offset)).block(block);
        f.render_widget(paragraph, area);
    }

    #[cfg(feature = "log_pane")]
    fn build_log_text(&self, area: Rect, visible_offset: usize) -> Text<'static> {
        let mut rendered_lines = Vec::new();
        let selection = self
            .log_selection
            .range()
            .filter(|(start, end)| start != end);
        let selection_style = Style::default().bg(palette::BLUE).fg(palette::BACKGROUND);
        let visible_lines = self
            .log_lines
            .iter()
            .skip(visible_offset)
            .take(area.height as usize);

        for (idx, line) in visible_lines.enumerate() {
            let line_index = visible_offset + idx;
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

    #[cfg(feature = "log_pane")]
    fn selected_log_text(&self) -> Option<String> {
        let (start, end) = self.log_selection.range()?;
        if start == end || self.log_lines.is_empty() {
            return None;
        }
        if start.row >= self.log_lines.len() || end.row >= self.log_lines.len() {
            return None;
        }

        let mut selected = Vec::new();
        for row in start.row..=end.row {
            let line = &self.log_lines[row];
            let line_len = line.text.chars().count();
            let Some((start_col, end_col)) = line_selection_bounds(row, line_len, start, end)
            else {
                selected.push(String::new());
                continue;
            };
            let (_, selection, _) = slice_char_range(&line.text, start_col, end_col);
            selected.push(selection.to_string());
        }

        Some(selected.join("\n"))
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
            .block(Block::default().style(Style::default().bg(base_bg)));
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

        let mut segments = vec![
            (tab_key_hint(), "Tabs", Color::Rgb(86, 114, 98), None),
            (
                "r".to_string(),
                "Reset latency",
                Color::Rgb(136, 92, 78),
                Some(HelpAction::ResetLatency),
            ),
            (
                "hjkl/←↑→↓".to_string(),
                "Scroll",
                Color::Rgb(92, 102, 150),
                None,
            ),
        ];
        if self.show_quit_hint {
            segments.push((
                "q".to_string(),
                "Quit",
                Color::Rgb(124, 118, 76),
                Some(HelpAction::Quit),
            ));
        }

        for (key, label, bg, action) in segments {
            let segment_len = segment_width(&key, label);
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
            .block(Block::default().style(Style::default().bg(base_bg)));
        f.render_widget(help, area);

        let clid_inner = {
            let stats = self.model.inner.copperlist_stats.lock().unwrap();
            let value = stats.last_seen_clid.unwrap_or(0);
            format!(" CL {:020} ", value)
        };
        let clid_width = (clid_inner.chars().count() + 2) as u16;
        if area.width > clid_width + 2 && area.height >= 1 {
            let clid_area = Rect {
                x: area
                    .x
                    .saturating_add(area.width.saturating_sub(clid_width + 1)),
                y: area.y,
                width: clid_width,
                height: 1,
            };
            let badge_bg = Color::Rgb(216, 157, 63);
            f.render_widget(
                Paragraph::new(Line::from(vec![
                    Span::styled("", Style::default().fg(badge_bg).bg(base_bg)),
                    Span::styled(
                        clid_inner,
                        Style::default()
                            .fg(palette::BACKGROUND)
                            .bg(badge_bg)
                            .add_modifier(Modifier::BOLD),
                    ),
                    Span::styled("", Style::default().fg(badge_bg).bg(base_bg)),
                ])),
                clid_area,
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn normalize_text_colors_replaces_reset_fg_and_bg() {
        let mut text = Text::from(Line::from(vec![Span::styled(
            "pfetch",
            Style::default().fg(Color::Reset).bg(Color::Reset),
        )]));

        palette::normalize_text_colors(&mut text, palette::FOREGROUND, palette::BACKGROUND);

        let span = &text.lines[0].spans[0];
        assert_eq!(span.style.fg, Some(palette::FOREGROUND));
        assert_eq!(span.style.bg, Some(palette::BACKGROUND));
    }
}

fn point_inside(px: u16, py: u16, x: u16, y: u16, width: u16, height: u16) -> bool {
    px >= x && px < x + width && py >= y && py < y + height
}

fn segment_width(key: &str, label: &str) -> u16 {
    (6 + key.chars().count() + label.chars().count()) as u16
}

fn screen_for_tab_key(key: char) -> Option<MonitorScreen> {
    TAB_DEFS
        .iter()
        .find(|tab| tab.key.len() == 1 && tab.key.starts_with(key))
        .map(|tab| tab.screen)
}

fn tab_key_hint() -> String {
    let keys = TAB_DEFS.iter().map(|tab| tab.key).collect::<Vec<_>>();
    if keys.is_empty() {
        return "tabs".to_string();
    }

    let numeric_keys = keys
        .iter()
        .map(|key| key.parse::<u8>())
        .collect::<Result<Vec<_>, _>>();

    if let Ok(numeric_keys) = numeric_keys {
        let is_contiguous = numeric_keys
            .windows(2)
            .all(|window| window[1] == window[0].saturating_add(1));
        if is_contiguous
            && let (Some(first), Some(last)) = (numeric_keys.first(), numeric_keys.last())
        {
            return if first == last {
                first.to_string()
            } else {
                format!("{first}-{last}")
            };
        }
    }

    keys.join("/")
}

#[cfg(feature = "log_pane")]
fn char_to_byte_index(text: &str, char_idx: usize) -> usize {
    text.char_indices()
        .nth(char_idx)
        .map(|(idx, _)| idx)
        .unwrap_or(text.len())
}

#[cfg(feature = "log_pane")]
fn slice_char_range(text: &str, start: usize, end: usize) -> (&str, &str, &str) {
    let start_idx = char_to_byte_index(text, start).min(text.len());
    let end_idx = char_to_byte_index(text, end).min(text.len());
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

#[cfg(feature = "log_pane")]
fn slice_chars_owned(text: &str, start: usize, end: usize) -> String {
    let start_idx = char_to_byte_index(text, start).min(text.len());
    let end_idx = char_to_byte_index(text, end).min(text.len());
    text[start_idx..end_idx].to_string()
}

#[cfg(feature = "log_pane")]
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

#[cfg(feature = "log_pane")]
fn spans_from_runs(line: &StyledLine) -> Vec<Span<'static>> {
    if line.runs.is_empty() {
        return vec![Span::raw(line.text.clone())];
    }

    let mut spans = Vec::new();
    let mut cursor = 0usize;
    let total_chars = line.text.chars().count();
    let mut runs = line.runs.clone();
    runs.sort_by_key(|run| run.start);

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
            spans.push(Span::styled(
                slice_chars_owned(&line.text, start, end),
                run.style,
            ));
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

fn format_bytes_or(bytes: u64, fallback: &str) -> String {
    if bytes > 0 {
        format_bytes(bytes as f64)
    } else {
        fallback.to_string()
    }
}

fn format_rate_bytes_or_na(bytes: u64, rate_hz: f64) -> String {
    if bytes > 0 {
        format!("{}/s", format_bytes((bytes as f64) * rate_hz))
    } else {
        "n/a".to_string()
    }
}

#[derive(Copy, Clone)]
enum NodeType {
    Unknown,
    Source,
    Sink,
    Task,
    Bridge,
}

impl std::fmt::Display for NodeType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
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
    fn color(self) -> Color {
        match self {
            Self::Unknown => palette::GRAY,
            Self::Source => Color::Rgb(255, 191, 0),
            Self::Sink => Color::Rgb(255, 102, 204),
            Self::Task => palette::WHITE,
            Self::Bridge => Color::Rgb(204, 153, 255),
        }
    }
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

    fn needs_rebuild(&self, key: GraphCacheKey) -> bool {
        self.dirty || self.graph.is_none() || self.key != Some(key)
    }
}

struct NodesScrollableWidgetState {
    model: MonitorModel,
    display_nodes: Vec<DisplayNode>,
    connections: Vec<Connection>,
    status_index_map: Vec<Option<ComponentId>>,
    nodes_scrollable_state: ScrollViewState,
    graph_cache: GraphCache,
}

impl NodesScrollableWidgetState {
    fn new(model: MonitorModel) -> Self {
        let mut display_nodes = Vec::new();
        let mut status_index_map = Vec::new();
        let mut node_lookup = HashMap::new();
        let component_id_by_name: HashMap<&'static str, ComponentId> = model
            .components()
            .iter()
            .enumerate()
            .map(|(idx, component)| (component.id(), ComponentId::new(idx)))
            .collect();

        for node in &model.topology().nodes {
            let node_type = match node.kind {
                ComponentType::Source => NodeType::Source,
                ComponentType::Task => NodeType::Task,
                ComponentType::Sink => NodeType::Sink,
                ComponentType::Bridge => NodeType::Bridge,
                _ => NodeType::Unknown,
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
            status_index_map.push(component_id_by_name.get(node.id.as_str()).copied());
        }

        let mut connections = Vec::with_capacity(model.topology().connections.len());
        for connection in &model.topology().connections {
            let Some(&src_idx) = node_lookup.get(&connection.src) else {
                continue;
            };
            let Some(&dst_idx) = node_lookup.get(&connection.dst) else {
                continue;
            };
            let src_node = &display_nodes[src_idx];
            let dst_node = &display_nodes[dst_idx];
            let src_port = connection
                .src_port
                .as_ref()
                .and_then(|port| src_node.outputs.iter().position(|name| name == port))
                .unwrap_or(0);
            let dst_port = connection
                .dst_port
                .as_ref()
                .and_then(|port| dst_node.inputs.iter().position(|name| name == port))
                .unwrap_or(0);

            connections.push(Connection::new(
                src_idx,
                src_port + NODE_PORT_ROW_OFFSET,
                dst_idx,
                dst_port + NODE_PORT_ROW_OFFSET,
            ));
        }

        if !display_nodes.is_empty() {
            let mut from_set = std::collections::HashSet::new();
            for connection in &connections {
                from_set.insert(connection.from_node);
            }
            if from_set.len() == display_nodes.len() {
                connections.retain(|connection| connection.from_node != 0);
            }
        }

        Self {
            model,
            display_nodes,
            connections,
            status_index_map,
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
                    Span::styled(
                        format!(" {} ", node.id),
                        Style::default().fg(palette::WHITE),
                    ),
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
const GRAPH_WIDTH_PADDING: u16 = NODE_WIDTH * 2;
const GRAPH_HEIGHT_PADDING: u16 = NODE_HEIGHT * 4;

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

impl StatefulWidget for NodesScrollableWidget<'_> {
    type State = NodesScrollableWidgetState;

    fn render(self, area: Rect, buf: &mut Buffer, state: &mut Self::State) {
        let content_size = state.ensure_graph_cache(area);
        let mut scroll_view = ScrollView::new(content_size);
        scroll_view.render_widget(
            Block::default().style(Style::default().bg(palette::BACKGROUND)),
            Rect::new(0, 0, content_size.width, content_size.height),
        );

        {
            let graph = state.graph();
            let zones = graph.split(scroll_view.area());

            let mut statuses = state.model.inner.component_statuses.lock().unwrap();
            for (idx, zone) in zones.into_iter().enumerate() {
                let status = state
                    .status_index_map
                    .get(idx)
                    .and_then(|component_id| *component_id)
                    .and_then(|component_id| statuses.get_mut(component_id.index()))
                    .map(|status| {
                        let snapshot: ComponentStatus = status.clone();
                        status.is_error = false;
                        snapshot
                    })
                    .unwrap_or_default();
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
                    Style::default().fg(palette::RED)
                } else {
                    Style::default().fg(palette::GREEN)
                };
                let mut lines = vec![
                    Line::styled(format!(" {}", type_label), base_style),
                    Line::styled(format!(" {}", status_text), base_style),
                ];

                let max_ports = node.inputs.len().max(node.outputs.len());
                if max_ports > 0 {
                    let left_width = (NODE_WIDTH_CONTENT as usize - 2) / 2;
                    let right_width = NODE_WIDTH_CONTENT as usize - 2 - left_width;
                    let input_style = Style::default().fg(palette::YELLOW);
                    let output_style = Style::default().fg(palette::CYAN);
                    let dotted_style = Style::default().fg(palette::DARK_GRAY);
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

                scroll_view.render_widget(Paragraph::new(Text::from(lines)), zone);
            }

            let content_area = Rect::new(0, 0, content_size.width, content_size.height);
            scroll_view.render_widget(graph, content_area);
        }

        scroll_view.render(area, buf, &mut state.nodes_scrollable_state);
    }
}
