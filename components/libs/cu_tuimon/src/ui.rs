use crate::MonitorModel;
use crate::model::MonitorFooterIdentity;
use crate::palette;
#[cfg(feature = "dag")]
use crate::tui_nodes::{Connection, NodeGraph, NodeLayout};
use ratatui::Frame;
#[cfg(feature = "dag")]
use ratatui::buffer::Buffer;
use ratatui::layout::{Alignment, Constraint, Direction, Layout, Position, Rect, Size};
use ratatui::prelude::Stylize;
use ratatui::style::{Color, Modifier, Style};
#[cfg(any(feature = "dag", feature = "sysinfo_pane", feature = "log_pane", test))]
use ratatui::text::Text;
use ratatui::text::{Line, Span};
use ratatui::widgets::{Block, BorderType, Borders, Cell, Paragraph, Row, StatefulWidget, Table};
#[cfg(feature = "dag")]
use std::collections::HashMap;
#[cfg(feature = "dag")]
use std::marker::PhantomData;
use tui_widgets::scrollview::{ScrollView, ScrollViewState};

use cu29::monitoring::{ComponentId, ComponentType, MonitorComponentMetadata};

#[cfg(feature = "log_pane")]
use crate::log_pane::{LogPane, SelectionPoint, StyledLine};

#[cfg(feature = "sysinfo_pane")]
use crate::sysinfo_pane::{SystemInfo, default_system_info};

#[cfg(all(native, feature = "sysinfo_pane"))]
use ansi_to_tui::IntoText;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MonitorScreen {
    #[cfg(feature = "sysinfo_pane")]
    System,
    #[cfg(feature = "dag")]
    Dag,
    #[cfg(feature = "neighbors")]
    Neighbors,
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
    Enter,
    Backspace,
    Tab,
    Esc,
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
    /// Scroll at a pointer position in terminal cells.
    ScrollAt {
        col: u16,
        row: u16,
        direction: ScrollDirection,
        steps: usize,
    },
    Scroll {
        direction: ScrollDirection,
        steps: usize,
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
}

#[derive(Clone, Debug, Default)]
pub struct MonitorUiOptions {
    pub show_quit_hint: bool,
}

#[derive(Clone, Copy)]
struct TabDef {
    screen: MonitorScreen,
    label: &'static str,
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

#[derive(Clone, Debug, PartialEq, Eq)]
struct FooterBadge {
    inner: String,
    bg: Color,
    fg: Color,
}

const TAB_DEFS: &[TabDef] = &[
    #[cfg(feature = "sysinfo_pane")]
    TabDef {
        screen: MonitorScreen::System,
        label: "SYS",
    },
    #[cfg(feature = "dag")]
    TabDef {
        screen: MonitorScreen::Dag,
        label: "DAG",
    },
    #[cfg(feature = "neighbors")]
    TabDef {
        screen: MonitorScreen::Neighbors,
        label: "HOP",
    },
    TabDef {
        screen: MonitorScreen::Latency,
        label: "LAT",
    },
    TabDef {
        screen: MonitorScreen::CopperList,
        label: "BW",
    },
    TabDef {
        screen: MonitorScreen::MemoryPools,
        label: "MEM",
    },
    #[cfg(feature = "log_pane")]
    TabDef {
        screen: MonitorScreen::Logs,
        label: "LOG",
    },
];

pub struct MonitorUi {
    model: MonitorModel,
    runtime_node_col_width: u16,
    active_screen: MonitorScreen,
    show_quit_hint: bool,
    tab_hitboxes: Vec<TabHitbox>,
    help_hitboxes: Vec<HelpHitbox>,
    #[cfg(feature = "dag")]
    nodes_scrollable_widget_state: NodesScrollableWidgetState,
    #[cfg(feature = "neighbors")]
    neighbors: crate::neighbors::NeighborsView,
    latency_scroll_state: ScrollViewState,
    bandwidth_scroll_state: ScrollViewState,
    stream_rates: Vec<crate::stream_panel::StreamRates>,

    #[cfg(feature = "sysinfo_pane")]
    system_info: SystemInfo,

    #[cfg(feature = "log_pane")]
    log_pane: LogPane,
}

impl MonitorUi {
    pub fn new(model: MonitorModel, options: MonitorUiOptions) -> Self {
        let runtime_node_col_width = Self::compute_runtime_node_col_width(model.components());
        #[cfg(feature = "dag")]
        let nodes_scrollable_widget_state = NodesScrollableWidgetState::new(model.clone());

        let stream_rates = model.streams.as_ref().map_or_else(Vec::new, |streams| {
            streams
                .iter()
                .map(|_| crate::stream_panel::StreamRates::default())
                .collect()
        });
        Self {
            #[cfg(feature = "neighbors")]
            neighbors: crate::neighbors::NeighborsView::new(model.clone()),
            model,
            runtime_node_col_width,
            active_screen: default_screen(),
            show_quit_hint: options.show_quit_hint,
            tab_hitboxes: Vec::new(),
            help_hitboxes: Vec::new(),
            #[cfg(feature = "dag")]
            nodes_scrollable_widget_state,
            latency_scroll_state: ScrollViewState::default(),
            bandwidth_scroll_state: ScrollViewState::default(),
            stream_rates,

            #[cfg(feature = "sysinfo_pane")]
            system_info: default_system_info(),

            #[cfg(feature = "log_pane")]
            log_pane: Default::default(),
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
            MonitorUiEvent::ScrollAt {
                col,
                row,
                direction,
                steps,
            } => {
                #[cfg(feature = "neighbors")]
                if self.active_screen == MonitorScreen::Neighbors {
                    self.neighbors.scroll_at(col, row, direction, steps);
                    return MonitorUiAction::None;
                }
                let _ = (col, row);
                self.scroll(direction, steps);
                MonitorUiAction::None
            }
            MonitorUiEvent::Scroll { direction, steps } => {
                self.scroll(direction, steps);
                MonitorUiAction::None
            }

            #[cfg(feature = "log_pane")]
            MonitorUiEvent::MouseDrag { col, row } => self.drag_log_selection(col, row),
            #[cfg(feature = "log_pane")]
            MonitorUiEvent::MouseUp { col, row } => self.finish_log_selection(col, row),
        }
    }

    pub fn handle_key(&mut self, key: MonitorUiKey) -> MonitorUiAction {
        #[cfg(feature = "neighbors")]
        if self.active_screen == MonitorScreen::Neighbors && self.neighbors.handle_key(key) {
            return MonitorUiAction::None;
        }
        match key {
            MonitorUiKey::Char(key) => {
                if let Some(screen) = screen_for_tab_key(key) {
                    self.active_screen = screen;
                } else {
                    match key {
                        'r' if self.active_screen == MonitorScreen::Latency => {
                            self.model.reset_latency();
                        }
                        'j' => self.scroll(ScrollDirection::Down, 1),
                        'k' => self.scroll(ScrollDirection::Up, 1),
                        'h' => self.scroll(ScrollDirection::Left, 5),
                        'l' => self.scroll(ScrollDirection::Right, 5),
                        'q' if self.show_quit_hint => {
                            return MonitorUiAction::QuitRequested;
                        }
                        _ => {}
                    }
                }
            }
            MonitorUiKey::Left => self.scroll(ScrollDirection::Left, 5),
            MonitorUiKey::Right => self.scroll(ScrollDirection::Right, 5),
            MonitorUiKey::Up => self.scroll(ScrollDirection::Up, 1),
            MonitorUiKey::Down => self.scroll(ScrollDirection::Down, 1),
            MonitorUiKey::Enter
            | MonitorUiKey::Backspace
            | MonitorUiKey::Tab
            | MonitorUiKey::Esc => {}
        }

        MonitorUiAction::None
    }

    pub fn handle_char_key(&mut self, key: char) -> MonitorUiAction {
        self.handle_key(MonitorUiKey::Char(key))
    }

    pub fn scroll(&mut self, direction: ScrollDirection, steps: usize) {
        match (self.active_screen, direction) {
            #[cfg(feature = "neighbors")]
            (MonitorScreen::Neighbors, direction) => self.neighbors.scroll(direction, steps),
            (MonitorScreen::CopperList, direction) => {
                for _ in 0..steps {
                    match direction {
                        ScrollDirection::Up => self.bandwidth_scroll_state.scroll_up(),
                        ScrollDirection::Down => self.bandwidth_scroll_state.scroll_down(),
                        ScrollDirection::Left => self.bandwidth_scroll_state.scroll_left(),
                        ScrollDirection::Right => self.bandwidth_scroll_state.scroll_right(),
                    }
                }
            }

            #[cfg(feature = "dag")]
            (MonitorScreen::Dag, ScrollDirection::Down) => {
                self.nodes_scrollable_widget_state
                    .nodes_scrollable_state
                    .scroll_down();
            }
            #[cfg(feature = "dag")]
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
            #[cfg(feature = "dag")]
            (MonitorScreen::Dag, ScrollDirection::Right) => {
                for _ in 0..steps {
                    self.nodes_scrollable_widget_state
                        .nodes_scrollable_state
                        .scroll_right();
                }
            }
            #[cfg(feature = "dag")]
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
                self.log_pane.offset_from_bottom =
                    self.log_pane.offset_from_bottom.saturating_add(steps);
            }
            #[cfg(feature = "log_pane")]
            (MonitorScreen::Logs, ScrollDirection::Down) => {
                self.log_pane.offset_from_bottom =
                    self.log_pane.offset_from_bottom.saturating_sub(steps);
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

        #[cfg(feature = "neighbors")]
        if self.active_screen == MonitorScreen::Neighbors {
            self.neighbors.click(x, y);
        }

        #[cfg(feature = "log_pane")]
        if self.active_screen == MonitorScreen::Logs {
            return self.start_log_selection(x, y);
        }

        MonitorUiAction::None
    }

    pub fn mark_graph_dirty(&mut self) {
        #[cfg(feature = "dag")]
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
            #[cfg(feature = "dag")]
            MonitorScreen::Dag => self.draw_nodes(f, area),
            #[cfg(feature = "neighbors")]
            MonitorScreen::Neighbors => self.neighbors.draw(f, area),
            MonitorScreen::Latency => self.draw_latency_table(f, area),
            MonitorScreen::CopperList => self.draw_copperlist_stats(f, area),
            MonitorScreen::MemoryPools => self.draw_memory_pools(f, area),

            #[cfg(feature = "sysinfo_pane")]
            MonitorScreen::System => self.draw_system_info(f, area),

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

    #[cfg(feature = "sysinfo_pane")]
    fn draw_system_info(&self, f: &mut Frame, area: Rect) {
        const VERSION: &str = env!("CARGO_PKG_VERSION");
        let mut lines = vec![
            Line::raw(""),
            Line::raw(format!("   -> Copper v{VERSION}")),
            Line::raw(""),
        ];

        #[cfg(native)]
        let mut body = self
            .system_info
            .clone()
            .into_text()
            .map(|text| text.to_owned())
            .unwrap_or_else(|_| Text::from(self.system_info.clone()));

        #[cfg(browser)]
        let mut body = self.system_info.clone();

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
                        Line::from(stat.handles_in_use.to_string()).alignment(Alignment::Right),
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

    fn draw_copperlist_stats(&mut self, f: &mut Frame, area: Rect) {
        let stats = self.model.inner.copperlist_stats.lock().unwrap();
        let size_display = format_bytes_or(stats.size_bytes as u64, "unknown");
        let raw_total = stats.raw_culist_bytes.max(stats.size_bytes as u64);
        let handles_display = format_bytes_or(stats.handle_bytes, "0 B");
        let mem_total = raw_total
            .saturating_add(stats.keyframe_bytes)
            .saturating_add(stats.structured_bytes_per_cl);
        let mem_total_display = format_bytes_or(mem_total, "unknown");
        let encoded_display = format_bytes_or(stats.encoded_bytes, "n/a");
        let space_saved_display = if raw_total > 0 && stats.encoded_bytes > 0 {
            let saved = 1.0 - (stats.encoded_bytes as f64) / (raw_total as f64);
            format!("{:.1}%", saved * 100.0)
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
        let dropped_copperlists_display = stats.dropped_copperlists_total.to_string();
        let dropped_keyframes_display = stats.dropped_keyframes_total.to_string();

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
        let dropped_copperlists_style = if stats.dropped_copperlists_total == 0 {
            Style::default()
        } else {
            Style::default()
                .fg(palette::LIGHT_RED)
                .add_modifier(Modifier::BOLD)
        };
        let dropped_keyframes_style = if stats.dropped_keyframes_total == 0 {
            Style::default()
        } else {
            Style::default()
                .fg(palette::LIGHT_RED)
                .add_modifier(Modifier::BOLD)
        };
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
            row("Dropped CopperLists", dropped_copperlists_display)
                .style(dropped_copperlists_style),
            row("Dropped keyframes", dropped_keyframes_display).style(dropped_keyframes_style),
            spacer.clone(),
            row("CL serialized size", encoded_display),
            row("Space saved", space_saved_display),
            row("Structured log / CL", structured_display),
            row("Structured BW", structured_bw),
            spacer.clone(),
            row("Total disk BW", disk_total_bw),
        ];

        let table = |rows, title: String| {
            Table::new(rows, [Constraint::Length(24), Constraint::Min(12)])
                .header(header.clone())
                .block(
                    Block::default()
                        .borders(Borders::ALL)
                        .border_type(BorderType::Rounded)
                        .title(title),
                )
        };
        let mem_table = table(mem_rows, " Memory BW ".into());
        let disk_table = table(disk_rows, " Disk / Encoding ".into());

        drop(stats);
        let mut telemetry = Vec::new();
        if let Some(streams) = &self.model.streams {
            for (monitor, rates) in streams.iter().zip(&mut self.stream_rates) {
                let snapshot = monitor.snapshot();
                rates.update(snapshot);
                telemetry.push((
                    format!(" Telemetry / TX: {} ", monitor.destination),
                    crate::stream_panel::rows(monitor, snapshot, rates),
                ));
            }
        }
        if telemetry.is_empty() {
            telemetry.push((
                " Telemetry / TX ".to_string(),
                vec![row("Status", "Not configured".to_string())],
            ));
        }
        let telemetry_height = telemetry
            .iter()
            .map(|(_, rows)| rows.len() + 4)
            .sum::<usize>();
        let content_size = Size::new(
            area.width.max(126),
            area.height
                .max(telemetry_height.min(u16::MAX as usize) as u16),
        );
        let offset = self.bandwidth_scroll_state.offset();
        self.bandwidth_scroll_state.set_offset(Position::new(
            offset.x.min(content_size.width.saturating_sub(area.width)),
            offset
                .y
                .min(content_size.height.saturating_sub(area.height)),
        ));
        let mut scroll = ScrollView::new(content_size);
        scroll.render_widget(
            Block::default().style(Style::default().bg(palette::BACKGROUND)),
            Rect::new(0, 0, content_size.width, content_size.height),
        );
        let panels = Layout::horizontal([Constraint::Fill(1); 3]).split(Rect::new(
            0,
            0,
            content_size.width,
            content_size.height,
        ));
        scroll.render_widget(mem_table, panels[0]);
        scroll.render_widget(disk_table, panels[1]);
        let mut y = 0;
        let last = telemetry.len() - 1;
        for (index, (title, rows)) in telemetry.into_iter().enumerate() {
            let remaining = content_size.height.saturating_sub(y);
            let height = if index == last {
                remaining
            } else {
                (rows.len() + 4).min(usize::from(remaining)) as u16
            };
            scroll.render_widget(
                table(rows, title),
                Rect::new(panels[2].x, y, panels[2].width, height),
            );
            y = y.saturating_add(height);
        }
        scroll.render(area, f.buffer_mut(), &mut self.bandwidth_scroll_state);
    }

    #[cfg(feature = "dag")]
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
        let Some(area) = self.log_pane.area else {
            self.log_pane.selection.clear();
            return MonitorUiAction::None;
        };
        if !point_inside(col, row, area.x, area.y, area.width, area.height) {
            self.log_pane.selection.clear();
            return MonitorUiAction::None;
        }

        let Some(point) = self.log_selection_point(col, row) else {
            return MonitorUiAction::None;
        };
        self.log_pane.selection.start(point);
        MonitorUiAction::None
    }

    #[cfg(feature = "log_pane")]
    fn drag_log_selection(&mut self, col: u16, row: u16) -> MonitorUiAction {
        let Some(point) = self.log_selection_point(col, row) else {
            return MonitorUiAction::None;
        };
        self.log_pane.selection.update(point);
        MonitorUiAction::None
    }

    #[cfg(feature = "log_pane")]
    fn finish_log_selection(&mut self, col: u16, row: u16) -> MonitorUiAction {
        let Some(point) = self.log_selection_point(col, row) else {
            self.log_pane.selection.clear();
            return MonitorUiAction::None;
        };
        self.log_pane.selection.update(point);
        self.selected_log_text()
            .map(MonitorUiAction::CopyLogSelection)
            .unwrap_or(MonitorUiAction::None)
    }

    #[cfg(feature = "log_pane")]
    fn log_selection_point(&self, col: u16, row: u16) -> Option<SelectionPoint> {
        let area = self.log_pane.area?;
        if !point_inside(col, row, area.x, area.y, area.width, area.height) {
            return None;
        }

        let rel_row = (row - area.y) as usize;
        let rel_col = (col - area.x) as usize;
        let line_index = self.visible_log_offset(area).saturating_add(rel_row);
        let line = self.log_pane.lines.get(line_index)?;
        Some(SelectionPoint {
            row: line_index,
            col: rel_col.min(line.text.chars().count()),
        })
    }

    #[cfg(feature = "log_pane")]
    fn visible_log_offset(&self, area: Rect) -> usize {
        let visible_rows = area.height as usize;
        let total_lines = self.log_pane.lines.len();
        let max_offset = total_lines.saturating_sub(visible_rows);
        let offset_from_bottom = self.log_pane.offset_from_bottom.min(max_offset);
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
        self.log_pane.area = Some(inner);
        self.log_pane.lines = self.model.log_lines();

        let visible_offset = self.visible_log_offset(inner);
        if let Some((start, end)) = self.log_pane.selection.range()
            && (start.row >= self.log_pane.lines.len() || end.row >= self.log_pane.lines.len())
        {
            self.log_pane.selection.clear();
        }

        let paragraph = Paragraph::new(self.build_log_text(inner, visible_offset)).block(block);
        f.render_widget(paragraph, area);
    }

    #[cfg(feature = "log_pane")]
    fn build_log_text(&self, area: Rect, visible_offset: usize) -> Text<'static> {
        let mut rendered_lines = Vec::new();
        let selection = self
            .log_pane
            .selection
            .range()
            .filter(|(start, end)| start != end);
        let selection_style = Style::default().bg(palette::BLUE).fg(palette::BACKGROUND);
        let visible_lines = self
            .log_pane
            .lines
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
        let (start, end) = self.log_pane.selection.range()?;
        if start == end || self.log_pane.lines.is_empty() {
            return None;
        }
        if start.row >= self.log_pane.lines.len() || end.row >= self.log_pane.lines.len() {
            return None;
        }

        let mut selected = Vec::new();
        for row in start.row..=end.row {
            let line = &self.log_pane.lines[row];
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
        let [area, mascot] =
            Layout::horizontal([Constraint::Min(0), Constraint::Length(4)]).areas(area);
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

        for (i, tab) in TAB_DEFS.iter().enumerate() {
            let key = ((b'1' + i as u8) as char).to_string();
            let is_active = self.active_screen == tab.screen;
            let bg = if is_active { active_bg } else { inactive_bg };
            let fg = if is_active { active_fg } else { inactive_fg };
            let label_style = if is_active {
                Style::default().fg(fg).bg(bg).add_modifier(Modifier::BOLD)
            } else {
                Style::default().fg(fg).bg(bg)
            };
            let tab_width = segment_width(&key, tab.label);
            self.tab_hitboxes.push(TabHitbox {
                screen: tab.screen,
                x: cursor_x,
                y: area.y,
                width: tab_width.min(area.right().saturating_sub(cursor_x)),
                height: area.height,
            });
            cursor_x = cursor_x.saturating_add(tab_width);

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
            spans.push(Span::styled(tab.label, label_style));
            spans.push(Span::styled(" ", Style::default().bg(bg)));
            spans.push(Span::styled("", Style::default().fg(bg).bg(base_bg)));
            spans.push(Span::styled(" ", Style::default().bg(base_bg)));
        }

        let tabs = Paragraph::new(Line::from(spans))
            .style(Style::default().bg(base_bg))
            .block(Block::default().style(Style::default().bg(base_bg)));
        f.render_widget(tabs, area);
        f.render_widget(
            Line::from(" 😼 ")
                .right_aligned()
                .style(Style::default().fg(active_fg).bg(base_bg)),
            mascot,
        );
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

        self.render_footer_badges(f, area, base_bg);
    }

    fn render_footer_badges(&self, f: &mut Frame, area: Rect, base_bg: Color) {
        if area.width == 0 || area.height == 0 {
            return;
        }

        let clid = self
            .model
            .inner
            .copperlist_stats
            .lock()
            .unwrap()
            .last_seen_clid
            .unwrap_or(0);
        let mut badges = footer_badges(self.model.footer_identity(), clid);
        while !badges.is_empty() {
            let (line, width) = footer_badge_line(&badges, base_bg);
            if width <= area.width {
                f.render_widget(
                    Paragraph::new(line),
                    Rect {
                        x: area.x.saturating_add(area.width.saturating_sub(width)),
                        y: area.y,
                        width,
                        height: 1,
                    },
                );
                return;
            }
            badges.remove(0);
        }
    }
}

fn point_inside(px: u16, py: u16, x: u16, y: u16, width: u16, height: u16) -> bool {
    px >= x && px < x + width && py >= y && py < y + height
}

fn segment_width(key: &str, label: &str) -> u16 {
    (6 + key.chars().count() + label.chars().count()) as u16
}

fn default_screen() -> MonitorScreen {
    #[cfg(feature = "dag")]
    {
        MonitorScreen::Dag
    }
    #[cfg(all(not(feature = "dag"), feature = "neighbors"))]
    {
        MonitorScreen::Neighbors
    }
    #[cfg(not(any(feature = "dag", feature = "neighbors")))]
    {
        MonitorScreen::Latency
    }
}

fn screen_for_tab_key(key: char) -> Option<MonitorScreen> {
    TAB_DEFS
        .iter()
        .enumerate()
        .find(|(i, _)| (b'1' + *i as u8) as char == key)
        .map(|(_, tab)| tab.screen)
}

fn tab_key_hint() -> String {
    let n = TAB_DEFS.len();
    if n == 0 {
        return "tabs".to_string();
    }
    if n == 1 {
        return "1".to_string();
    }
    format!("1-{n}")
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

pub(crate) fn format_bytes(bytes: f64) -> String {
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
#[cfg(feature = "dag")]
enum NodeType {
    Unknown,
    Source,
    Sink,
    Task,
    Bridge,
}

#[cfg(feature = "dag")]
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

#[cfg(feature = "dag")]
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
#[cfg(feature = "dag")]
struct DisplayNode {
    id: String,
    type_label: String,
    node_type: NodeType,
    inputs: Vec<String>,
    outputs: Vec<String>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg(feature = "dag")]
struct GraphCacheKey {
    area: Option<Size>,
    node_count: usize,
    connection_count: usize,
}

#[cfg(feature = "dag")]
struct GraphCache {
    graph: Option<NodeGraph<'static>>,
    content_size: Size,
    key: Option<GraphCacheKey>,
    dirty: bool,
}

#[cfg(feature = "dag")]
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

#[cfg(feature = "dag")]
struct NodesScrollableWidgetState {
    model: MonitorModel,
    display_nodes: Vec<DisplayNode>,
    connections: Vec<Connection>,
    status_index_map: Vec<Option<ComponentId>>,
    nodes_scrollable_state: ScrollViewState,
    graph_cache: GraphCache,
    initial_viewport_pending: bool,
    last_viewport_area: Option<Size>,
}

#[cfg(feature = "dag")]
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
            initial_viewport_pending: true,
            last_viewport_area: None,
        }
    }

    fn mark_graph_dirty(&mut self) {
        self.graph_cache.dirty = true;
    }

    fn ensure_graph_cache(&mut self, area: Rect) -> Size {
        let viewport_area: Size = area.into();
        let key = self.graph_cache_key(area);
        if self.graph_cache.needs_rebuild(key) {
            self.rebuild_graph_cache(area, key);
        } else if self.last_viewport_area != Some(viewport_area) {
            self.clamp_scroll_offset(area, self.graph_cache.content_size);
        }
        self.last_viewport_area = Some(viewport_area);
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
            area: self.display_nodes.is_empty().then_some(area.into()),
            node_count: self.display_nodes.len(),
            connection_count: self.connections.len(),
        }
    }

    fn estimate_initial_graph_size(&self, area: Rect) -> Size {
        if self.display_nodes.is_empty() {
            return Size::new(area.width.max(NODE_WIDTH), area.height.max(NODE_HEIGHT));
        }

        let node_layouts = self.build_node_layouts();
        let content_width = (node_layouts.len() as u16)
            .saturating_mul(NODE_WIDTH + 20)
            .max(NODE_WIDTH);
        // Disconnected subgraphs stack vertically, so the sum of node heights is a safe
        // upper bound for the initial routing grid.
        let stacked_height = node_layouts
            .iter()
            .fold(0u16, |height, node| height.saturating_add(node.size.1));
        let max_ports = self
            .display_nodes
            .iter()
            .map(|node| node.inputs.len().max(node.outputs.len()))
            .max()
            .unwrap_or_default();
        let min_height = (((max_ports + NODE_PORT_ROW_OFFSET) as u16) * 12).max(NODE_HEIGHT * 6);
        Size::new(content_width, stacked_height.max(min_height))
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
            self.estimate_initial_graph_size(area)
        } else {
            let graph = self.build_graph(self.estimate_initial_graph_size(area));
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

        let graph = self.build_graph(content_size);
        let graph_bounds = graph.content_bounds();
        self.graph_cache.graph = Some(graph);
        self.graph_cache.content_size = content_size;
        self.graph_cache.key = Some(key);
        self.graph_cache.dirty = false;
        self.last_viewport_area = Some(area.into());

        if self.initial_viewport_pending {
            self.nodes_scrollable_state
                .set_offset(initial_graph_scroll_offset(
                    area,
                    content_size,
                    graph_bounds,
                ));
            self.initial_viewport_pending = false;
        } else {
            self.clamp_scroll_offset(area, content_size);
        }
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

#[cfg(feature = "dag")]
struct NodesScrollableWidget<'a> {
    _marker: PhantomData<&'a ()>,
}

#[cfg(feature = "dag")]
const NODE_WIDTH: u16 = 29;
#[cfg(feature = "dag")]
const NODE_WIDTH_CONTENT: u16 = NODE_WIDTH - 2;
#[cfg(feature = "dag")]
const NODE_HEIGHT: u16 = 5;
#[cfg(feature = "dag")]
const NODE_META_LINES: usize = 2;
#[cfg(feature = "dag")]
const NODE_PORT_ROW_OFFSET: usize = NODE_META_LINES;
#[cfg(feature = "dag")]
const GRAPH_WIDTH_PADDING: u16 = NODE_WIDTH * 2;
#[cfg(feature = "dag")]
const GRAPH_HEIGHT_PADDING: u16 = NODE_HEIGHT * 4;
#[cfg(feature = "dag")]
const INITIAL_GRAPH_FOCUS_X_NUMERATOR: u32 = 5;
#[cfg(feature = "dag")]
const INITIAL_GRAPH_FOCUS_X_DENOMINATOR: u32 = 8;

#[cfg(feature = "dag")]
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

fn clip_with_ellipsis(value: &str, max_chars: usize) -> String {
    if max_chars == 0 {
        return String::new();
    }
    let char_count = value.chars().count();
    if char_count <= max_chars {
        return value.to_string();
    }
    if max_chars <= 3 {
        return value.chars().take(max_chars).collect();
    }
    let prefix: String = value.chars().take(max_chars - 3).collect();
    format!("{prefix}...")
}

fn footer_badges(identity: MonitorFooterIdentity, clid: u64) -> Vec<FooterBadge> {
    let mut badges = vec![FooterBadge {
        inner: format!(
            " {} ",
            clip_with_ellipsis(identity.system_name.as_str(), 18)
        ),
        bg: Color::Rgb(92, 102, 150),
        fg: Color::Rgb(236, 236, 236),
    }];
    if let Some(subsystem_name) = identity.subsystem_name {
        badges.push(FooterBadge {
            inner: format!(" {} ", clip_with_ellipsis(subsystem_name.as_str(), 18)),
            bg: Color::Rgb(116, 88, 128),
            fg: Color::Rgb(236, 236, 236),
        });
    }
    badges.extend([
        FooterBadge {
            inner: format!(" {} ", identity.instance_id),
            bg: Color::Rgb(136, 92, 78),
            fg: Color::Rgb(248, 231, 176),
        },
        FooterBadge {
            inner: format!(
                " {} ",
                clip_with_ellipsis(identity.mission_name.as_str(), 16)
            ),
            bg: Color::Rgb(86, 114, 98),
            fg: Color::Rgb(236, 236, 236),
        },
        FooterBadge {
            inner: format!(" {:020} ", clid),
            bg: Color::Rgb(198, 146, 64),
            fg: palette::BACKGROUND,
        },
    ]);
    badges
}

fn footer_badge_line(badges: &[FooterBadge], base_bg: Color) -> (Line<'static>, u16) {
    if badges.is_empty() {
        return (Line::default(), 0);
    }

    let mut spans = Vec::with_capacity(badges.len().saturating_mul(2).saturating_add(1));
    let mut width = 0u16;
    spans.push(Span::styled(
        "",
        Style::default().fg(badges[0].bg).bg(base_bg),
    ));
    width = width.saturating_add(1);

    for (idx, badge) in badges.iter().enumerate() {
        spans.push(Span::styled(
            badge.inner.clone(),
            Style::default()
                .fg(badge.fg)
                .bg(badge.bg)
                .add_modifier(Modifier::BOLD),
        ));
        width = width.saturating_add(badge.inner.chars().count() as u16);

        let next_bg = badges.get(idx + 1).map(|next| next.bg).unwrap_or(base_bg);
        spans.push(Span::styled("", Style::default().fg(badge.bg).bg(next_bg)));
        width = width.saturating_add(1);
    }

    (Line::from(spans), width)
}

#[cfg(feature = "dag")]
fn initial_graph_scroll_offset(area: Rect, content_size: Size, graph_bounds: Size) -> Position {
    let max_x = content_size
        .width
        .saturating_sub(area.width.saturating_sub(1));
    let max_y = content_size
        .height
        .saturating_sub(area.height.saturating_sub(1));
    let focus_x = (((graph_bounds.width as u32) * INITIAL_GRAPH_FOCUS_X_NUMERATOR)
        / INITIAL_GRAPH_FOCUS_X_DENOMINATOR) as u16;
    let focus_y = graph_bounds.height / 2;

    Position::new(
        focus_x.saturating_sub(area.width / 2).min(max_x),
        focus_y.saturating_sub(area.height / 2).min(max_y),
    )
}

#[cfg(feature = "dag")]
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

            let statuses = state.model.inner.component_statuses.lock().unwrap();
            for (idx, zone) in zones.into_iter().enumerate() {
                let status = state
                    .status_index_map
                    .get(idx)
                    .and_then(|component_id| *component_id)
                    .and_then(|component_id| statuses.get(component_id.index()))
                    .cloned()
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

#[cfg(test)]
mod tests;
