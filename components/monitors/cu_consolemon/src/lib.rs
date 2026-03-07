#[cfg(feature = "debug_pane")]
mod debug_pane;
pub mod sysinfo;

pub use cu_tuimon::{
    MonitorModel, MonitorScreen, MonitorUi, MonitorUiEvent, MonitorUiKey, MonitorUiOptions,
    ScrollDirection,
};

use color_eyre::config::HookBuilder;
use cu29::clock::CuDuration;
use cu29::context::CuContext;
use cu29::monitoring::{
    ComponentId, CopperListIoStats, CopperListView, CuComponentState, CuMonitor,
    CuMonitoringMetadata, CuMonitoringRuntime, Decision,
};
use cu29::{CuError, CuResult};
use cu29_log::CuLogLevel;
#[cfg(debug_assertions)]
use cu29_log_runtime::{
    format_message_only, register_live_log_listener, unregister_live_log_listener,
};
#[cfg(feature = "debug_pane")]
use debug_pane::{StyledLine, StyledRun, UIExt};
use ratatui::backend::CrosstermBackend;
use ratatui::crossterm::event::{
    DisableMouseCapture, EnableMouseCapture, Event, KeyCode, MouseButton, MouseEventKind,
};
use ratatui::crossterm::terminal::{
    EnterAlternateScreen, LeaveAlternateScreen, disable_raw_mode, enable_raw_mode,
};
use ratatui::crossterm::tty::IsTty;
use ratatui::crossterm::{event, execute};
use ratatui::layout::{Constraint, Direction, Layout, Rect};
use ratatui::style::{Color, Modifier, Style};
use ratatui::text::{Line, Span, Text};
use ratatui::widgets::{Block, Paragraph};
use ratatui::{Frame, Terminal};
use std::backtrace::Backtrace;
use std::collections::HashMap;
use std::io::{Write, stdin, stdout};
use std::process;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, OnceLock};
use std::thread::JoinHandle;
use std::time::Duration;
use std::{io, thread};

#[cfg(feature = "debug_pane")]
use arboard::Clipboard;

#[derive(Clone, Copy)]
struct TabDef {
    screen: ConsoleScreen,
    label: &'static str,
    key: &'static str,
}

#[derive(Clone, Copy)]
struct TabHitbox {
    screen: ConsoleScreen,
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

#[derive(Clone, Copy, PartialEq)]
enum ConsoleScreen {
    Base(MonitorScreen),
    #[cfg(feature = "debug_pane")]
    DebugOutput,
}

const TAB_DEFS: &[TabDef] = &[
    TabDef {
        screen: ConsoleScreen::Base(MonitorScreen::System),
        label: "SYS",
        key: "1",
    },
    TabDef {
        screen: ConsoleScreen::Base(MonitorScreen::Dag),
        label: "DAG",
        key: "2",
    },
    TabDef {
        screen: ConsoleScreen::Base(MonitorScreen::Latency),
        label: "LAT",
        key: "3",
    },
    TabDef {
        screen: ConsoleScreen::Base(MonitorScreen::CopperList),
        label: "BW",
        key: "4",
    },
    TabDef {
        screen: ConsoleScreen::Base(MonitorScreen::MemoryPools),
        label: "MEM",
        key: "5",
    },
    #[cfg(feature = "debug_pane")]
    TabDef {
        screen: ConsoleScreen::DebugOutput,
        label: "LOG",
        key: "6",
    },
];

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

/// A TUI based realtime console for Copper.
pub struct CuConsoleMon {
    model: MonitorModel,
    ui_handle: Option<JoinHandle<()>>,
    quitting: Arc<AtomicBool>,
}

impl CuConsoleMon {
    pub fn model(&self) -> MonitorModel {
        self.model.clone()
    }
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
    monitor_ui: MonitorUi,
    quitting: Arc<AtomicBool>,
    active_screen: ConsoleScreen,
    tab_hitboxes: Vec<TabHitbox>,
    help_hitboxes: Vec<HelpHitbox>,
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
}

impl UI {
    #[cfg(feature = "debug_pane")]
    fn new(
        model: MonitorModel,
        quitting: Arc<AtomicBool>,
        error_redirect: Option<gag::BufferRedirect>,
        debug_output: Option<debug_pane::DebugLog>,
    ) -> Self {
        init_error_hooks();
        Self {
            monitor_ui: MonitorUi::new(
                model,
                MonitorUiOptions {
                    system_info: sysinfo::pfetch_info(),
                    show_quit_hint: false,
                },
            ),
            quitting,
            active_screen: ConsoleScreen::Base(MonitorScreen::System),
            tab_hitboxes: Vec::new(),
            help_hitboxes: Vec::new(),
            error_redirect,
            debug_output,
            debug_output_area: None,
            debug_output_visible_offset: 0,
            debug_output_lines: Vec::new(),
            debug_selection: DebugSelection::default(),
            clipboard: None,
        }
    }

    #[cfg(not(feature = "debug_pane"))]
    fn new(model: MonitorModel, quitting: Arc<AtomicBool>) -> Self {
        init_error_hooks();
        Self {
            monitor_ui: MonitorUi::new(
                model,
                MonitorUiOptions {
                    system_info: sysinfo::pfetch_info(),
                    show_quit_hint: false,
                },
            ),
            quitting,
            active_screen: ConsoleScreen::Base(MonitorScreen::System),
            tab_hitboxes: Vec::new(),
            help_hitboxes: Vec::new(),
        }
    }

    fn set_screen(&mut self, screen: ConsoleScreen) {
        self.active_screen = screen;
        if let ConsoleScreen::Base(screen) = screen {
            self.monitor_ui.set_active_screen(screen);
        }
    }

    fn draw(&mut self, f: &mut Frame) {
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

        match self.active_screen {
            ConsoleScreen::Base(screen) => {
                self.monitor_ui.set_active_screen(screen);
                self.monitor_ui.draw_content(f, layout[1]);
            }
            #[cfg(feature = "debug_pane")]
            ConsoleScreen::DebugOutput => self.draw_debug_output(f, layout[1]),
        }
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

        let tab_hint = if cfg!(feature = "debug_pane") {
            "1-6"
        } else {
            "1-5"
        };
        let segments = [
            (tab_hint, "Tabs", Color::Rgb(86, 114, 98), None),
            (
                "r",
                "Reset latency",
                Color::Rgb(136, 92, 78),
                Some(HelpAction::ResetLatency),
            ),
            ("hjkl/←↑→↓", "Scroll", Color::Rgb(92, 102, 150), None),
            (
                "q",
                "Quit",
                Color::Rgb(124, 118, 76),
                Some(HelpAction::Quit),
            ),
        ];

        for (key, label, bg, action) in segments {
            let segment_len = segment_width(key, label);
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
            let value = self
                .monitor_ui
                .model()
                .last_seen_copperlist_id()
                .unwrap_or(0);
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
                            .fg(Color::Black)
                            .bg(badge_bg)
                            .add_modifier(Modifier::BOLD),
                    ),
                    Span::styled("", Style::default().fg(badge_bg).bg(base_bg)),
                ])),
                clid_area,
            );
        }
    }

    fn handle_tab_click(&mut self, mouse: event::MouseEvent) {
        if !matches!(mouse.kind, MouseEventKind::Down(MouseButton::Left)) {
            return;
        }
        for hitbox in &self.tab_hitboxes {
            if mouse_inside(&mouse, hitbox.x, hitbox.y, hitbox.width, hitbox.height) {
                self.set_screen(hitbox.screen);
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
                    if matches!(
                        self.active_screen,
                        ConsoleScreen::Base(MonitorScreen::Latency)
                    ) {
                        let _ = self
                            .monitor_ui
                            .handle_event(MonitorUiEvent::Key(MonitorUiKey::Char('r')));
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

    fn handle_key(&mut self, key: KeyCode) -> bool {
        match key {
            KeyCode::Char('1') => self.set_screen(ConsoleScreen::Base(MonitorScreen::System)),
            KeyCode::Char('2') => self.set_screen(ConsoleScreen::Base(MonitorScreen::Dag)),
            KeyCode::Char('3') => self.set_screen(ConsoleScreen::Base(MonitorScreen::Latency)),
            KeyCode::Char('4') => self.set_screen(ConsoleScreen::Base(MonitorScreen::CopperList)),
            KeyCode::Char('5') => self.set_screen(ConsoleScreen::Base(MonitorScreen::MemoryPools)),
            #[cfg(feature = "debug_pane")]
            KeyCode::Char('6') => self.set_screen(ConsoleScreen::DebugOutput),
            KeyCode::Char('r') => {
                if matches!(
                    self.active_screen,
                    ConsoleScreen::Base(MonitorScreen::Latency)
                ) {
                    let _ = self
                        .monitor_ui
                        .handle_event(MonitorUiEvent::Key(MonitorUiKey::Char('r')));
                }
            }
            KeyCode::Char('j') => self.handle_monitor_key(MonitorUiKey::Char('j')),
            KeyCode::Char('k') => self.handle_monitor_key(MonitorUiKey::Char('k')),
            KeyCode::Char('h') => self.handle_monitor_key(MonitorUiKey::Char('h')),
            KeyCode::Char('l') => self.handle_monitor_key(MonitorUiKey::Char('l')),
            KeyCode::Down => self.handle_monitor_key(MonitorUiKey::Down),
            KeyCode::Up => self.handle_monitor_key(MonitorUiKey::Up),
            KeyCode::Left => self.handle_monitor_key(MonitorUiKey::Left),
            KeyCode::Right => self.handle_monitor_key(MonitorUiKey::Right),
            KeyCode::Char('q') => {
                self.quitting.store(true, Ordering::SeqCst);
                return true;
            }
            _ => {}
        }
        false
    }

    fn handle_monitor_key(&mut self, key: MonitorUiKey) {
        if let ConsoleScreen::Base(_) = self.active_screen {
            let _ = self.monitor_ui.handle_event(MonitorUiEvent::Key(key));
        }
    }

    fn handle_scroll_mouse(&mut self, mouse: event::MouseEvent) {
        if !matches!(
            self.active_screen,
            ConsoleScreen::Base(MonitorScreen::Dag | MonitorScreen::Latency)
        ) {
            return;
        }
        match mouse.kind {
            MouseEventKind::ScrollDown => {
                let _ = self.monitor_ui.handle_event(MonitorUiEvent::Scroll {
                    direction: ScrollDirection::Down,
                    steps: 1,
                });
            }
            MouseEventKind::ScrollUp => {
                let _ = self.monitor_ui.handle_event(MonitorUiEvent::Scroll {
                    direction: ScrollDirection::Up,
                    steps: 1,
                });
            }
            MouseEventKind::ScrollRight => {
                let _ = self.monitor_ui.handle_event(MonitorUiEvent::Scroll {
                    direction: ScrollDirection::Right,
                    steps: 5,
                });
            }
            MouseEventKind::ScrollLeft => {
                let _ = self.monitor_ui.handle_event(MonitorUiEvent::Scroll {
                    direction: ScrollDirection::Left,
                    steps: 5,
                });
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

        if self.active_screen != ConsoleScreen::DebugOutput {
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
            MouseEventKind::Down(MouseButton::Left) => self.debug_selection.start(point),
            MouseEventKind::Drag(MouseButton::Left) => self.debug_selection.update(point),
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
        if start == end || self.debug_output_lines.is_empty() {
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

    fn run_app<B: ratatui::prelude::Backend<Error = io::Error>>(
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
                match event::read()? {
                    Event::Key(key) => {
                        if self.handle_key(key.code) {
                            break;
                        }
                    }
                    Event::Mouse(mouse) => self.handle_mouse_event(mouse),
                    Event::Resize(_columns, rows) => {
                        self.monitor_ui.mark_graph_dirty();
                        #[cfg(feature = "debug_pane")]
                        if let Some(debug_output) = self.debug_output.as_mut() {
                            debug_output.max_rows.store(rows, Ordering::SeqCst);
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
    fn new(metadata: CuMonitoringMetadata, _runtime: CuMonitoringRuntime) -> CuResult<Self> {
        Ok(Self {
            model: MonitorModel::from_metadata(&metadata),
            ui_handle: None,
            quitting: Arc::new(AtomicBool::new(false)),
        })
    }

    fn observe_copperlist_io(&self, stats: CopperListIoStats) {
        self.model.observe_copperlist_io(stats);
    }

    fn start(&mut self, _ctx: &CuContext) -> CuResult<()> {
        if !should_start_ui() {
            #[cfg(debug_assertions)]
            {
                register_live_log_listener(|entry, format_str, param_names| {
                    if let Some(line) = format_headless_log_line(entry, format_str, param_names) {
                        println!("{line}");
                    }
                });
            }
            return Ok(());
        }

        let model = self.model.clone();
        let quitting = self.quitting.clone();

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
                let error_redirect = match gag::BufferRedirect::stderr() {
                    Ok(redirect) => Some(redirect),
                    Err(err) => {
                        eprintln!(
                            "Failed to redirect stderr for debug pane; continuing without redirect: {err}"
                        );
                        None
                    }
                };

                let mut ui = UI::new(model.clone(), quitting.clone(), error_redirect, None);

                #[cfg(debug_assertions)]
                {
                    let max_lines = terminal.size().unwrap().height.saturating_sub(5);
                    let (mut debug_log, tx) = debug_pane::DebugLog::new(max_lines);

                    cu29_log_runtime::register_live_log_listener(
                        move |entry, format_str, param_names| {
                            let params: Vec<String> =
                                entry.params.iter().map(|value| value.to_string()).collect();
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
                            let _ = tx.try_send(line);
                        },
                    );

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
                let mut ui = UI::new(model, quitting.clone());
                if let Err(err) = ui.run_app(&mut terminal) {
                    let _ = restore_terminal();
                    eprintln!("CuConsoleMon UI exited with error: {err}");
                    return;
                }
                drop(stderr_gag);
            }

            quitting.store(true, Ordering::SeqCst);
            let _ = restore_terminal();
        });

        self.ui_handle = Some(handle);
        Ok(())
    }

    fn process_copperlist(&self, ctx: &CuContext, view: CopperListView<'_>) -> CuResult<()> {
        self.model.process_copperlist(ctx.cl_id(), view);
        if self.quitting.load(Ordering::SeqCst) {
            return Err("Exiting...".into());
        }
        Ok(())
    }

    fn process_error(
        &self,
        component_id: ComponentId,
        step: CuComponentState,
        error: &CuError,
    ) -> Decision {
        self.model
            .set_component_error(component_id, error.to_string());
        match step {
            CuComponentState::Start => Decision::Shutdown,
            CuComponentState::Preprocess => Decision::Abort,
            CuComponentState::Process => Decision::Ignore,
            CuComponentState::Postprocess => Decision::Ignore,
            CuComponentState::Stop => Decision::Shutdown,
        }
    }

    fn stop(&mut self, _ctx: &CuContext) -> CuResult<()> {
        self.quitting.store(true, Ordering::SeqCst);
        let _ = restore_terminal();

        if let Some(handle) = self.ui_handle.take() {
            let _ = handle.join();
        }

        #[cfg(debug_assertions)]
        if !should_start_ui() {
            unregister_live_log_listener();
        }

        self.model.reset_latency();
        Ok(())
    }
}

struct TerminalRestoreGuard;

impl Drop for TerminalRestoreGuard {
    fn drop(&mut self) {
        let _ = restore_terminal();
    }
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
        println!("CuConsoleMon panic: {info}");
        println!("Backtrace:\n{bt}");
        let _ = stdout().flush();
        process::exit(1);
    }));

    let _ = ONCE.set(());
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
fn format_ts(time: cu29::prelude::CuTime) -> String {
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
    let mut iter = format_str.char_indices().peekable();
    while let Some((idx, ch)) = iter.next() {
        if ch == '{' {
            let start_idx = idx + ch.len_utf8();
            if let Some(end) = format_str[start_idx..].find('}') {
                let end_idx = start_idx + end;
                let placeholder = &format_str[start_idx..end_idx];
                let replacement_opt = if placeholder.is_empty() {
                    anon_iter.next()
                } else {
                    named_params.get(placeholder)
                };
                if let Some(replacement) = replacement_opt {
                    let span_start = out.chars().count();
                    out.push_str(replacement);
                    let span_end = out.chars().count();
                    param_spans.push((span_start, span_end));
                    let skip_to = end_idx + '}'.len_utf8();
                    while let Some((next_idx, _)) = iter.peek().copied() {
                        if next_idx < skip_to {
                            iter.next();
                        } else {
                            break;
                        }
                    }
                    continue;
                }
            }
        }
        out.push(ch);
    }
    (out, param_spans)
}

#[cfg(feature = "debug_pane")]
fn styled_line_from_structured(
    time: cu29::prelude::CuTime,
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
    let mut param_spans_sorted = param_spans;
    param_spans_sorted.sort_by_key(|span| span.0);
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

    let prefix = format!("{ts} {level_txt} ");
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

#[cfg(debug_assertions)]
fn format_timestamp(time: CuDuration) -> String {
    let nanos = time.as_nanos();
    let total_seconds = nanos / 1_000_000_000;
    let hours = total_seconds / 3600;
    let minutes = (total_seconds / 60) % 60;
    let seconds = total_seconds % 60;
    let fractional_1e4 = (nanos % 1_000_000_000) / 100_000;
    format!("{hours:02}:{minutes:02}:{seconds:02}.{fractional_1e4:04}")
}

#[cfg(debug_assertions)]
fn format_headless_log_line(
    entry: &cu29_log::CuLogEntry,
    format_str: &str,
    param_names: &[&str],
) -> Option<String> {
    let params: Vec<String> = entry.params.iter().map(|value| value.to_string()).collect();
    let named: HashMap<String, String> = param_names
        .iter()
        .zip(params.iter())
        .map(|(key, value)| (key.to_string(), value.clone()))
        .collect();

    format_message_only(format_str, params.as_slice(), &named)
        .ok()
        .map(|msg| {
            let ts = format_timestamp(entry.time);
            format!("{} [{:?}] {}", ts, entry.level, msg)
        })
}

fn should_start_ui() -> bool {
    if !stdout().is_tty() || !stdin().is_tty() {
        return false;
    }

    #[cfg(unix)]
    {
        use std::os::unix::io::AsRawFd;
        let stdin_fd = stdin().as_raw_fd();
        let fg_pgrp = unsafe { libc::tcgetpgrp(stdin_fd) };
        if fg_pgrp == -1 {
            return false;
        }
        let pgrp = unsafe { libc::getpgrp() };
        if fg_pgrp != pgrp {
            return false;
        }
    }

    true
}
