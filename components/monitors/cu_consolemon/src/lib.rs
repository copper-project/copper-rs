#[cfg(feature = "debug_pane")]
use arboard::Clipboard;
use color_eyre::config::HookBuilder;
#[cfg(feature = "debug_pane")]
use cu_tuimon::MonitorLogCapture;
pub use cu_tuimon::{
    MonitorModel, MonitorScreen, MonitorUi, MonitorUiAction, MonitorUiEvent, MonitorUiKey,
    MonitorUiOptions, ScrollDirection,
};
use cu29::context::CuContext;
use cu29::monitoring::{
    ComponentId, CopperListIoStats, CopperListView, CuComponentState, CuMonitor,
    CuMonitoringMetadata, CuMonitoringRuntime, Decision, PanicHookRegistration,
};
use cu29::{CuError, CuResult};
use ratatui::backend::CrosstermBackend;
use ratatui::crossterm::event::{
    DisableMouseCapture, EnableMouseCapture, Event, KeyCode, MouseButton, MouseEventKind,
};
use ratatui::crossterm::terminal::{
    EnterAlternateScreen, LeaveAlternateScreen, disable_raw_mode, enable_raw_mode,
};
use ratatui::crossterm::tty::IsTty;
use ratatui::crossterm::{event, execute};
use ratatui::{Terminal, TerminalOptions, Viewport};
use std::io::{stdin, stdout};
#[cfg(feature = "debug_pane")]
use std::sync::Mutex;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, OnceLock};
use std::thread::JoinHandle;
use std::time::Duration;
use std::{io, thread};

/// A TUI based realtime console for Copper.
pub struct CuConsoleMon {
    model: MonitorModel,
    ui_handle: Option<JoinHandle<()>>,
    quitting: Arc<AtomicBool>,
    monitor_runtime: CuMonitoringRuntime,
    panic_cleanup: Option<PanicHookRegistration>,
    #[cfg(feature = "debug_pane")]
    log_capture: Option<Mutex<MonitorLogCapture>>,
}

impl CuConsoleMon {
    pub fn model(&self) -> MonitorModel {
        self.model.clone()
    }
}

impl Drop for CuConsoleMon {
    fn drop(&mut self) {
        self.quitting.store(true, Ordering::SeqCst);
        self.panic_cleanup = None;
        let _ = restore_terminal();
        if let Some(handle) = self.ui_handle.take() {
            let _ = handle.join();
        }
    }
}

struct UI {
    monitor_ui: MonitorUi,
    quitting: Arc<AtomicBool>,
    #[cfg(feature = "debug_pane")]
    clipboard: Option<Clipboard>,
}

impl UI {
    fn new(model: MonitorModel, quitting: Arc<AtomicBool>) -> Self {
        init_error_hooks();
        Self {
            monitor_ui: MonitorUi::new(
                model,
                MonitorUiOptions {
                    show_quit_hint: true,
                },
            ),
            quitting,
            #[cfg(feature = "debug_pane")]
            clipboard: None,
        }
    }

    fn draw(&mut self, frame: &mut ratatui::Frame) {
        self.monitor_ui.draw(frame);
    }

    fn handle_action(&mut self, action: MonitorUiAction) -> bool {
        match action {
            MonitorUiAction::None => false,
            MonitorUiAction::QuitRequested => {
                self.quitting.store(true, Ordering::SeqCst);
                true
            }
            #[cfg(feature = "debug_pane")]
            MonitorUiAction::CopyLogSelection(text) => {
                self.copy_text(text);
                false
            }
        }
    }

    fn handle_key(&mut self, key: KeyCode) -> bool {
        let action = match key {
            KeyCode::Char(ch) => {
                self.monitor_ui
                    .handle_event(MonitorUiEvent::Key(MonitorUiKey::Char(
                        ch.to_ascii_lowercase(),
                    )))
            }
            KeyCode::Left => self
                .monitor_ui
                .handle_event(MonitorUiEvent::Key(MonitorUiKey::Left)),
            KeyCode::Right => self
                .monitor_ui
                .handle_event(MonitorUiEvent::Key(MonitorUiKey::Right)),
            KeyCode::Up => self
                .monitor_ui
                .handle_event(MonitorUiEvent::Key(MonitorUiKey::Up)),
            KeyCode::Down => self
                .monitor_ui
                .handle_event(MonitorUiEvent::Key(MonitorUiKey::Down)),
            _ => MonitorUiAction::None,
        };

        self.handle_action(action)
    }

    fn handle_mouse_event(&mut self, mouse: event::MouseEvent) {
        let action = match mouse.kind {
            MouseEventKind::Down(MouseButton::Left) => {
                self.monitor_ui.handle_event(MonitorUiEvent::MouseDown {
                    col: mouse.column,
                    row: mouse.row,
                })
            }
            #[cfg(feature = "debug_pane")]
            MouseEventKind::Drag(MouseButton::Left) => {
                self.monitor_ui.handle_event(MonitorUiEvent::MouseDrag {
                    col: mouse.column,
                    row: mouse.row,
                })
            }
            #[cfg(feature = "debug_pane")]
            MouseEventKind::Up(MouseButton::Left) => {
                self.monitor_ui.handle_event(MonitorUiEvent::MouseUp {
                    col: mouse.column,
                    row: mouse.row,
                })
            }
            MouseEventKind::ScrollDown => self.monitor_ui.handle_event(MonitorUiEvent::Scroll {
                direction: ScrollDirection::Down,
                steps: 1,
            }),
            MouseEventKind::ScrollUp => self.monitor_ui.handle_event(MonitorUiEvent::Scroll {
                direction: ScrollDirection::Up,
                steps: 1,
            }),
            MouseEventKind::ScrollLeft => self.monitor_ui.handle_event(MonitorUiEvent::Scroll {
                direction: ScrollDirection::Left,
                steps: 5,
            }),
            MouseEventKind::ScrollRight => self.monitor_ui.handle_event(MonitorUiEvent::Scroll {
                direction: ScrollDirection::Right,
                steps: 5,
            }),
            _ => MonitorUiAction::None,
        };

        let _ = self.handle_action(action);
    }

    #[cfg(feature = "debug_pane")]
    fn copy_text(&mut self, text: String) {
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

    fn run_app<B: ratatui::prelude::Backend<Error = io::Error>>(
        &mut self,
        terminal: &mut Terminal<B>,
    ) -> io::Result<()> {
        loop {
            if self.quitting.load(Ordering::SeqCst) {
                break;
            }

            terminal.draw(|frame| {
                self.draw(frame);
            })?;

            if event::poll(Duration::from_millis(50))? {
                match event::read()? {
                    Event::Key(key) if self.handle_key(key.code) => {
                        break;
                    }
                    Event::Mouse(mouse) => self.handle_mouse_event(mouse),
                    Event::Resize(_, _) => self.monitor_ui.mark_graph_dirty(),
                    _ => {}
                }
            }
        }
        Ok(())
    }
}

impl CuMonitor for CuConsoleMon {
    fn new(metadata: CuMonitoringMetadata, runtime: CuMonitoringRuntime) -> CuResult<Self> {
        Ok(Self {
            model: MonitorModel::from_metadata(&metadata),
            ui_handle: None,
            quitting: Arc::new(AtomicBool::new(false)),
            monitor_runtime: runtime,
            panic_cleanup: None,
            #[cfg(feature = "debug_pane")]
            log_capture: None,
        })
    }

    fn observe_copperlist_io(&self, stats: CopperListIoStats) {
        self.model.observe_copperlist_io(stats);
    }

    fn start(&mut self, _ctx: &CuContext) -> CuResult<()> {
        #[cfg(feature = "debug_pane")]
        {
            self.log_capture = Some(Mutex::new(if should_start_ui() {
                MonitorLogCapture::to_model(self.model.clone())
            } else {
                MonitorLogCapture::to_stdout()
            }));
        }

        if !should_start_ui() {
            return Ok(());
        }

        self.panic_cleanup = Some(self.monitor_runtime.register_panic_cleanup(|_| {
            let _ = restore_terminal();
        }));

        let model = self.model.clone();
        let quitting = self.quitting.clone();
        let handle = thread::spawn(move || {
            let backend = CrosstermBackend::new(stdout());
            let _terminal_guard = TerminalRestoreGuard;

            if let Err(err) = setup_terminal() {
                eprintln!("Failed to prepare terminal UI: {err}");
                return;
            }

            let mut terminal = match Terminal::with_options(
                backend,
                TerminalOptions {
                    viewport: Viewport::Fullscreen,
                },
            ) {
                Ok(terminal) => terminal,
                Err(err) => {
                    eprintln!("Failed to initialize terminal backend: {err}");
                    return;
                }
            };

            let mut ui = UI::new(model, quitting.clone());
            if let Err(err) = ui.run_app(&mut terminal) {
                let _ = restore_terminal();
                eprintln!("CuConsoleMon UI exited with error: {err}");
                return;
            }

            quitting.store(true, Ordering::SeqCst);
            let _ = restore_terminal();
        });

        self.ui_handle = Some(handle);
        Ok(())
    }

    fn process_copperlist(&self, ctx: &CuContext, view: CopperListView<'_>) -> CuResult<()> {
        #[cfg(feature = "debug_pane")]
        if let Some(log_capture) = &self.log_capture {
            let mut log_capture = log_capture.lock().unwrap_or_else(|err| err.into_inner());
            log_capture.poll();
        }

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
        self.panic_cleanup = None;
        let _ = restore_terminal();

        if let Some(handle) = self.ui_handle.take() {
            let _ = handle.join();
        }

        #[cfg(feature = "debug_pane")]
        {
            self.log_capture = None;
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

    let (_unused_panic_hook, error) = HookBuilder::default().into_hooks();
    let error = error.into_eyre_hook();
    color_eyre::eyre::set_hook(Box::new(move |err| {
        let _ = restore_terminal();
        error(err)
    }))
    .unwrap();

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
