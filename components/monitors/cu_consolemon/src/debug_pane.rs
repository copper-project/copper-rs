use crate::UI;
use ratatui::Frame;
use ratatui::layout::Rect;
use ratatui::prelude::Stylize;
use ratatui::style::{Color, Style};
use ratatui::widgets::{Block, BorderType, Borders, Paragraph};
use std::sync::atomic::Ordering;
use std::sync::mpsc::SendError;
use {
    log::{Level, LevelFilter, Log, Metadata, Record},
    std::collections::VecDeque,
    std::io::Read,
    std::sync::atomic::AtomicU16,
    std::sync::mpsc::{Receiver, SyncSender},
};

#[derive(Debug)]
pub struct DebugLog {
    debug_log: VecDeque<StyledLine>,
    pub(crate) max_rows: AtomicU16,
    rx: Receiver<StyledLine>,
}

#[derive(Clone, Debug)]
pub struct StyledRun {
    /// Inclusive start / exclusive end in character indices
    pub start: usize,
    pub end: usize,
    pub style: Style,
}

#[derive(Clone, Debug)]
pub struct StyledLine {
    pub text: String,
    pub runs: Vec<StyledRun>,
}

impl DebugLog {
    #[allow(dead_code)]
    pub fn new(max_lines: u16) -> (Self, SyncSender<StyledLine>) {
        let (tx, rx) = std::sync::mpsc::sync_channel(1000);
        (
            Self {
                debug_log: VecDeque::new(),
                max_rows: AtomicU16::new(max_lines),
                rx,
            },
            tx,
        )
    }

    pub fn push_line(&mut self, line: StyledLine) {
        if line.text.is_empty() {
            return;
        }
        self.debug_log.push_back(line);
        let max_row = self.max_rows.load(Ordering::SeqCst) as usize;
        while self.debug_log.len() > max_row {
            self.debug_log.pop_front();
        }
    }

    pub fn update_logs(&mut self) {
        let max_row = self.max_rows.load(Ordering::SeqCst) as usize;

        for log in self.rx.try_iter() {
            self.debug_log.push_back(log);
            if self.debug_log.len() > max_row {
                self.debug_log.pop_front();
            }
        }
    }

    #[allow(dead_code)]
    pub fn lines(&self) -> Vec<StyledLine> {
        self.debug_log.iter().cloned().collect()
    }
}

#[derive(Clone)]
pub struct LogSubscriber {
    tx: SyncSender<String>,
}

impl LogSubscriber {
    #[allow(dead_code)]
    pub fn new(tx: SyncSender<String>) -> Self {
        let log_subscriber = Self { tx };
        if log::set_boxed_logger(Box::new(log_subscriber.clone())).is_err() {
            eprintln!("Failed to set `LogSubscriber` as global log subscriber")
        }
        log::set_max_level(LevelFilter::Debug);
        log_subscriber
    }

    pub fn push_logs(&self, log: String) {
        if let Err(err) = self.tx.send(log) {
            let SendError(msg) = err;
            eprintln!("Error Sending Logs to MPSC Channel: {msg}")
        }
    }
}

impl Log for LogSubscriber {
    fn enabled(&self, metadata: &Metadata) -> bool {
        metadata.level() <= Level::Debug
    }

    fn log(&self, record: &Record) {
        if self.enabled(record.metadata()) {
            let message = format!(
                "{} [{}] - {}\n",
                chrono::Local::now().time().format("%H:%M:%S"),
                record.level(),
                record.args()
            );

            self.push_logs(message);
        }
    }

    fn flush(&self) {}
}

pub trait UIExt {
    fn update_debug_output(&mut self);

    fn draw_debug_output(&mut self, f: &mut Frame, area: Rect);
}

impl UIExt for UI {
    fn update_debug_output(&mut self) {
        if let Some(debug_output) = self.debug_output.as_mut() {
            if let Some(error_redirect) = self.error_redirect.as_mut() {
                let mut error_buffer = String::new();
                if let Err(err) = error_redirect.read_to_string(&mut error_buffer) {
                    eprintln!("Failed to read stderr buffer for debug pane: {err}");
                }
                if !error_buffer.is_empty() {
                    for line in error_buffer.lines() {
                        debug_output.push_line(StyledLine {
                            text: line.to_string(),
                            runs: vec![StyledRun {
                                start: 0,
                                end: line.chars().count(),
                                style: Style::default().fg(Color::Red),
                            }],
                        });
                    }
                }
            }
            debug_output.update_logs();
        }
    }

    fn draw_debug_output(&mut self, f: &mut Frame, area: Rect) {
        if let Some(debug_output) = self.debug_output.as_mut() {
            let block = Block::default()
                .title(" Debug Output ")
                .title_bottom(format!("{} log entries", debug_output.debug_log.len()))
                .borders(Borders::ALL)
                .border_type(BorderType::Rounded);
            let inner = block.inner(area);
            self.debug_output_area = Some(inner);
            self.debug_output_lines = debug_output.lines();
            self.debug_output_visible_offset = self
                .debug_output_lines
                .len()
                .saturating_sub(inner.height as usize);
            if let Some((start, end)) = self.debug_selection.range()
                && (start.row >= self.debug_output_lines.len()
                    || end.row >= self.debug_output_lines.len())
            {
                self.debug_selection.clear();
            }

            let p = Paragraph::new(self.build_debug_output_text(inner)).block(block);
            f.render_widget(p, area);
        } else {
            self.debug_output_area = None;
            self.debug_output_visible_offset = 0;
            self.debug_output_lines.clear();
            self.debug_selection.clear();

            #[cfg(debug_assertions)]
            let text = "Text logger is disabled";

            #[cfg(not(debug_assertions))]
            let text = "Only available in dev profile";

            let p = Paragraph::new(text.italic());
            f.render_widget(p, area);
        }
    }
}
