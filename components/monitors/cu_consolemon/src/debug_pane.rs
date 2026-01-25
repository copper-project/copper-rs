use crate::UI;
use ratatui::Frame;
use ratatui::layout::Rect;
use ratatui::prelude::Stylize;
use ratatui::widgets::{Block, BorderType, Borders, Paragraph};
use std::sync::atomic::Ordering;
use std::sync::mpsc::SendError;
use {
    compact_str::CompactStringExt,
    log::{Level, LevelFilter, Log, Metadata, Record},
    std::collections::VecDeque,
    std::io::Read,
    std::sync::atomic::AtomicU16,
    std::sync::mpsc::{Receiver, SyncSender},
};

#[derive(Debug)]
pub struct DebugLog {
    debug_log: VecDeque<String>,
    pub(crate) max_rows: AtomicU16,
    rx: Receiver<String>,
}

impl DebugLog {
    #[allow(dead_code)]
    pub fn new(max_lines: u16) -> (Self, SyncSender<String>) {
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

    pub fn push_logs(&mut self, logs: String) {
        if logs.is_empty() {
            return;
        }

        self.debug_log.push_back(logs);
        let max_row = self.max_rows.load(Ordering::SeqCst) as usize;
        while self.debug_log.len() > max_row {
            self.debug_log.pop_front();
        }
    }

    pub fn update_logs(&mut self) {
        let max_row = self.max_rows.load(Ordering::SeqCst) as usize;

        for log in self.rx.try_iter() {
            if log.is_empty() {
                continue;
            }

            self.debug_log.push_back(log);
            if self.debug_log.len() > max_row {
                self.debug_log.pop_front();
            }
        }
    }

    #[allow(dead_code)]
    pub fn get_logs(&mut self) -> String {
        let logs = &self.debug_log;
        logs.concat_compact().to_string()
    }

    pub fn wrapped_lines(&self, width: usize) -> Vec<String> {
        if width == 0 {
            return Vec::new();
        }

        let mut wrapped = Vec::new();
        let mut pending_line = String::new();
        for entry in &self.debug_log {
            for ch in entry.chars() {
                match ch {
                    '\n' => {
                        push_wrapped_line(&mut wrapped, &pending_line, width);
                        pending_line.clear();
                    }
                    '\r' => {}
                    _ => pending_line.push(ch),
                }
            }
        }

        if !pending_line.is_empty() {
            push_wrapped_line(&mut wrapped, &pending_line, width);
        }

        wrapped
    }
}

fn push_wrapped_line(output: &mut Vec<String>, line: &str, width: usize) {
    if line.is_empty() {
        output.push(String::new());
        return;
    }

    let mut chunk = String::new();
    let mut count = 0;
    for ch in line.chars() {
        chunk.push(ch);
        count += 1;
        if count == width {
            output.push(chunk);
            chunk = String::new();
            count = 0;
        }
    }
    if !chunk.is_empty() {
        output.push(chunk);
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
                debug_output.push_logs(error_buffer);
            }
            debug_output.update_logs();
        }
    }

    fn draw_debug_output(&mut self, f: &mut Frame, area: Rect) {
        if let Some(debug_output) = self.debug_output.as_mut() {
            if let Some(error_redirect) = self.error_redirect.as_mut() {
                let mut error_buffer = String::new();
                if let Err(err) = error_redirect.read_to_string(&mut error_buffer) {
                    eprintln!("Failed to read stderr buffer for debug pane: {err}");
                }
                debug_output.push_logs(error_buffer);
            }

            let block = Block::default()
                .title(" Debug Output ")
                .title_bottom(format!("{} log entries", debug_output.debug_log.len()))
                .borders(Borders::ALL)
                .border_type(BorderType::Rounded);
            let inner = block.inner(area);
            self.debug_output_area = Some(inner);
            self.debug_output_lines = debug_output.wrapped_lines(inner.width as usize);
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
