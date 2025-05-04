use crate::UI;
use ratatui::layout::Rect;
use ratatui::prelude::Stylize;
use ratatui::widgets::{Block, Borders, Paragraph};
use ratatui::Frame;
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

    pub fn get_logs(&mut self) -> String {
        let logs = &self.debug_log;
        logs.concat_compact().to_string()
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
            let mut error_buffer = String::new();
            self.error_redirect
                .read_to_string(&mut error_buffer)
                .unwrap();
            debug_output.push_logs(error_buffer);
            debug_output.update_logs();
        }
    }

    fn draw_debug_output(&mut self, f: &mut Frame, area: Rect) {
        if let Some(debug_output) = self.debug_output.as_mut() {
            let mut error_buffer = String::new();
            self.error_redirect
                .read_to_string(&mut error_buffer)
                .unwrap();
            debug_output.push_logs(error_buffer);

            let debug_log = debug_output.get_logs();

            let p = Paragraph::new(debug_log).block(
                Block::default()
                    .title(" Debug Output ")
                    .title_bottom(format!("{} log entries", debug_output.debug_log.len()))
                    .borders(Borders::ALL),
            );
            f.render_widget(p, area);
        } else {
            #[cfg(debug_assertions)]
            let text = "Text logger is disabled";

            #[cfg(not(debug_assertions))]
            let text = "Only available in dev profile";

            let p = Paragraph::new(text.italic());
            f.render_widget(p, area);
        }
    }
}
