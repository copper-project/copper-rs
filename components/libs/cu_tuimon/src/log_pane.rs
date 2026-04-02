use crate::MonitorModel;
use crate::palette;
use ratatui::layout::Rect;
use ratatui::style::Style;

#[cfg(debug_assertions)]
mod cfg_debug_assertions {
    pub use cu29::clock::CuTime;
    pub use cu29_log::{CuLogEntry, CuLogLevel};
    pub use cu29_log_runtime::format_message_only;
    pub use cu29_log_runtime::register_live_log_listener;
    pub use ratatui::style::Color;
    pub use std::collections::HashMap;
}
#[cfg(debug_assertions)]
use cfg_debug_assertions::*;

#[cfg(feature = "stderr_capture")]
mod cfg_stderr_capture {
    pub use gag::BufferRedirect;
    pub use std::io::Read;
}
#[cfg(feature = "stderr_capture")]
use cfg_stderr_capture::*;

#[derive(Default)]
pub struct LogPane {
    pub area: Option<Rect>,
    pub lines: Vec<StyledLine>,
    pub selection: LogSelection,
    pub offset_from_bottom: usize,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct LogSelection {
    anchor: Option<SelectionPoint>,
    cursor: Option<SelectionPoint>,
}

impl LogSelection {
    pub fn clear(&mut self) {
        self.anchor = None;
        self.cursor = None;
    }

    pub fn start(&mut self, point: SelectionPoint) {
        self.anchor = Some(point);
        self.cursor = Some(point);
    }

    pub fn update(&mut self, point: SelectionPoint) {
        if self.anchor.is_some() {
            self.cursor = Some(point);
        }
    }

    pub fn range(&self) -> Option<(SelectionPoint, SelectionPoint)> {
        let anchor = self.anchor?;
        let cursor = self.cursor?;
        if (anchor.row, anchor.col) <= (cursor.row, cursor.col) {
            Some((anchor, cursor))
        } else {
            Some((cursor, anchor))
        }
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct SelectionPoint {
    pub row: usize,
    pub col: usize,
}

#[derive(Clone, Debug)]
pub struct StyledRun {
    pub start: usize,
    pub end: usize,
    pub style: Style,
}

#[derive(Clone, Debug)]
pub struct StyledLine {
    pub text: String,
    pub runs: Vec<StyledRun>,
}

#[cfg_attr(
    not(any(debug_assertions, feature = "stderr_capture")),
    allow(dead_code)
)]
#[derive(Clone)]
enum CaptureTarget {
    Model(MonitorModel),
    Stdout,
}

pub struct MonitorLogCapture {
    #[cfg_attr(not(feature = "stderr_capture"), allow(dead_code))]
    target: CaptureTarget,
    #[cfg(debug_assertions)]
    registered_live_listener: bool,
    #[cfg(feature = "stderr_capture")]
    stderr_redirect: Option<BufferRedirect>,
}

impl MonitorLogCapture {
    pub fn to_model(model: MonitorModel) -> Self {
        let target = CaptureTarget::Model(model);
        Self::new(target, true)
    }

    pub fn to_stdout() -> Self {
        Self::new(CaptureTarget::Stdout, false)
    }

    fn new(target: CaptureTarget, _capture_stderr: bool) -> Self {
        #[cfg(debug_assertions)]
        register_live_log_listener({
            let target = target.clone();
            move |entry, format_str, param_names| {
                target.handle_structured_log(entry, format_str, param_names);
            }
        });

        Self {
            target,
            #[cfg(debug_assertions)]
            registered_live_listener: true,
            #[cfg(feature = "stderr_capture")]
            stderr_redirect: if _capture_stderr {
                BufferRedirect::stderr().ok()
            } else {
                None
            },
        }
    }

    pub fn poll(&mut self) {
        #[cfg(feature = "stderr_capture")]
        if let Some(stderr_redirect) = self.stderr_redirect.as_mut() {
            let mut buffer = String::new();
            if let Err(err) = stderr_redirect.read_to_string(&mut buffer) {
                return eprintln!("cu_tuimon log pane: failed to read stderr buffer: {err}");
            }
            if buffer.is_empty() {
                return;
            }

            for line in buffer.lines() {
                let styled_line = styled_line_from_stderr(line);
                match &self.target {
                    CaptureTarget::Model(model) => model.push_log_line(styled_line),
                    CaptureTarget::Stdout => eprintln!("{line}"),
                }
            }
        }
    }
}

impl Drop for MonitorLogCapture {
    fn drop(&mut self) {
        self.poll();
        #[cfg(debug_assertions)]
        if self.registered_live_listener {
            cu29_log_runtime::unregister_live_log_listener();
        }
    }
}

impl CaptureTarget {
    #[cfg(debug_assertions)]
    fn handle_structured_log(&self, entry: &CuLogEntry, format_str: &str, param_names: &[&str]) {
        match self {
            Self::Model(model) => {
                model.push_log_line(styled_line_from_structured(entry, format_str, param_names));
            }
            Self::Stdout => {
                if let Some(line) = format_headless_log_line(entry, format_str, param_names) {
                    println!("{line}");
                }
            }
        }
    }
}

#[cfg(debug_assertions)]
fn format_headless_log_line(
    entry: &CuLogEntry,
    format_str: &str,
    param_names: &[&str],
) -> Option<String> {
    let params: Vec<String> = entry.params.iter().map(|value| value.to_string()).collect();
    let named_params: HashMap<String, String> = param_names
        .iter()
        .zip(params.iter())
        .map(|(name, value)| (name.to_string(), value.clone()))
        .collect();

    format_message_only(format_str, params.as_slice(), &named_params)
        .ok()
        .map(|message| {
            format!(
                "{} [{:?}] {}",
                format_timestamp(entry.time),
                entry.level,
                message
            )
        })
}

#[cfg(debug_assertions)]
fn styled_line_from_structured(
    entry: &CuLogEntry,
    format_str: &str,
    param_names: &[&str],
) -> StyledLine {
    let params: Vec<String> = entry.params.iter().map(|value| value.to_string()).collect();
    let named_params: HashMap<String, String> = param_names
        .iter()
        .zip(params.iter())
        .map(|(name, value)| (name.to_string(), value.clone()))
        .collect();

    let timestamp = format_timestamp(entry.time);
    let level_text = format!("[{:?}]", entry.level);
    let (message_text, param_spans) =
        build_message_with_runs(format_str, params.as_slice(), &named_params);
    let mut message_runs = Vec::new();
    let mut cursor = 0usize;
    let mut sorted_spans = param_spans;
    sorted_spans.sort_by_key(|span| span.0);

    for (start, end) in sorted_spans {
        if start > cursor {
            message_runs.push(StyledRun {
                start: cursor,
                end: start,
                style: Style::default().fg(palette::GRAY),
            });
        }
        message_runs.push(StyledRun {
            start,
            end,
            style: Style::default().fg(palette::MAGENTA),
        });
        cursor = end;
    }

    if cursor < message_text.chars().count() {
        message_runs.push(StyledRun {
            start: cursor,
            end: message_text.chars().count(),
            style: Style::default().fg(palette::GRAY),
        });
    }

    let prefix = format!("{timestamp} {level_text} ");
    let prefix_len = prefix.chars().count();
    let mut runs = vec![
        StyledRun {
            start: 0,
            end: timestamp.chars().count(),
            style: Style::default().fg(palette::BLUE),
        },
        StyledRun {
            start: timestamp.chars().count() + 1,
            end: timestamp.chars().count() + 1 + level_text.chars().count(),
            style: Style::default().fg(color_for_level(entry.level)),
        },
    ];

    for run in message_runs {
        runs.push(StyledRun {
            start: prefix_len + run.start,
            end: prefix_len + run.end,
            style: run.style,
        });
    }

    StyledLine {
        text: format!("{prefix}{message_text}"),
        runs,
    }
}

#[cfg_attr(not(feature = "stderr_capture"), allow(dead_code))]
pub fn styled_line_from_stderr(text: &str) -> StyledLine {
    StyledLine {
        text: text.to_string(),
        runs: vec![StyledRun {
            start: 0,
            end: text.chars().count(),
            style: Style::default().fg(palette::RED),
        }],
    }
}

#[cfg(debug_assertions)]
fn color_for_level(level: CuLogLevel) -> Color {
    match level {
        CuLogLevel::Debug => palette::GREEN,
        CuLogLevel::Info => palette::GRAY,
        CuLogLevel::Warning => palette::YELLOW,
        CuLogLevel::Error | CuLogLevel::Critical => palette::RED,
    }
}

#[cfg(debug_assertions)]
fn format_timestamp(time: CuTime) -> String {
    let nanos = time.as_nanos();
    let total_ms = nanos / 1_000_000;
    let millis = total_ms % 1_000;
    let total_seconds = total_ms / 1_000;
    let seconds = total_seconds % 60;
    let minutes = (total_seconds / 60) % 60;
    let hours = (total_seconds / 3_600) % 24;
    format!("{hours:02}:{minutes:02}:{seconds:02}.{millis:03}")
}

#[cfg(debug_assertions)]
fn build_message_with_runs(
    format_str: &str,
    params: &[String],
    named_params: &HashMap<String, String>,
) -> (String, Vec<(usize, usize)>) {
    let mut output = String::new();
    let mut param_spans = Vec::new();
    let mut anonymous = params.iter();
    let mut iter = format_str.char_indices().peekable();

    while let Some((index, ch)) = iter.next() {
        if ch == '{' {
            let start_idx = index + ch.len_utf8();
            if let Some(end) = format_str[start_idx..].find('}') {
                let end_idx = start_idx + end;
                let placeholder = &format_str[start_idx..end_idx];
                let replacement = if placeholder.is_empty() {
                    anonymous.next()
                } else {
                    named_params.get(placeholder)
                };

                if let Some(replacement) = replacement {
                    let span_start = output.chars().count();
                    output.push_str(replacement);
                    let span_end = output.chars().count();
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

        output.push(ch);
    }

    (output, param_spans)
}
