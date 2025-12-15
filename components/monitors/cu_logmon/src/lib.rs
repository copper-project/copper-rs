#![cfg_attr(not(feature = "std"), no_std)]

//! Lightweight Copper monitor that periodically dumps compact runtime stats over the
//! standard `debug!` / `info!` logging macros.
//!
//! This monitor is `no_std` friendly and keeps allocations to a minimum while still
//! reporting a per-second summary of the Copperlist cadence and latencies.

extern crate alloc;

use alloc::vec;
use alloc::vec::Vec;
use cu29::prelude::*;
use spin::Mutex;

const REPORT_INTERVAL_SECS: u64 = 1;
const MAX_LATENCY_SECS: u64 = 5;

struct WindowState {
    total_copperlists: u64,
    window_copperlists: u32,
    last_report_at: Option<CuTime>,
    last_log_duration: CuDuration,
    end_to_end: CuDurationStatistics,
    per_task: Vec<CuDurationStatistics>,
}

impl WindowState {
    fn new(task_count: usize) -> Self {
        let max_sample = CuDuration::from_secs(MAX_LATENCY_SECS);
        Self {
            total_copperlists: 0,
            window_copperlists: 0,
            last_report_at: None,
            last_log_duration: CuDuration::MIN,
            end_to_end: CuDurationStatistics::new(max_sample),
            per_task: vec![CuDurationStatistics::new(max_sample); task_count],
        }
    }

    fn reset_window(&mut self, now: CuTime) {
        self.window_copperlists = 0;
        self.last_report_at = Some(now);
        self.end_to_end.reset();
        for stat in &mut self.per_task {
            stat.reset();
        }
    }
}

struct Snapshot<'a> {
    copperlist_index: u64,
    rate_whole: u64,
    rate_tenths: u64,
    e2e_p50_us: u64,
    e2e_p90_us: u64,
    e2e_p99_us: u64,
    e2e_max_us: u64,
    slowest_task: &'a str,
    slowest_task_p99_us: u64,
    log_overhead_us: u64,
}

pub struct CuLogMon {
    taskids: &'static [&'static str],
    clock: Mutex<Option<RobotClock>>,
    window: Mutex<WindowState>,
}

impl CuLogMon {
    fn compute_snapshot<'a>(&'a self, state: &WindowState, now: CuTime) -> Option<Snapshot<'a>> {
        let last_report = state.last_report_at?;

        let elapsed = now - last_report;
        let elapsed_ns = elapsed.as_nanos();

        if elapsed_ns < CuDuration::from_secs(REPORT_INTERVAL_SECS).as_nanos() {
            return None;
        }

        let rate_x10 = if elapsed_ns > 0 {
            (state.window_copperlists as u64 * 10 * 1_000_000_000u64) / elapsed_ns
        } else {
            0
        };

        let slowest = find_slowest_task(&state.per_task);
        let (slowest_task, slowest_task_p99_us) = slowest
            .map(|(idx, dur)| {
                let name = self.taskids.get(idx).copied().unwrap_or("<?>");
                (name, dur.as_micros())
            })
            .unwrap_or(("none", 0));

        let e2e_p50 = state.end_to_end.percentile(0.5).as_micros();
        let e2e_p90 = state.end_to_end.percentile(0.9).as_micros();
        let e2e_p99 = state.end_to_end.percentile(0.99).as_micros();
        // Max can skew low if a bucket underflows; keep it at least as high as p99.
        let e2e_max = state.end_to_end.max().as_micros().max(e2e_p99);

        Some(Snapshot {
            copperlist_index: state.total_copperlists,
            rate_whole: rate_x10 / 10,
            rate_tenths: rate_x10 % 10,
            e2e_p50_us: e2e_p50,
            e2e_p90_us: e2e_p90,
            e2e_p99_us: e2e_p99,
            e2e_max_us: e2e_max,
            slowest_task,
            slowest_task_p99_us,
            log_overhead_us: state.last_log_duration.as_micros(),
        })
    }
}

fn task_duration(meta: &CuMsgMetadata) -> Option<CuDuration> {
    let start = Option::<CuTime>::from(meta.process_time.start)?;
    let end = Option::<CuTime>::from(meta.process_time.end)?;
    (end >= start).then_some(end - start)
}

fn end_to_end_latency(msgs: &[&CuMsgMetadata]) -> Option<CuDuration> {
    let start = msgs
        .first()
        .and_then(|m| Option::<CuTime>::from(m.process_time.start));
    let end = msgs
        .last()
        .and_then(|m| Option::<CuTime>::from(m.process_time.end));
    match (start, end) {
        (Some(s), Some(e)) if e >= s => Some(e - s),
        _ => None,
    }
}

fn find_slowest_task(per_task: &[CuDurationStatistics]) -> Option<(usize, CuDuration)> {
    per_task
        .iter()
        .enumerate()
        .filter_map(|(idx, stats)| {
            if stats.is_empty() {
                None
            } else {
                Some((idx, stats.percentile(0.99)))
            }
        })
        .max_by_key(|(_, dur)| dur.as_nanos())
}

fn task_state_label(state: &CuTaskState) -> &'static str {
    match state {
        CuTaskState::Start => "start",
        CuTaskState::Preprocess => "pre",
        CuTaskState::Process => "process",
        CuTaskState::Postprocess => "post",
        CuTaskState::Stop => "stop",
    }
}

impl CuMonitor for CuLogMon {
    fn new(_config: &CuConfig, taskids: &'static [&'static str]) -> CuResult<Self> {
        Ok(Self {
            taskids,
            clock: Mutex::new(None),
            window: Mutex::new(WindowState::new(taskids.len())),
        })
    }

    fn start(&mut self, clock: &RobotClock) -> CuResult<()> {
        *self.clock.lock() = Some(clock.clone());
        let mut window = self.window.lock();
        window.last_report_at = Some(clock.recent());
        info!("cu_logmon started ({} tasks)", self.taskids.len());
        Ok(())
    }

    fn process_copperlist(&self, msgs: &[&CuMsgMetadata]) -> CuResult<()> {
        let Some(clock) = self.clock.lock().clone() else {
            return Ok(());
        };

        let call_start = clock.recent();

        let snapshot = {
            let mut window = self.window.lock();
            window.last_report_at.get_or_insert(call_start);

            window.total_copperlists = window.total_copperlists.saturating_add(1);
            window.window_copperlists = window.window_copperlists.saturating_add(1);

            if let Some(latency) = end_to_end_latency(msgs) {
                window.end_to_end.record(latency);
            }

            for (idx, meta) in msgs.iter().enumerate() {
                if let Some(task_stat) = window.per_task.get_mut(idx)
                    && let Some(duration) = task_duration(meta)
                {
                    task_stat.record(duration);
                }
            }

            let snapshot = self.compute_snapshot(&window, call_start);
            if snapshot.is_some() {
                window.reset_window(call_start);
            }
            snapshot
        };

        if let Some(snapshot) = snapshot {
            let log_start = clock.recent();
            let use_color = cfg!(all(feature = "std", feature = "color_log"));
            let base = format!(
                "[CL {}] rate {}.{} Hz | slowest {} {}us | e2e p50 {}us p90 {}us p99 {}us max {}us | log_overhead {}us",
                snapshot.copperlist_index,
                snapshot.rate_whole,
                snapshot.rate_tenths,
                snapshot.slowest_task,
                snapshot.slowest_task_p99_us,
                snapshot.e2e_p50_us,
                snapshot.e2e_p90_us,
                snapshot.e2e_p99_us,
                snapshot.e2e_max_us,
                snapshot.log_overhead_us,
            );
            if use_color {
                // Colored labels for readability (values stay uncolored).
                const CL_COLOR: &str = "\x1b[94m"; // blue
                const LABEL_COLOR: &str = "\x1b[92m"; // green for main labels
                const SUBLABEL_COLOR: &str = "\x1b[93m"; // yellow for sublabels
                const TASK_NAME_COLOR: &str = "\x1b[38;5;208m"; // orange for task name
                const RESET: &str = "\x1b[0m";
                let colored = format!(
                    "[{cl_color}CL {cl}{reset}] {label}rate{reset} {rate_whole}.{rate_tenths} Hz | {label}slowest{reset} {task_color}{slow_task}{reset} {slow_p99}us | {label}e2e{reset} {sublabel}p50{reset} {p50}us {sublabel}p90{reset} {p90}us {sublabel}p99{reset} {p99}us {sublabel}max{reset} {max}us | {label}log_overhead{reset} {log_overhead}us",
                    cl_color = CL_COLOR,
                    label = LABEL_COLOR,
                    sublabel = SUBLABEL_COLOR,
                    task_color = TASK_NAME_COLOR,
                    reset = RESET,
                    cl = snapshot.copperlist_index,
                    rate_whole = snapshot.rate_whole,
                    rate_tenths = snapshot.rate_tenths,
                    slow_task = snapshot.slowest_task,
                    slow_p99 = snapshot.slowest_task_p99_us,
                    p50 = snapshot.e2e_p50_us,
                    p90 = snapshot.e2e_p90_us,
                    p99 = snapshot.e2e_p99_us,
                    max = snapshot.e2e_max_us,
                    log_overhead = snapshot.log_overhead_us,
                );
                info!("{}", &colored);
            } else {
                info!("{}", &base);
            }
            let log_end = clock.recent();
            self.window.lock().last_log_duration = log_end - log_start;
        }

        Ok(())
    }

    fn process_error(&self, taskid: usize, step: CuTaskState, _error: &CuError) -> Decision {
        let task_name = self.taskids.get(taskid).copied().unwrap_or("<??>");
        info!(
            "Task error {} during {}",
            task_name,
            task_state_label(&step)
        );
        Decision::Ignore
    }
}
