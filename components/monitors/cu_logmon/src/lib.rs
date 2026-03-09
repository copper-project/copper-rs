#![cfg_attr(not(feature = "std"), no_std)]

//! Lightweight Copper monitor that periodically dumps compact runtime stats over the
//! standard `debug!` / `info!` logging macros.
//!
//! This monitor is `no_std` friendly and keeps allocations to a minimum while still
//! reporting a per-second summary of the Copperlist cadence and latencies.

extern crate alloc;

use alloc::format;
use alloc::string::String;
use alloc::vec;
use alloc::vec::Vec;
use core::fmt::Write as _;
use cu29::prelude::*;
#[cfg(all(feature = "std", debug_assertions))]
use cu29_log_runtime::{
    format_message_only, register_live_log_listener, unregister_live_log_listener,
};
use spin::Mutex;
#[cfg(all(feature = "std", debug_assertions))]
use std::collections::HashMap;

const REPORT_INTERVAL_SECS: u64 = 1;
const MAX_LATENCY_SECS: u64 = 5;

#[cfg(all(feature = "std", debug_assertions))]
fn format_timestamp(time: CuTime) -> String {
    // Render CuTime (nanoseconds from an epoch) as HH:mm:ss.xxxx where xxxx is 1e-4 s.
    let nanos = time.as_nanos();
    let total_seconds = nanos / 1_000_000_000;
    let hours = total_seconds / 3600;
    let minutes = (total_seconds / 60) % 60;
    let seconds = total_seconds % 60;
    let fractional_1e4 = (nanos % 1_000_000_000) / 100_000; // 4 fractional digits.
    format!("{hours:02}:{minutes:02}:{seconds:02}.{fractional_1e4:04}")
}

struct WindowState {
    total_copperlists: u64,
    window_copperlists: u32,
    last_report_at: Option<CuTime>,
    last_log_duration: CuDuration,
    end_to_end: CuDurationStatistics,
    per_component: Vec<CuDurationStatistics>,
}

impl WindowState {
    fn new(component_count: usize, max_sample: CuDuration) -> Self {
        #[cfg(target_os = "none")]
        info!("WindowState::new: init end_to_end");
        let end_to_end = CuDurationStatistics::new(max_sample);
        #[cfg(target_os = "none")]
        info!("WindowState::new: init per_component");
        #[cfg(target_os = "none")]
        info!(
            "WindowState::new: stats_size={} per_component_bytes={}",
            core::mem::size_of::<CuDurationStatistics>(),
            core::mem::size_of::<CuDurationStatistics>() * component_count
        );
        let per_component = vec![CuDurationStatistics::new(max_sample); component_count];
        #[cfg(target_os = "none")]
        info!("WindowState::new: init done");
        Self {
            total_copperlists: 0,
            window_copperlists: 0,
            last_report_at: None,
            last_log_duration: CuDuration::MIN,
            end_to_end,
            per_component,
        }
    }

    fn reset_window(&mut self, now: CuTime) {
        self.window_copperlists = 0;
        self.last_report_at = Some(now);
        self.end_to_end.reset();
        for stat in &mut self.per_component {
            stat.reset();
        }
    }
}

fn monitor_max_sample(monitor_cfg: Option<&ComponentConfig>) -> CuResult<CuDuration> {
    if let Some(cfg) = monitor_cfg {
        if let Some(us) = cfg.get::<u64>("max_latency_us")? {
            if us == 0 {
                return Err(CuError::from("cu_logmon max_latency_us must be > 0"));
            }
            return Ok(CuDuration::from_micros(us));
        }
        if let Some(ms) = cfg.get::<u64>("max_latency_ms")? {
            if ms == 0 {
                return Err(CuError::from("cu_logmon max_latency_ms must be > 0"));
            }
            return Ok(CuDuration::from_millis(ms));
        }
        if let Some(secs) = cfg.get::<u64>("max_latency_secs")? {
            if secs == 0 {
                return Err(CuError::from("cu_logmon max_latency_secs must be > 0"));
            }
            return Ok(CuDuration::from_secs(secs));
        }
    }
    Ok(CuDuration::from_secs(MAX_LATENCY_SECS))
}

struct Snapshot {
    copperlist_index: u64,
    rate_whole: u64,
    rate_tenths: u64,
    e2e_p50_us: u64,
    e2e_p90_us: u64,
    e2e_p99_us: u64,
    e2e_max_us: u64,
    top4: String,
    overhead_us: u64,
}

pub struct CuLogMon {
    components: &'static [MonitorComponentMetadata],
    component_count: usize,
    window: Mutex<WindowState>,
}

impl CuLogMon {
    fn component_name(&self, component_id: ComponentId) -> &'static str {
        debug_assert!(component_id.index() < self.component_count);
        self.components[component_id.index()].id()
    }

    fn compute_snapshot(&self, state: &WindowState, now: CuTime) -> Option<Snapshot> {
        let last_report = state.last_report_at?;

        let elapsed = now - last_report;
        let elapsed_ns = elapsed.as_nanos();

        if elapsed_ns < CuDuration::from_secs(REPORT_INTERVAL_SECS).as_nanos() {
            return None;
        }

        let rate_x10 = (state.window_copperlists as u64 * 10 * 1_000_000_000u64)
            .checked_div(elapsed_ns)
            .unwrap_or(0);

        let top4_max_entries = find_top_components_by_max(&state.per_component, 4);
        let mut top4 = String::new();
        if top4_max_entries.is_empty() {
            top4.push_str("none");
        } else {
            for (rank, (component_id, dur)) in top4_max_entries.iter().enumerate() {
                if rank > 0 {
                    top4.push_str(", ");
                }
                let name = self.component_name(*component_id);
                let _ = write!(&mut top4, "{} {}us", name, dur.as_micros());
            }
        }

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
            top4,
            overhead_us: state.last_log_duration.as_micros(),
        })
    }
}

fn component_duration(meta: &CuMsgMetadata) -> Option<CuDuration> {
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

fn find_top_components_by_max(
    per_component: &[CuDurationStatistics],
    limit: usize,
) -> Vec<(ComponentId, CuDuration)> {
    let mut ranked: Vec<(ComponentId, CuDuration)> = per_component
        .iter()
        .enumerate()
        .filter_map(|(idx, stats)| {
            (!stats.is_empty()).then_some((ComponentId::new(idx), stats.max()))
        })
        .collect();
    ranked.sort_unstable_by(|a, b| {
        b.1.as_nanos()
            .cmp(&a.1.as_nanos())
            .then_with(|| a.0.index().cmp(&b.0.index()))
    });
    ranked.truncate(limit);
    ranked
}

fn component_state_label(state: &CuComponentState) -> &'static str {
    match state {
        CuComponentState::Start => "start",
        CuComponentState::Preprocess => "pre",
        CuComponentState::Process => "process",
        CuComponentState::Postprocess => "post",
        CuComponentState::Stop => "stop",
    }
}

impl CuMonitor for CuLogMon {
    fn new(metadata: CuMonitoringMetadata, _runtime: CuMonitoringRuntime) -> CuResult<Self> {
        let components = metadata.components();
        let component_count = components.len();
        #[cfg(target_os = "none")]
        info!("CuLogMon::new: component_count={}", component_count);
        let max_sample = monitor_max_sample(metadata.monitor_config())?;
        let window = WindowState::new(component_count, max_sample);
        #[cfg(target_os = "none")]
        info!("CuLogMon::new: window ready");
        Ok(Self {
            components,
            component_count,
            window: Mutex::new(window),
        })
    }

    fn start(&mut self, ctx: &CuContext) -> CuResult<()> {
        let mut window = self.window.lock();
        window.last_report_at = Some(ctx.recent());
        info!("cu_logmon started ({} components)", self.component_count);

        // Also listen to structured logs and print them with color.
        #[cfg(all(feature = "std", debug_assertions))]
        register_live_log_listener(|entry, format_str, param_names| {
            const PARAM_COLOR: &str = "\x1b[36m"; // cyan
            const RESET: &str = "\x1b[0m";

            let params: Vec<String> = entry.params.iter().map(|v| v.to_string()).collect();
            let colored_params: Vec<String> = params
                .iter()
                .map(|v| format!("{PARAM_COLOR}{v}{RESET}"))
                .collect();
            let colored_named: HashMap<String, String> = param_names
                .iter()
                .zip(params.iter())
                .map(|(k, v)| (k.to_string(), format!("{PARAM_COLOR}{v}{RESET}")))
                .collect();

            if let Ok(msg) =
                format_message_only(format_str, colored_params.as_slice(), &colored_named)
            {
                let level_color = match entry.level {
                    CuLogLevel::Debug => "\x1b[32m",   // green
                    CuLogLevel::Info => "\x1b[90m",    // gray
                    CuLogLevel::Warning => "\x1b[93m", // yellow
                    CuLogLevel::Error => "\x1b[91m",   // red
                    CuLogLevel::Critical => "\x1b[91m",
                };
                let ts_color = "\x1b[34m";
                let ts = format_timestamp(entry.time);
                println!(
                    "{ts_color}{ts}{reset} {level_color}[{:?}]{reset} {msg}",
                    entry.level,
                    ts = ts,
                    ts_color = ts_color,
                    level_color = level_color,
                    reset = RESET,
                    msg = msg
                );
            }
        });
        Ok(())
    }

    fn process_copperlist(&self, ctx: &CuContext, view: CopperListView<'_>) -> CuResult<()> {
        let call_start = ctx.recent();

        let snapshot = {
            let mut window = self.window.lock();
            window.last_report_at.get_or_insert(call_start);

            window.total_copperlists = window.total_copperlists.saturating_add(1);
            window.window_copperlists = window.window_copperlists.saturating_add(1);

            if let Some(latency) = end_to_end_latency(view.msgs()) {
                window.end_to_end.record(latency);
            }

            for entry in view.entries() {
                let component_index = entry.component_id.index();
                if let Some(component_stat) = window.per_component.get_mut(component_index)
                    && let Some(duration) = component_duration(entry.msg)
                {
                    component_stat.record(duration);
                } else {
                    debug_assert!(
                        component_index < window.per_component.len(),
                        "cu_logmon: component index {} out of bounds {}",
                        component_index,
                        window.per_component.len()
                    );
                }
            }

            let snapshot = self.compute_snapshot(&window, call_start);
            if snapshot.is_some() {
                window.reset_window(call_start);
            }
            snapshot
        };

        if let Some(snapshot) = snapshot {
            let log_start = ctx.recent();
            let use_color = cfg!(feature = "color_log");
            let base = format!(
                "[CL {}] rate {}.{} Hz | top4 {} | e2e p50 {}us p90 {}us p99 {}us max {}us | overhead {}us",
                snapshot.copperlist_index,
                snapshot.rate_whole,
                snapshot.rate_tenths,
                snapshot.top4,
                snapshot.e2e_p50_us,
                snapshot.e2e_p90_us,
                snapshot.e2e_p99_us,
                snapshot.e2e_max_us,
                snapshot.overhead_us,
            );
            if use_color {
                // Colored labels for readability (values stay uncolored).
                const CL_COLOR: &str = "\x1b[94m"; // blue
                const LABEL_COLOR: &str = "\x1b[92m"; // green for main labels
                const SUBLABEL_COLOR: &str = "\x1b[93m"; // yellow for sublabels
                const COMPONENT_NAME_COLOR: &str = "\x1b[38;5;208m"; // orange for component name
                const RESET: &str = "\x1b[0m";
                let colored = format!(
                    "[{cl_color}CL {cl}{reset}] {label}rate{reset} {rate_whole}.{rate_tenths} Hz | {label}top4{reset} {component_color}{top4}{reset} | {label}e2e{reset} {sublabel}p50{reset} {p50}us {sublabel}p90{reset} {p90}us {sublabel}p99{reset} {p99}us {sublabel}max{reset} {max}us | {label}overhead{reset} {overhead}us",
                    cl_color = CL_COLOR,
                    label = LABEL_COLOR,
                    sublabel = SUBLABEL_COLOR,
                    component_color = COMPONENT_NAME_COLOR,
                    reset = RESET,
                    cl = snapshot.copperlist_index,
                    rate_whole = snapshot.rate_whole,
                    rate_tenths = snapshot.rate_tenths,
                    p50 = snapshot.e2e_p50_us,
                    p90 = snapshot.e2e_p90_us,
                    p99 = snapshot.e2e_p99_us,
                    max = snapshot.e2e_max_us,
                    top4 = snapshot.top4,
                    overhead = snapshot.overhead_us,
                );
                info!("{}", &colored);
            } else {
                info!("{}", &base);
            }
            let log_end = ctx.recent();
            self.window.lock().last_log_duration = log_end - log_start;
        }

        Ok(())
    }

    fn process_error(
        &self,
        component_id: ComponentId,
        step: CuComponentState,
        error: &CuError,
    ) -> Decision {
        let component_name = self.component_name(component_id);
        error!(
            "Component {} @ {}: Error: {}.",
            component_name,
            component_state_label(&step),
            error,
        );
        Decision::Ignore
    }

    fn stop(&mut self, _ctx: &CuContext) -> CuResult<()> {
        #[cfg(all(feature = "std", debug_assertions))]
        unregister_live_log_listener();
        Ok(())
    }
}
