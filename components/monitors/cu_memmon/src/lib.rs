#![cfg_attr(not(feature = "std"), no_std)]

//! Per-task heap-allocation monitor for Copper.
//!
//! `CuMemMon` consumes the [`CuMonitor::observe_alloc`] hook the runtime emits
//! around every task and bridge lifecycle step and reports:
//! - per-step allocation deltas (running totals + per-call peak),
//! - per-task lifetime balance (`alloc - dealloc` since the task's `start`),
//! - a throttled `[mem]` one-liner from `process_copperlist`,
//! - a final per-task **leak report** at runtime shutdown.
//!
//! Optional realtime-strict mode (`realtime_strict: true` in the monitor's
//! RON config) latches any non-zero allocation observed during `Preprocess`,
//! `Process`, or `Postprocess` and surfaces it from the *next*
//! `process_copperlist` as a `CuError`. The runtime treats that as a fatal
//! monitor error and shuts down cleanly — so a violation in
//! e.g. `process` reliably aborts the run regardless of whether any other
//! component errored, and unrelated task errors are not affected.
//!
//! When the runtime is **not** built with `cu29/memory_monitoring`, the macro
//! emits no allocation scopes, so `observe_alloc` is never called.
//! `CuMemMon` detects this at `start()` by probing the public counter API
//! and emits a single `warning!` so the build configuration is obvious from
//! the log.

extern crate alloc;

use alloc::vec::Vec;
use core::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use cu29::__private::sync::Mutex;
use cu29::prelude::*;

#[cfg(all(feature = "std", debug_assertions))]
use alloc::string::String;
#[cfg(all(feature = "std", debug_assertions))]
use alloc::string::ToString;
#[cfg(all(feature = "std", debug_assertions))]
use std::collections::HashMap as StdHashMap;

/// Number of copperlists between throttled `[mem]` summary lines.
/// Picked to be roughly once per second at 100 Hz; users that want quieter
/// runs can disable via `summary_every: 0` in the monitor config.
const DEFAULT_SUMMARY_EVERY: u64 = 100;

/// One slot per (component, step). We collapse all five steps into separate
/// counters per component for compact storage and O(1) updates.
#[derive(Default, Clone, Copy)]
struct StepCounters {
    allocated: u64,
    deallocated: u64,
    calls: u64,
    /// Largest single-call allocation seen for this step.
    max_allocated: u64,
}

#[derive(Clone, Copy)]
struct ComponentCounters {
    start: StepCounters,
    preprocess: StepCounters,
    process: StepCounters,
    postprocess: StepCounters,
    stop: StepCounters,
}

impl ComponentCounters {
    const fn new() -> Self {
        const Z: StepCounters = StepCounters {
            allocated: 0,
            deallocated: 0,
            calls: 0,
            max_allocated: 0,
        };
        Self {
            start: Z,
            preprocess: Z,
            process: Z,
            postprocess: Z,
            stop: Z,
        }
    }

    fn slot(&mut self, step: CuComponentState) -> &mut StepCounters {
        match step {
            CuComponentState::Start => &mut self.start,
            CuComponentState::Preprocess => &mut self.preprocess,
            CuComponentState::Process => &mut self.process,
            CuComponentState::Postprocess => &mut self.postprocess,
            CuComponentState::Stop => &mut self.stop,
        }
    }

    fn lifetime_alloc(&self) -> u64 {
        self.start.allocated
            + self.preprocess.allocated
            + self.process.allocated
            + self.postprocess.allocated
            + self.stop.allocated
    }

    fn lifetime_dealloc(&self) -> u64 {
        self.start.deallocated
            + self.preprocess.deallocated
            + self.process.deallocated
            + self.postprocess.deallocated
            + self.stop.deallocated
    }
}

#[derive(Clone, Copy)]
struct RtViolation {
    step: CuComponentState,
    allocated: u64,
    deallocated: u64,
}

struct State {
    per_component: Vec<ComponentCounters>,
    /// True iff the runtime was compiled with `memory_monitoring`.
    /// Probed in `new()` by reading the public counters from the runtime.
    counters_available: bool,
    /// Pending realtime-strict violations to surface from the next
    /// `process_copperlist`. One slot per component; latest wins.
    rt_pending: Vec<Option<RtViolation>>,
}

pub struct CuMemMon {
    components: &'static [MonitorComponentMetadata],
    component_count: usize,
    realtime_strict: bool,
    summary_every: u64,
    /// Monotonic copperlist counter, drives the throttled summary.
    cl_counter: AtomicU64,
    state: Mutex<State>,
    /// True once we've emitted the "feature off" warning, so we don't spam.
    feature_off_warned: AtomicBool,
    /// Total observed allocations / deallocations across the whole runtime
    /// (cheap monotonic counters for the periodic summary header).
    total_alloc: AtomicU64,
    total_dealloc: AtomicU64,
}

fn lock_state(state: &Mutex<State>) -> StateGuard<'_> {
    #[cfg(feature = "std")]
    {
        state.lock().unwrap_or_else(|poison| poison.into_inner())
    }
    #[cfg(not(feature = "std"))]
    {
        state.lock()
    }
}

#[cfg(feature = "std")]
type StateGuard<'a> = std::sync::MutexGuard<'a, State>;
#[cfg(not(feature = "std"))]
type StateGuard<'a> = spin::MutexGuard<'a, State, spin::relax::Spin>;

fn parse_realtime_strict(cfg: Option<&ComponentConfig>) -> CuResult<bool> {
    let Some(cfg) = cfg else { return Ok(false) };
    if let Some(strict) = cfg.get::<bool>("realtime_strict")? {
        return Ok(strict);
    }
    Ok(false)
}

fn parse_summary_every(cfg: Option<&ComponentConfig>) -> CuResult<u64> {
    let Some(cfg) = cfg else {
        return Ok(DEFAULT_SUMMARY_EVERY);
    };
    if let Some(value) = cfg.get::<u32>("summary_every")? {
        return Ok(value as u64);
    }
    Ok(DEFAULT_SUMMARY_EVERY)
}

fn step_label(step: CuComponentState) -> &'static str {
    match step {
        CuComponentState::Start => "start",
        CuComponentState::Preprocess => "pre",
        CuComponentState::Process => "process",
        CuComponentState::Postprocess => "post",
        CuComponentState::Stop => "stop",
    }
}

#[inline]
fn is_realtime_step(step: CuComponentState) -> bool {
    matches!(
        step,
        CuComponentState::Preprocess | CuComponentState::Process | CuComponentState::Postprocess
    )
}

impl CuMemMon {
    fn component_name(&self, component_id: ComponentId) -> &'static str {
        debug_assert!(component_id.index() < self.component_count);
        self.components[component_id.index()].id()
    }
}

impl CuMonitor for CuMemMon {
    fn new(metadata: CuMonitoringMetadata, _runtime: CuMonitoringRuntime) -> CuResult<Self> {
        let components = metadata.components();
        let component_count = components.len();
        let realtime_strict = parse_realtime_strict(metadata.monitor_config())?;
        let summary_every = parse_summary_every(metadata.monitor_config())?;

        // Probe the runtime API: when `memory_monitoring` is off, the macro
        // never emits ScopedAllocCounter, so this hook would otherwise never
        // fire — `start()` uses this to print a single warning.
        let counters_available = cu29::monitoring::global_allocated_bytes().is_some();

        let state = State {
            per_component: alloc::vec![ComponentCounters::new(); component_count],
            counters_available,
            rt_pending: alloc::vec![None; component_count],
        };

        Ok(Self {
            components,
            component_count,
            realtime_strict,
            summary_every,
            cl_counter: AtomicU64::new(0),
            state: Mutex::new(state),
            feature_off_warned: AtomicBool::new(false),
            total_alloc: AtomicU64::new(0),
            total_dealloc: AtomicU64::new(0),
        })
    }

    fn start(&mut self, ctx: &CuContext) -> CuResult<()> {
        // Mirror NoMonitor: tee every live structured log entry to stdout in
        // std debug builds so apps that pick cu_memmon as their (only)
        // monitor still see info!/warning!/error! on the console.
        register_console_listener();

        let state = lock_state(&self.state);
        if !state.counters_available {
            warning!(
                ctx,
                "cu_memmon enabled but cu29 was built without `memory_monitoring`. \
                 No allocation scopes emitted; all reports will be zero."
            );
            self.feature_off_warned.store(true, Ordering::Relaxed);
        } else {
            info!(
                ctx,
                "cu_memmon started: {} components, realtime_strict={}, summary_every={}",
                self.component_count,
                self.realtime_strict,
                self.summary_every,
            );
        }
        Ok(())
    }

    fn process_copperlist(&self, ctx: &CuContext, _view: CopperListView<'_>) -> CuResult<()> {
        // 1) Surface realtime-strict violations: drain pending and return a
        //    CuError. The runtime will treat this as fatal and shut down.
        if self.realtime_strict {
            let drained = {
                let mut state = lock_state(&self.state);
                drain_rt_violations(&mut state.rt_pending)
            };
            if !drained.is_empty() {
                // Format only the first violation; multiples in the same
                // copperlist are rare and a single trip-wire suffices.
                let (component_id, viol) = drained[0];
                let name = self.component_name(component_id);
                let msg = format_rt_violation_message(name, viol);
                error!(ctx, "{}", msg.as_str());
                return Err(CuError::from(msg.as_str()));
            }
        }

        // 2) Throttled summary. `summary_every == 0` disables the summary.
        if self.summary_every == 0 {
            return Ok(());
        }
        let n = self.cl_counter.fetch_add(1, Ordering::Relaxed) + 1;
        if n % self.summary_every != 0 {
            return Ok(());
        }
        // Snapshot under the lock; format without holding it.
        let snapshot: Vec<(&'static str, ComponentCounters)> = {
            let state = lock_state(&self.state);
            if !state.counters_available {
                return Ok(());
            }
            self.components
                .iter()
                .zip(state.per_component.iter())
                .map(|(meta, counters)| (meta.id(), *counters))
                .collect()
        };
        for (name, c) in snapshot.iter() {
            let leak = c.lifetime_alloc().saturating_sub(c.lifetime_dealloc());
            info!(
                ctx,
                "[mem] {} start=+{}B -{}B | proc max=+{}B over {} calls | stop=+{}B -{}B | leak={}B",
                *name,
                c.start.allocated,
                c.start.deallocated,
                c.process.max_allocated,
                c.process.calls,
                c.stop.allocated,
                c.stop.deallocated,
                leak,
            );
        }
        Ok(())
    }

    fn observe_alloc(
        &self,
        component_id: ComponentId,
        step: CuComponentState,
        allocated_bytes: usize,
        deallocated_bytes: usize,
    ) {
        let alloc_u64 = allocated_bytes as u64;
        let dealloc_u64 = deallocated_bytes as u64;
        // Aggregate counters first — relaxed atomics suffice for an
        // eventually-correct periodic summary header.
        self.total_alloc.fetch_add(alloc_u64, Ordering::Relaxed);
        self.total_dealloc.fetch_add(dealloc_u64, Ordering::Relaxed);

        let mut state = lock_state(&self.state);
        let idx = component_id.index();
        debug_assert!(
            idx < state.per_component.len(),
            "cu_memmon: component id {idx} out of bounds {}",
            state.per_component.len()
        );
        let Some(component) = state.per_component.get_mut(idx) else {
            return;
        };
        let slot = component.slot(step);
        slot.allocated = slot.allocated.saturating_add(alloc_u64);
        slot.deallocated = slot.deallocated.saturating_add(dealloc_u64);
        slot.calls = slot.calls.saturating_add(1);
        if alloc_u64 > slot.max_allocated {
            slot.max_allocated = alloc_u64;
        }

        // Latch realtime violations: any non-zero alloc on a hot-path step
        // becomes a pending error that the next process_copperlist surfaces.
        if self.realtime_strict && is_realtime_step(step) && allocated_bytes > 0 {
            state.rt_pending[idx] = Some(RtViolation {
                step,
                allocated: alloc_u64,
                deallocated: dealloc_u64,
            });
        }
    }

    fn process_error(
        &self,
        _component_id: ComponentId,
        _step: CuComponentState,
        _error: &CuError,
    ) -> Decision {
        // cu_memmon is a passive observer for non-memory errors: hand the
        // runtime's default policy back. Memory violations are escalated via
        // process_copperlist returning Err, not through here.
        Decision::Ignore
    }

    fn stop(&mut self, ctx: &CuContext) -> CuResult<()> {
        let state = lock_state(&self.state);
        if !state.counters_available {
            unregister_console_listener();
            return Ok(());
        }
        info!(
            ctx,
            "cu_memmon final report: total +{}B / -{}B",
            self.total_alloc.load(Ordering::Relaxed),
            self.total_dealloc.load(Ordering::Relaxed),
        );
        for (idx, counters) in state.per_component.iter().enumerate() {
            let name = self.components[idx].id();
            let lifetime_alloc = counters.lifetime_alloc();
            let lifetime_dealloc = counters.lifetime_dealloc();
            let process_alloc = counters.process.allocated;
            let process_calls = counters.process.calls;
            if lifetime_alloc > lifetime_dealloc {
                let leak = lifetime_alloc - lifetime_dealloc;
                warning!(
                    ctx,
                    "cu_memmon: task {} leak={}B (alloc {}B, dealloc {}B); process allocated {}B over {} calls",
                    name,
                    leak,
                    lifetime_alloc,
                    lifetime_dealloc,
                    process_alloc,
                    process_calls,
                );
            } else if lifetime_alloc < lifetime_dealloc {
                // Symmetric anomaly: the task freed more than it allocated
                // within its own scopes. Usually means it returned memory
                // originally allocated outside its observed lifecycle.
                let excess = lifetime_dealloc - lifetime_alloc;
                warning!(
                    ctx,
                    "cu_memmon: task {} dealloc excess={}B (alloc {}B, dealloc {}B); process allocated {}B over {} calls — task may free memory it didn't allocate within its own steps",
                    name,
                    excess,
                    lifetime_alloc,
                    lifetime_dealloc,
                    process_alloc,
                    process_calls,
                );
            } else {
                info!(
                    ctx,
                    "cu_memmon: task {} balanced (alloc {}B, dealloc {}B); process allocated {}B over {} calls",
                    name,
                    lifetime_alloc,
                    lifetime_dealloc,
                    process_alloc,
                    process_calls,
                );
            }
        }
        drop(state);
        unregister_console_listener();
        Ok(())
    }
}

fn drain_rt_violations(
    pending: &mut [Option<RtViolation>],
) -> Vec<(ComponentId, RtViolation)> {
    let mut out = Vec::new();
    for (idx, slot) in pending.iter_mut().enumerate() {
        if let Some(v) = slot.take() {
            out.push((ComponentId::new(idx), v));
        }
    }
    out
}

#[cfg(feature = "std")]
fn format_rt_violation_message(name: &str, viol: RtViolation) -> String {
    alloc::format!(
        "cu_memmon: realtime violation: component {} allocated {} B (deallocated {} B) during {}",
        name,
        viol.allocated,
        viol.deallocated,
        step_label(viol.step),
    )
}

#[cfg(not(feature = "std"))]
fn format_rt_violation_message(name: &str, viol: RtViolation) -> &'static str {
    // In no_std builds we don't allocate the message; emit a fixed sentinel
    // so the user still sees *that* a violation happened. The kind/component
    // is already reported via error!(...) immediately before this is sent.
    let _ = (name, viol);
    "cu_memmon: realtime allocation violation"
}

#[cfg(all(feature = "std", debug_assertions))]
fn register_console_listener() {
    use std::println;
    register_live_log_listener(|entry, format_str, param_names| {
        let params: Vec<String> = entry.params.iter().map(|v| v.to_string()).collect();
        let named: StdHashMap<String, String> = param_names
            .iter()
            .zip(params.iter())
            .map(|(k, v)| (k.to_string(), v.clone()))
            .collect();

        if let Ok(msg) = format_message_only(format_str, params.as_slice(), &named) {
            println!("[{:?}] {}", entry.level, msg);
        }
    });
}

#[cfg(not(all(feature = "std", debug_assertions)))]
fn register_console_listener() {}

#[cfg(all(feature = "std", debug_assertions))]
fn unregister_console_listener() {
    unregister_live_log_listener();
}

#[cfg(not(all(feature = "std", debug_assertions)))]
fn unregister_console_listener() {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn step_label_covers_all_states() {
        assert_eq!(step_label(CuComponentState::Start), "start");
        assert_eq!(step_label(CuComponentState::Preprocess), "pre");
        assert_eq!(step_label(CuComponentState::Process), "process");
        assert_eq!(step_label(CuComponentState::Postprocess), "post");
        assert_eq!(step_label(CuComponentState::Stop), "stop");
    }

    #[test]
    fn is_realtime_step_only_flags_hot_path_states() {
        assert!(!is_realtime_step(CuComponentState::Start));
        assert!(is_realtime_step(CuComponentState::Preprocess));
        assert!(is_realtime_step(CuComponentState::Process));
        assert!(is_realtime_step(CuComponentState::Postprocess));
        assert!(!is_realtime_step(CuComponentState::Stop));
    }

    #[test]
    fn component_counters_track_per_step_and_lifetime_totals() {
        let mut c = ComponentCounters::new();
        c.slot(CuComponentState::Start).allocated += 1024;
        c.slot(CuComponentState::Process).allocated += 256;
        c.slot(CuComponentState::Process).deallocated += 256;
        c.slot(CuComponentState::Stop).deallocated += 1024;

        assert_eq!(c.start.allocated, 1024);
        assert_eq!(c.process.allocated, 256);
        assert_eq!(c.process.deallocated, 256);
        assert_eq!(c.stop.deallocated, 1024);

        assert_eq!(c.lifetime_alloc(), 1024 + 256);
        assert_eq!(c.lifetime_dealloc(), 256 + 1024);
        assert_eq!(
            c.lifetime_alloc().saturating_sub(c.lifetime_dealloc()),
            0
        );
    }

    #[test]
    fn drain_rt_violations_takes_pending_only() {
        let mut pending = alloc::vec![
            None,
            Some(RtViolation { step: CuComponentState::Process, allocated: 128, deallocated: 0 }),
            None,
            Some(RtViolation { step: CuComponentState::Preprocess, allocated: 32, deallocated: 0 }),
        ];
        let drained = drain_rt_violations(&mut pending);
        assert_eq!(drained.len(), 2);
        assert!(pending.iter().all(|s| s.is_none()));
        // Second drain reports nothing.
        assert!(drain_rt_violations(&mut pending).is_empty());

        let indices: Vec<usize> = drained.iter().map(|(id, _)| id.index()).collect();
        assert_eq!(indices, alloc::vec![1, 3]);
        assert!(matches!(drained[0].1.step, CuComponentState::Process));
        assert_eq!(drained[0].1.allocated, 128);
    }
}
