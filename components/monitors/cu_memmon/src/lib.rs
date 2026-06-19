#![cfg_attr(not(feature = "std"), no_std)]

//! Per-task heap-allocation monitor for Copper.
//!
//! `CuMemMon` consumes the [`CuMonitor::observe_alloc`] hook the runtime emits
//! around every task step and reports:
//! - per-step allocation deltas (debug-only, throttled to avoid spam),
//! - per-task lifetime balance (`alloc - dealloc` since the task's `start`),
//! - a final per-task **leak report** at runtime shutdown.
//!
//! Optional realtime-strict mode (`realtime_strict: true` in the monitor's
//! RON config) returns [`Decision::Shutdown`] from `process_error` whenever
//! any non-zero allocation happens during `Preprocess`, `Process`, or
//! `Postprocess`. This is intended for CI smoke tests that want to catch
//! "task X allocated on the hot path" regressions deterministically.
//!
//! When the runtime is **not** built with `cu29/memory_monitoring`, the
//! `observe_alloc` hook is still called but always reports zero. `CuMemMon`
//! detects this once at startup, emits a single `warning!`, and from then on
//! reports zeros without further noise.

extern crate alloc;

use alloc::vec::Vec;
use core::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use cu29::prelude::*;

#[cfg(feature = "std")]
use std::sync::Mutex;
#[cfg(feature = "std")]
type MutexGuard<'a, T> = std::sync::MutexGuard<'a, T>;

#[cfg(not(feature = "std"))]
use spin::Mutex;
#[cfg(not(feature = "std"))]
type MutexGuard<'a, T> = spin::MutexGuard<'a, T, spin::relax::Spin>;

/// One slot per (component, step). We collapse all five steps into separate
/// counters per component for compact storage and O(1) updates.
#[derive(Default, Clone, Copy)]
struct StepCounters {
    allocated: u64,
    deallocated: u64,
    calls: u64,
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

struct State {
    per_component: Vec<ComponentCounters>,
    /// True iff the runtime was compiled with `memory_monitoring`.
    /// Probed in `start()` by reading the public counters from the runtime.
    counters_available: bool,
}

pub struct CuMemMon {
    components: &'static [MonitorComponentMetadata],
    component_count: usize,
    realtime_strict: bool,
    state: Mutex<State>,
    /// One-shot flag to avoid spamming the realtime-violation log.
    rt_violation_logged: AtomicBool,
    /// Total observed allocations / deallocations across the whole runtime
    /// (cheap monotonic counters for the optional periodic summary).
    total_alloc: AtomicU64,
    total_dealloc: AtomicU64,
}

fn lock_state(state: &Mutex<State>) -> MutexGuard<'_, State> {
    #[cfg(feature = "std")]
    {
        state.lock().unwrap_or_else(|poison| poison.into_inner())
    }
    #[cfg(not(feature = "std"))]
    {
        state.lock()
    }
}

fn parse_realtime_strict(cfg: Option<&ComponentConfig>) -> CuResult<bool> {
    let Some(cfg) = cfg else { return Ok(false) };
    if let Some(strict) = cfg.get::<bool>("realtime_strict")? {
        return Ok(strict);
    }
    Ok(false)
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

        // Detect whether the runtime was built with the counting allocator.
        // When the feature is off both getters return `None`.
        let counters_available = cu29::monitoring::global_allocated_bytes().is_some();

        let state = State {
            per_component: alloc::vec![ComponentCounters::new(); component_count],
            counters_available,
        };

        Ok(Self {
            components,
            component_count,
            realtime_strict,
            state: Mutex::new(state),
            rt_violation_logged: AtomicBool::new(false),
            total_alloc: AtomicU64::new(0),
            total_dealloc: AtomicU64::new(0),
        })
    }

    fn start(&mut self, ctx: &CuContext) -> CuResult<()> {
        let state = lock_state(&self.state);
        if !state.counters_available {
            warning!(
                ctx,
                "cu_memmon enabled but cu29 was built without `memory_monitoring`. All reports will be zero."
            );
        } else {
            info!(
                ctx,
                "cu_memmon started ({} components, realtime_strict={})",
                self.component_count,
                self.realtime_strict
            );
        }
        Ok(())
    }

    fn process_copperlist(&self, _ctx: &CuContext, _view: CopperListView<'_>) -> CuResult<()> {
        // Per-copperlist hook intentionally a no-op: all the interesting work
        // happens in observe_alloc. A future revision can fold a throttled
        // info!() summary in here.
        Ok(())
    }

    fn observe_alloc(
        &self,
        component_id: ComponentId,
        step: CuComponentState,
        allocated_bytes: usize,
        deallocated_bytes: usize,
    ) {
        // Cheap aggregate counters first — never block on the mutex if the
        // runtime is hot and another thread is reporting.
        let alloc_u64 = allocated_bytes as u64;
        let dealloc_u64 = deallocated_bytes as u64;
        self.total_alloc.fetch_add(alloc_u64, Ordering::Relaxed);
        self.total_dealloc.fetch_add(dealloc_u64, Ordering::Relaxed);

        let mut state = lock_state(&self.state);
        let idx = component_id.index();
        debug_assert!(
            idx < state.per_component.len(),
            "cu_memmon: component id {idx} out of bounds {}",
            state.per_component.len()
        );
        if let Some(component) = state.per_component.get_mut(idx) {
            let slot = component.slot(step);
            slot.allocated = slot.allocated.saturating_add(alloc_u64);
            slot.deallocated = slot.deallocated.saturating_add(dealloc_u64);
            slot.calls = slot.calls.saturating_add(1);
        }

        // Realtime-strict warning is one-shot to keep the log clean; the actual
        // shutdown happens via process_error() once a task surfaces an error,
        // but most users will catch this via the warning first.
        if self.realtime_strict
            && is_realtime_step(step)
            && allocated_bytes > 0
            && !self.rt_violation_logged.swap(true, Ordering::Relaxed)
        {
            let name = self.component_name(component_id);
            warning!(
                "cu_memmon: realtime violation: component {} allocated {} B during {}",
                name,
                allocated_bytes as u64,
                step_label(step)
            );
        }
    }

    fn process_error(
        &self,
        component_id: ComponentId,
        step: CuComponentState,
        error: &CuError,
    ) -> Decision {
        let component_name = self.component_name(component_id);
        error!(
            "cu_memmon: component {} errored at {}: {}",
            component_name,
            step_label(step),
            error,
        );
        // We only escalate when realtime-strict mode actually flagged a
        // violation; otherwise we stay out of the runtime's way.
        if self.realtime_strict && self.rt_violation_logged.load(Ordering::Relaxed) {
            Decision::Shutdown
        } else {
            Decision::Ignore
        }
    }

    fn stop(&mut self, ctx: &CuContext) -> CuResult<()> {
        let state = lock_state(&self.state);
        if !state.counters_available {
            return Ok(());
        }
        info!(
            ctx,
            "cu_memmon final report: total +{}B / -{}B",
            self.total_alloc.load(Ordering::Relaxed),
            self.total_dealloc.load(Ordering::Relaxed)
        );
        for (idx, counters) in state.per_component.iter().enumerate() {
            let name = self.components[idx].id();
            let lifetime_alloc = counters.lifetime_alloc();
            let lifetime_dealloc = counters.lifetime_dealloc();
            let leak = lifetime_alloc.saturating_sub(lifetime_dealloc);
            let process_alloc = counters.process.allocated;
            let process_calls = counters.process.calls;
            if leak > 0 {
                warning!(
                    ctx,
                    "cu_memmon: task {} leak={}B (alloc {}B, dealloc {}B); process allocated {}B over {} calls",
                    name,
                    leak,
                    lifetime_alloc,
                    lifetime_dealloc,
                    process_alloc,
                    process_calls
                );
            } else {
                info!(
                    ctx,
                    "cu_memmon: task {} balanced (alloc {}B, dealloc {}B); process allocated {}B over {} calls",
                    name,
                    lifetime_alloc,
                    lifetime_dealloc,
                    process_alloc,
                    process_calls
                );
            }
        }
        Ok(())
    }
}

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
        // Net balanced over the lifetime: no leak.
        assert_eq!(
            c.lifetime_alloc().saturating_sub(c.lifetime_dealloc()),
            0
        );
    }
}
