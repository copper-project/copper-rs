use alloc::format;
use alloc::string::String;
use core::sync::atomic::{AtomicBool, AtomicI32, Ordering};
use cu29::prelude::*;
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

#[derive(Clone, Debug)]
struct LastProgress {
    at: Instant,
    clid: u64,
}

impl LastProgress {
    fn new() -> Self {
        Self {
            at: Instant::now(),
            clid: 0,
        }
    }
}

#[derive(Debug)]
struct SafetyCfg {
    copperlist_deadline: Duration,
    watchdog_period: Duration,
    exit_code_shutdown: i32,
    exit_code_lock: i32,
    exit_code_panic: i32,
}

impl SafetyCfg {
    fn from_monitor_config(monitor_cfg: Option<&ComponentConfig>) -> CuResult<Self> {
        let deadline_ms = read_u64(monitor_cfg, "copperlist_deadline_ms")?.unwrap_or(1000);
        if deadline_ms == 0 {
            return Err(CuError::from(
                "cu_safetymon copperlist_deadline_ms must be > 0",
            ));
        }

        let watchdog_period_ms =
            read_u64(monitor_cfg, "watchdog_period_ms")?.unwrap_or((deadline_ms / 4).max(10));
        if watchdog_period_ms == 0 {
            return Err(CuError::from("cu_safetymon watchdog_period_ms must be > 0"));
        }

        let exit_code_shutdown = read_i32(monitor_cfg, "exit_code_shutdown")?.unwrap_or(70);
        let exit_code_lock = read_i32(monitor_cfg, "exit_code_lock")?.unwrap_or(71);
        let exit_code_panic = read_i32(monitor_cfg, "exit_code_panic")?.unwrap_or(72);

        Ok(Self {
            copperlist_deadline: Duration::from_millis(deadline_ms),
            watchdog_period: Duration::from_millis(watchdog_period_ms),
            exit_code_shutdown,
            exit_code_lock,
            exit_code_panic,
        })
    }
}

#[derive(Debug)]
struct SharedState {
    last_progress: Mutex<LastProgress>,
    exit_requested: AtomicI32,
    stopping: AtomicBool,
    last_fault: Mutex<Option<String>>,
}

impl SharedState {
    fn new() -> Self {
        Self {
            last_progress: Mutex::new(LastProgress::new()),
            exit_requested: AtomicI32::new(0),
            stopping: AtomicBool::new(false),
            last_fault: Mutex::new(None),
        }
    }

    fn touch(&self, clid: u64) {
        let mut guard = self
            .last_progress
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        guard.at = Instant::now();
        guard.clid = clid;
    }

    fn request_exit(&self, code: i32, reason: String) {
        self.exit_requested.store(code, Ordering::SeqCst);
        *self
            .last_fault
            .lock()
            .unwrap_or_else(|poison| poison.into_inner()) = Some(reason);
    }
}

pub struct CuSafetyMon {
    components: &'static [MonitorComponentMetadata],
    shared: Arc<SharedState>,
    cfg: SafetyCfg,
    watchdog: Option<JoinHandle<()>>,
    monitor_runtime: CuMonitoringRuntime,
    panic_action: Option<PanicHookRegistration>,
    execution_probe: MonitorExecutionProbe,
}

impl CuSafetyMon {
    fn component_name(&self, component_id: ComponentId) -> &'static str {
        self.components[component_id.index()].id()
    }

    fn emit_fault(prefix: &str, detail: &str) {
        // Always emit to stderr so fault details are visible even without text log sinks.
        eprintln!("{} {}", prefix, detail);
        error!("{} {}", prefix, detail);
    }

    fn spawn_watchdog(&mut self) {
        let shared = Arc::clone(&self.shared);
        let deadline = self.cfg.copperlist_deadline;
        let period = self.cfg.watchdog_period;
        let exit_code = self.cfg.exit_code_lock;
        let components = self.components;
        let probe = self.execution_probe.clone();

        self.watchdog = Some(thread::spawn(move || {
            loop {
                if shared.stopping.load(Ordering::SeqCst) || runtime_panic_hook_active() {
                    break;
                }
                thread::park_timeout(period);

                if shared.stopping.load(Ordering::SeqCst) || runtime_panic_hook_active() {
                    break;
                }

                let (elapsed, last_culistid) = {
                    let guard = shared
                        .last_progress
                        .lock()
                        .unwrap_or_else(|poison| poison.into_inner());
                    (guard.at.elapsed(), guard.clid)
                };

                if elapsed > deadline {
                    if shared.stopping.load(Ordering::SeqCst) || runtime_panic_hook_active() {
                        break;
                    }

                    let marker = probe.marker();
                    let culist_info = format!("last_culist={last_culistid}");
                    let detail = if let Some(marker) = marker {
                        let component = components.get(marker.component_id.index()).map(|c| c.id());
                        match component {
                            Some(component) => format!(
                                "watchdog timeout after {:?}; last marker: component='{}' step={:?}; {}",
                                elapsed, component, marker.step, culist_info
                            ),
                            None => format!(
                                "watchdog timeout after {:?}; last marker: component_id={} step={:?}; {}",
                                elapsed,
                                marker.component_id.index(),
                                marker.step,
                                culist_info
                            ),
                        }
                    } else {
                        format!(
                            "watchdog timeout after {:?}; no execution marker seen; {}",
                            elapsed, culist_info
                        )
                    };

                    if shared.stopping.load(Ordering::SeqCst) || runtime_panic_hook_active() {
                        break;
                    }

                    shared.request_exit(exit_code, detail.clone());
                    Self::emit_fault("cu_safetymon lock fault:", &detail);
                    std::process::exit(exit_code);
                }
            }
        }));
    }

    fn panic_detail_from_report(report: &PanicReport) -> String {
        match report.location() {
            Some(location) => format!("panic at {}: {}", location, report.message()),
            None => format!("panic: {}", report.message()),
        }
    }

    fn install_panic_action(&mut self) {
        let components = self.components;
        let shared = Arc::clone(&self.shared);
        let panic_code = self.cfg.exit_code_panic;
        let probe = self.execution_probe.clone();
        self.panic_action = Some(self.monitor_runtime.register_panic_action(move |report| {
            let marker = probe.marker();
            let last_culistid = {
                let guard = shared
                    .last_progress
                    .lock()
                    .unwrap_or_else(|poison| poison.into_inner());
                guard.clid
            };

            let detail = if let Some(marker) = marker {
                let component = components.get(marker.component_id.index()).map(|c| c.id());
                match component {
                    Some(component) => format!(
                        "{}; last marker component='{}' step={:?}; last_culist={}",
                        Self::panic_detail_from_report(report),
                        component,
                        marker.step,
                        last_culistid
                    ),
                    None => format!(
                        "{}; last marker component={} step={:?}; last_culist={}",
                        Self::panic_detail_from_report(report),
                        marker.component_id.index(),
                        marker.step,
                        last_culistid
                    ),
                }
            } else {
                format!(
                    "{}; last_culist={}",
                    Self::panic_detail_from_report(report),
                    last_culistid
                )
            };

            shared.request_exit(panic_code, detail.clone());
            Self::emit_fault("cu_safetymon panic fault:", &detail);
            Some(panic_code)
        }));
    }

    fn maybe_exit_with_latched_code(&self) {
        let code = self.shared.exit_requested.load(Ordering::SeqCst);
        if code > 0 {
            let reason = self
                .shared
                .last_fault
                .lock()
                .unwrap_or_else(|poison| poison.into_inner())
                .clone()
                .unwrap_or_else(|| "no reason captured".to_string());
            let detail = format!("exiting process with code {} ({})", code, reason);
            Self::emit_fault("cu_safetymon", &detail);
            std::process::exit(code);
        }
    }
}

impl CuMonitor for CuSafetyMon {
    fn new(metadata: CuMonitoringMetadata, runtime: CuMonitoringRuntime) -> CuResult<Self> {
        let cfg = SafetyCfg::from_monitor_config(metadata.monitor_config())?;
        let execution_probe = runtime.execution_probe().clone();
        Ok(Self {
            components: metadata.components(),
            shared: Arc::new(SharedState::new()),
            cfg,
            watchdog: None,
            monitor_runtime: runtime,
            panic_action: None,
            execution_probe,
        })
    }

    fn start(&mut self, _ctx: &CuContext) -> CuResult<()> {
        self.shared.touch(0);
        self.install_panic_action();
        self.spawn_watchdog();
        info!(
            "cu_safetymon started: deadline={:?} period={:?} codes(shutdown={}, lock={}, panic={})",
            self.cfg.copperlist_deadline,
            self.cfg.watchdog_period,
            self.cfg.exit_code_shutdown,
            self.cfg.exit_code_lock,
            self.cfg.exit_code_panic,
        );
        Ok(())
    }

    fn process_copperlist(&self, ctx: &CuContext, _view: CopperListView<'_>) -> CuResult<()> {
        self.shared.touch(ctx.cl_id());
        Ok(())
    }

    fn process_error(
        &self,
        component_id: ComponentId,
        step: CuComponentState,
        error: &CuError,
    ) -> Decision {
        let component_name = self.component_name(component_id);
        let detail = format!(
            "runtime fault: component='{}' step={:?} error={} ",
            component_name, step, error
        );
        self.shared
            .request_exit(self.cfg.exit_code_shutdown, detail.clone());
        Self::emit_fault("cu_safetymon", &detail);
        Decision::Shutdown
    }

    fn process_panic(&self, panic_message: &str) {
        self.shared
            .request_exit(self.cfg.exit_code_panic, panic_message.to_string());
        Self::emit_fault("cu_safetymon panic observed:", panic_message);
    }

    fn stop(&mut self, _ctx: &CuContext) -> CuResult<()> {
        self.shared.stopping.store(true, Ordering::SeqCst);
        self.panic_action = None;

        if let Some(handle) = self.watchdog.take() {
            handle.thread().unpark();
            let _ = handle.join();
        }

        self.maybe_exit_with_latched_code();
        Ok(())
    }
}

fn read_u64(cfg: Option<&ComponentConfig>, key: &str) -> CuResult<Option<u64>> {
    match cfg {
        Some(cfg) => cfg.get::<u64>(key).map_err(Into::into),
        None => Ok(None),
    }
}

fn read_i32(cfg: Option<&ComponentConfig>, key: &str) -> CuResult<Option<i32>> {
    match cfg {
        Some(cfg) => cfg.get::<i32>(key).map_err(Into::into),
        None => Ok(None),
    }
}
