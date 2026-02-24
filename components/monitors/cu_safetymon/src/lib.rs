#![cfg_attr(not(feature = "std"), no_std)]

extern crate alloc;

use alloc::format;
use alloc::string::String;
use core::sync::atomic::{AtomicBool, AtomicI32, Ordering};
use cu29::prelude::*;

#[cfg(feature = "std")]
use std::panic::PanicHookInfo;
#[cfg(feature = "std")]
use std::sync::{Arc, Mutex};
#[cfg(feature = "std")]
use std::thread::{self, JoinHandle};
#[cfg(feature = "std")]
use std::time::{Duration, Instant};

#[cfg(feature = "std")]
type PanicHook = Box<dyn Fn(&PanicHookInfo<'_>) + Send + Sync + 'static>;

#[cfg(feature = "std")]
#[derive(Clone, Debug)]
struct LastProgress {
    at: Instant,
}

#[cfg(feature = "std")]
impl LastProgress {
    fn new() -> Self {
        Self { at: Instant::now() }
    }
}

#[cfg(feature = "std")]
#[derive(Debug)]
struct SafetyCfg {
    copperlist_deadline: Duration,
    watchdog_period: Duration,
    exit_code_shutdown: i32,
    exit_code_lock: i32,
    exit_code_panic: i32,
}

#[cfg(feature = "std")]
impl SafetyCfg {
    fn from_config(config: &CuConfig) -> CuResult<Self> {
        let monitor_cfg = config.get_monitor_config().and_then(|m| m.get_config());

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

#[cfg(feature = "std")]
#[derive(Debug)]
struct SharedState {
    last_progress: Mutex<LastProgress>,
    exit_requested: AtomicI32,
    stopping: AtomicBool,
    last_fault: Mutex<Option<String>>,
}

#[cfg(feature = "std")]
impl SharedState {
    fn new() -> Self {
        Self {
            last_progress: Mutex::new(LastProgress::new()),
            exit_requested: AtomicI32::new(0),
            stopping: AtomicBool::new(false),
            last_fault: Mutex::new(None),
        }
    }

    fn touch(&self) {
        let mut guard = self
            .last_progress
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        guard.at = Instant::now();
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
    taskids: &'static [&'static str],
    #[cfg(feature = "std")]
    shared: Arc<SharedState>,
    #[cfg(feature = "std")]
    cfg: SafetyCfg,
    #[cfg(feature = "std")]
    watchdog: Option<JoinHandle<()>>,
    #[cfg(feature = "std")]
    previous_panic_hook: Option<PanicHook>,
    #[cfg(feature = "std")]
    execution_probe: Option<ExecutionProbeHandle>,
}

impl CuSafetyMon {
    #[cfg(feature = "std")]
    fn spawn_watchdog(&mut self) {
        let shared = Arc::clone(&self.shared);
        let deadline = self.cfg.copperlist_deadline;
        let period = self.cfg.watchdog_period;
        let exit_code = self.cfg.exit_code_lock;
        let taskids: Vec<String> = self.taskids.iter().map(|id| (*id).to_string()).collect();
        let probe = self.execution_probe.clone();

        self.watchdog = Some(thread::spawn(move || {
            loop {
                if shared.stopping.load(Ordering::SeqCst) {
                    break;
                }
                thread::sleep(period);

                let elapsed = {
                    let guard = shared
                        .last_progress
                        .lock()
                        .unwrap_or_else(|poison| poison.into_inner());
                    guard.at.elapsed()
                };

                if elapsed > deadline {
                    let marker = probe.as_ref().and_then(|p| p.marker());
                    let detail = if let Some(marker) = marker {
                        let component = taskids.get(marker.component_id).map(|s| s.as_str());
                        match (component, marker.culistid) {
                            (Some(component), Some(clid)) => format!(
                                "watchdog timeout after {:?}; last marker: component='{}' step={:?} culist={}",
                                elapsed, component, marker.step, clid
                            ),
                            (Some(component), None) => format!(
                                "watchdog timeout after {:?}; last marker: component='{}' step={:?}",
                                elapsed, component, marker.step
                            ),
                            (None, Some(clid)) => format!(
                                "watchdog timeout after {:?}; last marker: component_id={} step={:?} culist={}",
                                elapsed, marker.component_id, marker.step, clid
                            ),
                            (None, None) => format!(
                                "watchdog timeout after {:?}; last marker: component_id={} step={:?}",
                                elapsed, marker.component_id, marker.step
                            ),
                        }
                    } else {
                        format!(
                            "watchdog timeout after {:?}; no execution marker seen",
                            elapsed
                        )
                    };

                    shared.request_exit(exit_code, detail.clone());
                    error!("cu_safetymon lock fault: {}", detail);
                    std::process::exit(exit_code);
                }
            }
        }));
    }

    #[cfg(feature = "std")]
    fn install_panic_hook(&mut self) {
        let shared = Arc::clone(&self.shared);
        let panic_code = self.cfg.exit_code_panic;
        let probe = self.execution_probe.clone();
        let previous = std::panic::take_hook();
        self.previous_panic_hook = Some(previous);

        std::panic::set_hook(Box::new(move |info| {
            let panic_message = if let Some(s) = info.payload().downcast_ref::<&str>() {
                (*s).to_string()
            } else if let Some(s) = info.payload().downcast_ref::<String>() {
                s.clone()
            } else {
                "panic with non-string payload".to_string()
            };

            let location = info
                .location()
                .map(|loc| format!("{}:{}:{}", loc.file(), loc.line(), loc.column()))
                .unwrap_or_else(|| "<unknown>".to_string());

            let marker = probe.as_ref().and_then(|p| p.marker());

            let detail = if let Some(marker) = marker {
                format!(
                    "panic at {}: {}; last marker component={} step={:?} culist={:?}",
                    location, panic_message, marker.component_id, marker.step, marker.culistid
                )
            } else {
                format!("panic at {}: {}", location, panic_message)
            };

            shared.request_exit(panic_code, detail.clone());
            error!("cu_safetymon panic fault: {}", detail);
            std::process::exit(panic_code);
        }));
    }

    #[cfg(feature = "std")]
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
            error!(
                "cu_safetymon exiting process with code {} ({})",
                code, reason
            );
            std::process::exit(code);
        }
    }
}

impl CuMonitor for CuSafetyMon {
    fn new(config: &CuConfig, taskids: &'static [&'static str]) -> CuResult<Self> {
        #[cfg(feature = "std")]
        {
            let cfg = SafetyCfg::from_config(config)?;
            Ok(Self {
                taskids,
                shared: Arc::new(SharedState::new()),
                cfg,
                watchdog: None,
                previous_panic_hook: None,
                execution_probe: None,
            })
        }

        #[cfg(not(feature = "std"))]
        {
            let _ = (config, taskids);
            Err(CuError::from(
                "cu_safetymon currently requires std (no_std support intentionally deferred)",
            ))
        }
    }

    fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
        #[cfg(feature = "std")]
        {
            self.shared.touch();
            self.install_panic_hook();
            self.spawn_watchdog();
            info!(
                "cu_safetymon started: deadline={:?} period={:?} codes(shutdown={}, lock={}, panic={})",
                self.cfg.copperlist_deadline,
                self.cfg.watchdog_period,
                self.cfg.exit_code_shutdown,
                self.cfg.exit_code_lock,
                self.cfg.exit_code_panic,
            );
        }
        Ok(())
    }

    fn process_copperlist(&self, _msgs: &[&CuMsgMetadata]) -> CuResult<()> {
        #[cfg(feature = "std")]
        {
            self.shared.touch();
        }
        Ok(())
    }

    fn process_error(&self, taskid: usize, step: CuTaskState, error: &CuError) -> Decision {
        #[cfg(feature = "std")]
        {
            let task_name = self.taskids.get(taskid).copied().unwrap_or("<?>");
            let detail = format!(
                "runtime fault: task='{}' step={:?} error={} ",
                task_name, step, error
            );
            self.shared
                .request_exit(self.cfg.exit_code_shutdown, detail.clone());
            error!("cu_safetymon {}", detail);
        }
        Decision::Shutdown
    }

    fn process_panic(&self, panic_message: &str) {
        #[cfg(feature = "std")]
        {
            self.shared
                .request_exit(self.cfg.exit_code_panic, panic_message.to_string());
            error!("cu_safetymon panic observed: {}", panic_message);
        }
    }

    #[cfg(feature = "std")]
    fn set_execution_probe(&mut self, probe: ExecutionProbeHandle) {
        self.execution_probe = Some(probe);
    }

    fn stop(&mut self, _clock: &RobotClock) -> CuResult<()> {
        #[cfg(feature = "std")]
        {
            self.shared.stopping.store(true, Ordering::SeqCst);

            if let Some(handle) = self.watchdog.take() {
                let _ = handle.join();
            }

            if let Some(previous_hook) = self.previous_panic_hook.take() {
                std::panic::set_hook(previous_hook);
            }

            self.maybe_exit_with_latched_code();
        }
        Ok(())
    }
}

#[cfg(feature = "std")]
fn read_u64(cfg: Option<&ComponentConfig>, key: &str) -> CuResult<Option<u64>> {
    match cfg {
        Some(cfg) => cfg.get::<u64>(key).map_err(Into::into),
        None => Ok(None),
    }
}

#[cfg(feature = "std")]
fn read_i32(cfg: Option<&ComponentConfig>, key: &str) -> CuResult<Option<i32>> {
    match cfg {
        Some(cfg) => cfg.get::<i32>(key).map_err(Into::into),
        None => Ok(None),
    }
}
