//! Allocation-free monitor callbacks with an application-owned UI handle.
use cu29::prelude::*;
use std::sync::Arc;
use std::sync::atomic::AtomicBool;
use std::sync::atomic::AtomicU64;
use std::sync::atomic::Ordering;

pub(crate) struct ComponentStats {
    pub(crate) duration_ns: AtomicU64,
    pub(crate) failed: AtomicBool,
}

pub(crate) struct MonitorState {
    pub(crate) metadata: CuMonitoringMetadata,
    pub(crate) components: Box<[ComponentStats]>,
    pub(crate) cycles: AtomicU64,
    pub(crate) errors: AtomicU64,
    pub(crate) running: AtomicBool,
}

pub(crate) struct GraphMonitor {
    state: Arc<MonitorState>,
    pub(crate) on_start: Option<std::sync::mpsc::Sender<Arc<MonitorState>>>,
}
impl CuMonitor for GraphMonitor {
    fn new(metadata: CuMonitoringMetadata, _runtime: CuMonitoringRuntime) -> CuResult<Self> {
        let components = (0..metadata.component_count())
            .map(|_| ComponentStats {
                duration_ns: AtomicU64::new(u64::MAX),
                failed: AtomicBool::new(false),
            })
            .collect();
        Ok(Self {
            on_start: None,
            state: Arc::new(MonitorState {
                metadata,
                components,
                cycles: AtomicU64::new(0),
                errors: AtomicU64::new(0),
                running: AtomicBool::new(false),
            }),
        })
    }
    fn start(&mut self, _ctx: &CuContext) -> CuResult<()> {
        self.state.running.store(true, Ordering::Relaxed);
        if let Some(sender) = self.on_start.take() {
            let _ = sender.send(Arc::clone(&self.state));
        }
        Ok(())
    }
    fn process_copperlist(&self, _ctx: &CuContext, view: CopperListView<'_>) -> CuResult<()> {
        for entry in view.entries() {
            if let (Some(start), Some(end)) = (
                Option::<CuTime>::from(entry.msg.process_time.start),
                Option::<CuTime>::from(entry.msg.process_time.end),
            ) && end >= start
            {
                self.state.components[entry.component_id.index()]
                    .duration_ns
                    .store((end - start).as_nanos(), Ordering::Relaxed);
            }
        }
        self.state.cycles.fetch_add(1, Ordering::Relaxed);
        Ok(())
    }
    fn process_error(
        &self,
        component: ComponentId,
        _step: CuComponentState,
        _error: &CuError,
    ) -> Decision {
        self.state.components[component.index()]
            .failed
            .store(true, Ordering::Relaxed);
        self.state.errors.fetch_add(1, Ordering::Relaxed);
        Decision::Shutdown
    }
    fn stop(&mut self, _ctx: &CuContext) -> CuResult<()> {
        self.state.running.store(false, Ordering::Relaxed);
        Ok(())
    }
}
