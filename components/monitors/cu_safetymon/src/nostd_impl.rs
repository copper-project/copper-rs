use cu29::prelude::*;

/// Placeholder `no_std` variant of `CuSafetyMon`.
///
/// Full watchdog/panic-handling behavior depends on `std` threading and process
/// termination semantics and is intentionally deferred.
pub struct CuSafetyMon {
    #[allow(dead_code)]
    component_ids: &'static [&'static str],
}

impl CuMonitor for CuSafetyMon {
    fn new(metadata: CuMonitoringMetadata, runtime: CuMonitoringRuntime) -> CuResult<Self> {
        let _ = (metadata, runtime);
        Err(CuError::from(
            "cu_safetymon currently requires std (no_std support intentionally deferred)",
        ))
    }

    fn process_copperlist(&self, _ctx: &CuContext, _view: CopperListView<'_>) -> CuResult<()> {
        Ok(())
    }

    fn process_error(
        &self,
        _component_id: ComponentId,
        _step: CuComponentState,
        _error: &CuError,
    ) -> Decision {
        Decision::Shutdown
    }
}
