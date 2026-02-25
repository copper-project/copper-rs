use cu29::prelude::*;

/// Placeholder `no_std` variant of `CuSafetyMon`.
///
/// Full watchdog/panic-handling behavior depends on `std` threading and process
/// termination semantics and is intentionally deferred.
pub struct CuSafetyMon {
    #[allow(dead_code)]
    taskids: &'static [&'static str],
}

impl CuMonitor for CuSafetyMon {
    fn new(_config: &CuConfig, taskids: &'static [&'static str]) -> CuResult<Self> {
        let _ = taskids;
        Err(CuError::from(
            "cu_safetymon currently requires std (no_std support intentionally deferred)",
        ))
    }

    fn process_copperlist(&self, _ctx: &CuContext, _msgs: &[&CuMsgMetadata]) -> CuResult<()> {
        Ok(())
    }

    fn process_error(&self, _taskid: usize, _step: CuTaskState, _error: &CuError) -> Decision {
        Decision::Shutdown
    }
}
