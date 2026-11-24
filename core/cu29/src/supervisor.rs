//! Startup configuration shared by the live mission apps of a generated supervisor.
use crate::__private::sync::{Arc, Mutex};
use alloc::boxed::Box;
use cu29_clock::{CuDuration, RobotClock};
use cu29_runtime::app::{ActiveMission, CuRunExit, CuWaitError};
use cu29_runtime::resource::ResourceManager;
use cu29_traits::{CuError, CuResult};
use cu29_unifiedlog::{NoopLogger, NoopSectionStorage, SectionStorage, UnifiedLogWrite};

#[cfg(not(feature = "static-config"))]
type ResourceConfig = cu29_runtime::config::CuConfig;
#[cfg(feature = "static-config")]
type ResourceConfig = ();

type ResourceFactory<'a> = Box<dyn FnMut(&ResourceConfig) -> CuResult<ResourceManager> + 'a>;

#[cfg(feature = "clock-sync")]
type Reference = (
    Box<dyn crate::clock_sync::ClockReference>,
    crate::clock_sync::MaintenanceConfig,
);

/// Settings consumed by generated code at a mission boundary.
#[doc(hidden)]
pub struct MissionSettings<'a> {
    pub clock: Option<RobotClock>,
    pub instance_id: u32,
    pub save_effective_config: bool,
    pub resources: Option<ResourceFactory<'a>>,
    #[cfg(all(feature = "std", not(feature = "static-config")))]
    pub config: Option<cu29_runtime::config::CuConfig>,
    #[cfg(feature = "clock-sync")]
    pub reference: Option<Box<dyn FnMut() -> CuResult<Reference> + 'a>>,
    #[cfg(all(feature = "telemetry", not(feature = "async-cl-io")))]
    pub manual_telemetry:
        Option<Box<dyn FnMut() -> CuResult<crate::logstream::manual::ManualDestination> + 'a>>,
    #[cfg(feature = "logstream")]
    pub telemetry:
        Option<Box<dyn FnMut() -> CuResult<crate::logstream::TelemetryDestination> + 'a>>,
}

/// Implemented by the mission enum emitted by the runtime macro.
#[doc(hidden)]
pub trait ConfiguredMission: Sized {
    type Active<S: SectionStorage + 'static, L: UnifiedLogWrite<S> + 'static>: ActiveMission;
    fn activate<S: SectionStorage + 'static, L: UnifiedLogWrite<S> + 'static>(
        self,
        settings: &mut MissionSettings<'_>,
        logger: Arc<Mutex<L>>,
    ) -> CuResult<Self::Active<S, L>>;
    fn activate_default(
        self,
        settings: &mut MissionSettings<'_>,
        logger: Arc<Mutex<NoopLogger>>,
    ) -> CuResult<Self::Active<NoopSectionStorage, NoopLogger>> {
        self.activate(settings, logger)
    }
}

type ActivateMission<M, S, L> = fn(
    M,
    &mut MissionSettings<'_>,
    Arc<Mutex<L>>,
) -> CuResult<<M as ConfiguredMission>::Active<S, L>>;

/// Configures all live missions of an app before starting its supervisor.
pub struct SupervisorBuilder<
    'a,
    M: ConfiguredMission,
    S: SectionStorage + 'static = NoopSectionStorage,
    L: UnifiedLogWrite<S> + 'static = NoopLogger,
> {
    settings: MissionSettings<'a>,
    logger: Arc<Mutex<L>>,
    activate: ActivateMission<M, S, L>,
}

impl<M: ConfiguredMission> Default for SupervisorBuilder<'_, M> {
    fn default() -> Self {
        Self {
            settings: MissionSettings {
                #[cfg(all(feature = "std", not(feature = "clock-sync")))]
                clock: Some(RobotClock::default()),
                #[cfg(not(all(feature = "std", not(feature = "clock-sync"))))]
                clock: None,
                instance_id: 0,
                save_effective_config: true,
                resources: None,
                #[cfg(all(feature = "std", not(feature = "static-config")))]
                config: None,
                #[cfg(feature = "clock-sync")]
                reference: None,
                #[cfg(feature = "logstream")]
                telemetry: None,
                #[cfg(all(feature = "telemetry", not(feature = "async-cl-io")))]
                manual_telemetry: None,
            },
            logger: Arc::new(Mutex::new(NoopLogger::new())),
            activate: M::activate_default,
        }
    }
}

impl<'a, M: ConfiguredMission, S: SectionStorage + 'static, L: UnifiedLogWrite<S> + 'static>
    SupervisorBuilder<'a, M, S, L>
{
    /// Use this execution clock for every activation.
    pub fn with_clock(mut self, clock: RobotClock) -> Self {
        self.settings.clock = Some(clock);
        self
    }

    /// Apply the same instance ID to every mission in the recording.
    pub fn with_instance_id(mut self, instance_id: u32) -> Self {
        self.settings.instance_id = instance_id;
        self
    }

    /// Every mission writes to the supplied logger with its own mission and run context.
    pub fn with_logger<S2: SectionStorage + 'static, L2: UnifiedLogWrite<S2> + 'static>(
        self,
        logger: Arc<Mutex<L2>>,
    ) -> SupervisorBuilder<'a, M, S2, L2> {
        SupervisorBuilder {
            settings: self.settings,
            logger,
            activate: M::activate,
        }
    }

    #[cfg(feature = "std")]
    pub fn with_unified_logger(
        self,
        logger: Arc<Mutex<cu29_unifiedlog::UnifiedLoggerWrite>>,
    ) -> SupervisorBuilder<
        'a,
        M,
        cu29_unifiedlog::memmap::MmapSectionStorage,
        cu29_unifiedlog::UnifiedLoggerWrite,
    > {
        self.with_logger(logger)
    }

    #[cfg(feature = "std")]
    /// Open one archive shared by all mission runs, using the app builder's slab size default.
    pub fn with_log_path(
        self,
        path: impl AsRef<std::path::Path>,
        slab_size: Option<usize>,
    ) -> CuResult<
        SupervisorBuilder<
            'a,
            M,
            cu29_unifiedlog::memmap::MmapSectionStorage,
            cu29_unifiedlog::UnifiedLoggerWrite,
        >,
    > {
        Ok(self.with_unified_logger(open_log(path.as_ref(), slab_size)?))
    }

    #[cfg(all(feature = "std", not(feature = "static-config")))]
    pub fn with_config(mut self, config: cu29_runtime::config::CuConfig) -> Self {
        self.settings.config = Some(config);
        self
    }

    /// Save each mission's effective configuration in log metadata (default: true).
    /// Static-config builds always omit it.
    pub fn save_effective_config(mut self, save: bool) -> Self {
        self.settings.save_effective_config = save;
        self
    }

    /// Supply owned resources for each activation. The previous graph has been released.
    pub fn with_resources(
        mut self,
        resources: impl FnMut(&ResourceConfig) -> CuResult<ResourceManager> + 'a,
    ) -> Self {
        self.settings.resources = Some(Box::new(resources));
        self
    }

    /// Open an owned clock reference for each activation.
    #[cfg(feature = "clock-sync")]
    pub fn with_clock_reference<P: crate::clock_sync::ClockReference>(
        mut self,
        mut open: impl FnMut() -> CuResult<P> + 'a,
        settings: crate::clock_sync::MaintenanceConfig,
    ) -> Self {
        self.settings.reference = Some(Box::new(move || Ok((Box::new(open()?), settings))));
        self
    }

    /// Open a fresh session and native transmitter for each mission activation.
    #[cfg(all(feature = "telemetry", not(feature = "async-cl-io")))]
    pub fn with_telemetry_tx<T: crate::logstream::protocol::CuTelemetryTx + 'static>(
        mut self,
        mut open: impl FnMut() -> CuResult<([u8; 16], T)> + 'a,
        buffers: crate::logstream::TelemetryBufferConfig,
    ) -> Self {
        self.settings.manual_telemetry = Some(Box::new(move || {
            let (session_id, tx) = open()?;
            Ok(crate::logstream::manual::ManualDestination {
                session_id,
                buffers,
                make_tx: Box::new(move |_| Ok(Box::new(tx))),
            })
        }));
        self
    }

    /// Open the transport after the previous mission's sender has shut down.
    #[cfg(feature = "logstream")]
    pub fn with_logstream<T: crate::logstream::CuStreamTx + 'static>(
        mut self,
        mut open: impl FnMut() -> CuResult<T> + 'a,
        config: crate::logstream::LogStreamSenderConfig,
    ) -> Self {
        self.settings.telemetry = Some(Box::new(move || {
            Ok(crate::logstream::TelemetryDestination::Fec(
                Box::new(open()?),
                Box::new(config.clone()),
            ))
        }));
        self
    }

    #[cfg(feature = "logstream")]
    /// Construct the native sender on each activation using its transport clock.
    pub fn with_telemetry_tx<T: crate::logstream::protocol::CuTelemetryTx + 'static>(
        mut self,
        make_tx: impl FnMut(RobotClock) -> CuResult<T> + Send + 'static,
        buffers: crate::logstream::TelemetryBufferConfig,
    ) -> Self {
        let make_tx = Arc::new(Mutex::new(make_tx));
        self.settings.telemetry = Some(Box::new(move || {
            let make_tx = make_tx.clone();
            Ok(crate::logstream::TelemetryDestination::Native {
                make_tx: Box::new(move |clock| {
                    let mut open = make_tx
                        .lock()
                        .map_err(|_| CuError::from("telemetry factory lock poisoned"))?;
                    Ok(Box::new(open(clock)?))
                }),
                buffers,
            })
        }));
        self
    }

    /// Build an idle supervisor. Each start uses the same platform settings.
    pub fn build(self) -> CuResult<Supervisor<'a, M, S, L>> {
        Ok(Supervisor {
            builder: self,
            active: None,
        })
    }
}

/// Owns the platform settings and the active generated app. Use the app-specific supervisor alias.
pub struct Supervisor<
    'a,
    M: ConfiguredMission,
    S: SectionStorage + 'static = NoopSectionStorage,
    L: UnifiedLogWrite<S> + 'static = NoopLogger,
> {
    builder: SupervisorBuilder<'a, M, S, L>,
    active: Option<M::Active<S, L>>,
}

impl<'a, M: ConfiguredMission> Supervisor<'a, M> {
    pub fn builder() -> SupervisorBuilder<'a, M> {
        SupervisorBuilder::default()
    }
}

impl<M: ConfiguredMission, S: SectionStorage + 'static, L: UnifiedLogWrite<S> + 'static>
    Supervisor<'_, M, S, L>
{
    /// Build and start the selected graph on the calling thread.
    pub fn start(&mut self, mission: M) -> CuResult<()> {
        if self.active.is_some() {
            return Err("a mission is already running".into());
        }
        self.active = Some((self.builder.activate)(
            mission,
            &mut self.builder.settings,
            self.builder.logger.clone(),
        )?);
        Ok(())
    }

    /// Drive the running graph until the predicate succeeds or Copper requests shutdown.
    pub fn run_until(&mut self, requested: impl FnMut() -> bool) -> CuResult<CuRunExit> {
        self.active_mission()?.run_until(requested)
    }

    /// Drive the active mission until its readiness predicate succeeds or the wait ends.
    pub fn wait_until_ready(
        &mut self,
        timeout: CuDuration,
        ready: impl FnMut() -> bool,
    ) -> Result<(), CuWaitError> {
        self.active_mission()
            .map_err(CuWaitError::Execution)?
            .wait_until_ready(timeout, ready)
    }

    /// Drive the active mission until its drain predicate succeeds or the wait ends.
    pub fn drain_until(
        &mut self,
        timeout: CuDuration,
        drained: impl FnMut() -> bool,
    ) -> Result<(), CuWaitError> {
        self.active_mission()
            .map_err(CuWaitError::Execution)?
            .drain_until(timeout, drained)
    }

    fn active_mission(&mut self) -> CuResult<&mut M::Active<S, L>> {
        self.active
            .as_mut()
            .ok_or_else(|| CuError::from("no active mission"))
    }

    /// Release the old app before constructing the target. A failed switch leaves the supervisor idle.
    pub fn transition_to(&mut self, mission: M) -> CuResult<()> {
        self.stop()?;
        self.start(mission)
    }

    /// Stop and release the current app. The supervisor can start another mission.
    pub fn stop(&mut self) -> CuResult<()> {
        self.active.take().map_or(Ok(()), ActiveMission::stop)
    }
}

impl<M: ConfiguredMission, S: SectionStorage + 'static, L: UnifiedLogWrite<S> + 'static> Drop
    for Supervisor<'_, M, S, L>
{
    fn drop(&mut self) {
        let _ = self.stop();
    }
}

#[cfg(feature = "std")]
#[doc(hidden)]
pub fn open_log(
    path: &std::path::Path,
    slab_size: Option<usize>,
) -> CuResult<Arc<Mutex<cu29_unifiedlog::UnifiedLoggerWrite>>> {
    use cu29_unifiedlog::{UnifiedLogger, UnifiedLoggerBuilder};
    let logger = UnifiedLoggerBuilder::new()
        .write(true)
        .create(true)
        .file_base_name(path)
        .preallocated_size(slab_size.unwrap_or(10 * 1024 * 1024))
        .build()
        .map_err(|error| CuError::new_with_cause("Failed to create unified logger", error))?;
    match logger {
        UnifiedLogger::Write(logger) => Ok(Arc::new(Mutex::new(logger))),
        UnifiedLogger::Read(_) => {
            Err("UnifiedLoggerBuilder did not create a write-capable logger".into())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    enum Mission {
        StopFails,
        StartFails,
        Unreachable,
        Normal,
    }

    impl ConfiguredMission for Mission {
        type Active<S: SectionStorage + 'static, L: UnifiedLogWrite<S> + 'static> = Self;
        fn activate<S: SectionStorage + 'static, L: UnifiedLogWrite<S> + 'static>(
            self,
            _: &mut MissionSettings<'_>,
            _: Arc<Mutex<L>>,
        ) -> CuResult<Self> {
            match self {
                Self::Unreachable => panic!("target constructed before successful stop"),
                Self::StartFails => Err("start failed".into()),
                mission => Ok(mission),
            }
        }
    }

    impl ActiveMission for Mission {
        fn run_until(&mut self, _: impl FnMut() -> bool) -> CuResult<CuRunExit> {
            Ok(CuRunExit::Requested)
        }
        fn wait_until_ready(
            &mut self,
            _: CuDuration,
            _: impl FnMut() -> bool,
        ) -> Result<(), CuWaitError> {
            unreachable!("idle wait must fail before dispatch")
        }
        fn drain_until(
            &mut self,
            _: CuDuration,
            _: impl FnMut() -> bool,
        ) -> Result<(), CuWaitError> {
            unreachable!("idle wait must fail before dispatch")
        }
        fn stop(self) -> CuResult<()> {
            if matches!(self, Self::StopFails) {
                Err("stop failed".into())
            } else {
                Ok(())
            }
        }
    }

    #[test]
    fn failed_switch_leaves_supervisor_idle_and_reusable() {
        let mut supervisor = Supervisor::builder().build().unwrap();
        supervisor.start(Mission::StopFails).unwrap();
        assert!(supervisor.start(Mission::Unreachable).is_err());
        assert!(supervisor.transition_to(Mission::Unreachable).is_err());
        assert!(supervisor.run_until(|| true).is_err());
        assert!(matches!(
            supervisor.wait_until_ready(CuDuration::from_nanos(0), || true),
            Err(CuWaitError::Execution(_))
        ));
        assert!(matches!(
            supervisor.drain_until(CuDuration::from_nanos(0), || true),
            Err(CuWaitError::Execution(_))
        ));
        assert!(supervisor.transition_to(Mission::StartFails).is_err());
        supervisor.start(Mission::Normal).unwrap();
        supervisor.stop().unwrap();
        supervisor.start(Mission::Normal).unwrap();
    }
}
