use crate::curuntime::KeyFrame;
use cu29_clock::RobotClock;
use cu29_traits::CuResult;
use cu29_unifiedlog::{SectionStorage, UnifiedLogWrite};

#[cfg(feature = "std")]
use crate::copperlist::CopperList;
#[cfg(feature = "std")]
use cu29_clock::RobotClockMock;
#[cfg(feature = "std")]
use cu29_traits::CopperListTuple;

#[cfg(not(feature = "std"))]
mod imp {
    pub use alloc::string::String;
    pub use alloc::sync::Arc;
    pub use spin::Mutex;
}

#[cfg(feature = "std")]
mod imp {
    pub use crate::config::CuConfig;
    pub use crate::simulation::SimOverride;
    pub use cu29_unifiedlog::memmap::MmapSectionStorage;
    pub use std::sync::{Arc, Mutex};
}

use imp::*;

/// Convenience trait for CuApplication when it is just a std App
#[cfg(feature = "std")]
pub trait CuStdApplication:
    CuApplication<MmapSectionStorage, cu29_unifiedlog::UnifiedLoggerWrite>
{
}

#[cfg(feature = "std")]
impl<T> CuStdApplication for T where
    T: CuApplication<MmapSectionStorage, cu29_unifiedlog::UnifiedLoggerWrite>
{
}

/// Compile-time subsystem identity embedded in generated Copper applications.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Subsystem {
    id: Option<&'static str>,
    code: u16,
}

impl Subsystem {
    #[inline]
    pub const fn new(id: Option<&'static str>, code: u16) -> Self {
        Self { id, code }
    }

    #[inline]
    pub const fn id(self) -> Option<&'static str> {
        self.id
    }

    #[inline]
    pub const fn code(self) -> u16 {
        self.code
    }
}

/// Compile-time subsystem identity embedded in generated Copper applications.
pub trait CuSubsystemMetadata {
    /// Multi-Copper subsystem identity for this generated application.
    fn subsystem() -> Subsystem;
}

/// A trait that defines the structure and behavior of a CuApplication.
///
/// CuApplication is the normal, running on robot version of an application and its runtime.
///
/// The `CuApplication` trait outlines the necessary functions required for managing an application lifecycle,
/// including configuration management, initialization, task execution, and runtime control. It is meant to be
/// implemented by types that represent specific applications, providing them with unified control and execution features.
///
/// This is the more generic version that allows you to specify a custom unified logger.
pub trait CuApplication<S: SectionStorage, L: UnifiedLogWrite<S> + 'static> {
    /// Returns the original configuration as a string, typically loaded from a RON file.
    /// This configuration represents the default settings for the application before any overrides.
    fn get_original_config() -> String;

    /// Creates a new application.
    ///
    /// # Arguments
    ///
    /// * `clock` - A `RobotClock` instance to be used for time-related operations in the implementing struct.
    /// * `unified_logger` - A thread-safe, shared reference to `UnifiedLoggerWrite`, enabling logging functionalities.
    /// * `config_override` - An optional `CuConfig` instance that allows overriding the default configuration values.
    ///   - If `Some`, the provided configuration will be used.
    ///   - If `None`, the default configuration will be applied.
    ///
    /// # Returns
    ///
    /// A result containing either:
    /// - An instantiated object of the implementing type (`Self`), or
    /// - A `CuResult` error in case of failure during initialization.
    ///
    fn new(
        clock: RobotClock,
        unified_logger: Arc<Mutex<L>>,
        #[cfg(feature = "std")] config_override: Option<CuConfig>, // No config override in no-std, the bundled config is always the config
    ) -> CuResult<Self>
    where
        Self: Sized;

    /// Starts all tasks managed by the application/runtime.
    ///
    /// # Returns
    /// * `Ok(())` - If all tasks are started successfully.
    /// * `Err(CuResult)` - If an error occurs while attempting to start one
    ///   or more tasks.
    fn start_all_tasks(&mut self) -> CuResult<()>;

    /// Executes a single iteration of copper-generated runtime (generating and logging one copperlist)
    ///
    /// # Returns
    ///
    /// * `CuResult<()>` - Returns `Ok(())` if the iteration completes successfully, or an error
    ///   wrapped in `CuResult` if something goes wrong during execution.
    ///
    fn run_one_iteration(&mut self) -> CuResult<()>;

    /// Runs indefinitely looping over run_one_iteration
    ///
    /// # Returns
    ///
    /// Returns a `CuResult<()>`, which indicates the success or failure of the
    /// operation.
    /// - On success, the result is `Ok(())`.
    /// - On failure, an appropriate error wrapped in `CuResult` is returned.
    fn run(&mut self) -> CuResult<()>;

    /// Stops all tasks managed by the application/runtime.
    ///
    /// # Returns
    ///
    /// Returns a `CuResult<()>`, which indicates the success or failure of the
    /// operation.
    /// - On success, the result is `Ok(())`.
    /// - On failure, an appropriate error wrapped in `CuResult` is returned.
    ///
    fn stop_all_tasks(&mut self) -> CuResult<()>;

    /// Restore all tasks from the given frozen state
    fn restore_keyframe(&mut self, freezer: &KeyFrame) -> CuResult<()>;
}

/// A trait that defines the structure and behavior of a simulation-enabled CuApplication.
///
/// CuSimApplication is the simulation version of an application and its runtime, allowing
/// overriding of steps with simulated behavior.
///
/// The `CuSimApplication` trait outlines the necessary functions required for managing an application lifecycle
/// in simulation mode, including configuration management, initialization, task execution, and runtime control.
#[cfg(feature = "std")]
pub trait CuSimApplication<S: SectionStorage, L: UnifiedLogWrite<S> + 'static> {
    /// The type representing a simulation step that can be overridden
    type Step<'z>;

    /// Returns the original configuration as a string, typically loaded from a RON file.
    /// This configuration represents the default settings for the application before any overrides.
    fn get_original_config() -> String;

    /// Creates a new simulation-enabled application.
    ///
    /// # Arguments
    ///
    /// * `clock` - A `RobotClock` instance to be used for time-related operations in the implementing struct.
    /// * `unified_logger` - A thread-safe, shared reference to `UnifiedLoggerWrite`, enabling logging functionalities.
    /// * `config_override` - An optional `CuConfig` instance that allows overriding the default configuration values.
    ///   - If `Some`, the provided configuration will be used.
    ///   - If `None`, the default configuration will be applied.
    /// * `sim_callback` - A mutable function reference that allows overriding individual simulation steps.
    ///   The callback receives a Step parameter and returns a SimOverride indicating how to handle the step.
    ///
    /// # Returns
    ///
    /// A result containing either:
    /// - An instantiated object of the implementing type (`Self`), or
    /// - A `CuResult` error in case of failure during initialization.
    fn new(
        clock: RobotClock,
        unified_logger: Arc<Mutex<L>>,
        config_override: Option<CuConfig>,
        sim_callback: &mut impl for<'z> FnMut(Self::Step<'z>) -> SimOverride,
    ) -> CuResult<Self>
    where
        Self: Sized;

    /// Starts all tasks managed by the application/runtime in simulation mode.
    ///
    /// # Arguments
    /// * `sim_callback` - A mutable function reference that allows overriding individual simulation steps.
    ///
    /// # Returns
    /// * `Ok(())` - If all tasks are started successfully.
    /// * `Err(CuResult)` - If an error occurs while attempting to start one
    ///   or more tasks.
    fn start_all_tasks(
        &mut self,
        sim_callback: &mut impl for<'z> FnMut(Self::Step<'z>) -> SimOverride,
    ) -> CuResult<()>;

    /// Executes a single iteration of copper-generated runtime in simulation mode.
    ///
    /// # Arguments
    /// * `sim_callback` - A mutable function reference that allows overriding individual simulation steps.
    ///
    /// # Returns
    ///
    /// * `CuResult<()>` - Returns `Ok(())` if the iteration completes successfully, or an error
    ///   wrapped in `CuResult` if something goes wrong during execution.
    fn run_one_iteration(
        &mut self,
        sim_callback: &mut impl for<'z> FnMut(Self::Step<'z>) -> SimOverride,
    ) -> CuResult<()>;

    /// Runs indefinitely looping over run_one_iteration in simulation mode
    ///
    /// # Arguments
    /// * `sim_callback` - A mutable function reference that allows overriding individual simulation steps.
    ///
    /// # Returns
    ///
    /// Returns a `CuResult<()>`, which indicates the success or failure of the
    /// operation.
    /// - On success, the result is `Ok(())`.
    /// - On failure, an appropriate error wrapped in `CuResult` is returned.
    fn run(
        &mut self,
        sim_callback: &mut impl for<'z> FnMut(Self::Step<'z>) -> SimOverride,
    ) -> CuResult<()>;

    /// Stops all tasks managed by the application/runtime in simulation mode.
    ///
    /// # Arguments
    /// * `sim_callback` - A mutable function reference that allows overriding individual simulation steps.
    ///
    /// # Returns
    ///
    /// Returns a `CuResult<()>`, which indicates the success or failure of the
    /// operation.
    /// - On success, the result is `Ok(())`.
    /// - On failure, an appropriate error wrapped in `CuResult` is returned.
    fn stop_all_tasks(
        &mut self,
        sim_callback: &mut impl for<'z> FnMut(Self::Step<'z>) -> SimOverride,
    ) -> CuResult<()>;

    /// Restore all tasks from the given frozen state
    fn restore_keyframe(&mut self, freezer: &KeyFrame) -> CuResult<()>;
}

/// Simulation-enabled applications that can replay a recorded CopperList verbatim.
///
/// This is the exact-output replay primitive used by deterministic re-sim flows:
/// task outputs and bridge receives are overridden from the recorded CopperList,
/// bridge sends are skipped, and an optional recorded keyframe can be injected
/// verbatim when the current CL is expected to capture one.
#[cfg(feature = "std")]
pub trait CuRecordedReplayApplication<S: SectionStorage, L: UnifiedLogWrite<S> + 'static>:
    CuSimApplication<S, L>
{
    /// The generated recorded CopperList payload set for this application.
    type RecordedDataSet: CopperListTuple;

    /// Replay one recorded CopperList exactly as logged.
    fn replay_recorded_copperlist(
        &mut self,
        clock_mock: &RobotClockMock,
        copperlist: &CopperList<Self::RecordedDataSet>,
        keyframe: Option<&KeyFrame>,
    ) -> CuResult<()>;
}
