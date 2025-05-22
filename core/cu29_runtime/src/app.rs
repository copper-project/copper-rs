use crate::config::CuConfig;
use crate::simulation::SimOverride;
use cu29_clock::RobotClock;
use cu29_traits::CuResult;
use cu29_unifiedlog::UnifiedLoggerWrite;
use std::sync::{Arc, Mutex};

/// A trait that defines the structure and behavior of a CuApplication.
///
/// CuApplication is the normal, running on robot version of an application and its runtime.
///
/// The `CuApplication` trait outlines the necessary functions required for managing an application lifecycle,
/// including configuration management, initialization, task execution, and runtime control. It is meant to be
/// implemented by types that represent specific applications, providing them with unified control and execution features.
///
pub trait CuApplication {
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
        unified_logger: Arc<Mutex<UnifiedLoggerWrite>>,
        config_override: Option<CuConfig>,
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
}

/// A trait that defines the structure and behavior of a simulation-enabled CuApplication.
///
/// CuSimApplication is the simulation version of an application and its runtime, allowing
/// overriding of steps with simulated behavior.
///
/// The `CuSimApplication` trait outlines the necessary functions required for managing an application lifecycle
/// in simulation mode, including configuration management, initialization, task execution, and runtime control.
pub trait CuSimApplication {
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
        unified_logger: Arc<Mutex<UnifiedLoggerWrite>>,
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
}
