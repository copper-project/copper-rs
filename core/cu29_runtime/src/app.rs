use crate::curuntime::KeyFrame;
use core::fmt;
use core::marker::PhantomData;
use cu29_traits::CopperListTuple;
use cu29_traits::{CuError, CuResult};
use cu29_unifiedlog::{SectionStorage, UnifiedLogWrite};

#[cfg(feature = "std")]
use crate::copperlist::CopperList;
#[cfg(not(feature = "std"))]
use alloc::vec::Vec;
#[cfg(feature = "std")]
use cu29_clock::RobotClockMock;
#[cfg(feature = "std")]
use std::vec::Vec;

#[cfg(not(feature = "std"))]
mod imp {
    pub use alloc::string::String;
}

#[cfg(feature = "std")]
mod imp {
    pub use crate::config::CuConfig;
    pub use crate::simulation::SimOverride;
    pub use cu29_clock::RobotClock;
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

    /// Returns the mission id this generated application is bound to, when applicable.
    fn mission_id() -> Option<&'static str> {
        None
    }

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

/// Optional introspection hook exposing the latest runtime-generated CopperList snapshot.
///
/// This is remote-debug-only: debugger conveniences must not add unconditional
/// runtime-path overhead to normal Copper builds. Non-`remote-debug` builds
/// should implement this as a cheap `None`.
pub trait CurrentRuntimeCopperList<P: CopperListTuple> {
    fn current_runtime_copperlist_bytes(&self) -> Option<&[u8]>;

    fn set_current_runtime_copperlist_bytes(&mut self, snapshot: Option<Vec<u8>>) {
        let _ = snapshot;
    }
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

/// Simulation-enabled applications that can be instantiated for distributed replay.
///
/// This extends exact-output replay with the one extra capability the
/// distributed engine needs: build a replayable app for a specific
/// deployment `instance_id` while keeping app construction type-safe.
#[cfg(feature = "std")]
pub trait CuDistributedReplayApplication<S: SectionStorage, L: UnifiedLogWrite<S> + 'static>:
    CuRecordedReplayApplication<S, L> + CuSubsystemMetadata
{
    /// Build this app for deterministic distributed replay.
    fn build_distributed_replay(
        clock: RobotClock,
        unified_logger: Arc<Mutex<L>>,
        instance_id: u32,
        config_override: Option<CuConfig>,
    ) -> CuResult<Self>
    where
        Self: Sized;
}

/// Typestate marker: the application is built but its tasks have not been started yet.
pub struct Initialized;

/// Typestate marker: tasks are started and iterations can be executed.
pub struct Running;

/// Typestate marker: tasks are stopped. The application can be started again,
/// which is useful to chain missions within the same process.
pub struct Stopped;

/// Typestate marker: a lifecycle transition failed and the application may be
/// partially started or partially stopped. The only way forward is a
/// best-effort cleanup through `stop_all_tasks`.
pub struct Faulted;

mod sealed {
    /// Seals the lifecycle state traits so external code cannot add new states
    /// and bypass the transitions enforced by [`CuAppLifecycle`](super::CuAppLifecycle).
    pub trait Sealed {}
    impl Sealed for super::Initialized {}
    impl Sealed for super::Running {}
    impl Sealed for super::Stopped {}
    impl Sealed for super::Faulted {}
}

/// Lifecycle states from which the application tasks can be started (`Initialized`, `Stopped`).
#[diagnostic::on_unimplemented(
    message = "the application cannot be started from the `{Self}` lifecycle state",
    label = "`start_all_tasks` and `run` require an `Initialized` or `Stopped` application",
    note = "a `Running` application is already started and a `Faulted` application must first be cleaned up with `stop_all_tasks`"
)]
pub trait Startable: sealed::Sealed {}
impl Startable for Initialized {}
impl Startable for Stopped {}

/// Lifecycle states from which the application tasks can be stopped (`Running`, `Faulted`).
#[diagnostic::on_unimplemented(
    message = "the application cannot be stopped from the `{Self}` lifecycle state",
    label = "`stop_all_tasks` requires a `Running` or `Faulted` application",
    note = "start the application first with `start_all_tasks`"
)]
pub trait Stoppable: sealed::Sealed {}
impl Stoppable for Running {}
impl Stoppable for Faulted {}

/// Compile-time enforced lifecycle for a [`CuApplication`].
///
/// Wrapping an application moves lifecycle mistakes (double start, iterating
/// before start, mixing `start_all_tasks` with `run`...) from runtime surprises
/// to compile errors: each state is a distinct type exposing only the
/// transitions that are legal from it.
///
/// ```text
///                 start_all_tasks
///   Initialized ------------------> Running <---+
///        |                            |  |      | run_one_iteration
///        | run                        |  +------+
///        |                            | stop_all_tasks
///        +--------> Stopped <---------+
///                    |   ^
///                    |   | stop_all_tasks (cleanup)
///        (restart)   |  Faulted <--- any failed transition
///                    +---> start_all_tasks / run again
/// ```
///
/// Transitions consume the wrapper and return it typed with the new state, so
/// a stale pre-transition handle cannot be reused. When a transition fails,
/// the application is handed back inside [`LifecycleError`], typed [`Faulted`],
/// so cleanup is still possible and nothing is lost.
///
/// This wrapper is opt-in: the underlying [`CuApplication`] methods remain
/// available on the unwrapped application for existing code.
pub struct CuAppLifecycle<S, L, A, State = Initialized> {
    app: A,
    _lifecycle: PhantomData<(S, L, State)>,
}

impl<S, L, A, State> fmt::Debug for CuAppLifecycle<S, L, A, State> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("CuAppLifecycle")
            .field("state", &core::any::type_name::<State>())
            .finish_non_exhaustive()
    }
}

/// Convenience alias for applications using the default std unified logger,
/// mirroring [`CuStdApplication`].
#[cfg(feature = "std")]
pub type CuStdAppLifecycle<A, State = Initialized> =
    CuAppLifecycle<MmapSectionStorage, cu29_unifiedlog::UnifiedLoggerWrite, A, State>;

/// Result of a lifecycle transition: on success the application typed with its
/// new state, on failure [`LifecycleError`] carrying the application typed [`Faulted`].
pub type TransitionResult<S, L, A, Next> =
    Result<CuAppLifecycle<S, L, A, Next>, LifecycleError<S, L, A>>;

/// A failed lifecycle transition.
///
/// Because transitions consume the application by value, the failure path hands
/// it back typed as [`Faulted`]: `app` only exposes `stop_all_tasks` for
/// best-effort cleanup.
pub struct LifecycleError<S, L, A> {
    /// The error reported by the underlying application.
    pub error: CuError,
    /// The application, in the `Faulted` state. Call `stop_all_tasks` on it to clean up.
    pub app: CuAppLifecycle<S, L, A, Faulted>,
}

impl<S, L, A> fmt::Debug for LifecycleError<S, L, A> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("LifecycleError")
            .field("error", &self.error)
            .finish_non_exhaustive()
    }
}

impl<S, L, A> fmt::Display for LifecycleError<S, L, A> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "lifecycle transition failed: {}", self.error)
    }
}

impl<S, L, A> core::error::Error for LifecycleError<S, L, A> {
    fn source(&self) -> Option<&(dyn core::error::Error + 'static)> {
        Some(&self.error)
    }
}

/// Allows `?` in functions returning `CuResult` when the faulted application
/// does not need to be recovered.
impl<S, L, A> From<LifecycleError<S, L, A>> for CuError {
    fn from(value: LifecycleError<S, L, A>) -> Self {
        value.error
    }
}

impl<S, L, A, State> CuAppLifecycle<S, L, A, State> {
    /// Rewraps the application under a new typestate. Private: state changes
    /// only happen through the lifecycle transitions.
    fn into_state<Next>(self) -> CuAppLifecycle<S, L, A, Next> {
        CuAppLifecycle {
            app: self.app,
            _lifecycle: PhantomData,
        }
    }

    /// Read-only access to the wrapped application.
    pub fn inner(&self) -> &A {
        &self.app
    }

    /// Consumes the wrapper and returns the application, dropping the
    /// compile-time lifecycle tracking.
    pub fn into_inner(self) -> A {
        self.app
    }
}

impl<S, L, A> CuAppLifecycle<S, L, A, Initialized>
where
    S: SectionStorage,
    L: UnifiedLogWrite<S> + 'static,
    A: CuApplication<S, L>,
{
    /// Wraps a freshly built application in the `Initialized` state.
    pub fn new(app: A) -> Self {
        CuAppLifecycle {
            app,
            _lifecycle: PhantomData,
        }
    }
}

impl<S, L, A, State> CuAppLifecycle<S, L, A, State>
where
    S: SectionStorage,
    L: UnifiedLogWrite<S> + 'static,
    A: CuApplication<S, L>,
    State: Startable,
{
    /// Starts all tasks, transitioning to `Running`.
    ///
    /// On failure the application may be partially started; it is returned
    /// typed `Faulted` inside the error so it can be cleaned up.
    pub fn start_all_tasks(mut self) -> TransitionResult<S, L, A, Running> {
        match self.app.start_all_tasks() {
            Ok(()) => Ok(self.into_state()),
            Err(error) => Err(LifecycleError {
                error,
                app: self.into_state(),
            }),
        }
    }

    /// Runs the full lifecycle (start, iterate until shutdown, stop),
    /// transitioning to `Stopped` on success.
    pub fn run(mut self) -> TransitionResult<S, L, A, Stopped> {
        match self.app.run() {
            Ok(()) => Ok(self.into_state()),
            Err(error) => Err(LifecycleError {
                error,
                app: self.into_state(),
            }),
        }
    }
}

impl<S, L, A> CuAppLifecycle<S, L, A, Running>
where
    S: SectionStorage,
    L: UnifiedLogWrite<S> + 'static,
    A: CuApplication<S, L>,
{
    /// Executes one iteration of the runtime. An iteration error does not
    /// change the lifecycle state: the caller decides whether to keep
    /// iterating or stop.
    pub fn run_one_iteration(&mut self) -> CuResult<()> {
        self.app.run_one_iteration()
    }
}

impl<S, L, A, State> CuAppLifecycle<S, L, A, State>
where
    S: SectionStorage,
    L: UnifiedLogWrite<S> + 'static,
    A: CuApplication<S, L>,
    State: Stoppable,
{
    /// Stops all tasks, transitioning to `Stopped`. From `Faulted` this is a
    /// best-effort cleanup. On failure the application is handed back typed
    /// `Faulted` again.
    pub fn stop_all_tasks(mut self) -> TransitionResult<S, L, A, Stopped> {
        match self.app.stop_all_tasks() {
            Ok(()) => Ok(self.into_state()),
            Err(error) => Err(LifecycleError {
                error,
                app: self.into_state(),
            }),
        }
    }
}

#[cfg(all(test, feature = "std"))]
mod lifecycle_tests {
    use super::*;

    #[derive(Default)]
    struct MockApp {
        started: u32,
        stopped: u32,
        iterations: u32,
        runs: u32,
        fail_start: bool,
        fail_stop: bool,
    }

    impl<S: SectionStorage, L: UnifiedLogWrite<S> + 'static> CuApplication<S, L> for MockApp {
        fn get_original_config() -> String {
            String::new()
        }

        fn start_all_tasks(&mut self) -> CuResult<()> {
            if self.fail_start {
                return Err("mock start failure".into());
            }
            self.started += 1;
            Ok(())
        }

        fn run_one_iteration(&mut self) -> CuResult<()> {
            self.iterations += 1;
            Ok(())
        }

        fn run(&mut self) -> CuResult<()> {
            if self.fail_start {
                return Err("mock run failure".into());
            }
            self.runs += 1;
            Ok(())
        }

        fn stop_all_tasks(&mut self) -> CuResult<()> {
            if self.fail_stop {
                return Err("mock stop failure".into());
            }
            self.stopped += 1;
            Ok(())
        }

        fn restore_keyframe(&mut self, _freezer: &KeyFrame) -> CuResult<()> {
            Ok(())
        }
    }

    type Lifecycle = CuStdAppLifecycle<MockApp>;

    #[test]
    fn full_cycle_with_restart() {
        let app = Lifecycle::new(MockApp::default());
        let mut running = app.start_all_tasks().unwrap();
        running.run_one_iteration().unwrap();
        running.run_one_iteration().unwrap();
        let stopped = running.stop_all_tasks().unwrap();

        // Restarting a stopped application is legal (mission chaining).
        let running = stopped.start_all_tasks().unwrap();
        let stopped = running.stop_all_tasks().unwrap();

        let mock = stopped.into_inner();
        assert_eq!(mock.started, 2);
        assert_eq!(mock.iterations, 2);
        assert_eq!(mock.stopped, 2);
    }

    #[test]
    fn run_transitions_to_stopped_and_can_run_again() {
        let app = Lifecycle::new(MockApp::default());
        let stopped = app.run().unwrap();
        let stopped = stopped.run().unwrap();
        assert_eq!(stopped.inner().runs, 2);
    }

    #[test]
    fn failed_start_hands_back_a_faulted_app_for_cleanup() {
        let app = Lifecycle::new(MockApp {
            fail_start: true,
            ..Default::default()
        });
        let err = app.start_all_tasks().unwrap_err();
        assert!(err.error.to_string().contains("mock start failure"));

        // The only legal transition from Faulted is the cleanup.
        let stopped = err.app.stop_all_tasks().unwrap();
        let mock = stopped.into_inner();
        assert_eq!(mock.started, 0);
        assert_eq!(mock.stopped, 1);
    }

    #[test]
    fn failed_stop_hands_back_a_faulted_app() {
        let app = Lifecycle::new(MockApp {
            fail_stop: true,
            ..Default::default()
        });
        let running = app.start_all_tasks().unwrap();
        let err = running.stop_all_tasks().unwrap_err();
        assert!(err.error.to_string().contains("mock stop failure"));
    }

    #[test]
    fn lifecycle_error_propagates_with_question_mark() {
        fn drive() -> CuResult<()> {
            let app = Lifecycle::new(MockApp {
                fail_start: true,
                ..Default::default()
            });
            let _running = app.start_all_tasks()?;
            Ok(())
        }
        let error = drive().unwrap_err();
        assert!(error.to_string().contains("mock start failure"));
    }
}
