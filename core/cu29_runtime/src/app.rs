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
    pub use alloc::boxed::Box;
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
    #[deprecated(
        since = "1.2.0",
        note = "use the typed transition `start()` on the handle returned by `build()`"
    )]
    fn start_all_tasks(&mut self) -> CuResult<()>;

    /// Executes a single iteration of copper-generated runtime (generating and logging one copperlist)
    ///
    /// # Returns
    ///
    /// * `CuResult<()>` - Returns `Ok(())` if the iteration completes successfully, or an error
    ///   wrapped in `CuResult` if something goes wrong during execution.
    ///
    #[deprecated(
        since = "1.2.0",
        note = "use `run_one_iteration()` on the Running handle from `build()?.start()?`"
    )]
    fn run_one_iteration(&mut self) -> CuResult<()>;

    /// Runs indefinitely looping over run_one_iteration
    ///
    /// # Returns
    ///
    /// Returns a `CuResult<()>`, which indicates the success or failure of the
    /// operation.
    /// - On success, the result is `Ok(())`.
    /// - On failure, an appropriate error wrapped in `CuResult` is returned.
    #[deprecated(
        since = "1.2.0",
        note = "use the typed transition `run_until_shutdown()` on the handle returned by `build()`"
    )]
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
    #[deprecated(
        since = "1.2.0",
        note = "use the typed transition `stop()` on the Running handle"
    )]
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
    #[deprecated(
        since = "1.2.0",
        note = "use the typed transition `start()` on the handle returned by `build()`"
    )]
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
    #[deprecated(
        since = "1.2.0",
        note = "use `run_one_iteration()` on the Running handle from `build()?.start()?`"
    )]
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
    #[deprecated(
        since = "1.2.0",
        note = "use the typed transition `run_until_shutdown()` on the handle returned by `build()`"
    )]
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
    #[deprecated(
        since = "1.2.0",
        note = "use the typed transition `stop()` on the Running handle"
    )]
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
    ///
    /// Generated implementations validate continuity, drain prior asynchronous
    /// output, and align the allocator to the recorded id before executing the
    /// captured-output callback. This is an offline operation and may wait for
    /// logging workers; it is not a real-time task-path API. A matching keyframe
    /// permits a jump across missing history and preserves its recorded timing.
    /// Call [`CuSimApplication::restore_keyframe`] separately when restoring task
    /// state at that boundary.
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
///                      start
///   Initialized ------------------> Running <---+
///        |                            |  |      | run_one_iteration
///        | run_until_shutdown         |  +------+
///        |                            | stop
///        +--------> Stopped <---------+
///                    |   ^
///                    |   | stop (cleanup)
///        (restart)   |  Faulted <--- any failed transition
///                    +---> start / run_until_shutdown again
/// ```
///
/// Transitions consume the wrapper and return it typed with the new state, so
/// a stale pre-transition handle cannot be reused. When a transition fails,
/// the application is handed back inside [`LifecycleError`], typed [`Faulted`],
/// so cleanup is still possible and nothing is lost.
///
/// This is the default construction path: the generated application builder
/// hands one out from `build_app()`. The raw [`CuApplication`] methods remain
/// available (deprecated on the generated application) for framework-level
/// harnesses such as replay engines; [`into_inner`](CuAppLifecycle::into_inner)
/// drops back to that level when needed.
///
/// The application is boxed internally: generated applications embed their
/// copperlist pools and can be megabytes, so the consuming transitions move a
/// pointer instead of the application itself (which in debug builds would
/// blow through a test thread's stack).
pub struct CuAppLifecycle<S, L, A, State = Initialized> {
    app: Box<A>,
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
        *self.app
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
            app: Box::new(app),
            _lifecycle: PhantomData,
        }
    }

    /// Mutable access to the wrapped application, only before it is started.
    ///
    /// This exists for pre-start configuration (attaching monitors,
    /// inspecting the runtime...). Once started, only the read-only
    /// [`inner`](CuAppLifecycle::inner) view remains available so the
    /// lifecycle transitions cannot be bypassed mid-flight.
    pub fn inner_mut(&mut self) -> &mut A {
        &mut self.app
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
    #[allow(deprecated)] // forwards to the raw trait, deprecated for direct use only
    pub fn start(mut self) -> TransitionResult<S, L, A, Running> {
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
    #[allow(deprecated)] // forwards to the raw trait, deprecated for direct use only
    pub fn run_until_shutdown(mut self) -> TransitionResult<S, L, A, Stopped> {
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
    #[allow(deprecated)] // forwards to the raw trait, deprecated for direct use only
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
    #[allow(deprecated)] // forwards to the raw trait, deprecated for direct use only
    pub fn stop(mut self) -> TransitionResult<S, L, A, Stopped> {
        match self.app.stop_all_tasks() {
            Ok(()) => Ok(self.into_state()),
            Err(error) => Err(LifecycleError {
                error,
                app: self.into_state(),
            }),
        }
    }
}

/// Read access to the wrapped application in every lifecycle state, so
/// conveniences like `app.clock()` keep working without ceremony.
impl<S, L, A, State> core::ops::Deref for CuAppLifecycle<S, L, A, State> {
    type Target = A;

    fn deref(&self) -> &A {
        &self.app
    }
}

/// Mutable access only before the application is started: after a typed
/// start, raw mutable access could bypass the lifecycle guarantees.
impl<S, L, A> core::ops::DerefMut for CuAppLifecycle<S, L, A, Initialized> {
    fn deref_mut(&mut self) -> &mut A {
        &mut self.app
    }
}

/// Legacy escape hatch: the raw (deprecated) lifecycle API remains callable
/// on what `build()` returns, so pre-typestate code compiles unchanged and
/// only picks up deprecation warnings. Raw calls do not advance the
/// typestate; mixing them with typed transitions is outside the typestate
/// guarantees.
#[allow(deprecated)]
impl<S, L, A> CuApplication<S, L> for CuAppLifecycle<S, L, A, Initialized>
where
    S: SectionStorage,
    L: UnifiedLogWrite<S> + 'static,
    A: CuApplication<S, L>,
{
    fn get_original_config() -> String {
        A::get_original_config()
    }

    fn start_all_tasks(&mut self) -> CuResult<()> {
        self.app.start_all_tasks()
    }

    fn run_one_iteration(&mut self) -> CuResult<()> {
        self.app.run_one_iteration()
    }

    fn run(&mut self) -> CuResult<()> {
        self.app.run()
    }

    fn stop_all_tasks(&mut self) -> CuResult<()> {
        self.app.stop_all_tasks()
    }

    fn restore_keyframe(&mut self, freezer: &KeyFrame) -> CuResult<()> {
        self.app.restore_keyframe(freezer)
    }
}

/// Compile-time enforced lifecycle for a [`CuSimApplication`].
///
/// Simulation counterpart of [`CuAppLifecycle`]: same states and transitions,
/// with each transition threading the `sim_callback` the simulation runtime
/// requires. Replay primitives (`replay_recorded_copperlist`,
/// `restore_keyframe`) are framework-level and stay on the raw traits.
#[cfg(feature = "std")]
pub struct CuSimAppLifecycle<S, L, A, State = Initialized> {
    app: Box<A>,
    _lifecycle: PhantomData<(S, L, State)>,
}

#[cfg(feature = "std")]
impl<S, L, A, State> fmt::Debug for CuSimAppLifecycle<S, L, A, State> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("CuSimAppLifecycle")
            .field("state", &core::any::type_name::<State>())
            .finish_non_exhaustive()
    }
}

/// Convenience alias for simulation applications using the default std
/// unified logger, mirroring [`CuStdAppLifecycle`].
#[cfg(feature = "std")]
pub type CuStdSimAppLifecycle<A, State = Initialized> =
    CuSimAppLifecycle<MmapSectionStorage, cu29_unifiedlog::UnifiedLoggerWrite, A, State>;

/// Result of a simulation lifecycle transition: on success the application
/// typed with its new state, on failure [`SimLifecycleError`] carrying the
/// application typed [`Faulted`].
#[cfg(feature = "std")]
pub type SimTransitionResult<S, L, A, Next> =
    Result<CuSimAppLifecycle<S, L, A, Next>, SimLifecycleError<S, L, A>>;

/// A failed simulation lifecycle transition, mirroring [`LifecycleError`].
#[cfg(feature = "std")]
pub struct SimLifecycleError<S, L, A> {
    /// The error reported by the underlying application.
    pub error: CuError,
    /// The application, in the `Faulted` state. Call `stop_all_tasks` on it to clean up.
    pub app: CuSimAppLifecycle<S, L, A, Faulted>,
}

#[cfg(feature = "std")]
impl<S, L, A> fmt::Debug for SimLifecycleError<S, L, A> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("SimLifecycleError")
            .field("error", &self.error)
            .finish_non_exhaustive()
    }
}

#[cfg(feature = "std")]
impl<S, L, A> fmt::Display for SimLifecycleError<S, L, A> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "lifecycle transition failed: {}", self.error)
    }
}

#[cfg(feature = "std")]
impl<S, L, A> core::error::Error for SimLifecycleError<S, L, A> {
    fn source(&self) -> Option<&(dyn core::error::Error + 'static)> {
        Some(&self.error)
    }
}

/// Allows `?` in functions returning `CuResult` when the faulted application
/// does not need to be recovered.
#[cfg(feature = "std")]
impl<S, L, A> From<SimLifecycleError<S, L, A>> for CuError {
    fn from(value: SimLifecycleError<S, L, A>) -> Self {
        value.error
    }
}

#[cfg(feature = "std")]
impl<S, L, A, State> CuSimAppLifecycle<S, L, A, State> {
    /// Rewraps the application under a new typestate. Private: state changes
    /// only happen through the lifecycle transitions.
    fn into_state<Next>(self) -> CuSimAppLifecycle<S, L, A, Next> {
        CuSimAppLifecycle {
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
        *self.app
    }
}

#[cfg(feature = "std")]
impl<S, L, A> CuSimAppLifecycle<S, L, A, Initialized>
where
    S: SectionStorage,
    L: UnifiedLogWrite<S> + 'static,
    A: CuSimApplication<S, L>,
{
    /// Wraps a freshly built simulation application in the `Initialized` state.
    pub fn new(app: A) -> Self {
        CuSimAppLifecycle {
            app: Box::new(app),
            _lifecycle: PhantomData,
        }
    }
}

#[cfg(feature = "std")]
impl<S, L, A, State> CuSimAppLifecycle<S, L, A, State>
where
    S: SectionStorage,
    L: UnifiedLogWrite<S> + 'static,
    A: CuSimApplication<S, L>,
    State: Startable,
{
    /// Starts all tasks, transitioning to `Running`.
    ///
    /// On failure the application may be partially started; it is returned
    /// typed `Faulted` inside the error so it can be cleaned up.
    #[allow(deprecated)] // forwards to the raw trait, deprecated for direct use only
    pub fn start(
        mut self,
        sim_callback: &mut impl for<'z> FnMut(<A as CuSimApplication<S, L>>::Step<'z>) -> SimOverride,
    ) -> SimTransitionResult<S, L, A, Running> {
        match self.app.start_all_tasks(sim_callback) {
            Ok(()) => Ok(self.into_state()),
            Err(error) => Err(SimLifecycleError {
                error,
                app: self.into_state(),
            }),
        }
    }

    /// Runs the full lifecycle (start, iterate until shutdown, stop),
    /// transitioning to `Stopped` on success.
    #[allow(deprecated)] // forwards to the raw trait, deprecated for direct use only
    pub fn run_until_shutdown(
        mut self,
        sim_callback: &mut impl for<'z> FnMut(<A as CuSimApplication<S, L>>::Step<'z>) -> SimOverride,
    ) -> SimTransitionResult<S, L, A, Stopped> {
        match self.app.run(sim_callback) {
            Ok(()) => Ok(self.into_state()),
            Err(error) => Err(SimLifecycleError {
                error,
                app: self.into_state(),
            }),
        }
    }
}

#[cfg(feature = "std")]
impl<S, L, A> CuSimAppLifecycle<S, L, A, Running>
where
    S: SectionStorage,
    L: UnifiedLogWrite<S> + 'static,
    A: CuSimApplication<S, L>,
{
    /// Executes one iteration of the runtime. An iteration error does not
    /// change the lifecycle state: the caller decides whether to keep
    /// iterating or stop.
    #[allow(deprecated)] // forwards to the raw trait, deprecated for direct use only
    pub fn run_one_iteration(
        &mut self,
        sim_callback: &mut impl for<'z> FnMut(<A as CuSimApplication<S, L>>::Step<'z>) -> SimOverride,
    ) -> CuResult<()> {
        self.app.run_one_iteration(sim_callback)
    }
}

#[cfg(feature = "std")]
impl<S, L, A, State> CuSimAppLifecycle<S, L, A, State>
where
    S: SectionStorage,
    L: UnifiedLogWrite<S> + 'static,
    A: CuSimApplication<S, L>,
    State: Stoppable,
{
    /// Stops all tasks, transitioning to `Stopped`. From `Faulted` this is a
    /// best-effort cleanup. On failure the application is handed back typed
    /// `Faulted` again.
    #[allow(deprecated)] // forwards to the raw trait, deprecated for direct use only
    pub fn stop(
        mut self,
        sim_callback: &mut impl for<'z> FnMut(<A as CuSimApplication<S, L>>::Step<'z>) -> SimOverride,
    ) -> SimTransitionResult<S, L, A, Stopped> {
        match self.app.stop_all_tasks(sim_callback) {
            Ok(()) => Ok(self.into_state()),
            Err(error) => Err(SimLifecycleError {
                error,
                app: self.into_state(),
            }),
        }
    }
}

/// Read access to the wrapped application in every lifecycle state.
#[cfg(feature = "std")]
impl<S, L, A, State> core::ops::Deref for CuSimAppLifecycle<S, L, A, State> {
    type Target = A;

    fn deref(&self) -> &A {
        &self.app
    }
}

/// Mutable access only before the application is started, mirroring
/// [`CuAppLifecycle`].
#[cfg(feature = "std")]
impl<S, L, A> core::ops::DerefMut for CuSimAppLifecycle<S, L, A, Initialized> {
    fn deref_mut(&mut self) -> &mut A {
        &mut self.app
    }
}

/// Legacy escape hatch mirroring the [`CuApplication`] impl on
/// [`CuAppLifecycle`]: pre-typestate simulation code compiles unchanged
/// against what `build()` returns and only picks up deprecation warnings.
#[cfg(feature = "std")]
#[allow(deprecated)]
impl<S, L, A> CuSimApplication<S, L> for CuSimAppLifecycle<S, L, A, Initialized>
where
    S: SectionStorage,
    L: UnifiedLogWrite<S> + 'static,
    A: CuSimApplication<S, L>,
{
    type Step<'z> = <A as CuSimApplication<S, L>>::Step<'z>;

    fn get_original_config() -> String {
        A::get_original_config()
    }

    fn mission_id() -> Option<&'static str> {
        A::mission_id()
    }

    fn start_all_tasks(
        &mut self,
        sim_callback: &mut impl for<'z> FnMut(Self::Step<'z>) -> SimOverride,
    ) -> CuResult<()> {
        self.app.start_all_tasks(sim_callback)
    }

    fn run_one_iteration(
        &mut self,
        sim_callback: &mut impl for<'z> FnMut(Self::Step<'z>) -> SimOverride,
    ) -> CuResult<()> {
        self.app.run_one_iteration(sim_callback)
    }

    fn run(
        &mut self,
        sim_callback: &mut impl for<'z> FnMut(Self::Step<'z>) -> SimOverride,
    ) -> CuResult<()> {
        self.app.run(sim_callback)
    }

    fn stop_all_tasks(
        &mut self,
        sim_callback: &mut impl for<'z> FnMut(Self::Step<'z>) -> SimOverride,
    ) -> CuResult<()> {
        self.app.stop_all_tasks(sim_callback)
    }

    fn restore_keyframe(&mut self, freezer: &KeyFrame) -> CuResult<()> {
        self.app.restore_keyframe(freezer)
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

    #[allow(deprecated)] // mocks implement the deprecated raw trait methods
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
        let mut running = app.start().unwrap();
        running.run_one_iteration().unwrap();
        running.run_one_iteration().unwrap();
        let stopped = running.stop().unwrap();

        // Restarting a stopped application is legal (mission chaining).
        let running = stopped.start().unwrap();
        let stopped = running.stop().unwrap();

        let mock = stopped.into_inner();
        assert_eq!(mock.started, 2);
        assert_eq!(mock.iterations, 2);
        assert_eq!(mock.stopped, 2);
    }

    #[test]
    fn run_transitions_to_stopped_and_can_run_again() {
        let app = Lifecycle::new(MockApp::default());
        let stopped = app.run_until_shutdown().unwrap();
        let stopped = stopped.run_until_shutdown().unwrap();
        assert_eq!(stopped.inner().runs, 2);
    }

    #[test]
    fn failed_start_hands_back_a_faulted_app_for_cleanup() {
        let app = Lifecycle::new(MockApp {
            fail_start: true,
            ..Default::default()
        });
        let err = app.start().unwrap_err();
        assert!(err.error.to_string().contains("mock start failure"));

        // The only legal transition from Faulted is the cleanup.
        let stopped = err.app.stop().unwrap();
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
        let running = app.start().unwrap();
        let err = running.stop().unwrap_err();
        assert!(err.error.to_string().contains("mock stop failure"));
    }

    #[test]
    fn lifecycle_error_propagates_with_question_mark() {
        fn drive() -> CuResult<()> {
            let app = Lifecycle::new(MockApp {
                fail_start: true,
                ..Default::default()
            });
            let _running = app.start()?;
            Ok(())
        }
        let error = drive().unwrap_err();
        assert!(error.to_string().contains("mock start failure"));
    }

    #[derive(Default)]
    struct MockSimApp {
        started: u32,
        stopped: u32,
        iterations: u32,
        fail_start: bool,
    }

    struct MockStep;

    #[allow(deprecated)] // mocks implement the deprecated raw trait methods
    impl<S: SectionStorage, L: UnifiedLogWrite<S> + 'static> CuSimApplication<S, L> for MockSimApp {
        type Step<'z> = MockStep;

        fn get_original_config() -> String {
            String::new()
        }

        fn start_all_tasks(
            &mut self,
            sim_callback: &mut impl for<'z> FnMut(Self::Step<'z>) -> SimOverride,
        ) -> CuResult<()> {
            if self.fail_start {
                return Err("mock sim start failure".into());
            }
            let _ = sim_callback(MockStep);
            self.started += 1;
            Ok(())
        }

        fn run_one_iteration(
            &mut self,
            sim_callback: &mut impl for<'z> FnMut(Self::Step<'z>) -> SimOverride,
        ) -> CuResult<()> {
            let _ = sim_callback(MockStep);
            self.iterations += 1;
            Ok(())
        }

        fn run(
            &mut self,
            sim_callback: &mut impl for<'z> FnMut(Self::Step<'z>) -> SimOverride,
        ) -> CuResult<()> {
            let _ = sim_callback(MockStep);
            Ok(())
        }

        fn stop_all_tasks(
            &mut self,
            sim_callback: &mut impl for<'z> FnMut(Self::Step<'z>) -> SimOverride,
        ) -> CuResult<()> {
            let _ = sim_callback(MockStep);
            self.stopped += 1;
            Ok(())
        }

        fn restore_keyframe(&mut self, _freezer: &KeyFrame) -> CuResult<()> {
            Ok(())
        }
    }

    type SimLifecycle = CuStdSimAppLifecycle<MockSimApp>;

    #[test]
    fn sim_full_cycle_threads_the_callback() {
        let mut callback_calls = 0u32;
        let mut cb = |_step: MockStep| -> SimOverride {
            callback_calls += 1;
            SimOverride::ExecuteByRuntime
        };

        let app = SimLifecycle::new(MockSimApp::default());
        let mut running = app.start(&mut cb).unwrap();
        running.run_one_iteration(&mut cb).unwrap();
        let stopped = running.stop(&mut cb).unwrap();

        let mock = stopped.into_inner();
        assert_eq!(mock.started, 1);
        assert_eq!(mock.iterations, 1);
        assert_eq!(mock.stopped, 1);
        assert_eq!(callback_calls, 3);
    }

    #[test]
    fn sim_failed_start_hands_back_a_faulted_app_for_cleanup() {
        let mut cb = |_step: MockStep| -> SimOverride { SimOverride::ExecuteByRuntime };
        let app = SimLifecycle::new(MockSimApp {
            fail_start: true,
            ..Default::default()
        });
        let err = app.start(&mut cb).unwrap_err();
        assert!(err.error.to_string().contains("mock sim start failure"));
        let stopped = err.app.stop(&mut cb).unwrap();
        assert_eq!(stopped.inner().stopped, 1);
    }
}
