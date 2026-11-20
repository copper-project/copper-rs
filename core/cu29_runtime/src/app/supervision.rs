use super::*;
use alloc::{format, string::ToString};
use cu29_clock::{ClockProvider, CuDuration};

/// Reason a managed application loop returned.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CuRunExit {
    Requested,
    Shutdown,
}

/// Optional capability of generated live applications.
pub trait CuManagedApplication<S: SectionStorage, L: UnifiedLogWrite<S> + 'static>:
    CuApplication<S, L>
{
    /// Drive an already-started app. Check shutdown before the caller's predicate
    /// at each iteration boundary. Return with the app still running.
    fn run_managed(&mut self, should_stop: impl FnMut() -> bool) -> CuResult<CuRunExit>;
}

impl<S, L, A> CuAppLifecycle<S, L, A, Running>
where
    S: SectionStorage,
    L: UnifiedLogWrite<S> + 'static,
    A: CuManagedApplication<S, L>,
{
    /// Run on this thread until `should_stop` returns true or Copper requests shutdown.
    /// The predicate must be nonblocking. The app remains running on return.
    pub fn run_until(&mut self, should_stop: impl FnMut() -> bool) -> CuResult<CuRunExit> {
        self.app.run_managed(should_stop)
    }
}

impl<S, L, A> CuAppLifecycle<S, L, A, Running>
where
    S: SectionStorage,
    L: UnifiedLogWrite<S> + 'static,
    A: CuManagedApplication<S, L> + ClockProvider,
{
    fn wait_for(
        &mut self,
        timeout: CuDuration,
        mut observed: impl FnMut() -> bool,
    ) -> Result<(), CuWaitError> {
        let clock = self.app.get_clock();
        let start = clock.raw_now();
        let expired = || clock.raw_now() - start >= timeout;
        let mut passed = false;
        let mut first = true;
        let exit = self
            .run_until(|| {
                if !first && expired() {
                    return true;
                }
                first = false;
                passed = observed();
                passed || expired()
            })
            .map_err(CuWaitError::Execution)?;
        match exit {
            CuRunExit::Shutdown => Err(CuWaitError::Shutdown),
            CuRunExit::Requested if passed => Ok(()),
            CuRunExit::Requested => Err(CuWaitError::Timeout),
        }
    }

    /// Run iterations until the device reports readiness, the deadline expires, or Copper requests shutdown.
    pub fn wait_until_ready(
        &mut self,
        timeout: CuDuration,
        ready: impl FnMut() -> bool,
    ) -> Result<(), CuWaitError> {
        self.wait_for(timeout, ready)
    }

    /// Run iterations until the device finishes draining, the deadline expires, or Copper requests shutdown.
    pub fn drain_until(
        &mut self,
        timeout: CuDuration,
        drained: impl FnMut() -> bool,
    ) -> Result<(), CuWaitError> {
        self.wait_for(timeout, drained)
    }
}

pub(super) fn cleanup<T>(result: CuResult<T>, stopped: CuResult<()>) -> CuResult<T> {
    match (result, stopped) {
        (Err(error), Err(stop)) => Err(CuError::from(format!(
            "{error}; cleanup also failed: {stop}"
        ))),
        (Err(error), _) | (_, Err(error)) => Err(error),
        (Ok(value), Ok(())) => Ok(value),
    }
}

#[doc(hidden)]
pub fn start_managed<S, L, A>(
    app: CuAppLifecycle<S, L, A>,
) -> CuResult<CuAppLifecycle<S, L, A, Running>>
where
    S: SectionStorage,
    L: UnifiedLogWrite<S> + 'static,
    A: CuApplication<S, L>,
{
    match app.start() {
        Ok(running) => Ok(running),
        Err(failed) => cleanup(
            Err(failed.error),
            failed.app.stop().map(drop).map_err(|failed| failed.error),
        ),
    }
}

/// Failure while iterating towards a readiness or drain barrier.
#[derive(Debug)]
pub enum CuWaitError {
    Timeout,
    Shutdown,
    Execution(CuError),
}

impl fmt::Display for CuWaitError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Timeout => f.write_str("mission observation timed out"),
            Self::Shutdown => f.write_str("shutdown during mission observation"),
            Self::Execution(error) => write!(f, "{error}"),
        }
    }
}

impl core::error::Error for CuWaitError {}

impl From<CuWaitError> for CuError {
    fn from(error: CuWaitError) -> Self {
        match error {
            CuWaitError::Execution(error) => error,
            other => other.to_string().into(),
        }
    }
}

/// Implemented by the generated enum of running mission apps.
#[doc(hidden)]
pub trait ActiveMission {
    fn run_until(&mut self, requested: impl FnMut() -> bool) -> CuResult<CuRunExit>;
    fn stop(self) -> CuResult<()>;
}
