use super::*;

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

/// Failure while iterating towards a readiness or drain barrier.
#[cfg(feature = "std")]
#[derive(Debug)]
pub enum CuWaitError {
    Timeout,
    Shutdown,
    Execution(CuError),
}

#[cfg(feature = "std")]
impl fmt::Display for CuWaitError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Timeout => f.write_str("mission observation timed out"),
            Self::Shutdown => f.write_str("shutdown during mission observation"),
            Self::Execution(error) => write!(f, "{error}"),
        }
    }
}

#[cfg(feature = "std")]
impl std::error::Error for CuWaitError {}

#[cfg(feature = "std")]
impl From<CuWaitError> for CuError {
    fn from(error: CuWaitError) -> Self {
        match error {
            CuWaitError::Execution(error) => error,
            other => other.to_string().into(),
        }
    }
}

/// Device observations and deadlines for one mission build. All hooks run on
/// the owner thread. Observations must be quick and nonblocking.
#[cfg(feature = "std")]
pub trait CuMissionPolicy {
    fn ready(&mut self) -> bool;
    fn readiness_timeout(&self) -> core::time::Duration;
    fn request_drain(&mut self);
    fn drained(&mut self) -> bool;
    fn drain_timeout(&self) -> core::time::Duration;
}
