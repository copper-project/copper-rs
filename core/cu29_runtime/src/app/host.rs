use super::supervision::{cleanup, start_managed};
use super::*;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread::{self, JoinHandle};

/// Nonblocking stop request. Completion is acknowledged by joining the owner.
#[derive(Clone)]
pub struct CuRunnerControl {
    stop: Arc<AtomicBool>,
    owner: thread::Thread,
}

impl CuRunnerControl {
    pub fn request_stop(&self) {
        self.stop.store(true, Ordering::Release);
        self.owner.unpark();
    }
}

/// Owns one Copper thread. Drop requests stop and blocks until cleanup completes.
pub struct CuRunner {
    control: CuRunnerControl,
    owner: Option<JoinHandle<CuResult<CuRunExit>>>,
}

impl CuRunner {
    fn launch(
        work: impl FnOnce(&AtomicBool) -> CuResult<CuRunExit> + Send + 'static,
    ) -> CuResult<Self> {
        let stop = Arc::new(AtomicBool::new(false));
        let worker_stop = stop.clone();
        let owner = thread::Builder::new()
            .name("copper".into())
            .spawn(move || work(&worker_stop))
            .map_err(|error| CuError::from(error.to_string()))?;
        Ok(Self {
            control: CuRunnerControl {
                stop,
                owner: owner.thread().clone(),
            },
            owner: Some(owner),
        })
    }

    /// Build, start, run, stop and destroy the app on one thread. Only the factory crosses threads.
    pub fn spawn<S, L, A>(
        factory: impl FnOnce() -> CuResult<CuAppLifecycle<S, L, A>> + Send + 'static,
    ) -> CuResult<Self>
    where
        S: SectionStorage,
        L: UnifiedLogWrite<S> + 'static,
        A: CuManagedApplication<S, L>,
    {
        Self::launch(move |stop| {
            let mut running = start_managed(factory()?)?;
            let result = running.run_until(|| stop.load(Ordering::Acquire));
            cleanup(
                result,
                running.stop().map(drop).map_err(|failed| failed.error),
            )
        })
    }

    pub fn control(&self) -> CuRunnerControl {
        self.control.clone()
    }

    /// Block until resource release and return execution and cleanup failures.
    pub fn join(mut self) -> CuResult<CuRunExit> {
        self.owner
            .take()
            .expect("owned join handle")
            .join()
            .map_err(|panic| {
                CuError::from(crate::monitoring::panic_payload_to_string(panic.as_ref()))
            })?
    }
}

impl Drop for CuRunner {
    fn drop(&mut self) {
        if let Some(thread) = self.owner.take() {
            self.control.request_stop();
            let _ = thread.join();
        }
    }
}
