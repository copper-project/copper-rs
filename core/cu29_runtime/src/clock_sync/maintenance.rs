//! Experimental lifecycle integration for a single typed reference clock.
//!
//! Linux reference I/O runs on a worker. Embedded references are polled between
//! iterations and must bound their own work. Consumers start only after lock.

use super::ClockSyncRecord;
use crate::resource::ResourceBundleDecl;
use alloc::boxed::Box;
use core::time::Duration;
use cu29_clock::RobotClock;
use cu29_clock::sync::{
    ClockDomain, ClockObservation, ClockSync, SyncConfig, SyncError, SyncState, SyncStatus,
};
use cu29_traits::{CuError, CuResult};

/// Experimental reference provider implemented by Linux adapters and BSPs.
pub trait ClockReference: Send + Sync + 'static {
    /// Constructs the execution counter when no explicit clock was injected.
    fn create_clock(&self) -> CuResult<RobotClock>;
    /// Starts reference service handles before acquisition.
    fn start(&mut self) -> CuResult<()>;
    /// Current shared epoch/session, also reported in every observation.
    fn domain(&self) -> ClockDomain;
    /// Polls outside task process(), returning a fresh capture when usable.
    fn poll(&mut self, clock: &RobotClock) -> CuResult<Option<ClockObservation>>;
    /// Stops the controller before releasing any transport handles.
    fn stop(&mut self) -> CuResult<()>;
}

/// Experimental resource contract: a statically resolved owned reference slot.
pub trait ClockReferenceBundle: ResourceBundleDecl {
    /// Concrete type registered at the configured reference slot.
    type Reference: ClockReference;
}

fn sync_error(error: SyncError) -> CuError {
    CuError::new_with_cause("Clock synchronization failed", error)
}

/// Experimental maintenance timing, measured against the undisciplined counter.
#[derive(Clone, Copy, Debug)]
pub struct MaintenanceConfig {
    /// Raw-time acquisition deadline.
    pub acquisition_timeout: Duration,
    /// Reference sampling cadence.
    pub sample_interval: Duration,
    /// Discipline and uncertainty bounds.
    pub sync: SyncConfig,
}

impl MaintenanceConfig {
    fn validate(self) -> CuResult<()> {
        if self.acquisition_timeout.is_zero()
            || self.sample_interval.is_zero()
            || self.acquisition_timeout.as_nanos() > u128::from(u64::MAX)
            || self.sample_interval.as_nanos() > u128::from(u64::MAX)
        {
            return Err(CuError::from(
                "Clock maintenance durations must be positive u64 nanoseconds",
            ));
        }
        Ok(())
    }
}

#[cfg(feature = "std")]
struct Worker {
    // Preserve generated applications' Sync bound. Access uses get_mut(), which
    // requires exclusive ownership and never acquires the mutex on the RT path.
    rx: std::sync::Mutex<rtrb::Consumer<Option<ClockObservation>>>,
    stop: alloc::sync::Arc<portable_atomic::AtomicBool>,
    thread: Option<std::thread::JoinHandle<CuResult<Box<dyn ClockReference>>>>,
}

/// Runtime-owned reference and discipline state, polled outside process().
#[doc(hidden)]
pub struct ClockMaintenance {
    reference: Option<Box<dyn ClockReference>>,
    clock: RobotClock,
    sync: ClockSync,
    config: MaintenanceConfig,
    next_sample: u64,
    started: bool,
    changed: bool,
    record_pending: bool,
    #[cfg(feature = "std")]
    worker: Option<Worker>,
}

impl ClockMaintenance {
    /// Attaches the provider to an execution clock before constructing consumers.
    pub fn new(
        reference: Box<dyn ClockReference>,
        clock: RobotClock,
        config: MaintenanceConfig,
    ) -> CuResult<Self> {
        config.validate()?;
        let sync = ClockSync::new(&clock, reference.domain(), config.sync).map_err(sync_error)?;
        Ok(Self {
            reference: Some(reference),
            clock,
            sync,
            config,
            next_sample: 0,
            started: false,
            changed: false,
            record_pending: false,
            #[cfg(feature = "std")]
            worker: None,
        })
    }

    fn accept(&mut self, capture: CuResult<Option<ClockObservation>>) -> CuResult<()> {
        self.changed = true;
        match capture {
            Ok(Some(sample)) => {
                if sample.domain != self.sync.status().domain {
                    // A changed parent/session stops consumers; restart reacquires.
                    return Err(CuError::from(
                        "Clock reference domain/session changed; restart to reacquire",
                    ));
                }
                self.sync.observe(sample).map_err(sync_error)?;
            }
            Ok(None) | Err(_) => self.sync.reference_lost(),
        }
        Ok(())
    }

    /// Acquires the reference before any consumer start hook is called.
    pub fn start(&mut self) -> CuResult<()> {
        if self.started {
            return Ok(());
        }
        let parent = self
            .reference
            .as_mut()
            .ok_or(CuError::from("Clock reference missing"))?;
        if let Err(error) = parent.start() {
            let _ = parent.stop();
            return Err(error);
        }
        self.started = true;
        let result = self.acquire();
        if result.is_err() {
            let _ = self.stop();
        }
        result
    }

    fn acquire(&mut self) -> CuResult<()> {
        let start = self.clock.raw_now().0;
        let domain = self
            .reference
            .as_ref()
            .ok_or(CuError::from("Clock reference missing"))?
            .domain();
        self.sync.resync(domain).map_err(sync_error)?;
        self.next_sample = start;
        loop {
            let raw = self.clock.raw_now().0;
            if raw.saturating_sub(start) >= self.config.acquisition_timeout.as_nanos() as u64 {
                return Err(CuError::from("Clock synchronization acquisition timed out"));
            }
            if raw >= self.next_sample {
                let capture = self
                    .reference
                    .as_mut()
                    .ok_or(CuError::from("Clock reference missing"))?
                    .poll(&self.clock);
                // The first poll may discover a usable parent identity.
                if let Ok(Some(sample)) = capture.as_ref()
                    && sample.domain != self.sync.status().domain
                {
                    self.sync.resync(sample.domain).map_err(sync_error)?;
                }
                self.accept(capture)?;
                self.next_sample =
                    raw.saturating_add(self.config.sample_interval.as_nanos() as u64);
            }
            match self.sync.update() {
                Ok(status) if status.state == SyncState::Locked => break,
                Ok(status) if status.state == SyncState::Expired => {
                    return Err(CuError::from("Clock reference expired during acquisition"));
                }
                Err(SyncError::ReadersActive) | Ok(_) => {}
                Err(error) => return Err(sync_error(error)),
            }
            #[cfg(feature = "std")]
            std::thread::sleep(Duration::from_millis(1));
            #[cfg(not(feature = "std"))]
            core::hint::spin_loop();
        }
        self.record_pending = true;
        self.changed = false;
        #[cfg(feature = "std")]
        self.start_worker()?;
        Ok(())
    }

    #[cfg(feature = "std")]
    fn start_worker(&mut self) -> CuResult<()> {
        use portable_atomic::{AtomicBool, Ordering};
        let mut parent = self
            .reference
            .take()
            .ok_or(CuError::from("Clock reference missing"))?;
        let clock = self.clock.clone();
        let interval = self.config.sample_interval;
        let (mut tx, rx) = rtrb::RingBuffer::new(2);
        let stop = alloc::sync::Arc::new(AtomicBool::new(false));
        let worker_stop = stop.clone();
        // spawn() can fail; keep the parent in a recoverable shared slot until the
        // worker has started, so shutdown still calls stop on all failure paths.
        let parent_slot = alloc::sync::Arc::new(std::sync::Mutex::new(Some(parent)));
        let worker_parent = parent_slot.clone();
        let thread = std::thread::Builder::new()
            .name("cu-clock-reference".into())
            .spawn(move || {
                let mut parent = worker_parent
                    .lock()
                    .map_err(|_| CuError::from("Clock worker mutex poisoned"))?
                    .take()
                    .ok_or(CuError::from("Clock worker reference missing"))?;
                while !worker_stop.load(Ordering::Acquire) {
                    if tx.slots() > 0 {
                        // Errors own strings; dispose of them on the I/O worker. The
                        // real-time mailbox carries only fixed-size observations/loss.
                        let sample = parent.poll(&clock).unwrap_or_default();
                        let _ = tx.push(sample);
                    }
                    std::thread::park_timeout(interval);
                }
                Ok(parent)
            });
        match thread {
            Ok(thread) => {
                self.worker = Some(Worker {
                    rx: std::sync::Mutex::new(rx),
                    stop,
                    thread: Some(thread),
                });
                Ok(())
            }
            Err(error) => {
                parent = parent_slot
                    .lock()
                    .map_err(|_| CuError::from("Clock worker mutex poisoned"))?
                    .take()
                    .ok_or(CuError::from(
                        "Clock reference missing after worker failure",
                    ))?;
                self.reference = Some(parent);
                Err(CuError::new_with_cause(
                    "Failed to start clock reference worker",
                    error,
                ))
            }
        }
    }

    /// Checks health and publishes one coherent correction before an iteration.
    pub fn maintain(&mut self) -> CuResult<SyncStatus> {
        if !self.started {
            return Err(CuError::from(
                "Clock synchronization must start before consumers",
            ));
        }
        #[cfg(feature = "std")]
        {
            // Drain the fixed-capacity mailbox; there are at most two observations.
            for _ in 0..2 {
                let capture = match self.worker.as_mut() {
                    Some(worker) => worker
                        .rx
                        .get_mut()
                        .map_err(|_| CuError::from("Clock mailbox poisoned"))?
                        .pop()
                        .ok(),
                    None => None,
                };
                if let Some(capture) = capture {
                    self.accept(Ok(capture))?;
                }
            }
        }
        #[cfg(not(feature = "std"))]
        {
            let raw = self.clock.raw_now().0;
            if raw >= self.next_sample {
                let capture = self
                    .reference
                    .as_mut()
                    .ok_or(CuError::from("Clock reference missing"))?
                    .poll(&self.clock);
                self.accept(capture)?;
                self.next_sample =
                    raw.saturating_add(self.config.sample_interval.as_nanos() as u64);
            }
        }
        let status = if self.changed {
            match self.sync.update() {
                Ok(status) => {
                    self.changed = false;
                    self.record_pending = true;
                    status
                }
                Err(SyncError::ReadersActive) => self.sync.status(),
                Err(error) => return Err(sync_error(error)),
            }
        } else {
            self.sync.status()
        };
        if matches!(status.state, SyncState::Acquiring | SyncState::Expired) {
            return Err(CuError::from(
                "Clock synchronization is unusable; consumers must stop",
            ));
        }
        Ok(status)
    }

    /// Takes a pending correction record once, outside process().
    pub fn take_record(&mut self, culistid: u64) -> Option<ClockSyncRecord> {
        if !core::mem::take(&mut self.record_pending) {
            return None;
        }
        Some(ClockSyncRecord {
            culistid,
            snapshot: self.sync.snapshot(),
        })
    }

    /// Stops reference I/O and returns the provider for a later restart.
    pub fn stop(&mut self) -> CuResult<()> {
        if !self.started {
            return Ok(());
        }
        #[cfg(feature = "std")]
        if let Some(mut worker) = self.worker.take() {
            worker.stop.store(true, portable_atomic::Ordering::Release);
            if let Some(thread) = worker.thread.take() {
                thread.thread().unpark();
                self.reference = Some(
                    thread
                        .join()
                        .map_err(|_| CuError::from("Clock reference worker panicked"))??,
                );
            }
        }
        self.started = false;
        self.reference
            .as_mut()
            .map_or(Ok(()), |parent| parent.stop())
    }

    /// Clock controller access used by replay and recording outside process().
    pub fn controller(&mut self) -> &mut ClockSync {
        &mut self.sync
    }
}

impl Drop for ClockMaintenance {
    fn drop(&mut self) {
        if self.started {
            let _ = self.stop();
        }
    }
}

#[cfg(all(test, feature = "std"))]
mod tests {
    use super::*;
    use core::sync::atomic::{AtomicUsize, Ordering};
    use std::sync::Arc;

    struct MissingReference {
        stops: Arc<AtomicUsize>,
    }
    impl ClockReference for MissingReference {
        fn create_clock(&self) -> CuResult<RobotClock> {
            Ok(RobotClock::new())
        }
        fn domain(&self) -> ClockDomain {
            ClockDomain {
                id: 0,
                identity: [0; 8],
                session: 0,
            }
        }
        fn start(&mut self) -> CuResult<()> {
            Ok(())
        }
        fn poll(&mut self, _: &RobotClock) -> CuResult<Option<ClockObservation>> {
            Ok(None)
        }
        fn stop(&mut self) -> CuResult<()> {
            self.stops.fetch_add(1, Ordering::SeqCst);
            Ok(())
        }
    }
    #[test]
    fn acquisition_timeout_stops_parent_and_shutdown_is_idempotent() {
        let stops = Arc::new(AtomicUsize::new(0));
        let reference = MissingReference {
            stops: stops.clone(),
        };
        let config = MaintenanceConfig {
            acquisition_timeout: Duration::from_millis(4),
            sample_interval: Duration::from_millis(1),
            sync: SyncConfig::new(cu29_clock::CuDuration(100_000)),
        };
        let mut maintenance =
            ClockMaintenance::new(Box::new(reference), RobotClock::new(), config).unwrap();
        assert!(
            maintenance
                .start()
                .unwrap_err()
                .to_string()
                .contains("timed out")
        );
        maintenance.stop().unwrap();
        drop(maintenance);
        assert_eq!(stops.load(Ordering::SeqCst), 1);
    }
}
