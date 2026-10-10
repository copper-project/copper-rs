//! Read-only paired captures from a board's existing synchronized PTP service.

use cu29::clock::sync::{ClockDomain, ClockObservation};
use cu29::clock_sync::ClockReference;
use cu29::prelude::{CuDuration, CuError, CuResult, RobotClock};

/// BSP hooks for an existing PTP service, run in foreground on bare metal.
///
/// Experimental. `poll` advances the board's service with bounded work. `read`
/// returns a fresh TAI reading and its upstream uncertainty, or `None` while
/// upstream quality is unusable. The BSP extends timer rollovers and initializes
/// Copper's architecture counter before the runtime builder runs.
pub struct BoardPtpHooks {
    /// Acquires the parent/controller handles.
    pub start: fn() -> CuResult<()>,
    /// Executes one bounded service step.
    pub poll: fn() -> CuResult<()>,
    /// Returns the current PTP root/session.
    pub domain: fn() -> ClockDomain,
    /// Captures parent nanoseconds and its upstream uncertainty.
    pub read: fn() -> CuResult<Option<(u64, CuDuration)>>,
    /// Stops the controller before transport teardown.
    pub stop: fn() -> CuResult<()>,
}

/// BSP reference adapter with a statically known architecture counter frequency.
///
/// Export an owned `BoardPtp<HZ>` in a resource bundle implementing
/// `ClockReferenceBundle<Reference = BoardPtp<HZ>>`. On Cortex-M, all counter
/// reads remain in the foreground; ISR code enqueues parent captures separately.
pub struct BoardPtp<const HZ: u64> {
    hooks: BoardPtpHooks,
}

impl<const HZ: u64> BoardPtp<HZ> {
    /// Wraps a board's existing PTP service hooks without starting it.
    pub const fn new(hooks: BoardPtpHooks) -> Self {
        Self { hooks }
    }
}

impl<const HZ: u64> ClockReference for BoardPtp<HZ> {
    fn create_clock(&self) -> CuResult<RobotClock> {
        RobotClock::new_with_frequency(HZ)
            .map_err(|e| CuError::new_with_cause("Invalid board raw-counter frequency", e))
    }
    fn start(&mut self) -> CuResult<()> {
        (self.hooks.start)()
    }
    fn domain(&self) -> ClockDomain {
        (self.hooks.domain)()
    }
    fn poll(&mut self, clock: &RobotClock) -> CuResult<Option<ClockObservation>> {
        (self.hooks.poll)()?;
        let domain = self.domain();
        let before = clock.raw_now();
        let sample = (self.hooks.read)()?;
        let after = clock.raw_now();
        if after < before {
            return Err(CuError::from(
                "Board raw counter moved backward during capture",
            ));
        }
        if self.domain() != domain {
            return Ok(None);
        }
        sample
            .map(|(parent_ns, upstream)| {
                let error = upstream
                    .0
                    .checked_add((after - before).as_nanos().div_ceil(2))
                    .ok_or(CuError::from("Board capture uncertainty overflow"))?;
                Ok(ClockObservation {
                    raw_local: before + (after - before) / 2u64,
                    parent_ns,
                    uncertainty: CuDuration(error),
                    domain,
                })
            })
            .transpose()
    }
    fn stop(&mut self) -> CuResult<()> {
        (self.hooks.stop)()
    }
}
