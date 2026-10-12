//! Read-only paired captures from a board's existing synchronized PTP service.

use cu29::clock::ClockOrigin;
use cu29::clock::sync::{ClockDomain, ClockObservation};
use cu29::clock_sync::ClockReference;
use cu29::prelude::{CuDuration, CuError, CuResult, RobotClock};

/// BSP hooks for an existing PTP service, run in foreground on bare metal.
///
/// Experimental. `poll` advances the board's service with bounded work. `read`
/// returns reference nanoseconds and upstream uncertainty, or `None` while
/// upstream quality is unusable. The BSP extends timer rollovers and initializes
/// Copper's architecture counter before the runtime builder runs.
pub struct BoardPtpHooks {
    /// Acquires the parent/controller handles.
    pub start: fn() -> CuResult<()>,
    /// Executes one bounded service step.
    pub poll: fn() -> CuResult<()>,
    /// Returns the current PTP root/session.
    pub domain: fn() -> ClockDomain,
    /// Returns the advertised source and epoch of captured reference nanoseconds.
    pub origin: fn() -> ClockOrigin,
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
        let origin = (self.hooks.origin)();
        let before = clock.raw_now();
        let sample = (self.hooks.read)()?;
        let after = clock.raw_now();
        if after < before {
            return Err(CuError::from(
                "Board raw counter moved backward during capture",
            ));
        }
        if self.domain() != domain || (self.hooks.origin)() != origin {
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
                    origin,
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

#[cfg(test)]
mod tests {
    use super::*;
    use cu29::clock::sync::{ClockSync, SyncConfig};
    use cu29::clock::{ClockEpoch, ClockSource};

    #[test]
    fn board_reference_keeps_bare_gps_time_and_provenance() {
        let origin = ClockOrigin {
            source: ClockSource::Gnss,
            epoch: ClockEpoch::Gps,
        };
        let domain = ClockDomain {
            id: 0,
            identity: *b"gps-root",
            session: 0,
        };
        let mut provider = BoardPtp::<1_000_000>::new(BoardPtpHooks {
            start: || Ok(()),
            poll: || Ok(()),
            stop: || Ok(()),
            domain: || ClockDomain {
                id: 0,
                identity: *b"gps-root",
                session: 0,
            },
            origin: || ClockOrigin {
                source: ClockSource::Gnss,
                epoch: ClockEpoch::Gps,
            },
            read: || Ok(Some((1_400_000_000_000_000_000, CuDuration(100)))),
        });
        let (clock, _) = RobotClock::mock();
        let sample = provider.poll(&clock).unwrap().unwrap();
        assert_eq!(sample.parent_ns, 1_400_000_000_000_000_000);
        assert_eq!(sample.origin, origin);
        let mut sync =
            ClockSync::new(&clock, domain, SyncConfig::new(CuDuration(200_000))).unwrap();
        sync.observe(sample).unwrap();
        sync.update().unwrap();
        assert_eq!(clock.origin(), origin);
        assert_eq!(clock.now().0, sample.parent_ns);
    }
}
