use core::ops::{Add, Sub};
pub use quanta::Instant;
use quanta::{Clock, Mock};
use std::sync::Arc;
use std::time::Duration;

#[cfg(test)]
#[macro_use]
extern crate approx;

/// For Robot times, the underlying type is a u64 representing nanoseconds.
/// It is always positive to simplify the reasoning on the user side.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct CuDuration(u64);

/// bridge the API with standard Durations.
impl From<Duration> for CuDuration {
    fn from(duration: Duration) -> Self {
        CuDuration(duration.as_nanos() as u64)
    }
}

impl Into<Duration> for CuDuration {
    fn into(self) -> Duration {
        Duration::from_nanos(self.0)
    }
}

impl From<u64> for CuDuration {
    fn from(duration: u64) -> Self {
        CuDuration(duration)
    }
}

impl Into<u64> for CuDuration {
    fn into(self) -> u64 {
        self.0
    }
}

impl Sub for CuDuration {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        CuDuration(self.0 - rhs.0)
    }
}

impl Add for CuDuration {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        CuDuration(self.0 + rhs.0)
    }
}

/// A robot time is just a duration from a fixed point in time.
pub type CuTime = CuDuration;

/// A running Robot clock.
pub struct RobotClock {
    inner: Clock,
    ref_time: Instant,
}

impl RobotClock {
    /// Creates a RobotClock using the call time as its reference point.
    pub fn new() -> Self {
        let clock = Clock::new();
        let ref_time = clock.now();
        RobotClock {
            inner: clock,
            ref_time,
        }
    }

    /// Builds a monotonic clock starting at the given reference time.
    pub fn from_ref_time(ref_time_ns: u64) -> Self {
        let clock = Clock::new();
        let ref_time = clock.now() - Duration::from_nanos(ref_time_ns);
        RobotClock {
            inner: Clock::new(),
            ref_time,
        }
    }

    /// Build a fake clock with a reference time of 0.
    /// The Mock interface enables you to increment and decrement the time.
    pub fn mock() -> (Self, Arc<Mock>) {
        let (clock, mock) = Clock::mock();
        let ref_time = clock.now();
        (
            RobotClock {
                inner: clock,
                ref_time,
            },
            mock,
        )
    }

    // Now returns the time that passed since the reference time, usually the start time.
    // It is a monotonically increasing value.
    pub fn now(&self) -> CuTime {
        // TODO: this could be further optimized to avoid this constant conversion from 2 fields to one under the hood.
        // Let's say this is the default implementation.
        (self.inner.now() - self.ref_time).into()
    }
}

impl Default for RobotClock {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mock() {
        let (clock, mock) = RobotClock::mock();
        assert_eq!(clock.now(), Duration::from_secs(0).into());
        mock.increment(Duration::from_secs(1));
        assert_eq!(clock.now(), Duration::from_secs(1).into());
    }

    #[test]
    fn test_from_ref_time() {
        let tolerance_ms = 10;
        let clock = RobotClock::from_ref_time(1_000_000_000);
        assert_relative_eq!(
            <CuDuration as Into<Duration>>::into(clock.now()).as_millis() as f64,
            Duration::from_secs(1).as_millis() as f64,
            epsilon = tolerance_ms as f64
        );
    }

    #[test]
    fn longuest_duration() {
        let maxcu = CuDuration(u64::MAX);
        let maxd: Duration = maxcu.into();
        assert_eq!(maxd.as_nanos(), u64::MAX as u128);
        let s = maxd.as_secs();
        let y = s / 60 / 60 / 24 / 365;
        assert!(y >= 584); // 584 years of robot uptime, we should be good.
    }

    #[test]
    fn test_some_time_arithmetics() {
        let a: CuDuration = 10.into();
        let b: CuDuration = 20.into();
        let c = a + b;
        assert_eq!(c.0, 30);
        let d = b - a;
        assert_eq!(d.0, 10);
    }
}
