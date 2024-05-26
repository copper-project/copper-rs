pub use quanta::Instant;
use quanta::{Clock, Mock};
use std::sync::Arc;
use std::time::Duration;

#[cfg(test)]
#[macro_use]
extern crate approx;

pub struct RobotClock {
    inner: Clock,
    ref_time: Instant,
}

impl RobotClock {
    pub fn new() -> Self {
        let clock = Clock::new();
        let ref_time = clock.now();
        RobotClock {
            inner: clock,
            ref_time,
        }
    }

    // Builds a monotonic clock starting at the given reference time.
    pub fn from_ref_time(ref_time_ns: u64) -> Self {
        let clock = Clock::new();
        let ref_time = clock.now() - Duration::from_nanos(ref_time_ns);
        RobotClock {
            inner: Clock::new(),
            ref_time,
        }
    }

    // Build a fake clock with a reference time of 0.
    // The Mock interface enables you to increment and decrement the time.
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

    pub fn elapsed(&self) -> Duration {
        self.inner.now() - self.ref_time
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
        assert_eq!(clock.elapsed(), Duration::from_secs(0));
        mock.increment(Duration::from_secs(1));
        assert_eq!(clock.elapsed(), Duration::from_secs(1));
    }

    #[test]
    fn test_from_ref_time() {
        let tolerance_ms = 10;
        let clock = RobotClock::from_ref_time(1_000_000_000);
        assert_relative_eq!(
            clock.elapsed().as_millis() as f64,
            Duration::from_secs(1).as_millis() as f64,
            epsilon = tolerance_ms as f64
        );
    }
}
