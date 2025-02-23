#[cfg(test)]
#[macro_use]
extern crate approx;
use bincode::de::BorrowDecoder;
use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::BorrowDecode;
use bincode::{Decode, Encode};
use core::ops::{Add, Sub};
pub use quanta::Instant;
use quanta::{Clock, Mock};
use serde::{Deserialize, Serialize};
use std::convert::Into;
use std::fmt::{Display, Formatter};
use std::ops::{AddAssign, Div, Mul, SubAssign};
use std::sync::Arc;
use std::time::Duration;

/// For Robot times, the underlying type is a u64 representing nanoseconds.
/// It is always positive to simplify the reasoning on the user side.
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize, Default)]
pub struct CuDuration(pub u64);

impl CuDuration {
    // Lowest value a CuDuration can have.
    pub const MIN: CuDuration = CuDuration(0u64);
    // Highest value a CuDuration can have reserving the max value for None.
    pub const MAX: CuDuration = CuDuration(NONE_VALUE - 1);
    pub fn max(self, other: CuDuration) -> CuDuration {
        let Self(lhs) = self;
        let Self(rhs) = other;
        CuDuration(lhs.max(rhs))
    }

    pub fn min(self, other: CuDuration) -> CuDuration {
        let Self(lhs) = self;
        let Self(rhs) = other;
        CuDuration(lhs.min(rhs))
    }

    pub fn as_nanos(&self) -> u64 {
        let Self(nanos) = self;
        *nanos
    }
}

/// bridge the API with standard Durations.
impl From<Duration> for CuDuration {
    fn from(duration: Duration) -> Self {
        CuDuration(duration.as_nanos() as u64)
    }
}

impl From<CuDuration> for Duration {
    fn from(val: CuDuration) -> Self {
        let CuDuration(nanos) = val;
        Duration::from_nanos(nanos)
    }
}

impl From<u64> for CuDuration {
    fn from(duration: u64) -> Self {
        CuDuration(duration)
    }
}

impl From<CuDuration> for u64 {
    fn from(val: CuDuration) -> Self {
        let CuDuration(nanos) = val;
        nanos
    }
}

impl Sub for CuDuration {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        let CuDuration(lhs) = self;
        let CuDuration(rhs) = rhs;
        CuDuration(lhs - rhs)
    }
}

impl Add for CuDuration {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        let CuDuration(lhs) = self;
        let CuDuration(rhs) = rhs;
        CuDuration(lhs + rhs)
    }
}

impl AddAssign for CuDuration {
    fn add_assign(&mut self, rhs: Self) {
        let CuDuration(lhs) = self;
        let CuDuration(rhs) = rhs;
        *lhs += rhs;
    }
}

impl SubAssign for CuDuration {
    fn sub_assign(&mut self, rhs: Self) {
        let CuDuration(lhs) = self;
        let CuDuration(rhs) = rhs;
        *lhs -= rhs;
    }
}

// a way to divide a duration by a scalar.
// useful to compute averages for example.
impl<T> Div<T> for CuDuration
where
    T: Into<u64>,
{
    type Output = Self;
    fn div(self, rhs: T) -> Self {
        let CuDuration(lhs) = self;
        CuDuration(lhs / rhs.into())
    }
}
//
// a way to multiply a duration by a scalar.
// useful to compute offsets for example.
// CuDuration * scalar
impl<T> Mul<T> for CuDuration
where
    T: Into<u64>,
{
    type Output = CuDuration;

    fn mul(self, rhs: T) -> CuDuration {
        let CuDuration(lhs) = self;
        CuDuration(lhs * rhs.into())
    }
}

// u64 * CuDuration
impl Mul<CuDuration> for u64 {
    type Output = CuDuration;

    fn mul(self, rhs: CuDuration) -> CuDuration {
        let CuDuration(nanos) = rhs;
        CuDuration(self * nanos)
    }
}

// u32 * CuDuration
impl Mul<CuDuration> for u32 {
    type Output = CuDuration;

    fn mul(self, rhs: CuDuration) -> CuDuration {
        let CuDuration(nanos) = rhs;
        CuDuration(self as u64 * nanos)
    }
}

// i32 * CuDuration
impl Mul<CuDuration> for i32 {
    type Output = CuDuration;

    fn mul(self, rhs: CuDuration) -> CuDuration {
        let CuDuration(nanos) = rhs;
        CuDuration(self as u64 * nanos)
    }
}

impl Encode for CuDuration {
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        let CuDuration(nanos) = self;
        nanos.encode(encoder)
    }
}

impl Decode for CuDuration {
    fn decode<D: Decoder>(decoder: &mut D) -> Result<Self, DecodeError> {
        Ok(CuDuration(u64::decode(decoder)?))
    }
}

impl<'de> BorrowDecode<'de> for CuDuration {
    fn borrow_decode<D: BorrowDecoder<'de>>(decoder: &mut D) -> Result<Self, DecodeError> {
        Ok(CuDuration(u64::decode(decoder)?))
    }
}

impl Display for CuDuration {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        let Self(nanos) = *self;
        if nanos >= 86_400_000_000_000 {
            write!(f, "{:.3} d", nanos as f64 / 86_400_000_000_000.0)
        } else if nanos >= 3_600_000_000_000 {
            write!(f, "{:.3} h", nanos as f64 / 3_600_000_000_000.0)
        } else if nanos >= 60_000_000_000 {
            write!(f, "{:.3} m", nanos as f64 / 60_000_000_000.0)
        } else if nanos >= 1_000_000_000 {
            write!(f, "{:.3} s", nanos as f64 / 1_000_000_000.0)
        } else if nanos >= 1_000_000 {
            write!(f, "{:.3} ms", nanos as f64 / 1_000_000.0)
        } else if nanos >= 1_000 {
            write!(f, "{:.3} µs", nanos as f64 / 1_000.0)
        } else {
            write!(f, "{nanos} ns")
        }
    }
}

/// A robot time is just a duration from a fixed point in time.
pub type CuTime = CuDuration;

/// Homebrewed `Option<CuDuration>` to avoid using 128bits just to represent an Option.
#[derive(Copy, Clone, Debug, PartialEq, Encode, Decode, Serialize, Deserialize)]
pub struct OptionCuTime(CuTime);

const NONE_VALUE: u64 = 0xFFFFFFFFFFFFFFFF;

impl OptionCuTime {
    #[inline]
    pub fn is_none(&self) -> bool {
        let Self(CuDuration(nanos)) = self;
        *nanos == NONE_VALUE
    }

    #[inline]
    pub fn none() -> Self {
        OptionCuTime(CuDuration(NONE_VALUE))
    }

    #[inline]
    pub fn unwrap(self) -> CuTime {
        if self.is_none() {
            panic!("called `OptionCuTime::unwrap()` on a `None` value");
        }
        self.0
    }
}

impl Display for OptionCuTime {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        if self.is_none() {
            write!(f, "None")
        } else {
            write!(f, "{}", self.0)
        }
    }
}

impl Default for OptionCuTime {
    fn default() -> Self {
        Self::none()
    }
}

impl From<Option<CuTime>> for OptionCuTime {
    #[inline]
    fn from(duration: Option<CuTime>) -> Self {
        match duration {
            Some(duration) => OptionCuTime(duration),
            None => OptionCuTime(CuDuration(NONE_VALUE)),
        }
    }
}

impl From<OptionCuTime> for Option<CuTime> {
    #[inline]
    fn from(val: OptionCuTime) -> Self {
        let OptionCuTime(CuDuration(nanos)) = val;
        if nanos == NONE_VALUE {
            None
        } else {
            Some(CuDuration(nanos))
        }
    }
}

impl From<CuTime> for OptionCuTime {
    #[inline]
    fn from(val: CuTime) -> Self {
        Some(val).into()
    }
}

/// Represents a time range.
#[derive(Copy, Clone, Debug, Encode, Decode, Serialize, Deserialize, PartialEq)]
pub struct CuTimeRange {
    pub start: CuTime,
    pub end: CuTime,
}

/// Builds a time range from a slice of CuTime.
/// This is an O(n) operation.
impl From<&[CuTime]> for CuTimeRange {
    fn from(slice: &[CuTime]) -> Self {
        CuTimeRange {
            start: *slice.iter().min().expect("Empty slice"),
            end: *slice.iter().max().expect("Empty slice"),
        }
    }
}

/// Represents a time range with possible undefined start or end or both.
#[derive(Copy, Clone, Debug, Encode, Decode, Serialize, Deserialize)]
pub struct PartialCuTimeRange {
    pub start: OptionCuTime,
    pub end: OptionCuTime,
}

impl Default for PartialCuTimeRange {
    fn default() -> Self {
        PartialCuTimeRange {
            start: OptionCuTime::none(),
            end: OptionCuTime::none(),
        }
    }
}

/// The time of validity of a message can be more than one time but can be a time range of Tovs.
/// For example a sub scan for a lidar, a set of images etc... can have a range of validity.
#[derive(Default, Clone, Debug, PartialEq, Encode, Decode, Serialize, Deserialize)]
pub enum Tov {
    #[default]
    None,
    Time(CuTime),
    Range(CuTimeRange),
}

impl From<Option<CuDuration>> for Tov {
    fn from(duration: Option<CuDuration>) -> Self {
        match duration {
            Some(duration) => Tov::Time(duration),
            None => Tov::None,
        }
    }
}

impl From<CuDuration> for Tov {
    fn from(duration: CuDuration) -> Self {
        Tov::Time(duration)
    }
}

/// A running Robot clock.
/// The clock is a monotonic clock that starts at an arbitrary reference time.
/// It is clone resilient, ie a clone will be the same clock, even when mocked.
#[derive(Clone, Debug)]
pub struct RobotClock {
    inner: Clock,      // This is a wrapper on quanta::Clock today.
    ref_time: Instant, // The reference instant on which this clock is based.
}

/// A mock clock that can be controlled by the user.
#[derive(Debug, Clone)]
pub struct RobotClockMock(Arc<Mock>); // wraps the Mock from quanta today.

impl RobotClockMock {
    pub fn increment(&self, amount: Duration) {
        let Self(mock) = self;
        mock.increment(amount);
    }

    /// Decrements the time by the given amount.
    /// Be careful this brakes the monotonicity of the clock.
    pub fn decrement(&self, amount: Duration) {
        let Self(mock) = self;
        mock.decrement(amount);
    }

    /// Gets the current value of time.
    pub fn value(&self) -> u64 {
        let Self(mock) = self;
        mock.value()
    }

    /// A convenient way to get the current time from the mocking side.
    pub fn now(&self) -> CuTime {
        let Self(mock) = self;
        mock.value().into()
    }

    /// Sets the absolute value of the time.
    pub fn set_value(&self, value: u64) {
        let Self(mock) = self;
        let v = mock.value();
        // had to work around the quata API here.
        if v < value {
            self.increment(Duration::from_nanos(value) - Duration::from_nanos(v));
        } else {
            self.decrement(Duration::from_nanos(v) - Duration::from_nanos(value));
        }
    }
}

impl RobotClock {
    /// Creates a RobotClock using now as its reference time.
    /// It will start a 0ns incrementing monotonically.
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
    /// The RobotMock interface enables you to control all the clones of the clock given.
    pub fn mock() -> (Self, RobotClockMock) {
        let (clock, mock) = Clock::mock();
        let ref_time = clock.now();
        (
            RobotClock {
                inner: clock,
                ref_time,
            },
            RobotClockMock(mock),
        )
    }

    // Now returns the time that passed since the reference time, usually the start time.
    // It is a monotonically increasing value.
    #[inline]
    pub fn now(&self) -> CuTime {
        // TODO: this could be further optimized to avoid this constant conversion from 2 fields to one under the hood.
        // Let's say this is the default implementation.
        (self.inner.now() - self.ref_time).into()
    }

    // A less precise but quicker time
    #[inline]
    pub fn recent(&self) -> CuTime {
        (self.inner.recent() - self.ref_time).into()
    }
}

impl Default for RobotClock {
    fn default() -> Self {
        Self::new()
    }
}

/// A trait to provide a clock to the runtime.
pub trait ClockProvider {
    fn get_clock(&self) -> RobotClock;
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
    fn test_mock_clone() {
        let (clock, mock) = RobotClock::mock();
        assert_eq!(clock.now(), Duration::from_secs(0).into());
        let clock_clone = clock.clone();
        mock.increment(Duration::from_secs(1));
        assert_eq!(clock_clone.now(), Duration::from_secs(1).into());
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
    fn longest_duration() {
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

    #[test]
    fn test_build_range_from_slice() {
        let range = CuTimeRange::from(&[20.into(), 10.into(), 30.into()][..]);
        assert_eq!(range.start, 10.into());
        assert_eq!(range.end, 30.into());
    }
}
