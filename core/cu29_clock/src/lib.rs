#![cfg_attr(not(feature = "std"), no_std)]
#[cfg(not(feature = "std"))]
extern crate alloc;
#[cfg(test)]
extern crate approx;

use bincode::de::BorrowDecoder;
use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::BorrowDecode;
use bincode::{Decode, Encode};
use core::ops::{Add, Sub};
use serde::{Deserialize, Serialize};

// We use this to be able to support embedded 32bit platforms
use portable_atomic::{AtomicU64, Ordering};

#[cfg(not(feature = "std"))]
mod imp {
    pub use alloc::format;
    pub use alloc::sync::Arc;
    pub use core::fmt::{Display, Formatter};
    pub use core::ops::{AddAssign, Div, Mul, SubAssign};
}

#[cfg(feature = "std")]
mod imp {
    pub use std::convert::Into;
    pub use std::fmt::{Display, Formatter};
    pub use std::ops::{AddAssign, Div, Mul, SubAssign};
    pub use std::sync::Arc;
}

use imp::*;

// Platform-specific high-precision timer implementations
mod platform {
    use super::{AtomicU64, Ordering};

    #[cfg(any(target_arch = "x86", target_arch = "x86_64"))]
    pub fn read_raw_counter() -> u64 {
        unsafe { core::arch::x86_64::_rdtsc() }
    }

    #[cfg(target_arch = "aarch64")]
    pub fn read_raw_counter() -> u64 {
        let mut counter: u64;
        unsafe {
            core::arch::asm!("mrs {}, cntvct_el0", out(reg) counter);
        }
        counter
    }

    #[cfg(target_arch = "arm")]
    pub fn read_raw_counter() -> u64 {
        let mut counter_lo: u32;
        let mut counter_hi: u32;
        unsafe {
            core::arch::asm!(
                "mrrc p15, 1, {0}, {1}, c14",
                out(reg) counter_lo,
                out(reg) counter_hi,
            );
        }
        ((counter_hi as u64) << 32) | (counter_lo as u64)
    }

    #[cfg(target_arch = "riscv64")]
    pub fn read_raw_counter() -> u64 {
        let counter: u64;
        unsafe {
            core::arch::asm!("rdcycle {}", out(reg) counter);
        }
        counter
    }

    #[cfg(not(any(
        target_arch = "x86",
        target_arch = "x86_64",
        target_arch = "aarch64",
        target_arch = "arm",
        target_arch = "riscv64"
    )))]
    pub fn read_raw_counter() -> u64 {
        // Fallback implementation for unsupported architectures
        #[cfg(feature = "std")]
        {
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos() as u64
        }
        #[cfg(not(feature = "std"))]
        {
            // For no-std environments on unsupported platforms, we need a compile-time error
            compile_error!("Unsupported target architecture for high-precision timing");
        }
    }

    // Frequency estimation for converting raw counter values to nanoseconds
    static FREQUENCY_NS: AtomicU64 = AtomicU64::new(0);
    static INIT_COUNTER: AtomicU64 = AtomicU64::new(0);
    static INIT_TIME_NS: AtomicU64 = AtomicU64::new(0);

    pub fn initialize_frequency() {
        #[cfg(feature = "std")]
        {
            let start_counter = read_raw_counter();
            let start_time = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos() as u64;

            std::thread::sleep(std::time::Duration::from_millis(10));

            let end_counter = read_raw_counter();
            let end_time = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos() as u64;

            let counter_diff = end_counter.saturating_sub(start_counter);
            let time_diff_ns = end_time.saturating_sub(start_time);

            if counter_diff > 0 {
                let freq_ns =
                    ((counter_diff as u128 * 1_000_000_000) / time_diff_ns as u128) as u64;
                FREQUENCY_NS.store(freq_ns, Ordering::Relaxed);
                INIT_COUNTER.store(start_counter, Ordering::Relaxed);
                INIT_TIME_NS.store(start_time, Ordering::Relaxed);
            }
        }
        #[cfg(not(feature = "std"))]
        {
            // In no-std environments, we can't do runtime frequency calibration
            // Use a reasonable default frequency (e.g., 1GHz = 1 tick per nanosecond)
            // This is a simplification and may not be accurate for all platforms
            FREQUENCY_NS.store(1_000_000_000, Ordering::Relaxed);
            INIT_COUNTER.store(0, Ordering::Relaxed);
            INIT_TIME_NS.store(0, Ordering::Relaxed);
        }
    }

    pub fn counter_to_nanos(counter: u64) -> u64 {
        let freq = FREQUENCY_NS.load(Ordering::Relaxed);
        if freq == 0 {
            initialize_frequency();
            let freq = FREQUENCY_NS.load(Ordering::Relaxed);
            if freq == 0 {
                return counter; // Fallback if frequency estimation fails
            }
        }

        let init_counter = INIT_COUNTER.load(Ordering::Relaxed);
        let init_time_ns = INIT_TIME_NS.load(Ordering::Relaxed);
        let counter_diff = counter.saturating_sub(init_counter);

        init_time_ns.saturating_add(((counter_diff as u128 * 1_000_000_000) / freq as u128) as u64)
    }
}

/// High-precision instant in time, represented as nanoseconds since an arbitrary epoch
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct CuInstant(u64);

pub type Instant = CuInstant; // Backward compatibility

impl CuInstant {
    pub fn now() -> Self {
        let raw_counter = platform::read_raw_counter();
        CuInstant(platform::counter_to_nanos(raw_counter))
    }

    pub fn as_nanos(&self) -> u64 {
        self.0
    }
}

impl Sub for CuInstant {
    type Output = CuDuration;

    fn sub(self, other: CuInstant) -> CuDuration {
        CuDuration(self.0.saturating_sub(other.0))
    }
}

impl Sub<CuDuration> for CuInstant {
    type Output = CuInstant;

    fn sub(self, duration: CuDuration) -> CuInstant {
        CuInstant(self.0.saturating_sub(duration.as_nanos()))
    }
}

impl Add<CuDuration> for CuInstant {
    type Output = CuInstant;

    fn add(self, duration: CuDuration) -> CuInstant {
        CuInstant(self.0.saturating_add(duration.as_nanos()))
    }
}

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

    pub fn as_micros(&self) -> u64 {
        let Self(nanos) = self;
        nanos / 1_000
    }

    pub fn as_millis(&self) -> u64 {
        let Self(nanos) = self;
        nanos / 1_000_000
    }

    pub fn as_secs(&self) -> u64 {
        let Self(nanos) = self;
        nanos / 1_000_000_000
    }

    pub fn from_nanos(nanos: u64) -> Self {
        CuDuration(nanos)
    }

    pub fn from_micros(micros: u64) -> Self {
        CuDuration(micros * 1_000)
    }

    pub fn from_millis(millis: u64) -> Self {
        CuDuration(millis * 1_000_000)
    }

    pub fn from_secs(secs: u64) -> Self {
        CuDuration(secs * 1_000_000_000)
    }
}

/// bridge the API with standard Durations.
#[cfg(feature = "std")]
impl From<std::time::Duration> for CuDuration {
    fn from(duration: std::time::Duration) -> Self {
        CuDuration(duration.as_nanos() as u64)
    }
}

#[cfg(not(feature = "std"))]
impl From<core::time::Duration> for CuDuration {
    fn from(duration: core::time::Duration) -> Self {
        CuDuration(duration.as_nanos() as u64)
    }
}

#[cfg(feature = "std")]
impl From<CuDuration> for std::time::Duration {
    fn from(val: CuDuration) -> Self {
        let CuDuration(nanos) = val;
        std::time::Duration::from_nanos(nanos)
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

impl<Context> Decode<Context> for CuDuration {
    fn decode<D: Decoder>(decoder: &mut D) -> Result<Self, DecodeError> {
        Ok(CuDuration(u64::decode(decoder)?))
    }
}

impl<'de, Context> BorrowDecode<'de, Context> for CuDuration {
    fn borrow_decode<D: BorrowDecoder<'de>>(decoder: &mut D) -> Result<Self, DecodeError> {
        Ok(CuDuration(u64::decode(decoder)?))
    }
}

impl Display for CuDuration {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
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

/// A busy looping function based on this clock for a duration.
/// Mainly useful for embedded to spinlocking.
#[inline(always)]
pub fn busy_wait_for(duration: CuDuration) {
    busy_wait_until(CuInstant::now() + duration);
}

/// A busy looping function based on this until a specific time.
/// Mainly useful for embedded to spinlocking.
#[inline(always)]
pub fn busy_wait_until(time: CuInstant) {
    while CuInstant::now() < time {
        core::hint::spin_loop();
    }
}

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
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
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
#[derive(Default, Copy, Clone, Debug, Encode, Decode, Serialize, Deserialize)]
pub struct PartialCuTimeRange {
    pub start: OptionCuTime,
    pub end: OptionCuTime,
}

impl Display for PartialCuTimeRange {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        let start = if self.start.is_none() {
            "…"
        } else {
            &format!("{}", self.start)
        };
        let end = if self.end.is_none() {
            "…"
        } else {
            &format!("{}", self.end)
        };
        write!(f, "[{start} – {end}]")
    }
}

/// The time of validity of a message can be more than one time but can be a time range of Tovs.
/// For example a sub scan for a lidar, a set of images etc... can have a range of validity.
#[derive(Default, Clone, Debug, PartialEq, Encode, Decode, Serialize, Deserialize, Copy)]
pub enum Tov {
    #[default]
    None,
    Time(CuTime),
    Range(CuTimeRange),
}

impl Display for Tov {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        match self {
            Tov::None => write!(f, "None"),
            Tov::Time(t) => write!(f, "{t}"),
            Tov::Range(r) => write!(f, "[{} – {}]", r.start, r.end),
        }
    }
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

/// Internal clock implementation that provides high-precision timing
#[derive(Clone, Debug)]
struct InternalClock {
    // For real clocks, this stores the initialization time
    // For mock clocks, this references the mock state
    mock_state: Option<Arc<AtomicU64>>,
}

impl InternalClock {
    fn new() -> Self {
        // Initialize the frequency calibration
        platform::initialize_frequency();
        InternalClock { mock_state: None }
    }

    fn mock() -> (Self, Arc<AtomicU64>) {
        let mock_state = Arc::new(AtomicU64::new(0));
        let clock = InternalClock {
            mock_state: Some(Arc::clone(&mock_state)),
        };
        (clock, mock_state)
    }

    fn now(&self) -> CuInstant {
        if let Some(ref mock_state) = self.mock_state {
            CuInstant(mock_state.load(Ordering::Relaxed))
        } else {
            CuInstant::now()
        }
    }

    fn recent(&self) -> CuInstant {
        // For simplicity, we use the same implementation as now()
        // In a more sophisticated implementation, this could use a cached value
        self.now()
    }
}

/// A running Robot clock.
/// The clock is a monotonic clock that starts at an arbitrary reference time.
/// It is clone resilient, ie a clone will be the same clock, even when mocked.
#[derive(Clone, Debug)]
pub struct RobotClock {
    inner: InternalClock,
    ref_time: CuInstant,
}

/// A mock clock that can be controlled by the user.
#[derive(Debug, Clone)]
pub struct RobotClockMock(Arc<AtomicU64>);

impl RobotClockMock {
    pub fn increment(&self, amount: CuDuration) {
        let Self(mock_state) = self;
        mock_state.fetch_add(amount.as_nanos(), Ordering::Relaxed);
    }

    /// Decrements the time by the given amount.
    /// Be careful this breaks the monotonicity of the clock.
    pub fn decrement(&self, amount: CuDuration) {
        let Self(mock_state) = self;
        mock_state.fetch_sub(amount.as_nanos(), Ordering::Relaxed);
    }

    /// Gets the current value of time.
    pub fn value(&self) -> u64 {
        let Self(mock_state) = self;
        mock_state.load(Ordering::Relaxed)
    }

    /// A convenient way to get the current time from the mocking side.
    pub fn now(&self) -> CuTime {
        let Self(mock_state) = self;
        CuDuration(mock_state.load(Ordering::Relaxed))
    }

    /// Sets the absolute value of the time.
    pub fn set_value(&self, value: u64) {
        let Self(mock_state) = self;
        mock_state.store(value, Ordering::Relaxed);
    }
}

impl RobotClock {
    /// Creates a RobotClock using now as its reference time.
    /// It will start at 0ns incrementing monotonically.
    pub fn new() -> Self {
        let clock = InternalClock::new();
        let ref_time = clock.now();
        RobotClock {
            inner: clock,
            ref_time,
        }
    }

    /// Builds a monotonic clock starting at the given reference time.
    pub fn from_ref_time(ref_time_ns: u64) -> Self {
        let clock = InternalClock::new();
        let ref_time = clock.now() - CuDuration(ref_time_ns);
        RobotClock {
            inner: clock,
            ref_time,
        }
    }

    /// Build a fake clock with a reference time of 0.
    /// The RobotMock interface enables you to control all the clones of the clock given.
    pub fn mock() -> (Self, RobotClockMock) {
        let (clock, mock_state) = InternalClock::mock();
        let ref_time = clock.now();
        (
            RobotClock {
                inner: clock,
                ref_time,
            },
            RobotClockMock(mock_state),
        )
    }

    /// Now returns the time that passed since the reference time, usually the start time.
    /// It is a monotonically increasing value.
    #[inline]
    pub fn now(&self) -> CuTime {
        self.inner.now() - self.ref_time
    }

    /// A less precise but quicker time
    #[inline]
    pub fn recent(&self) -> CuTime {
        self.inner.recent() - self.ref_time
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
    use approx::assert_relative_eq;

    #[test]
    fn test_cuduration_comparison_operators() {
        let a = CuDuration(100);
        let b = CuDuration(200);

        assert!(a < b);
        assert!(b > a);
        assert_ne!(a, b);
        assert_eq!(a, CuDuration(100));
    }

    #[test]
    fn test_cuduration_arithmetic_operations() {
        let a = CuDuration(100);
        let b = CuDuration(50);

        assert_eq!(a + b, CuDuration(150));
        assert_eq!(a - b, CuDuration(50));
        assert_eq!(a * 2u32, CuDuration(200));
        assert_eq!(a / 2u32, CuDuration(50));
    }

    #[test]
    fn test_robot_clock_monotonic() {
        let clock = RobotClock::new();
        let t1 = clock.now();
        let t2 = clock.now();
        assert!(t2 >= t1);
    }

    #[test]
    fn test_robot_clock_mock() {
        let (clock, mock) = RobotClock::mock();
        let t1 = clock.now();
        mock.increment(CuDuration::from_millis(100));
        let t2 = clock.now();
        assert!(t2 > t1);
        assert_eq!(t2 - t1, CuDuration(100_000_000)); // 100ms in nanoseconds
    }

    #[test]
    fn test_robot_clock_clone_consistency() {
        let (clock1, mock) = RobotClock::mock();
        let clock2 = clock1.clone();

        mock.set_value(1_000_000_000); // 1 second
        assert_eq!(clock1.now(), clock2.now());
    }

    #[test]
    fn test_from_ref_time() {
        let tolerance_ms = 10f64;
        let clock = RobotClock::from_ref_time(1_000_000_000);
        assert_relative_eq!(
            clock.now().as_millis() as f64,
            CuDuration::from_secs(1).as_millis() as f64,
            epsilon = tolerance_ms
        );
    }

    #[test]
    fn longest_duration() {
        let maxcu = CuDuration(u64::MAX);
        assert_eq!(maxcu.as_nanos(), u64::MAX);
        let s = maxcu.as_secs();
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

    #[test]
    fn test_time_range_operations() {
        // Test creating a time range and checking its properties
        let start = CuTime::from(100u64);
        let end = CuTime::from(200u64);
        let range = CuTimeRange { start, end };

        assert_eq!(range.start, start);
        assert_eq!(range.end, end);

        // Test creating from a slice
        let times = [
            CuTime::from(150u64),
            CuTime::from(120u64),
            CuTime::from(180u64),
        ];
        let range_from_slice = CuTimeRange::from(&times[..]);

        // Range should capture min and max values
        assert_eq!(range_from_slice.start, CuTime::from(120u64));
        assert_eq!(range_from_slice.end, CuTime::from(180u64));
    }

    #[test]
    fn test_partial_time_range() {
        // Test creating a partial time range with defined start/end
        let start = CuTime::from(100u64);
        let end = CuTime::from(200u64);

        let partial_range = PartialCuTimeRange {
            start: OptionCuTime::from(start),
            end: OptionCuTime::from(end),
        };

        // Test converting to Option
        let opt_start: Option<CuTime> = partial_range.start.into();
        let opt_end: Option<CuTime> = partial_range.end.into();

        assert_eq!(opt_start, Some(start));
        assert_eq!(opt_end, Some(end));

        // Test partial range with undefined values
        let partial_undefined = PartialCuTimeRange::default();
        assert!(partial_undefined.start.is_none());
        assert!(partial_undefined.end.is_none());
    }

    #[test]
    fn test_tov_conversions() {
        // Test different Time of Validity (Tov) variants
        let time = CuTime::from(100u64);

        // Test conversion from CuTime
        let tov_time: Tov = time.into();
        assert!(matches!(tov_time, Tov::Time(_)));

        if let Tov::Time(t) = tov_time {
            assert_eq!(t, time);
        }

        // Test conversion from Option<CuTime>
        let some_time = Some(time);
        let tov_some: Tov = some_time.into();
        assert!(matches!(tov_some, Tov::Time(_)));

        let none_time: Option<CuDuration> = None;
        let tov_none: Tov = none_time.into();
        assert!(matches!(tov_none, Tov::None));

        // Test range
        let start = CuTime::from(100u64);
        let end = CuTime::from(200u64);
        let range = CuTimeRange { start, end };
        let tov_range = Tov::Range(range);

        assert!(matches!(tov_range, Tov::Range(_)));
    }

    #[cfg(feature = "std")]
    #[test]
    fn test_cuduration_display() {
        // Test the display implementation for different magnitudes
        let nano = CuDuration(42);
        assert_eq!(nano.to_string(), "42 ns");

        let micro = CuDuration(42_000);
        assert_eq!(micro.to_string(), "42.000 µs");

        let milli = CuDuration(42_000_000);
        assert_eq!(milli.to_string(), "42.000 ms");

        let sec = CuDuration(1_500_000_000);
        assert_eq!(sec.to_string(), "1.500 s");

        let min = CuDuration(90_000_000_000);
        assert_eq!(min.to_string(), "1.500 m");

        let hour = CuDuration(3_600_000_000_000);
        assert_eq!(hour.to_string(), "1.000 h");

        let day = CuDuration(86_400_000_000_000);
        assert_eq!(day.to_string(), "1.000 d");
    }

    #[test]
    fn test_robot_clock_precision() {
        // Test that RobotClock::now() and RobotClock::recent() return different values
        // and that recent() is always <= now()
        let clock = RobotClock::new();

        // We can't guarantee the exact values, but we can check relationships
        let recent = clock.recent();
        let now = clock.now();

        // recent() should be less than or equal to now()
        assert!(recent <= now);

        // Test precision of from_ref_time
        let ref_time_ns = 1_000_000_000; // 1 second
        let clock = RobotClock::from_ref_time(ref_time_ns);

        // Clock should start at approximately ref_time_ns
        let now = clock.now();
        let now_ns: u64 = now.into();

        // Allow reasonable tolerance for clock initialization time
        let tolerance_ns = 50_000_000; // 50ms tolerance
        assert!(now_ns >= ref_time_ns);
        assert!(now_ns < ref_time_ns + tolerance_ns);
    }

    #[test]
    fn test_mock_clock_advanced_operations() {
        // Test more complex operations with the mock clock
        let (clock, mock) = RobotClock::mock();

        // Test initial state
        assert_eq!(clock.now(), CuDuration(0));

        // Test increment
        mock.increment(CuDuration::from_secs(10));
        assert_eq!(clock.now(), CuDuration::from_secs(10));

        // Test decrement (unusual but supported)
        mock.decrement(CuDuration::from_secs(5));
        assert_eq!(clock.now(), CuDuration::from_secs(5));

        // Test setting absolute value
        mock.set_value(30_000_000_000); // 30 seconds in ns
        assert_eq!(clock.now(), CuDuration::from_secs(30));

        // Test that getting the time from the mock directly works
        assert_eq!(mock.now(), CuDuration::from_secs(30));
        assert_eq!(mock.value(), 30_000_000_000);
    }

    #[test]
    fn test_cuduration_min_max() {
        // Test MIN and MAX constants
        assert_eq!(CuDuration::MIN, CuDuration(0));

        // Test min/max methods
        let a = CuDuration(100);
        let b = CuDuration(200);

        assert_eq!(a.min(b), a);
        assert_eq!(a.max(b), b);
        assert_eq!(b.min(a), a);
        assert_eq!(b.max(a), b);

        // Edge cases
        assert_eq!(a.min(a), a);
        assert_eq!(a.max(a), a);

        // Test with MIN/MAX constants
        assert_eq!(a.min(CuDuration::MIN), CuDuration::MIN);
        assert_eq!(a.max(CuDuration::MAX), CuDuration::MAX);
    }

    #[test]
    fn test_clock_provider_trait() {
        // Test implementing the ClockProvider trait
        struct TestClockProvider {
            clock: RobotClock,
        }

        impl ClockProvider for TestClockProvider {
            fn get_clock(&self) -> RobotClock {
                self.clock.clone()
            }
        }

        // Create a provider with a mock clock
        let (clock, mock) = RobotClock::mock();
        let provider = TestClockProvider { clock };

        // Test that provider returns a clock synchronized with the original
        let provider_clock = provider.get_clock();
        assert_eq!(provider_clock.now(), CuDuration(0));

        // Advance the mock clock and check that the provider's clock also advances
        mock.increment(CuDuration::from_secs(5));
        assert_eq!(provider_clock.now(), CuDuration::from_secs(5));
    }
}
