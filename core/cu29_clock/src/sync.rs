//! Experimental, transport-independent reference discipline with fixed storage.
//!
//! One controller owns a clock's curve. Reference I/O belongs to the caller;
//! readers only evaluate a published integer affine mapping.

#[cfg(feature = "clock-sync")]
use crate::RobotClock;
use crate::{CuDuration, CuTime};
use bincode::{Decode, Encode};
#[cfg(feature = "clock-sync")]
use core::cell::UnsafeCell;
use core::fmt;
#[cfg(feature = "clock-sync")]
use portable_atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};
use serde::{Deserialize, Serialize};

const BILLION: i128 = 1_000_000_000;
#[cfg(feature = "clock-sync")]
const WINDOW: usize = 8;

/// Reference epoch identity. PTP uses TAI and the grandmaster identity.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, Encode, Decode)]
pub struct ClockDomain {
    /// PTP domain number or application domain identifier.
    pub id: u8,
    /// Root clock identity, unchanged across followers of the same parent.
    pub identity: [u8; 8],
    /// Reference session generation, changed when the reference resets.
    pub session: u64,
}

/// Paired counter/reference measurement at the same event.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, Encode, Decode)]
pub struct ClockObservation {
    /// Undisciplined local counter in nanoseconds.
    pub raw_local: CuTime,
    /// TAI/reference time at the capture event.
    pub parent_ns: u64,
    /// Upstream and capture error combined.
    pub uncertainty: CuDuration,
    /// Epoch and session identity.
    pub domain: ClockDomain,
}

/// Error and correction bounds. All rates are parts per billion.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, Encode, Decode)]
pub struct SyncConfig {
    /// Maximum usable timestamp uncertainty.
    pub max_error: CuDuration,
    /// Maximum age of an accepted reference measurement.
    pub max_age: CuDuration,
    /// Relative oscillator error during holdover.
    pub drift_bound_ppb: u32,
    /// Additional phase correction rate; must leave the total rate positive.
    pub max_slew_ppb: u32,
}

impl SyncConfig {
    /// Conservative starting bounds: five seconds, 100 ppm drift, 500 ppm slew.
    pub fn new(max_error: CuDuration) -> Self {
        Self {
            max_error,
            max_age: CuDuration::from_secs(5),
            drift_bound_ppb: 100_000,
            max_slew_ppb: 500_000,
        }
    }

    /// Validates positive error/age bounds and a strictly positive total rate.
    pub fn validate(self) -> Result<(), SyncError> {
        if self.max_error.0 == 0
            || self.max_age.0 == 0
            || u64::from(self.drift_bound_ppb) + u64::from(self.max_slew_ppb) >= 1_000_000_000
        {
            return Err(SyncError::InvalidConfig);
        }
        Ok(())
    }
}

/// Reference acquisition and health.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, Encode, Decode)]
pub enum SyncState {
    /// Collecting samples or slewing into the requested error budget.
    Acquiring,
    /// Tracking a usable reference within the error budget.
    Locked,
    /// Reference unavailable; extrapolating the last curve within its bounds.
    Holdover,
    /// Budget, age, or parent continuity violated; reacquisition is required.
    Expired,
}

/// Current quality of the execution timeline.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, Encode, Decode)]
pub struct SyncStatus {
    /// Current acquisition/health state.
    pub state: SyncState,
    /// Reference epoch identity.
    pub domain: ClockDomain,
    /// Raw elapsed time since the last accepted sample.
    pub sample_age: CuDuration,
    /// Capture, upstream, extrapolation and remaining phase error combined.
    pub estimated_error: CuDuration,
    /// Maximum combined timestamp uncertainty accepted by this clock.
    pub max_error: CuDuration,
    /// Signed parent minus execution time in nanoseconds.
    pub phase_offset_ns: i64,
    /// Estimated reference/counter rate difference in ppb.
    pub drift_ppb: i64,
    /// Oscillator error bound used to age uncertainty during holdover.
    pub drift_bound_ppb: u32,
}

/// Bounded errors that do not allocate, including on embedded targets.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SyncError {
    /// Invalid error/rate bounds or counter frequency.
    InvalidConfig,
    /// A controller already owns this clock.
    AlreadyControlled,
    /// Observation belongs to another epoch/session.
    WrongDomain,
    /// Observation is future-dated, stale, duplicate or reordered.
    InvalidSample,
    /// Reference jumped or measurements violate the oscillator bound.
    Discontinuity,
    /// Timestamp or arithmetic cannot be represented.
    OutOfRange,
    /// Synchronization expired and requires explicit resync.
    Expired,
    /// A publication was deferred because readers were active.
    ReadersActive,
}

impl fmt::Display for SyncError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "clock synchronization: {self:?}")
    }
}
impl core::error::Error for SyncError {}

/// Recorded affine mapping and quality, restored without reference I/O.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, Encode, Decode)]
pub struct ClockSnapshot {
    raw_anchor: u64,
    time_anchor: u64,
    rate_ppb: i64,
    quality: Option<Quality>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, Encode, Decode)]
struct Quality {
    domain: ClockDomain,
    config: SyncConfig,
    state: SyncState,
    sample: Option<ClockObservation>,
    drift_ppb: i64,
}

fn scaled(delta: i128, rate_ppb: i64) -> i128 {
    delta * (BILLION + i128::from(rate_ppb)) / BILLION
}
fn checked_time(value: i128) -> Result<u64, SyncError> {
    if !(0..=i128::from(CuTime::MAX.0)).contains(&value) {
        return Err(SyncError::OutOfRange);
    }
    Ok(value as u64)
}
fn ceil_drift(age: u64, ppb: u32) -> u64 {
    (u128::from(age) * u128::from(ppb))
        .div_ceil(BILLION as u128)
        .min(u128::from(u64::MAX)) as u64
}

impl ClockSnapshot {
    /// Evaluates the recorded execution curve at an undisciplined counter time.
    pub fn at(&self, raw: u64) -> Result<u64, SyncError> {
        checked_time(
            i128::from(self.time_anchor)
                + scaled(i128::from(raw) - i128::from(self.raw_anchor), self.rate_ppb),
        )
    }

    /// Ages the recorded quality against an undisciplined counter time.
    pub fn status(&self, raw: u64) -> Option<SyncStatus> {
        let q = self.quality?;
        let mut result = SyncStatus {
            state: q.state,
            domain: q.domain,
            sample_age: CuDuration(0),
            estimated_error: CuDuration::MAX,
            max_error: q.config.max_error,
            phase_offset_ns: 0,
            drift_ppb: q.drift_ppb,
            drift_bound_ppb: q.config.drift_bound_ppb,
        };
        if let Some(sample) = q.sample {
            let age = raw.saturating_sub(sample.raw_local.0);
            let parent = i128::from(sample.parent_ns) + scaled(i128::from(age), q.drift_ppb);
            let phase = self.at(raw).map(|now| parent - i128::from(now));
            result.sample_age = CuDuration(age);
            if let Ok(phase) = phase
                && let Ok(offset) = i64::try_from(phase)
            {
                result.phase_offset_ns = offset;
                result.estimated_error = CuDuration(
                    sample
                        .uncertainty
                        .0
                        .saturating_add(ceil_drift(age, q.config.drift_bound_ppb))
                        .saturating_add(offset.unsigned_abs()),
                );
            }
            if age > q.config.max_age.0 || result.estimated_error > q.config.max_error {
                // Acquisition is allowed to slew a large initial phase error.
                if q.state != SyncState::Acquiring || age > q.config.max_age.0 {
                    result.state = SyncState::Expired;
                }
            }
        }
        Some(result)
    }
}

/// Two immutable slots and a reader gate. Readers never wait for a writer.
#[cfg(feature = "clock-sync")]
#[derive(Debug)]
pub(crate) struct SharedClock {
    slots: [UnsafeCell<ClockSnapshot>; 2],
    active: AtomicUsize,
    readers: AtomicUsize,
    controlled: AtomicBool,
    last: AtomicU64,
}

// SAFETY: There is exactly one writer (claimed by ClockSync). It only writes
// the inactive slot after observing zero readers. Readers increment before
// loading active and decrement after copying. Sequential consistency orders
// this gate with publication: a reader starting after the writer's check can
// only read the unchanged active slot or the fully published new slot. A
// subsequent write cannot reuse the old slot until every reader has exited.
#[cfg(feature = "clock-sync")]
unsafe impl Sync for SharedClock {}

#[cfg(feature = "clock-sync")]
impl SharedClock {
    pub(crate) fn new(raw: u64, time: u64) -> Self {
        let curve = ClockSnapshot {
            raw_anchor: raw,
            time_anchor: time,
            rate_ppb: 0,
            quality: None,
        };
        Self {
            slots: [UnsafeCell::new(curve), UnsafeCell::new(curve)],
            active: AtomicUsize::new(0),
            readers: AtomicUsize::new(0),
            controlled: AtomicBool::new(false),
            last: AtomicU64::new(time),
        }
    }

    fn read_with<T>(&self, read: impl FnOnce(&ClockSnapshot) -> T) -> T {
        self.readers.fetch_add(1, Ordering::SeqCst);
        let index = self.active.load(Ordering::SeqCst);
        // SAFETY: the reader gate protects this immutable slot (see Sync impl).
        let value = read(unsafe { &*self.slots[index].get() });
        self.readers.fetch_sub(1, Ordering::SeqCst);
        value
    }

    pub(crate) fn read(&self) -> ClockSnapshot {
        self.read_with(|curve| *curve)
    }

    fn publish(&self, curve: ClockSnapshot) -> Result<(), SyncError> {
        if self.readers.load(Ordering::SeqCst) != 0 {
            return Err(SyncError::ReadersActive);
        }
        let next = 1 - self.active.load(Ordering::SeqCst);
        // SAFETY: controller ownership and reader gate protect the inactive slot.
        unsafe {
            *self.slots[next].get() = curve;
        }
        self.active.store(next, Ordering::SeqCst);
        Ok(())
    }

    pub(crate) fn now(&self, raw: CuTime) -> CuTime {
        let (time, synced) = self.read_with(|curve| {
            (
                curve.at(raw.0).unwrap_or(CuTime::MAX.0),
                curve.quality.is_some(),
            )
        });
        if synced {
            CuTime(self.last.fetch_max(time, Ordering::SeqCst).max(time))
        } else {
            CuTime(time)
        }
    }
}

/// Exclusive reference controller; observations and publication run outside process().
#[cfg(feature = "clock-sync")]
pub struct ClockSync {
    clock: RobotClock,
    domain: ClockDomain,
    config: SyncConfig,
    observations: [Option<ClockObservation>; WINDOW],
    len: usize,
    aligned: bool,
    invalid: bool,
    unavailable: bool,
}

#[cfg(feature = "clock-sync")]
impl ClockSync {
    /// Attaches a single controller to the shared execution clock.
    pub fn new(
        clock: &RobotClock,
        domain: ClockDomain,
        config: SyncConfig,
    ) -> Result<Self, SyncError> {
        config.validate()?;
        clock
            .mapping
            .controlled
            .compare_exchange(false, true, Ordering::SeqCst, Ordering::SeqCst)
            .map_err(|_| SyncError::AlreadyControlled)?;
        let aligned = clock.mapping.read().quality.is_some();
        Ok(Self {
            clock: clock.clone(),
            domain,
            config,
            observations: [None; WINDOW],
            len: 0,
            aligned,
            invalid: false,
            unavailable: false,
        })
    }

    /// Validates a paired capture using raw time, never corrected execution time.
    pub fn observe(&mut self, sample: ClockObservation) -> Result<(), SyncError> {
        if self.invalid
            || self
                .clock
                .sync_status()
                .is_some_and(|s| s.state == SyncState::Expired)
        {
            self.invalid = true;
            return Err(SyncError::Expired);
        }
        if sample.domain != self.domain {
            return Err(SyncError::WrongDomain);
        }
        let raw = self.clock.raw_now().0;
        if sample.raw_local.0 > raw
            || raw - sample.raw_local.0 > self.config.max_age.0
            || sample.uncertainty > self.config.max_error
        {
            return Err(SyncError::InvalidSample);
        }
        checked_time(i128::from(sample.parent_ns))?;
        if let Some(last) = self.len.checked_sub(1).and_then(|i| self.observations[i]) {
            if sample.raw_local <= last.raw_local {
                return Err(SyncError::InvalidSample);
            }
            let elapsed = sample.raw_local.0 - last.raw_local.0;
            let residual =
                (i128::from(sample.parent_ns) - i128::from(last.parent_ns) - i128::from(elapsed))
                    .unsigned_abs();
            let bound = u128::from(ceil_drift(elapsed, self.config.drift_bound_ppb))
                + u128::from(last.uncertainty.0)
                + u128::from(sample.uncertainty.0);
            if residual > bound {
                self.invalid = true;
                return Err(SyncError::Discontinuity);
            }
        }
        if self.len == WINDOW {
            self.observations.rotate_left(1);
            self.len -= 1;
        }
        self.observations[self.len] = Some(sample);
        self.len += 1;
        self.unavailable = false;
        Ok(())
    }

    /// Marks unavailable upstream quality even when the hardware clock still ticks.
    pub fn reference_lost(&mut self) {
        self.unavailable = true;
    }

    /// Publishes a continuous curve. Initial acquisition is the only epoch step.
    pub fn update(&mut self) -> Result<SyncStatus, SyncError> {
        let raw = self.clock.raw_now().0;
        let mut curve = self.clock.mapping.read();
        if curve
            .status(raw)
            .is_some_and(|s| s.state == SyncState::Expired)
        {
            self.invalid = true;
        }
        let sample = self.len.checked_sub(1).and_then(|i| self.observations[i]);
        let mut drift = 0;
        if let (Some(first), Some(last)) = (self.observations[0], sample)
            && first.raw_local < last.raw_local
        {
            let delta = i128::from(last.raw_local.0 - first.raw_local.0);
            drift = (((i128::from(last.parent_ns) - i128::from(first.parent_ns) - delta) * BILLION
                / delta)
                .clamp(
                    -i128::from(self.config.drift_bound_ppb),
                    i128::from(self.config.drift_bound_ppb),
                )) as i64;
        }
        let state = if self.invalid {
            SyncState::Expired
        } else if self.unavailable && self.aligned {
            SyncState::Holdover
        } else if curve.quality.is_some_and(|q| q.state == SyncState::Locked) {
            SyncState::Locked
        } else {
            SyncState::Acquiring
        };
        curve.quality = Some(Quality {
            domain: self.domain,
            config: self.config,
            state,
            sample,
            drift_ppb: drift,
        });
        if let Some(sample) = sample
            && !self.invalid
        {
            let age = raw.saturating_sub(sample.raw_local.0);
            let parent =
                checked_time(i128::from(sample.parent_ns) + scaled(i128::from(age), drift))?;
            if !self.aligned {
                curve.raw_anchor = raw;
                curve.time_anchor = parent;
                curve.rate_ppb = drift;
            } else {
                let current = curve
                    .at(raw)?
                    .max(self.clock.mapping.last.load(Ordering::SeqCst));
                let phase = i128::from(parent) - i128::from(current);
                curve.raw_anchor = raw;
                curve.time_anchor = current;
                // Correct phase over one second, bounded by the requested slew.
                let slew = phase.clamp(
                    -i128::from(self.config.max_slew_ppb),
                    i128::from(self.config.max_slew_ppb),
                );
                if !self.unavailable {
                    curve.rate_ppb = drift + slew as i64;
                }
            }
            let status = curve.status(raw).ok_or(SyncError::InvalidSample)?;
            if status.state != SyncState::Expired
                && status.estimated_error <= self.config.max_error
                && self.len >= 2
                && let Some(q) = &mut curve.quality
            {
                q.state = if self.unavailable {
                    SyncState::Holdover
                } else {
                    SyncState::Locked
                };
            }
        }
        self.clock.mapping.publish(curve)?;
        if sample.is_some() {
            self.aligned = true;
        }
        let status = curve.status(raw).ok_or(SyncError::InvalidSample)?;
        if status.state == SyncState::Expired {
            self.invalid = true;
        }
        Ok(status)
    }

    /// Current health, aged against the raw counter.
    pub fn status(&self) -> SyncStatus {
        let mut status = self.clock.sync_status().unwrap_or(SyncStatus {
            state: SyncState::Acquiring,
            domain: self.domain,
            sample_age: CuDuration(0),
            estimated_error: CuDuration::MAX,
            max_error: self.config.max_error,
            phase_offset_ns: 0,
            drift_ppb: 0,
            drift_bound_ppb: self.config.drift_bound_ppb,
        });
        if self.invalid {
            status.state = SyncState::Expired;
        }
        status
    }

    /// Clears reference history while preserving the already published timeline.
    pub fn resync(&mut self, domain: ClockDomain) -> Result<(), SyncError> {
        self.domain = domain;
        self.len = 0;
        self.observations = [None; WINDOW];
        self.invalid = false;
        self.unavailable = false;
        self.update()?;
        Ok(())
    }

    /// Captures the published curve and quality for offline replay.
    pub fn snapshot(&self) -> ClockSnapshot {
        self.clock.mapping.read()
    }

    /// Attaches a replay controller to a recorded curve without a live reference.
    pub fn from_snapshot(clock: &RobotClock, snapshot: ClockSnapshot) -> Result<Self, SyncError> {
        let quality = snapshot.quality.ok_or(SyncError::InvalidSample)?;
        let mut sync = Self::new(clock, quality.domain, quality.config)?;
        sync.restore(snapshot)?;
        Ok(sync)
    }

    /// Sets a replay mock from a recorded execution timestamp, converting back
    /// to the recorded raw-counter timeline before evaluating the clock curve.
    pub fn set_replay_time(
        &mut self,
        mock: &crate::RobotClockMock,
        time: CuTime,
    ) -> Result<(), SyncError> {
        if !self.clock.is_mock() {
            return Err(SyncError::InvalidConfig);
        }
        let curve = self.snapshot();
        let delta = i128::from(time.0) - i128::from(curve.time_anchor);
        let rate = BILLION + i128::from(curve.rate_ppb);
        let scaled = delta * BILLION;
        let ticks =
            scaled.div_euclid(rate) + i128::from(delta >= 0 && scaled.rem_euclid(rate) != 0);
        let raw = checked_time(i128::from(curve.raw_anchor) + ticks)?;
        if curve.at(raw)? != time.0 {
            return Err(SyncError::OutOfRange);
        }
        mock.set_value(raw);
        self.clock.mapping.last.store(time.0, Ordering::SeqCst);
        Ok(())
    }

    /// Restores a recorded curve on this controller's clock without parent I/O.
    pub fn restore(&mut self, snapshot: ClockSnapshot) -> Result<(), SyncError> {
        if snapshot.rate_ppb.unsigned_abs() >= 1_000_000_000 {
            return Err(SyncError::OutOfRange);
        }
        checked_time(i128::from(snapshot.time_anchor))?;
        if let Some(quality) = snapshot.quality {
            quality.config.validate()?;
        }
        self.clock.mapping.publish(snapshot)?;
        self.clock
            .mapping
            .last
            .store(snapshot.time_anchor, Ordering::SeqCst);
        self.aligned = snapshot.quality.is_some();
        Ok(())
    }
}

#[cfg(feature = "clock-sync")]
impl Drop for ClockSync {
    fn drop(&mut self) {
        self.clock.mapping.controlled.store(false, Ordering::SeqCst);
    }
}

#[cfg(all(test, feature = "clock-sync"))]
mod tests {
    use super::*;
    const DOMAIN: ClockDomain = ClockDomain {
        id: 0,
        identity: *b"testroot",
        session: 1,
    };
    const EPOCH: u64 = 1_800_000_000_000_000_000;

    fn observe(sync: &mut ClockSync, raw: u64, parent: u64, error: u64) -> Result<(), SyncError> {
        sync.observe(ClockObservation {
            raw_local: CuTime(raw),
            parent_ns: parent,
            uncertainty: CuDuration(error),
            domain: DOMAIN,
        })
    }

    fn locked() -> (RobotClock, crate::RobotClockMock, ClockSync) {
        let (clock, mock) = RobotClock::mock();
        let mut sync =
            ClockSync::new(&clock, DOMAIN, SyncConfig::new(CuDuration(200_000))).unwrap();
        observe(&mut sync, 0, EPOCH, 100).unwrap();
        sync.update().unwrap();
        mock.set_value(1_000_000_000);
        observe(&mut sync, mock.value(), EPOCH + mock.value(), 100).unwrap();
        assert_eq!(sync.update().unwrap().state, SyncState::Locked);
        (clock, mock, sync)
    }

    #[cfg(feature = "std")]
    #[test]
    fn test_first_known_frequency_constructor_supports_instant_clock() {
        let clock = RobotClock::new_with_frequency(1_000_000_000).unwrap();
        let instant = crate::CuInstant::now();
        assert!(crate::CuInstant::now() >= instant);
        assert!(clock.now().0 < 1_000_000_000);
    }

    #[cfg(feature = "std")]
    #[test]
    fn test_custom_and_known_frequency_clocks_do_not_recalibrate_existing_clones() {
        let clock = RobotClock::new();
        let clone = clock.clone();
        let before = clock.now();
        let rtc = alloc::sync::Arc::new(portable_atomic::AtomicU64::new(0));
        let custom =
            RobotClock::new_with_rtc(move || rtc.fetch_add(10_000_000, Ordering::Relaxed), |_| {});
        let _known = RobotClock::new_with_frequency(1).unwrap();
        assert!(clock.now() >= before);
        assert!(clone.now().0 - before.0 < 1_000_000_000);
        assert_eq!(clock.inner.frequency, clone.inner.frequency);
        assert_ne!(clock.inner.frequency, custom.inner.frequency);
        assert_eq!(
            RobotClock::new_with_frequency(0).unwrap_err(),
            SyncError::InvalidConfig
        );
    }

    #[test]
    fn test_recorded_curve_seek_and_raw_mock_semantics() {
        let (clock, mock, mut sync) = locked();
        mock.set_value(2_000_000_000);
        observe(&mut sync, mock.value(), EPOCH + mock.value() + 20_000, 100).unwrap();
        sync.update().unwrap();
        let snapshot = sync.snapshot();
        let recorded =
            [1_999_999_000, 2_000_000_000, 2_000_001_000].map(|raw| snapshot.at(raw).unwrap());
        for time in recorded.into_iter().rev() {
            sync.restore(snapshot).unwrap();
            sync.set_replay_time(&mock, CuTime(time)).unwrap();
            assert_eq!(clock.now().0, time);
            assert_eq!(clock.recent().0, time);
            assert_eq!(snapshot.at(mock.value()).unwrap(), time);
            assert_eq!(clock.sync_status().unwrap().domain, DOMAIN);
        }
    }

    #[test]
    fn test_two_boot_epochs_and_counter_rates_share_reference() {
        let (a, ma) = RobotClock::mock();
        let (b, mb) = RobotClock::mock();
        let config = SyncConfig::new(CuDuration(200_000));
        let mut sa = ClockSync::new(&a, DOMAIN, config).unwrap();
        let mut sb = ClockSync::new(&b, DOMAIN, config).unwrap();
        for second in 0..10u64 {
            let parent = EPOCH + second * 1_000_000_000;
            ma.set_value(5_000_000_000 + second * 1_000_020_000);
            mb.set_value(80_000_000_000 + second * 999_980_000);
            observe(&mut sa, ma.value(), parent, 100).unwrap();
            observe(&mut sb, mb.value(), parent, 100).unwrap();
            let qa = sa.update().unwrap();
            let qb = sb.update().unwrap();
            if second > 0 {
                assert_eq!(qa.state, SyncState::Locked);
                assert_eq!(qb.state, SyncState::Locked);
                assert!(a.now().0.abs_diff(b.now().0) < 50_000);
            }
        }
        assert!(sa.status().drift_ppb < 0);
        assert!(sb.status().drift_ppb > 0);
        assert_eq!(a.recent(), a.clone().now());
        assert_eq!(a.raw_now(), ma.now());
    }

    #[test]
    fn test_slew_and_resync_never_step_backward() {
        let (clock, mock, mut sync) = locked();
        let before = clock.now();
        mock.increment(CuDuration::from_secs(1));
        observe(&mut sync, mock.value(), EPOCH + mock.value() - 30_000, 100).unwrap();
        let status = sync.update().unwrap();
        assert!(status.phase_offset_ns < 0);
        assert_eq!(clock.now().0, before.0 + 1_000_000_000);
        let mut previous = clock.now();
        for _ in 0..100 {
            mock.increment(CuDuration::from_millis(1));
            let current = clock.now();
            assert!(current >= previous);
            previous = current;
        }
        sync.resync(DOMAIN).unwrap();
        assert_eq!(clock.now(), previous);
        observe(&mut sync, mock.value(), EPOCH + mock.value() + 20_000, 100).unwrap();
        sync.update().unwrap();
        assert_eq!(clock.now(), previous);
    }

    #[test]
    fn test_reference_loss_error_growth_and_sticky_expiry() {
        let (clock, mock, mut sync) = locked();
        sync.reference_lost();
        assert_eq!(sync.update().unwrap().state, SyncState::Holdover);
        mock.increment(CuDuration::from_secs(1));
        assert_eq!(sync.status().estimated_error.0, 100_100);
        mock.increment(CuDuration::from_secs(1));
        assert_eq!(sync.status().state, SyncState::Expired);
        assert_eq!(sync.update().unwrap().state, SyncState::Expired);
        assert_eq!(
            observe(&mut sync, mock.value(), EPOCH + mock.value(), 100),
            Err(SyncError::Expired)
        );
        assert_eq!(clock.now().0, EPOCH + mock.value());
    }

    #[test]
    fn test_reject_domains_reordering_future_and_parent_reset() {
        let (_, mock, mut sync) = locked();
        let sample = ClockObservation {
            raw_local: mock.now(),
            parent_ns: EPOCH,
            uncertainty: CuDuration(100),
            domain: ClockDomain {
                session: 2,
                ..DOMAIN
            },
        };
        assert_eq!(sync.observe(sample), Err(SyncError::WrongDomain));
        assert_eq!(
            observe(&mut sync, mock.value(), EPOCH, 100),
            Err(SyncError::InvalidSample)
        );
        assert_eq!(
            observe(&mut sync, mock.value() + 1, EPOCH, 100),
            Err(SyncError::InvalidSample)
        );
        mock.increment(CuDuration::from_secs(1));
        assert_eq!(
            observe(&mut sync, mock.value(), EPOCH, 100),
            Err(SyncError::Discontinuity)
        );
        assert_eq!(sync.status().state, SyncState::Expired);
        assert_eq!(sync.update().unwrap().state, SyncState::Expired);
    }

    #[test]
    fn test_single_owner_config_validation_and_overflow() {
        let (clock, mock) = RobotClock::mock();
        assert!(matches!(
            ClockSync::new(&clock, DOMAIN, SyncConfig::new(CuDuration(0))),
            Err(SyncError::InvalidConfig)
        ));
        let mut sync =
            ClockSync::new(&clock, DOMAIN, SyncConfig::new(CuDuration(1_000_000))).unwrap();
        assert!(matches!(
            ClockSync::new(&clock.clone(), DOMAIN, sync.config),
            Err(SyncError::AlreadyControlled)
        ));
        assert_eq!(
            observe(&mut sync, 0, u64::MAX, 1),
            Err(SyncError::OutOfRange)
        );
        observe(&mut sync, 0, CuTime::MAX.0 - 10, 1).unwrap();
        mock.set_value(100);
        assert!(matches!(sync.update(), Err(SyncError::OutOfRange)));
        drop(sync);
        assert!(ClockSync::new(&clock, DOMAIN, SyncConfig::new(CuDuration(1))).is_ok());
        assert!(matches!(
            RobotClock::new_with_frequency(0),
            Err(SyncError::InvalidConfig)
        ));
    }

    #[test]
    fn test_snapshot_roundtrip_replays_without_live_parent() {
        let (clock, mock, sync) = locked();
        let bytes = bincode::encode_to_vec(sync.snapshot(), bincode::config::standard()).unwrap();
        let (snapshot, _): (ClockSnapshot, _) =
            bincode::decode_from_slice(&bytes, bincode::config::standard()).unwrap();
        let (replay, replay_mock) = RobotClock::mock();
        replay_mock.set_value(mock.value());
        let mut replay_sync = ClockSync::new(&replay, DOMAIN, sync.config).unwrap();
        replay_sync.restore(snapshot).unwrap();
        for _ in 0..100 {
            mock.increment(CuDuration::from_millis(1));
            replay_mock.increment(CuDuration::from_millis(1));
            assert_eq!(clock.now(), replay.now());
            assert_eq!(clock.sync_status(), replay.sync_status());
        }
    }

    #[test]
    fn test_readers_defer_publication_without_waiting() {
        let (_, _, mut sync) = locked();
        sync.clock.mapping.readers.fetch_add(1, Ordering::SeqCst);
        assert_eq!(sync.update(), Err(SyncError::ReadersActive));
        assert_eq!(sync.clock.now().0, EPOCH + 1_000_000_000);
        sync.clock.mapping.readers.fetch_sub(1, Ordering::SeqCst);
        assert!(sync.update().is_ok());
    }

    #[cfg(feature = "std")]
    #[test]
    fn test_concurrent_readers_see_coherent_monotonic_curves() {
        let (clock, mock, mut sync) = locked();
        std::thread::scope(|scope| {
            for _ in 0..4 {
                let clock = &clock;
                scope.spawn(move || {
                    let mut last = clock.now();
                    for _ in 0..20_000 {
                        let now = clock.now();
                        assert!(now >= last);
                        last = now;
                        let q = clock.sync_status().unwrap();
                        assert_eq!(q.domain, DOMAIN);
                        assert!(q.estimated_error.0 < 300_000);
                    }
                });
            }
            for _ in 0..1000 {
                mock.increment(CuDuration::from_micros(1));
                if let Err(err) = sync.update() {
                    assert_eq!(err, SyncError::ReadersActive);
                }
            }
        });
    }
}
