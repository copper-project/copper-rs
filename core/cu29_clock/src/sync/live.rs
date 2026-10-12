//! Live reference discipline and publication to the shared execution clock.

use super::{
    BILLION, ClockDomain, ClockObservation, ClockSnapshot, Quality, SyncConfig, SyncError,
    SyncState, SyncStatus, ceil_drift, checked_time, scaled,
};
use crate::{CuDuration, CuInstant, CuTime, RobotClock};
use core::cell::UnsafeCell;
use portable_atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};

const WINDOW: usize = 8;

/// Two immutable slots and a reader gate. Readers never wait for a writer.
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
unsafe impl Sync for SharedClock {}

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

    pub(crate) fn now(&self, raw: CuInstant) -> CuTime {
        let (time, synced) = self.read_with(|curve| {
            (
                curve.at(raw).unwrap_or(CuTime::MAX).as_nanos(),
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
        let raw = self.clock.raw_now();
        if sample.raw_local > raw
            || raw - sample.raw_local > self.config.max_age
            || sample.uncertainty > self.config.max_error
        {
            return Err(SyncError::InvalidSample);
        }
        checked_time(i128::from(sample.parent_ns))?;
        if let Some(last) = self.len.checked_sub(1).and_then(|i| self.observations[i]) {
            if sample.raw_local <= last.raw_local {
                return Err(SyncError::InvalidSample);
            }
            let elapsed = (sample.raw_local - last.raw_local).as_nanos();
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
        self.update_at(self.clock.raw_now())
    }

    fn update_at(&mut self, raw: CuInstant) -> Result<SyncStatus, SyncError> {
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
            let delta = i128::from((last.raw_local - first.raw_local).as_nanos());
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
            let age = raw.as_nanos().saturating_sub(sample.raw_local.as_nanos());
            let parent =
                checked_time(i128::from(sample.parent_ns) + scaled(i128::from(age), drift))?;
            if !self.aligned {
                curve.raw_anchor = raw.as_nanos();
                curve.time_anchor = parent;
                curve.rate_ppb = drift;
            } else {
                // A reader may have advanced last beyond this captured instant.
                // Keep the anchors paired; now() applies the monotonic floor.
                let current = curve.at(raw)?.as_nanos();
                let phase = i128::from(parent) - i128::from(current);
                curve.raw_anchor = raw.as_nanos();
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

    /// Restarts reference acquisition while preserving the execution curve.
    ///
    /// A deferred publication leaves the controller and its history unchanged.
    pub fn resync(&mut self, domain: ClockDomain) -> Result<(), SyncError> {
        let mut curve = self.clock.mapping.read();
        curve.quality = Some(Quality {
            domain,
            config: self.config,
            state: SyncState::Acquiring,
            sample: None,
            drift_ppb: 0,
        });
        self.clock.mapping.publish(curve)?;
        self.domain = domain;
        self.len = 0;
        self.observations = [None; WINDOW];
        self.invalid = false;
        self.unavailable = false;
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
        if curve.at(CuInstant::from_nanos(raw))? != time {
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

impl Drop for ClockSync {
    fn drop(&mut self) {
        self.clock.mapping.controlled.store(false, Ordering::SeqCst);
    }
}

#[cfg(test)]
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
            raw_local: CuInstant::from_nanos(raw),
            parent_ns: parent,
            uncertainty: CuDuration(error),
            domain: DOMAIN,
        })
    }

    fn locked() -> (RobotClock, crate::RobotClockMock, ClockSync) {
        locked_with_config(SyncConfig::new(CuDuration(200_000)))
    }

    fn locked_with_config(config: SyncConfig) -> (RobotClock, crate::RobotClockMock, ClockSync) {
        let (clock, mock) = RobotClock::mock();
        let mut sync = ClockSync::new(&clock, DOMAIN, config).unwrap();
        observe(&mut sync, 0, EPOCH, 100).unwrap();
        sync.update().unwrap();
        mock.set_value(1_000_000_000);
        observe(&mut sync, mock.value(), EPOCH + mock.value(), 100).unwrap();
        assert_eq!(sync.update().unwrap().state, SyncState::Locked);
        (clock, mock, sync)
    }

    #[test]
    fn test_updates_preserve_curve_when_readers_advance_after_raw_capture() {
        for config in [
            SyncConfig::new(CuDuration(200_000)),
            SyncConfig {
                drift_bound_ppb: 0,
                max_slew_ppb: 0,
                ..SyncConfig::new(CuDuration(200_000))
            },
        ] {
            let (clock, mock, mut sync) = locked_with_config(config);
            let captured_raw = clock.raw_now();
            let captured_time = sync.snapshot().at(captured_raw).unwrap();

            // A reader runs after update captures raw time, before it reanchors.
            mock.increment(CuDuration::from_millis(10));
            let before = clock.now();
            assert_eq!(before.0, captured_time.0 + 10_000_000);

            assert_eq!(
                sync.update_at(captured_raw).unwrap().state,
                SyncState::Locked
            );
            assert_eq!(sync.snapshot().at(captured_raw).unwrap(), captured_time);
            assert_eq!(sync.snapshot().rate_ppb, 0);
            assert_eq!(clock.now(), before);
            assert_eq!(clock.recent(), before);
            assert_eq!(sync.status().phase_offset_ns, 0);

            mock.increment(CuDuration::from_millis(1));
            assert_eq!(clock.now().0, before.0 + 1_000_000);
        }
    }

    #[test]
    fn test_updates_preserve_slewing_curve_when_readers_advance_after_raw_capture() {
        let (clock, mock, mut sync) = locked();
        mock.increment(CuDuration::from_secs(1));
        observe(&mut sync, mock.value(), EPOCH + mock.value() + 20_000, 100).unwrap();
        sync.update().unwrap();
        let captured_raw = clock.raw_now();
        let curve = sync.snapshot();
        assert_eq!(curve.rate_ppb, 30_000);

        mock.increment(CuDuration::from_millis(10));
        let before = clock.now();
        assert_eq!(
            sync.update_at(captured_raw).unwrap().state,
            SyncState::Locked
        );
        assert_eq!(sync.snapshot(), curve);
        assert_eq!(clock.now(), before);
        assert_eq!(clock.recent(), before);
    }

    #[cfg(feature = "std")]
    #[test]
    fn test_known_frequency_counter_is_local_to_clock() {
        let clock = RobotClock::new_with_frequency(1_000_000_000).unwrap();
        let raw = clock.raw_now();
        let clone = clock.clone();
        assert!(clone.raw_now() >= raw);
        assert!(clock.now().0 < 1_000_000_000);
    }

    #[cfg(feature = "std")]
    #[test]
    fn test_known_frequency_clock_does_not_recalibrate_existing_clones() {
        let clock = RobotClock::new();
        let clone = clock.clone();
        let before = clock.now();
        let _known = RobotClock::new_with_frequency(1).unwrap();
        assert!(clock.now() >= before);
        assert!(clone.now().0 - before.0 < 1_000_000_000);
        assert_eq!(clock.inner.frequency, clone.inner.frequency);
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
        let recorded = [1_999_999_000, 2_000_000_000, 2_000_001_000]
            .map(|raw| snapshot.at(CuInstant::from_nanos(raw)).unwrap());
        for time in recorded.into_iter().rev() {
            sync.restore(snapshot).unwrap();
            sync.set_replay_time(&mock, time).unwrap();
            assert_eq!(clock.now(), time);
            assert_eq!(clock.recent(), time);
            assert_eq!(snapshot.at(mock.raw_now()).unwrap(), time);
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
        assert_eq!(a.raw_now(), ma.raw_now());
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
        assert_eq!(sync.status().state, SyncState::Acquiring);
        assert_eq!(sync.status().estimated_error, CuDuration::MAX);
        observe(&mut sync, mock.value(), EPOCH + mock.value() + 20_000, 100).unwrap();
        assert_eq!(sync.update().unwrap().state, SyncState::Acquiring);
        assert_eq!(clock.now(), previous);
    }

    #[test]
    fn test_resync_after_age_or_error_expiry_reacquires_without_step() {
        for (config, elapsed) in [
            (
                SyncConfig::new(CuDuration(200_000)),
                CuDuration::from_secs(2),
            ),
            (
                SyncConfig::new(CuDuration::from_millis(10)),
                CuDuration::from_secs(6),
            ),
        ] {
            for publish_expiry in [false, true] {
                let (clock, mock, mut sync) = locked_with_config(config);
                sync.reference_lost();
                sync.update().unwrap();
                mock.increment(elapsed);
                assert_eq!(clock.sync_status().unwrap().state, SyncState::Expired);
                if publish_expiry {
                    assert_eq!(sync.update().unwrap().state, SyncState::Expired);
                }
                let curve = sync.snapshot();
                let before = clock.now();

                sync.resync(DOMAIN).unwrap();
                let restarted = sync.snapshot();
                assert_eq!(restarted.raw_anchor, curve.raw_anchor);
                assert_eq!(restarted.time_anchor, curve.time_anchor);
                assert_eq!(restarted.rate_ppb, curve.rate_ppb);
                assert_eq!(clock.now(), before);
                assert_eq!(clock.sync_status().unwrap().state, SyncState::Acquiring);

                observe(&mut sync, mock.value(), EPOCH + mock.value(), 100).unwrap();
                assert_eq!(sync.update().unwrap().state, SyncState::Acquiring);
                assert_eq!(clock.now(), before);
                mock.increment(CuDuration::from_secs(1));
                observe(&mut sync, mock.value(), EPOCH + mock.value(), 100).unwrap();
                assert_eq!(sync.update().unwrap().state, SyncState::Locked);
                assert_eq!(clock.now().0, before.0 + 1_000_000_000);
            }
        }
    }

    #[test]
    fn test_resync_after_discontinuity_reacquires_new_domain() {
        let (clock, mock, mut sync) = locked();
        mock.increment(CuDuration::from_secs(1));
        assert_eq!(
            observe(
                &mut sync,
                mock.value(),
                EPOCH + mock.value() + 10_000_000,
                100
            ),
            Err(SyncError::Discontinuity)
        );
        assert_eq!(sync.update().unwrap().state, SyncState::Expired);
        let before = clock.now();
        let domain = ClockDomain {
            session: DOMAIN.session + 1,
            ..DOMAIN
        };
        sync.resync(domain).unwrap();
        assert_eq!(clock.now(), before);
        assert_eq!(clock.sync_status().unwrap().domain, domain);
        assert_eq!(sync.status().state, SyncState::Acquiring);
        assert_eq!(
            observe(&mut sync, mock.value(), EPOCH + mock.value(), 100),
            Err(SyncError::WrongDomain)
        );

        for expected in [SyncState::Acquiring, SyncState::Locked] {
            sync.observe(ClockObservation {
                raw_local: mock.raw_now(),
                parent_ns: EPOCH + mock.value(),
                uncertainty: CuDuration(100),
                domain,
            })
            .unwrap();
            assert_eq!(sync.update().unwrap().state, expected);
            assert_eq!(clock.now().0, EPOCH + mock.value());
            mock.increment(CuDuration::from_secs(1));
        }
    }

    #[test]
    fn test_resync_slews_large_positive_and_negative_offsets_into_lock() {
        for offset in [-250_000_000_i64, 250_000_000] {
            let (clock, mock, mut sync) = locked();
            sync.reference_lost();
            sync.update().unwrap();
            mock.increment(CuDuration::from_secs(60));
            assert_eq!(sync.update().unwrap().state, SyncState::Expired);
            let before = clock.now();
            let parent_at = |raw: u64| {
                checked_time(i128::from(EPOCH) + i128::from(raw) + i128::from(offset)).unwrap()
            };

            sync.resync(DOMAIN).unwrap();
            observe(&mut sync, mock.value(), parent_at(mock.value()), 100).unwrap();
            let status = sync.update().unwrap();
            assert_eq!(status.state, SyncState::Acquiring);
            assert_eq!(status.phase_offset_ns, offset);
            assert_eq!(clock.now(), before);

            let mut previous = before;
            for second in 1..=500_i64 {
                mock.increment(CuDuration::from_secs(1));
                let current = clock.now();
                let elapsed = if offset > 0 {
                    1_000_500_000
                } else {
                    999_500_000
                };
                assert_eq!((current - previous).as_nanos(), elapsed);
                observe(&mut sync, mock.value(), parent_at(mock.value()), 100).unwrap();
                let status = sync.update().unwrap();
                assert_eq!(clock.now(), current);
                assert_eq!(
                    status.phase_offset_ns,
                    offset - offset.signum() * second * 500_000
                );
                assert_eq!(
                    status.state,
                    if second == 500 {
                        SyncState::Locked
                    } else {
                        SyncState::Acquiring
                    }
                );
                previous = current;
            }
            assert_eq!(clock.now().0, parent_at(mock.value()));
            assert!(sync.status().estimated_error <= sync.status().max_error);
        }
    }

    #[test]
    fn test_resync_defers_without_clearing_expired_controller() {
        let (clock, mock, mut sync) = locked();
        sync.reference_lost();
        sync.update().unwrap();
        mock.increment(CuDuration::from_secs(2));
        assert_eq!(sync.update().unwrap().state, SyncState::Expired);
        let curve = sync.snapshot();
        let observations = sync.observations;
        let last = clock.mapping.last.load(Ordering::SeqCst);
        let domain = ClockDomain {
            session: 2,
            ..DOMAIN
        };

        clock.mapping.readers.fetch_add(1, Ordering::SeqCst);
        let result = sync.resync(domain);
        clock.mapping.readers.fetch_sub(1, Ordering::SeqCst);
        assert_eq!(result, Err(SyncError::ReadersActive));
        assert_eq!(sync.snapshot(), curve);
        assert_eq!(clock.mapping.last.load(Ordering::SeqCst), last);
        assert_eq!(sync.domain, DOMAIN);
        assert_eq!(sync.observations, observations);
        assert_eq!(sync.len, 2);
        assert!(sync.invalid);
        assert!(sync.unavailable);

        sync.resync(domain).unwrap();
        assert_eq!(sync.status().state, SyncState::Acquiring);
        assert_eq!(sync.status().domain, domain);
        assert_eq!(clock.mapping.last.load(Ordering::SeqCst), last);
        assert_eq!(sync.len, 0);
        assert!(!sync.invalid);
        assert!(!sync.unavailable);
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
            raw_local: mock.raw_now(),
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
