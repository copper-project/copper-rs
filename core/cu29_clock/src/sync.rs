//! Transport-independent reference discipline with fixed storage.
//!
//! One controller owns a clock's curve. Reference I/O belongs to the caller;
//! readers only evaluate a published integer affine mapping.

use crate::{ClockOrigin, CuDuration, CuInstant, CuTime};
use bincode::{Decode, Encode};
use core::fmt;
use serde::{Deserialize, Serialize};

const BILLION: i128 = 1_000_000_000;

/// Reference identity and session, independent of timestamp epoch.
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
    pub raw_local: CuInstant,
    /// Provider reference time at the capture event, without epoch conversion.
    pub parent_ns: u64,
    /// Source and epoch of the supplied reference time.
    pub origin: ClockOrigin,
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
    origin: ClockOrigin,
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
    /// Recorded provenance and epoch of the execution mapping.
    pub fn origin(&self) -> ClockOrigin {
        self.origin
    }

    /// Evaluates the recorded execution curve at an undisciplined counter time.
    pub fn at(&self, raw: CuInstant) -> Result<CuTime, SyncError> {
        checked_time(
            i128::from(self.time_anchor)
                + scaled(
                    i128::from(raw.as_nanos()) - i128::from(self.raw_anchor),
                    self.rate_ppb,
                ),
        )
        .map(CuTime)
    }

    /// Ages the recorded quality against an undisciplined counter time.
    pub fn status(&self, raw: CuInstant) -> Option<SyncStatus> {
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
            let age = raw.as_nanos().saturating_sub(sample.raw_local.as_nanos());
            let parent = i128::from(sample.parent_ns) + scaled(i128::from(age), q.drift_ppb);
            let phase = self.at(raw).map(|now| parent - i128::from(now.as_nanos()));
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

#[cfg(feature = "clock-sync")]
mod live;

#[cfg(feature = "clock-sync")]
pub use live::ClockSync;
#[cfg(feature = "clock-sync")]
pub(crate) use live::SharedClock;
