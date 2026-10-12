//! Informational provenance and epoch of execution timestamps.

use bincode::{Decode, Encode};
use serde::{Deserialize, Serialize};

/// Advertised upstream source, independent of transport and timestamp epoch.
///
/// This is diagnostic metadata, not a guarantee of traceability or accuracy.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, Encode, Decode)]
pub enum ClockSource {
    /// The architecture counter used by a local execution clock.
    LocalMonotonic,
    /// A controlled test or replay counter.
    Mock,
    /// An application-provided local real-time clock.
    LocalRtc,
    /// A global navigation satellite system receiver.
    Gnss,
    /// An advertised atomic clock source.
    AtomicClock,
    /// A terrestrial radio time service.
    TerrestrialRadio,
    /// An upstream PTP source whose root provenance is not specified further.
    Ptp,
    /// An upstream NTP source.
    Ntp,
    /// A manually set clock.
    HandSet,
    /// An internal oscillator.
    InternalOscillator,
    /// A provider-defined source code.
    Other(u8),
    /// The provider did not identify the source.
    #[default]
    Unknown,
}

/// Meaning of zero on the execution timeline. Every timestamp remains nanoseconds.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, Encode, Decode)]
pub enum ClockEpoch {
    /// Elapsed time since this execution clock started.
    Local,
    /// Nanoseconds since 1970-01-01 00:00:00 TAI, as used by PTP.
    Tai {
        /// Provider-advertised TAI minus UTC, when valid for this reference.
        /// Informational only; RobotClock never applies this offset.
        tai_minus_utc_seconds: Option<i16>,
    },
    /// Nanoseconds since 1980-01-06 00:00:00 GPS time.
    Gps,
    /// Unix-style UTC nanoseconds since 1970-01-01 00:00:00 UTC.
    Utc,
    /// An application-defined epoch identifier; interpretation belongs to the application.
    Other(u32),
    /// No epoch interpretation was supplied.
    #[default]
    Unknown,
}

#[cfg(feature = "clock-sync")]
impl ClockEpoch {
    pub(crate) fn same_timeline(self, other: Self) -> bool {
        matches!((self, other), (Self::Tai { .. }, Self::Tai { .. })) || self == other
    }
}

/// Compact metadata describing what [`crate::RobotClock::now`] returns.
///
/// Providers may supply GPS, TAI, UTC, or application-defined nanoseconds.
/// The clock disciplines the supplied timeline without calendar conversion.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, Encode, Decode)]
pub struct ClockOrigin {
    /// Advertised upstream provenance.
    pub source: ClockSource,
    /// Epoch of the supplied timestamps.
    pub epoch: ClockEpoch,
}
