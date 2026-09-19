//! ROS 2 QoS for the bridge.
//!
//! A topic's QoS is advertised to ROS 2 peers through the rmw_zenoh liveliness
//! token. rmw_zenoh encodes it as a single keyexpr chunk. This module mirrors
//! `qos_to_keyexpr()` from `rmw_zenoh_cpp/src/detail/liveliness_utils.cpp`
//! bit-for-bit, so that a ROS 2 subscriber reads the publisher's real QoS
//! during discovery instead of falling back to defaults the bridge does not
//! honour.
//!
//! Layout (each value is the numeric RMW QoS policy enum; a slot is left empty
//! when it equals the rmw_zenoh default):
//!
//! ```text
//! reliability:durability,history:depth,deadline_sec:deadline_nsec,lifespan_sec:lifespan_nsec,liveliness:lease_sec:lease_nsec
//! ```
//!
//! With every slot at its rmw_zenoh default this collapses to the empty
//! profile `::,:,:,:,,`.

use std::fmt::Write as _;

/// Reliability QoS policy (numeric `rmw_qos_reliability_policy_t` values).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Reliability {
    Reliable = 1,
    BestEffort = 2,
}

/// Durability QoS policy (numeric `rmw_qos_durability_policy_t` values).
///
/// The full policy set is kept for parity with rmw_zenoh; the bridge currently
/// only publishes `Volatile`, the others become relevant with per-channel QoS.
#[allow(dead_code)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Durability {
    TransientLocal = 1,
    Volatile = 2,
}

/// History QoS policy (numeric `rmw_qos_history_policy_t` values).
///
/// The full policy set is kept for parity with rmw_zenoh; the bridge currently
/// only publishes `KeepLast`, `KeepAll` becomes relevant with per-channel QoS.
#[allow(dead_code)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum History {
    KeepLast = 1,
    KeepAll = 2,
}

/// rmw_zenoh defaults (see `rmw_zenoh_cpp/src/detail/qos.cpp`). A slot equal to
/// its default is encoded as empty.
const RMW_DEFAULT_DEPTH: u32 = 42;

/// A ROS 2 QoS profile for a bridged topic.
///
/// Deadline, lifespan, liveliness and liveliness lease are not set by the
/// bridge, so they are always encoded as the rmw_zenoh default (empty slots).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Qos {
    reliability: Reliability,
    durability: Durability,
    history: History,
    depth: u32,
}

impl Qos {
    /// QoS matching the bridge's actual behaviour: the bridge publishes with
    /// zenoh's default `CongestionControl::Drop`, so messages may be dropped
    /// when the queue is full — i.e. best-effort, keep-last delivery. This
    /// mirrors `rmw_qos_profile_sensor_data` (`BEST_EFFORT` + `KEEP_LAST(5)`),
    /// so the advertisement no longer claims the retransmission it cannot
    /// provide.
    pub fn sensor_data() -> Self {
        Self {
            reliability: Reliability::BestEffort,
            durability: Durability::Volatile,
            history: History::KeepLast,
            depth: 5,
        }
    }

    /// Encode the profile into the rmw_zenoh liveliness-token chunk.
    pub fn to_keyexpr(self) -> String {
        let mut keyexpr = String::new();

        // Reliability.
        if self.reliability != Reliability::Reliable {
            let _ = write!(keyexpr, "{}", self.reliability as u8);
        }
        keyexpr.push(':');

        // Durability.
        if self.durability != Durability::Volatile {
            let _ = write!(keyexpr, "{}", self.durability as u8);
        }
        keyexpr.push(':');

        // History.
        if self.history != History::KeepLast {
            let _ = write!(keyexpr, "{}", self.history as u8);
        }
        keyexpr.push(',');
        if self.depth != RMW_DEFAULT_DEPTH {
            let _ = write!(keyexpr, "{}", self.depth);
        }
        keyexpr.push(':');

        // Deadline (rmw_zenoh default: infinite) — always empty.
        keyexpr.push(',');
        keyexpr.push(':');

        // Lifespan (rmw_zenoh default: infinite) — always empty.
        keyexpr.push(',');
        keyexpr.push(':');

        // Liveliness (rmw_zenoh default: automatic, infinite lease) — always
        // empty.
        keyexpr.push(',');
        keyexpr.push(',');

        keyexpr
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sensor_data_profile_matches_rmw_zenoh_encoding() {
        assert_eq!(Qos::sensor_data().to_keyexpr(), "2::,5:,:,:,,");
    }

    #[test]
    fn all_rmw_defaults_encode_to_the_empty_profile() {
        let qos = Qos {
            reliability: Reliability::Reliable,
            durability: Durability::Volatile,
            history: History::KeepLast,
            depth: RMW_DEFAULT_DEPTH,
        };
        assert_eq!(qos.to_keyexpr(), "::,:,:,:,,");
    }
}
