use bincode::{Decode, Encode};
use core::fmt;
use cu29::prelude::*;
use cu29::units::si::f32::Length;
use cu29::units::si::length::meter;
use serde::{Deserialize, Serialize};

pub const RANGE_PEER_ID_CAPACITY: usize = 24;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RangePeerIdError {
    Empty,
    TooLong { len: usize, max: usize },
}

impl fmt::Display for RangePeerIdError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Empty => write!(f, "range peer id must not be empty"),
            Self::TooLong { len, max } => {
                write!(f, "range peer id length {len} exceeds {max} bytes")
            }
        }
    }
}

impl core::error::Error for RangePeerIdError {}

/// Fixed-capacity peer identifier used by range observations.
///
/// This stays allocation-free on the runtime path while remaining flexible
/// enough for common tag, anchor, beacon, or node identifiers.
#[derive(
    Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, Encode, Decode, Reflect,
)]
pub struct RangePeerId {
    bytes: [u8; RANGE_PEER_ID_CAPACITY],
    len: u8,
}

impl RangePeerId {
    pub fn new(peer_id: &str) -> Result<Self, RangePeerIdError> {
        Self::try_from(peer_id)
    }

    pub fn as_str(&self) -> &str {
        let len = self.len as usize;
        core::str::from_utf8(&self.bytes[..len]).expect("RangePeerId stores valid UTF-8")
    }

    pub fn len(&self) -> usize {
        self.len as usize
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }
}

impl TryFrom<&str> for RangePeerId {
    type Error = RangePeerIdError;

    fn try_from(peer_id: &str) -> Result<Self, Self::Error> {
        if peer_id.is_empty() {
            return Err(RangePeerIdError::Empty);
        }
        if peer_id.len() > RANGE_PEER_ID_CAPACITY {
            return Err(RangePeerIdError::TooLong {
                len: peer_id.len(),
                max: RANGE_PEER_ID_CAPACITY,
            });
        }

        let mut bytes = [0_u8; RANGE_PEER_ID_CAPACITY];
        bytes[..peer_id.len()].copy_from_slice(peer_id.as_bytes());

        Ok(Self {
            bytes,
            len: peer_id.len() as u8,
        })
    }
}

/// Standardized single-peer range measurement.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, Encode, Decode, Reflect)]
pub struct RangeObservation {
    pub peer_id: RangePeerId,
    pub distance: Length,
    pub rssi_dbm: Option<i16>,
}

impl Default for RangeObservation {
    fn default() -> Self {
        Self {
            peer_id: RangePeerId::default(),
            distance: Length::new::<meter>(0.0),
            rssi_dbm: None,
        }
    }
}

impl RangeObservation {
    pub fn from_meters(peer_id: RangePeerId, distance_m: f32, rssi_dbm: Option<i16>) -> Self {
        Self {
            peer_id,
            distance: Length::new::<meter>(distance_m),
            rssi_dbm,
        }
    }

    pub fn from_centimeters(peer_id: RangePeerId, distance_cm: u32, rssi_dbm: Option<i16>) -> Self {
        Self::from_meters(peer_id, distance_cm as f32 / 100.0, rssi_dbm)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bincode::config;

    #[test]
    fn range_peer_id_round_trip() {
        let peer_id = RangePeerId::new("DAVID123").unwrap();
        assert_eq!(peer_id.as_str(), "DAVID123");
        assert_eq!(peer_id.len(), 8);
    }

    #[test]
    fn range_peer_id_rejects_too_long_values() {
        let err = RangePeerId::new("1234567890123456789012345").unwrap_err();
        assert!(matches!(err, RangePeerIdError::TooLong { .. }));
    }

    #[test]
    fn range_observation_round_trip_encode_decode() {
        let payload =
            RangeObservation::from_centimeters(RangePeerId::new("TAG-01").unwrap(), 245, Some(-71));

        let cfg = config::standard();
        let mut buffer = [0u8; 128];
        let len = bincode::encode_into_slice(payload, &mut buffer, cfg).unwrap();
        let (decoded, used) =
            bincode::decode_from_slice::<RangeObservation, _>(&buffer[..len], cfg).unwrap();

        assert_eq!(used, len);
        assert_eq!(decoded.peer_id.as_str(), "TAG-01");
        assert_eq!(decoded.rssi_dbm, Some(-71));
        assert_eq!(decoded.distance.get::<meter>(), 2.45);
    }
}
