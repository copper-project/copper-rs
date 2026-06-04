use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{Decode, Encode};
use core::fmt;
use cu29::clock::{CuTime, CuTimeRange};
use cu29::prelude::*;
use cu29::units::si::f32::Length;
use cu29::units::si::length::meter;
use serde::{Deserialize, Deserializer, Serialize, Serializer, de, ser::SerializeStruct};

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

/// Standardized distance measurement to an identified peer.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, Encode, Decode, Reflect)]
pub struct PeerRangeObservation {
    pub peer_id: RangePeerId,
    pub distance: Length,
    pub rssi_dbm: Option<i16>,
}

impl Default for PeerRangeObservation {
    fn default() -> Self {
        Self {
            peer_id: RangePeerId::default(),
            distance: Length::new::<meter>(0.0),
            rssi_dbm: None,
        }
    }
}

impl PeerRangeObservation {
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

/// A peer range measurement with its own time of validity.
///
/// Use this inside aggregate payloads when samples were not captured at the same instant. For a
/// single `CuMsg<PeerRangeObservation>`, prefer carrying the time in the Copper message envelope.
#[derive(
    Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize, Encode, Decode, Reflect,
)]
pub struct PeerRangeSample {
    pub tov: CuTime,
    pub observation: PeerRangeObservation,
}

impl PeerRangeSample {
    pub fn new(tov: CuTime, observation: PeerRangeObservation) -> Self {
        Self { tov, observation }
    }
}

/// Bounded collection of timestamped peer ranges.
///
/// The enclosing Copper message should use `Tov::Range` when the samples span more than one
/// validity instant.
#[derive(Clone, Copy, Debug, PartialEq, Reflect)]
pub struct PeerRangeSnapshot<const N: usize> {
    pub len: usize,
    pub samples: [PeerRangeSample; N],
}

impl<const N: usize> Encode for PeerRangeSnapshot<N> {
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.samples().len(), encoder)?;
        for sample in self.samples() {
            Encode::encode(sample, encoder)?;
        }
        Ok(())
    }
}

impl<const N: usize> Decode<()> for PeerRangeSnapshot<N> {
    fn decode<D: Decoder<Context = ()>>(decoder: &mut D) -> Result<Self, DecodeError> {
        let len = <usize as Decode<()>>::decode(decoder)?;
        if len > N {
            return Err(DecodeError::ArrayLengthMismatch {
                required: N,
                found: len,
            });
        }

        let mut snapshot = Self::default();
        for _ in 0..len {
            let sample = <PeerRangeSample as Decode<()>>::decode(decoder)?;
            snapshot
                .push(sample)
                .map_err(|_| DecodeError::ArrayLengthMismatch {
                    required: N,
                    found: len,
                })?;
        }

        Ok(snapshot)
    }
}

impl<const N: usize> Serialize for PeerRangeSnapshot<N> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let mut state = serializer.serialize_struct("PeerRangeSnapshot", 2)?;
        state.serialize_field("len", &self.samples().len())?;
        state.serialize_field("samples", self.samples())?;
        state.end()
    }
}

impl<'de, const N: usize> Deserialize<'de> for PeerRangeSnapshot<N> {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct PeerRangeSnapshotSerde {
            len: usize,
            samples: alloc::vec::Vec<PeerRangeSample>,
        }

        let decoded = PeerRangeSnapshotSerde::deserialize(deserializer)?;
        if decoded.len != decoded.samples.len() {
            return Err(de::Error::custom(
                "peer range snapshot len does not match samples",
            ));
        }
        if decoded.samples.len() > N {
            return Err(de::Error::custom("peer range snapshot exceeds capacity"));
        }

        let mut snapshot = Self::default();
        for sample in decoded.samples {
            snapshot
                .push(sample)
                .map_err(|_| de::Error::custom("peer range snapshot exceeds capacity"))?;
        }

        Ok(snapshot)
    }
}

impl<const N: usize> Default for PeerRangeSnapshot<N> {
    fn default() -> Self {
        Self {
            len: 0,
            samples: [PeerRangeSample::default(); N],
        }
    }
}

impl<const N: usize> PeerRangeSnapshot<N> {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub fn is_full(&self) -> bool {
        self.len >= N
    }

    pub fn samples(&self) -> &[PeerRangeSample] {
        &self.samples[..self.len.min(N)]
    }

    pub fn push(&mut self, sample: PeerRangeSample) -> Result<(), PeerRangeSnapshotFull> {
        if self.is_full() {
            return Err(PeerRangeSnapshotFull { capacity: N });
        }

        self.samples[self.len] = sample;
        self.len += 1;
        Ok(())
    }

    pub fn tov_range(&self) -> Option<CuTimeRange> {
        let samples = self.samples();
        if samples.is_empty() {
            return None;
        }

        let mut start = samples[0].tov;
        let mut end = samples[0].tov;
        for sample in &samples[1..] {
            start = start.min(sample.tov);
            end = end.max(sample.tov);
        }

        Some(CuTimeRange { start, end })
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PeerRangeSnapshotFull {
    pub capacity: usize,
}

impl fmt::Display for PeerRangeSnapshotFull {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "peer range snapshot capacity {} is full", self.capacity)
    }
}

impl core::error::Error for PeerRangeSnapshotFull {}

#[cfg(test)]
mod tests {
    use super::*;
    use bincode::config;
    use cu29::units::si::length::meter;

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
    fn peer_range_observation_round_trip_encode_decode() {
        let payload = PeerRangeObservation::from_centimeters(
            RangePeerId::new("TAG-01").unwrap(),
            245,
            Some(-71),
        );

        let cfg = config::standard();
        let mut buffer = [0u8; 128];
        let len = bincode::encode_into_slice(payload, &mut buffer, cfg).unwrap();
        let (decoded, used) =
            bincode::decode_from_slice::<PeerRangeObservation, _>(&buffer[..len], cfg).unwrap();

        assert_eq!(used, len);
        assert_eq!(decoded.peer_id.as_str(), "TAG-01");
        assert_eq!(decoded.rssi_dbm, Some(-71));
        assert_eq!(decoded.distance.get::<meter>(), 2.45);
    }

    #[test]
    fn peer_range_snapshot_tracks_sample_time_range() {
        let mut snapshot = PeerRangeSnapshot::<2>::new();
        snapshot
            .push(PeerRangeSample::new(
                CuTime::from_nanos(20),
                PeerRangeObservation::from_centimeters(
                    RangePeerId::new("TAG-02").unwrap(),
                    245,
                    None,
                ),
            ))
            .unwrap();
        snapshot
            .push(PeerRangeSample::new(
                CuTime::from_nanos(10),
                PeerRangeObservation::from_centimeters(
                    RangePeerId::new("TAG-01").unwrap(),
                    120,
                    Some(-71),
                ),
            ))
            .unwrap();

        let range = snapshot.tov_range().unwrap();
        assert_eq!(range.start, CuTime::from_nanos(10));
        assert_eq!(range.end, CuTime::from_nanos(20));
        assert_eq!(snapshot.samples()[1].observation.peer_id.as_str(), "TAG-01");
    }

    #[test]
    fn peer_range_snapshot_rejects_over_capacity_push() {
        let mut snapshot = PeerRangeSnapshot::<1>::new();
        let sample = PeerRangeSample::new(
            CuTime::from_nanos(1),
            PeerRangeObservation::from_centimeters(RangePeerId::new("TAG-01").unwrap(), 100, None),
        );

        snapshot.push(sample).unwrap();
        assert_eq!(
            snapshot.push(sample),
            Err(PeerRangeSnapshotFull { capacity: 1 })
        );
    }

    #[test]
    fn peer_range_snapshot_round_trip_encode_decode() {
        let mut snapshot = PeerRangeSnapshot::<4>::new();
        snapshot
            .push(PeerRangeSample::new(
                CuTime::from_nanos(10),
                PeerRangeObservation::from_centimeters(
                    RangePeerId::new("TAG-01").unwrap(),
                    100,
                    Some(-70),
                ),
            ))
            .unwrap();
        snapshot
            .push(PeerRangeSample::new(
                CuTime::from_nanos(20),
                PeerRangeObservation::from_centimeters(
                    RangePeerId::new("TAG-02").unwrap(),
                    200,
                    None,
                ),
            ))
            .unwrap();

        let cfg = config::standard();
        let mut buffer = [0u8; 256];
        let len = bincode::encode_into_slice(snapshot, &mut buffer, cfg).unwrap();
        let (decoded, used) =
            bincode::decode_from_slice::<PeerRangeSnapshot<4>, _>(&buffer[..len], cfg).unwrap();

        assert_eq!(used, len);
        assert_eq!(decoded.len, 2);
        assert_eq!(decoded.samples()[0].observation.peer_id.as_str(), "TAG-01");
        assert_eq!(decoded.samples()[1].observation.peer_id.as_str(), "TAG-02");
    }
}
