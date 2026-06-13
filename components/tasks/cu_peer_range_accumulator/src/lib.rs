#![cfg_attr(not(feature = "std"), no_std)]

use cu_sensor_payloads::{PeerRangeObservation, PeerRangeSample, PeerRangeSnapshot};
use cu29::clock::{CuDuration, CuTime};
use cu29::prelude::*;
use serde::Deserialize;

const DEFAULT_MAX_SAMPLE_AGE_MS: u64 = 1000;

#[derive(Clone, Copy, Debug, Eq, PartialEq, Reflect)]
pub enum SampleRetention {
    MaxAge(CuDuration),
    Forever,
}

impl Default for SampleRetention {
    fn default() -> Self {
        Self::MaxAge(CuDuration::from_millis(DEFAULT_MAX_SAMPLE_AGE_MS))
    }
}

#[derive(Clone, Copy, Debug, Deserialize)]
enum SampleRetentionConfig {
    MaxAgeMs(u32),
    Forever,
}

impl SampleRetentionConfig {
    fn into_policy(self) -> SampleRetention {
        match self {
            Self::MaxAgeMs(ms) => SampleRetention::MaxAge(CuDuration::from_millis(ms as u64)),
            Self::Forever => SampleRetention::Forever,
        }
    }
}

#[derive(Reflect)]
pub struct PeerRangeAccumulatorTask<const N: usize> {
    samples: [Option<PeerRangeSample>; N],
    min_samples: usize,
    sample_retention: SampleRetention,
}

impl<const N: usize> Freezable for PeerRangeAccumulatorTask<N> {}

impl<const N: usize> PeerRangeAccumulatorTask<N> {
    pub fn new_with_limits(min_samples: usize, sample_retention: SampleRetention) -> Self {
        Self {
            samples: [None; N],
            min_samples: min_samples.clamp(1, N),
            sample_retention,
        }
    }

    pub fn latest_samples(&self) -> impl Iterator<Item = PeerRangeSample> + '_ {
        self.samples.iter().flatten().copied()
    }

    pub fn update(
        &mut self,
        sample: PeerRangeSample,
    ) -> Result<Option<PeerRangeSnapshot<N>>, PeerRangeAccumulatorError> {
        if let Some(existing) = self
            .samples
            .iter_mut()
            .flatten()
            .find(|existing| existing.observation.peer_id == sample.observation.peer_id)
        {
            *existing = sample;
        } else if let Some(slot) = self.samples.iter_mut().find(|slot| slot.is_none()) {
            *slot = Some(sample);
        } else {
            return Err(PeerRangeAccumulatorError::SnapshotFull { capacity: N });
        }

        Ok(self.snapshot_at(sample.tov))
    }

    pub fn snapshot_at(&mut self, now: CuTime) -> Option<PeerRangeSnapshot<N>> {
        if let SampleRetention::MaxAge(max_sample_age) = self.sample_retention {
            for slot in &mut self.samples {
                let Some(sample) = slot else {
                    continue;
                };
                if now - sample.tov > max_sample_age {
                    *slot = None;
                }
            }
        }

        let mut snapshot = PeerRangeSnapshot::<N>::new();
        for sample in self.latest_samples() {
            snapshot
                .push(sample)
                .expect("snapshot capacity matches accumulator capacity");
        }

        (snapshot.len >= self.min_samples).then_some(snapshot)
    }
}

impl<const N: usize> CuTask for PeerRangeAccumulatorTask<N> {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(PeerRangeObservation);
    type Output<'m> = output_msg!(PeerRangeSnapshot<N>);

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let min_samples = config
            .map(|config| config.get::<u32>("min_samples"))
            .transpose()?
            .flatten()
            .map_or(N, |value| value as usize);

        let sample_retention = config
            .map(|config| config.get_value::<SampleRetentionConfig>("sample_retention"))
            .transpose()?
            .flatten()
            .map_or_else(SampleRetention::default, SampleRetentionConfig::into_policy);

        Ok(Self::new_with_limits(min_samples, sample_retention))
    }

    fn process(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        output.tov = input.tov;
        let Some(observation) = input.payload().copied() else {
            output.clear_payload();
            return Ok(());
        };
        let Tov::Time(tov) = input.tov else {
            return Err("peer range accumulator expects Tov::Time inputs".into());
        };

        match self
            .update(PeerRangeSample::new(tov, observation))
            .map_err(|_| CuError::from("peer range accumulator capacity is full"))?
        {
            Some(snapshot) => {
                output.tov = snapshot.tov_range().map_or(Tov::None, Tov::Range);
                output.set_payload(snapshot);
            }
            None => output.clear_payload(),
        }

        Ok(())
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PeerRangeAccumulatorError {
    SnapshotFull { capacity: usize },
}

impl core::fmt::Display for PeerRangeAccumulatorError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::SnapshotFull { capacity } => {
                write!(f, "peer range accumulator capacity {capacity} is full")
            }
        }
    }
}

impl core::error::Error for PeerRangeAccumulatorError {}

#[cfg(test)]
mod tests {
    use super::*;
    use cu_sensor_payloads::RangePeerId;
    use cu29::units::si::length::meter;

    fn observation(peer_id: &str, meters: f32) -> PeerRangeObservation {
        PeerRangeObservation::from_meters(RangePeerId::new(peer_id).unwrap(), meters, None)
    }

    #[test]
    fn replaces_latest_sample_for_same_peer() {
        let mut accumulator =
            PeerRangeAccumulatorTask::<3>::new_with_limits(1, SampleRetention::default());
        accumulator
            .update(PeerRangeSample::new(
                CuTime::from_nanos(10),
                observation("A", 1.0),
            ))
            .unwrap();

        let snapshot = accumulator
            .update(PeerRangeSample::new(
                CuTime::from_nanos(20),
                observation("A", 2.0),
            ))
            .unwrap()
            .unwrap();

        assert_eq!(snapshot.len, 1);
        assert_eq!(snapshot.samples()[0].tov, CuTime::from_nanos(20));
        assert_eq!(
            snapshot.samples()[0].observation.distance.get::<meter>(),
            2.0
        );
    }

    #[test]
    fn rejects_new_peer_when_capacity_is_full() {
        let mut accumulator =
            PeerRangeAccumulatorTask::<1>::new_with_limits(1, SampleRetention::Forever);
        accumulator
            .update(PeerRangeSample::new(
                CuTime::from_nanos(10),
                observation("A", 1.0),
            ))
            .unwrap();

        assert_eq!(
            accumulator.update(PeerRangeSample::new(
                CuTime::from_nanos(20),
                observation("B", 2.0)
            )),
            Err(PeerRangeAccumulatorError::SnapshotFull { capacity: 1 })
        );
    }

    #[test]
    fn evicts_stale_samples_before_snapshot() {
        let mut accumulator = PeerRangeAccumulatorTask::<3>::new_with_limits(
            2,
            SampleRetention::MaxAge(CuDuration::from_millis(5)),
        );
        accumulator
            .update(PeerRangeSample::new(
                CuTime::from_millis(10),
                observation("A", 1.0),
            ))
            .unwrap();

        let snapshot = accumulator
            .update(PeerRangeSample::new(
                CuTime::from_millis(20),
                observation("B", 2.0),
            ))
            .unwrap();

        assert!(snapshot.is_none());
        assert_eq!(accumulator.latest_samples().count(), 1);
    }

    #[test]
    fn forever_retention_keeps_old_samples() {
        let mut accumulator =
            PeerRangeAccumulatorTask::<3>::new_with_limits(2, SampleRetention::Forever);
        accumulator
            .update(PeerRangeSample::new(
                CuTime::from_millis(10),
                observation("A", 1.0),
            ))
            .unwrap();

        let snapshot = accumulator
            .update(PeerRangeSample::new(
                CuTime::from_millis(2000),
                observation("B", 2.0),
            ))
            .unwrap()
            .unwrap();

        assert_eq!(snapshot.len, 2);
    }

    #[test]
    fn task_emits_snapshot_with_range_tov_after_min_samples() {
        let mut accumulator =
            PeerRangeAccumulatorTask::<3>::new_with_limits(2, SampleRetention::default());
        let ctx = CuContext::new_with_clock();
        let mut input = CuMsg::new(Some(observation("A", 1.0)));
        input.tov = Tov::Time(CuTime::from_nanos(10));
        let mut output = CuMsg::<PeerRangeSnapshot<3>>::default();

        accumulator.process(&ctx, &input, &mut output).unwrap();
        assert!(output.payload().is_none());

        input.set_payload(observation("B", 2.0));
        input.tov = Tov::Time(CuTime::from_nanos(20));
        accumulator.process(&ctx, &input, &mut output).unwrap();

        assert_eq!(output.payload().unwrap().len, 2);
        assert_eq!(
            output.tov,
            Tov::Range(cu29::clock::CuTimeRange {
                start: CuTime::from_nanos(10),
                end: CuTime::from_nanos(20),
            })
        );
    }
}
