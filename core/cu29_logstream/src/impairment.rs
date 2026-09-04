use crate::{Error, Result};
use alloc::vec::Vec;

/// Seeded deterministic datagram impairment profile, expressed in basis points.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ImpairmentConfig {
    pub seed: u64,
    pub drop_basis_points: u16,
    pub corrupt_basis_points: u16,
    pub duplicate_basis_points: u16,
    pub reorder: bool,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ImpairmentStats {
    pub input_datagrams: usize,
    pub dropped_datagrams: usize,
    pub corrupted_datagrams: usize,
    pub duplicated_datagrams: usize,
    pub output_datagrams: usize,
}

impl ImpairmentStats {
    pub const fn destruction_basis_points(self) -> u16 {
        if self.input_datagrams == 0 {
            return 0;
        }
        let destroyed = self
            .dropped_datagrams
            .saturating_add(self.corrupted_datagrams);
        ((destroyed as u64 * 10_000) / self.input_datagrams as u64) as u16
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ImpairmentOutput {
    pub datagrams: Vec<Vec<u8>>,
    pub stats: ImpairmentStats,
}

/// Apply deterministic loss, corruption, duplication, and reordering.
pub fn impair(datagrams: &[Vec<u8>], config: ImpairmentConfig) -> Result<ImpairmentOutput> {
    if config.drop_basis_points > 10_000
        || config.corrupt_basis_points > 10_000
        || config.duplicate_basis_points > 10_000
    {
        return Err(Error::InvalidConfig(
            "impairment probabilities cannot exceed 10,000 basis points",
        ));
    }
    let mut rng = SplitMix64::new(config.seed);
    let capacity = datagrams
        .len()
        .checked_mul(2)
        .ok_or(Error::InvalidConfig("impaired datagram capacity overflow"))?;
    let mut output = Vec::with_capacity(capacity);
    let mut stats = ImpairmentStats {
        input_datagrams: datagrams.len(),
        ..ImpairmentStats::default()
    };

    for datagram in datagrams {
        if rng.basis_points() < config.drop_basis_points {
            stats.dropped_datagrams = stats.dropped_datagrams.saturating_add(1);
            continue;
        }
        let mut delivered = datagram.clone();
        if !delivered.is_empty() && rng.basis_points() < config.corrupt_basis_points {
            let index = rng.index(delivered.len());
            delivered[index] ^= 1_u8 << (rng.next() & 7);
            stats.corrupted_datagrams = stats.corrupted_datagrams.saturating_add(1);
        }
        let duplicate = rng.basis_points() < config.duplicate_basis_points;
        output.push(delivered.clone());
        if duplicate {
            output.push(delivered);
            stats.duplicated_datagrams = stats.duplicated_datagrams.saturating_add(1);
        }
    }

    if config.reorder {
        for index in (1..output.len()).rev() {
            let other = rng.index(index + 1);
            output.swap(index, other);
        }
    }
    stats.output_datagrams = output.len();
    Ok(ImpairmentOutput {
        datagrams: output,
        stats,
    })
}

struct SplitMix64(u64);

impl SplitMix64 {
    const fn new(seed: u64) -> Self {
        Self(seed)
    }

    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut value = self.0;
        value = (value ^ (value >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        value = (value ^ (value >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        value ^ (value >> 31)
    }

    fn basis_points(&mut self) -> u16 {
        (self.next() % 10_000) as u16
    }

    fn index(&mut self, upper_bound: usize) -> usize {
        (self.next() % upper_bound as u64) as usize
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec;

    #[test]
    fn impairment_is_reproducible() {
        let datagrams = (0..20).map(|value| vec![value; 8]).collect::<Vec<_>>();
        let config = ImpairmentConfig {
            seed: 42,
            drop_basis_points: 2_000,
            corrupt_basis_points: 1_000,
            duplicate_basis_points: 1_000,
            reorder: true,
        };
        assert_eq!(
            impair(&datagrams, config).unwrap(),
            impair(&datagrams, config).unwrap()
        );
    }
}
