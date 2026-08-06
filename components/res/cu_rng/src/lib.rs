//! Seeded pseudo-random number generator resource for Copper.
//!
//! [`CuRng`] is a deterministic, reproducible RNG (ChaCha8) usable anywhere
//! [`rand::Rng`] is expected. It is wired into a Copper app as a
//! single-slot resource bundle, [`CuRngBundle`], via the standard resource
//! mechanism.
//!
//! Use [`prelude`] to bring the normal `rand` sampling API into scope:
//!
//! ```rust
//! use cu_rng::prelude::*;
//!
//! let mut rng = CuRng::from_seed(12345);
//! let unit_f64: f64 = rng.random();
//! let unit_f32: f32 = rng.random();
//! let noise: f64 = rng.random_range(-0.05..0.05);
//! let enabled: bool = rng.random();
//! let dropout: bool = rng.random_bool(0.02);
//! let vector: [f32; 3] = rng.random();
//!
//! assert!((0.0..1.0).contains(&unit_f64));
//! assert!((0.0..1.0).contains(&unit_f32));
//! assert!((-0.05..0.05).contains(&noise));
//! let _ = (enabled, dropout, vector);
//! ```
//!
//! # Example config
//!
//! ```ron
//! (
//!     resources: [
//!         ( id: "planner_rng", provider: "cu_rng::CuRngBundle", config: {"seed": 12345} ),
//!     ],
//!     tasks: [
//!         ( id: "planner", type: "tasks::MyPlanner",
//!           resources: { "rng": "planner_rng.rng" } ),
//!     ],
//! )
//! ```
//!
//! On the task side, declare the binding with the `resources!` macro:
//!
//! ```rust,ignore
//! use cu29::prelude::*;
//! use cu29::resources;
//! use cu_rng::prelude::*;
//!
//! mod planner_resources {
//!     use super::*;
//!     resources!({ rng => Owned<CuRng> });
//! }
//! ```

#![cfg_attr(not(feature = "std"), no_std)]

use cu29::bundle_resources;
use cu29::prelude::*;
use cu29::resource::{BundleContext, ResourceBundle, ResourceManager};

use rand::Rng;
use rand::rand_core::{Infallible, SeedableRng, TryRng};
use rand_chacha::ChaCha8Rng;

/// Common imports for using [`CuRng`] as a typed random source.
pub mod prelude {
    pub use crate::{CuRng, CuRngBundle};
    pub use rand::distr::{Bernoulli, Distribution, Uniform};
    pub use rand::{Rng, RngExt};
}

/// RON config key that carries the seed value.
pub const SEED_KEY: &str = "seed";

/// Slot name exposed by [`CuRngBundle`]. Tasks reference it as
/// `<bundle_id>.rng` in the `resources: { ... }` mapping.
pub const RNG_NAME: &str = "rng";

/// Deterministic, seedable RNG resource.
///
/// Backed by ChaCha8: same seed, same stream, on every platform. This is
/// what replay of randomized tasks relies on.
///
/// Not a CSPRNG — do not use for keys or nonces.
pub struct CuRng {
    inner: ChaCha8Rng,
}

impl CuRng {
    /// Build a fresh RNG from the given 64-bit seed.
    pub fn from_seed(seed: u64) -> Self {
        Self {
            inner: ChaCha8Rng::seed_from_u64(seed),
        }
    }

    /// Reset the stream to step 0 with a new seed.
    ///
    /// Equivalent to replacing `self` with `from_seed(seed)`; it does not
    /// fast-forward to a mid-stream position, so it cannot resume a task
    /// mid-run from just a stored seed.
    pub fn reseed(&mut self, seed: u64) {
        self.inner = ChaCha8Rng::seed_from_u64(seed);
    }
}

impl TryRng for CuRng {
    type Error = Infallible;

    fn try_next_u32(&mut self) -> Result<u32, Self::Error> {
        Ok(self.inner.next_u32())
    }

    fn try_next_u64(&mut self) -> Result<u64, Self::Error> {
        Ok(self.inner.next_u64())
    }

    fn try_fill_bytes(&mut self, dest: &mut [u8]) -> Result<(), Self::Error> {
        self.inner.fill_bytes(dest);
        Ok(())
    }
}

/// Single-slot bundle provider exposing a [`CuRng`] as an owned resource.
///
/// Reads the seed from the bundle's RON `config: {"seed": <u64>}` and hands
/// out the RNG through the standard `Owned<CuRng>` binding.
pub struct CuRngBundle;

bundle_resources!(CuRngBundle: Rng = "rng");

impl ResourceBundle for CuRngBundle {
    fn build(
        bundle: BundleContext<Self>,
        config: Option<&ComponentConfig>,
        manager: &mut ResourceManager,
    ) -> CuResult<()> {
        let seed = config
            .and_then(|cfg| cfg.get::<u64>(SEED_KEY).transpose())
            .transpose()?
            .ok_or_else(|| {
                CuError::from("CuRngBundle: missing required config key 'seed' (u64)")
            })?;
        manager.add_owned(bundle.key(CuRngBundleId::Rng), CuRng::from_seed(seed))?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::prelude::*;
    use cu29::resource::{NamedResourceBundleDecl, resource_index_by_name};

    #[test]
    fn same_seed_produces_same_stream() {
        let mut a = CuRng::from_seed(42);
        let mut b = CuRng::from_seed(42);
        for _ in 0..64 {
            assert_eq!(a.next_u64(), b.next_u64());
        }
    }

    #[test]
    fn different_seeds_diverge() {
        let mut a = CuRng::from_seed(1);
        let mut b = CuRng::from_seed(2);
        let av: [u64; 4] = core::array::from_fn(|_| a.next_u64());
        let bv: [u64; 4] = core::array::from_fn(|_| b.next_u64());
        assert_ne!(av, bv);
    }

    #[test]
    fn reseed_restarts_stream() {
        let mut a = CuRng::from_seed(7);
        let first = a.next_u64();
        for _ in 0..10 {
            let _ = a.next_u64();
        }
        a.reseed(7);
        assert_eq!(a.next_u64(), first);
    }

    #[test]
    fn fill_bytes_is_deterministic() {
        let mut a = CuRng::from_seed(0xC0FFEE);
        let mut b = CuRng::from_seed(0xC0FFEE);
        let mut a_bytes = [0u8; 32];
        let mut b_bytes = [0u8; 32];
        a.fill_bytes(&mut a_bytes);
        b.fill_bytes(&mut b_bytes);
        assert_eq!(a_bytes, b_bytes);
    }

    #[test]
    fn supports_typed_sampling_api() {
        let mut rng = CuRng::from_seed(0xA11CE);

        let unit_f64: f64 = rng.random();
        let unit_f32: f32 = rng.random();
        let ranged_float: f64 = rng.random_range(-0.25..0.25);
        let ranged_int: usize = rng.random_range(0..16);
        let coin: bool = rng.random();
        let rare_event = rng.random_bool(0.05);
        let vector: [f32; 3] = rng.random();

        assert!((0.0..1.0).contains(&unit_f64));
        assert!((0.0..1.0).contains(&unit_f32));
        assert!((-0.25..0.25).contains(&ranged_float));
        assert!(ranged_int < 16);
        let _ = (coin, rare_event, vector);
    }

    #[test]
    fn supports_prebuilt_distributions() {
        let mut rng = CuRng::from_seed(0xB0B);
        let noise = Uniform::new(-0.05, 0.05).expect("valid float range");
        let dropout = Bernoulli::new(0.02).expect("valid probability");

        let sampled_noise = noise.sample(&mut rng);
        let sampled_dropout = dropout.sample(&mut rng);

        assert!((-0.05..0.05).contains(&sampled_noise));
        let _ = sampled_dropout;
    }

    #[test]
    fn bundle_declares_named_rng_slot() {
        let names = <CuRngBundle as NamedResourceBundleDecl>::NAMES;
        assert_eq!(names, &[RNG_NAME]);
        assert_eq!(resource_index_by_name::<CuRngBundle>(RNG_NAME), 0);
    }
}
