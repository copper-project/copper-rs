//! Seeded pseudo-random number generator resource for Copper.
//!
//! [`CuRng`] is a deterministic, reproducible RNG (ChaCha8) usable anywhere
//! [`rand_core::RngCore`] is expected. It is wired into a Copper app as a
//! single-slot resource bundle, [`CuRngBundle`], via the standard resource
//! mechanism.
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
//! use cu_rng::CuRng;
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

use rand_chacha::ChaCha8Rng;
use rand_core::{RngCore, SeedableRng};

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

impl RngCore for CuRng {
    fn next_u32(&mut self) -> u32 {
        self.inner.next_u32()
    }

    fn next_u64(&mut self) -> u64 {
        self.inner.next_u64()
    }

    fn fill_bytes(&mut self, dest: &mut [u8]) {
        self.inner.fill_bytes(dest)
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
        let mut ba = [0u8; 32];
        let mut bb = [0u8; 32];
        a.fill_bytes(&mut ba);
        b.fill_bytes(&mut bb);
        assert_eq!(ba, bb);
    }

    #[test]
    fn bundle_declares_named_rng_slot() {
        let names = <CuRngBundle as NamedResourceBundleDecl>::NAMES;
        assert_eq!(names, &[RNG_NAME]);
        assert_eq!(resource_index_by_name::<CuRngBundle>(RNG_NAME), 0);
    }
}
