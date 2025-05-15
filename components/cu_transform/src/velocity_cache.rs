use crate::velocity::VelocityTransform;
use cu29::clock::CuTime;
use dashmap::DashMap;
use std::fmt::Debug;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, Instant};

/// The cache entry for a velocity transform query
#[derive(Clone)]
pub(crate) struct VelocityTransformCacheEntry<T: Copy + Debug + 'static> {
    /// The cached velocity transform result
    pub(crate) velocity: VelocityTransform<T>,
    /// The time for which this velocity was calculated
    pub(crate) time: CuTime,
    /// When this cache entry was last accessed
    pub(crate) last_access: Instant,
    /// Path hash used to calculate this velocity
    pub(crate) path_hash: u64,
}

/// A cache for velocity transforms to avoid recalculating frequently accessed paths
pub(crate) struct VelocityTransformCache<T: Copy + Debug + 'static> {
    /// Map from (source, target) frames to cached velocity transforms
    entries: DashMap<(String, String), VelocityTransformCacheEntry<T>>,
    /// Maximum size of the cache
    max_size: usize,
    /// Maximum age of cache entries before invalidation
    max_age: Duration,
    /// Last time the cache was cleaned up (as nanoseconds since epoch)
    last_cleanup: AtomicU64,
    /// Cleanup interval - only clean every N seconds
    cleanup_interval: Duration,
}

/// Get current time as nanoseconds since some epoch
/// This is just used for comparing intervals, not for absolute time
fn now_timestamp_nanos() -> u64 {
    let now = Instant::now();
    // We can't get nanoseconds from Instant directly, but we can use this trick
    // to get a stable value that increases monotonically
    let duration = now.elapsed();
    (duration.as_secs() * 1_000_000_000 + duration.subsec_nanos() as u64) % u64::MAX
}

impl<T: Copy + Debug + 'static> VelocityTransformCache<T> {
    pub(crate) fn new(max_size: usize, max_age: Duration) -> Self {
        Self {
            entries: DashMap::with_capacity(max_size),
            max_size,
            max_age,
            last_cleanup: AtomicU64::new(now_timestamp_nanos()),
            cleanup_interval: Duration::from_secs(5), // Clean every 5 seconds
        }
    }

    /// Get a cached velocity transform if it exists and is still valid
    pub(crate) fn get(
        &self,
        from: &str,
        to: &str,
        time: CuTime,
        path_hash: u64,
    ) -> Option<VelocityTransform<T>> {
        let key = (from.to_string(), to.to_string());

        if let Some(mut entry) = self.entries.get_mut(&key) {
            let now = Instant::now();

            // Check if the cache entry is for the same time and path
            if entry.time == time && entry.path_hash == path_hash {
                // Check if the entry is still valid (not too old)
                if now.duration_since(entry.last_access) <= self.max_age {
                    // Update last access time
                    entry.last_access = now;
                    return Some(entry.velocity.clone());
                }
            }
        }

        None
    }

    /// Add a new velocity transform to the cache
    pub(crate) fn insert(
        &self,
        from: &str,
        to: &str,
        velocity: VelocityTransform<T>,
        time: CuTime,
        path_hash: u64,
    ) {
        let now = Instant::now();
        let key = (from.to_string(), to.to_string());

        // If the cache is at capacity, remove the oldest entry
        if self.entries.len() >= self.max_size {
            // Find the oldest entry (this requires iterating through entries)
            let mut oldest_key = None;
            let mut oldest_time = now;

            for entry in self.entries.iter() {
                if entry.last_access < oldest_time {
                    oldest_time = entry.last_access;
                    oldest_key = Some(entry.key().clone());
                }
            }

            if let Some(key_to_remove) = oldest_key {
                self.entries.remove(&key_to_remove);
            }
        }

        // Insert the new entry
        self.entries.insert(
            key,
            VelocityTransformCacheEntry {
                velocity,
                time,
                last_access: now,
                path_hash,
            },
        );
    }

    /// Check if it's time to clean up the cache
    pub(crate) fn should_cleanup(&self) -> bool {
        let now = now_timestamp_nanos();
        let last = self.last_cleanup.load(Ordering::Relaxed);
        let interval_nanos = self.cleanup_interval.as_nanos() as u64;

        now.saturating_sub(last) >= interval_nanos
    }

    /// Clear old entries from the cache
    pub(crate) fn cleanup(&self) {
        let now = Instant::now();
        let mut keys_to_remove = Vec::new();

        // Identify keys to remove
        for entry in self.entries.iter() {
            if now.duration_since(entry.last_access) > self.max_age {
                keys_to_remove.push(entry.key().clone());
            }
        }

        // Remove expired entries
        for key in keys_to_remove {
            self.entries.remove(&key);
        }

        // Update last cleanup time using atomic
        self.last_cleanup
            .store(now_timestamp_nanos(), Ordering::Relaxed);
    }

    /// Clear all entries
    pub(crate) fn clear(&self) {
        self.entries.clear();
    }
}
