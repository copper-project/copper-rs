use crate::velocity::VelocityTransform;
use cu29::clock::{CuTime, RobotClock};
use dashmap::DashMap;
use std::fmt::Debug;

/// The cache entry for a velocity transform query
#[derive(Clone)]
pub(crate) struct VelocityTransformCacheEntry<T: Copy + Debug + 'static> {
    /// The cached velocity transform result
    pub(crate) velocity: VelocityTransform<T>,
    /// The time for which this velocity was calculated
    pub(crate) time: CuTime,
    /// When this cache entry was last accessed (robot time)
    pub(crate) last_access: CuTime,
    /// Path hash used to calculate this velocity
    pub(crate) path_hash: u64,
}

/// A cache for velocity transforms to avoid recalculating frequently accessed paths
pub(crate) struct VelocityTransformCache<T: Copy + Debug + 'static> {
    /// Map from (source, target) frames to cached velocity transforms
    entries: DashMap<(String, String), VelocityTransformCacheEntry<T>>,
    /// Maximum size of the cache
    max_size: usize,
    /// Maximum age of cache entries before invalidation (in nanoseconds)
    max_age_nanos: u64,
    /// Cleanup interval - only clean every N nanoseconds
    cleanup_interval_nanos: u64,
    /// Last time the cache was cleaned up (needs to be mutable)
    last_cleanup_cell: std::sync::Mutex<CuTime>,
}

impl<T: Copy + Debug + 'static> VelocityTransformCache<T> {
    pub(crate) fn new(max_size: usize, max_age_nanos: u64) -> Self {
        Self {
            entries: DashMap::with_capacity(max_size),
            max_size,
            max_age_nanos,
            cleanup_interval_nanos: 5_000_000_000, // Clean every 5 seconds
            last_cleanup_cell: std::sync::Mutex::new(CuTime::from(0u64)),
        }
    }

    /// Get a cached velocity transform if it exists and is still valid
    pub(crate) fn get(
        &self,
        from: &str,
        to: &str,
        time: CuTime,
        path_hash: u64,
        robot_clock: &RobotClock,
    ) -> Option<VelocityTransform<T>> {
        let key = (from.to_string(), to.to_string());

        if let Some(mut entry) = self.entries.get_mut(&key) {
            let now = robot_clock.now();

            // Check if the cache entry is for the same time and path
            if entry.time == time && entry.path_hash == path_hash {
                // Check if the entry is still valid (not too old)
                let age = now.as_nanos().saturating_sub(entry.last_access.as_nanos());
                if age <= self.max_age_nanos {
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
        robot_clock: &RobotClock,
    ) {
        let now = robot_clock.now();
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
    pub(crate) fn should_cleanup(&self, robot_clock: &RobotClock) -> bool {
        let now = robot_clock.now();
        let last_cleanup = *self.last_cleanup_cell.lock().unwrap();
        let elapsed = now.as_nanos().saturating_sub(last_cleanup.as_nanos());
        elapsed >= self.cleanup_interval_nanos
    }

    /// Clear old entries from the cache
    pub(crate) fn cleanup(&self, robot_clock: &RobotClock) {
        let now = robot_clock.now();
        let mut keys_to_remove = Vec::new();

        // Identify keys to remove
        for entry in self.entries.iter() {
            let age = now.as_nanos().saturating_sub(entry.last_access.as_nanos());
            if age > self.max_age_nanos {
                keys_to_remove.push(entry.key().clone());
            }
        }

        // Remove expired entries
        for key in keys_to_remove {
            self.entries.remove(&key);
        }

        // Update last cleanup time
        *self.last_cleanup_cell.lock().unwrap() = now;
    }

    /// Clear all entries
    pub(crate) fn clear(&self) {
        self.entries.clear();
    }
}