use crate::FrameIdString;
use crate::error::{TransformError, TransformResult};
use crate::transform::{StampedTransform, TransformBuffer, TransformStore};
use crate::transform_payload::StampedFrameTransform;
use crate::velocity::VelocityTransform;
use crate::velocity_cache::VelocityTransformCache;
use cu_spatial_payloads::Transform3D;
use cu29::clock::{CuTime, RobotClock, Tov};
use dashmap::DashMap;
use petgraph::algo::dijkstra;
use petgraph::graph::{DiGraph, NodeIndex};
use serde::Serialize;
use serde::de::DeserializeOwned;
use std::collections::HashMap;
use std::fmt::Debug;
use std::ops::Neg;
use std::time::Duration;

/// Trait for types that can compute their inverse transformation
pub trait HasInverse<T: Copy + Debug + 'static> {
    fn inverse(&self) -> Self;
}

impl HasInverse<f32> for Transform3D<f32> {
    fn inverse(&self) -> Self {
        self.inverse()
    }
}

impl HasInverse<f64> for Transform3D<f64> {
    fn inverse(&self) -> Self {
        self.inverse()
    }
}

/// The cache entry for a transform query
#[derive(Clone)]
struct TransformCacheEntry<T: Copy + Debug + 'static> {
    /// The cached transform result
    transform: Transform3D<T>,
    /// The time for which this transform was calculated
    time: CuTime,
    /// When this cache entry was last accessed
    last_access: CuTime,
    /// Path used to calculate this transform
    path_hash: u64,
}

/// A cache for transforms to avoid recalculating frequently accessed paths
struct TransformCache<T: Copy + Debug + 'static> {
    /// Map from (source, target) frames to cached transforms
    entries: DashMap<(FrameIdString, FrameIdString), TransformCacheEntry<T>>,
    /// Maximum size of the cache
    max_size: usize,
    /// Maximum age of cache entries before invalidation (in nanoseconds)
    max_age_nanos: u64,
    /// Last time the cache was cleaned up (needs to be mutable)
    last_cleanup_cell: std::sync::Mutex<CuTime>,
    /// Cleanup interval (in nanoseconds)
    cleanup_interval_nanos: u64,
}

impl<T: Copy + Debug + 'static> TransformCache<T> {
    fn new(max_size: usize, max_age: Duration) -> Self {
        Self {
            entries: DashMap::with_capacity(max_size),
            max_size,
            max_age_nanos: max_age.as_nanos() as u64,
            last_cleanup_cell: std::sync::Mutex::new(CuTime::from(0u64)),
            cleanup_interval_nanos: 5_000_000_000, // Clean every 5 seconds
        }
    }

    /// Get a cached transform if it exists and is still valid
    fn get(
        &self,
        from: &str,
        to: &str,
        time: CuTime,
        path_hash: u64,
        robot_clock: &RobotClock,
    ) -> Option<Transform3D<T>> {
        let key = (
            FrameIdString::from(from).expect("Frame name too long"),
            FrameIdString::from(to).expect("Frame name too long"),
        );

        if let Some(mut entry) = self.entries.get_mut(&key) {
            let now = robot_clock.now();

            // Check if the cache entry is for the same time and path
            if entry.time == time && entry.path_hash == path_hash {
                // Check if the entry is still valid (not too old)
                let age = now.as_nanos().saturating_sub(entry.last_access.as_nanos());
                if age <= self.max_age_nanos {
                    // Update last access time
                    entry.last_access = now;
                    return Some(entry.transform);
                }
            }
        }

        None
    }

    /// Add a new transform to the cache
    fn insert(
        &self,
        from: &str,
        to: &str,
        transform: Transform3D<T>,
        time: CuTime,
        path_hash: u64,
        robot_clock: &RobotClock,
    ) {
        let now = robot_clock.now();
        let key = (
            FrameIdString::from(from).expect("Frame name too long"),
            FrameIdString::from(to).expect("Frame name too long"),
        );

        // If the cache is at capacity, remove the oldest entry
        if self.entries.len() >= self.max_size {
            let oldest_key = self
                .entries
                .iter()
                .min_by_key(|entry| entry.last_access)
                .map(|entry| *entry.key());
            if let Some(key_to_remove) = oldest_key {
                self.entries.remove(&key_to_remove);
            }
        }

        // Insert the new entry
        self.entries.insert(
            key,
            TransformCacheEntry {
                transform,
                time,
                last_access: now,
                path_hash,
            },
        );
    }

    /// Check if it's time to clean up the cache
    fn should_cleanup(&self, robot_clock: &RobotClock) -> bool {
        let now = robot_clock.now();
        let last_cleanup = *self.last_cleanup_cell.lock().unwrap();
        let elapsed = now.as_nanos().saturating_sub(last_cleanup.as_nanos());
        elapsed >= self.cleanup_interval_nanos
    }

    /// Clear old entries from the cache
    fn cleanup(&self, robot_clock: &RobotClock) {
        let now = robot_clock.now();
        let mut keys_to_remove = Vec::new();

        // Identify keys to remove
        for entry in self.entries.iter() {
            let age = now.as_nanos().saturating_sub(entry.last_access.as_nanos());
            if age > self.max_age_nanos {
                keys_to_remove.push(*entry.key());
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
    fn clear(&self) {
        self.entries.clear();
    }
}

pub struct TransformTree<T: Copy + Debug + Default + 'static> {
    graph: DiGraph<FrameIdString, ()>,
    frame_indices: HashMap<FrameIdString, NodeIndex>,
    transform_store: TransformStore<T>,
    // Concurrent cache for transform lookups
    cache: TransformCache<T>,
    // Concurrent cache for velocity transform lookups
    velocity_cache: VelocityTransformCache<T>,
}

/// Trait for types that can provide a value representing "one"
pub trait One {
    /// Returns a value representing "one" for this type
    fn one() -> Self;
}

// Implement One for common numeric types
impl One for f32 {
    fn one() -> Self {
        1.0
    }
}

impl One for f64 {
    fn one() -> Self {
        1.0
    }
}

impl One for i32 {
    fn one() -> Self {
        1
    }
}

impl One for i64 {
    fn one() -> Self {
        1
    }
}

impl One for u32 {
    fn one() -> Self {
        1
    }
}

impl One for u64 {
    fn one() -> Self {
        1
    }
}

// We need to limit T to types where Transform3D<T> has Clone and inverse method
// and now we also require T to implement One
impl<T: Copy + Debug + Default + One + Serialize + DeserializeOwned + 'static + Neg<Output = T>>
    TransformTree<T>
where
    Transform3D<T>: Clone + HasInverse<T> + std::ops::Mul<Output = Transform3D<T>>,
    T: std::ops::Add<Output = T>
        + std::ops::Sub<Output = T>
        + std::ops::Mul<Output = T>
        + std::ops::Div<Output = T>
        + std::ops::AddAssign
        + std::ops::SubAssign
        + num_traits::NumCast,
{
    /// Default cache size (number of transforms to cache)
    const DEFAULT_CACHE_SIZE: usize = 100;

    /// Default cache entry lifetime (5 seconds)
    const DEFAULT_CACHE_AGE: Duration = Duration::from_secs(5);

    /// Create a new transform tree with default settings
    pub fn new() -> Self {
        Self {
            graph: DiGraph::new(),
            frame_indices: HashMap::new(),
            transform_store: TransformStore::new(),
            cache: TransformCache::new(Self::DEFAULT_CACHE_SIZE, Self::DEFAULT_CACHE_AGE),
            velocity_cache: VelocityTransformCache::new(
                Self::DEFAULT_CACHE_SIZE,
                Self::DEFAULT_CACHE_AGE.as_nanos() as u64,
            ),
        }
    }

    /// Create a new transform tree with custom cache settings
    pub fn with_cache_settings(cache_size: usize, cache_age: Duration) -> Self {
        Self {
            graph: DiGraph::new(),
            frame_indices: HashMap::new(),
            transform_store: TransformStore::new(),
            cache: TransformCache::new(cache_size, cache_age),
            velocity_cache: VelocityTransformCache::new(cache_size, cache_age.as_nanos() as u64),
        }
    }

    /// Clear the transform cache
    pub fn clear_cache(&self) {
        self.cache.clear();
        self.velocity_cache.clear();
    }

    /// Perform scheduled cache cleanup operation
    pub fn cleanup_cache(&self, robot_clock: &RobotClock) {
        self.cache.cleanup(robot_clock);
        self.velocity_cache.cleanup(robot_clock);
    }

    /// Creates an identity transform matrix in a type-safe way
    fn create_identity_transform() -> Transform3D<T> {
        let mut mat = [[T::default(); 4]; 4];

        // Set the diagonal elements to one
        let one = T::one();
        mat[0][0] = one;
        mat[1][1] = one;
        mat[2][2] = one;
        mat[3][3] = one;

        Transform3D::from_matrix(mat)
    }

    fn ensure_frame(&mut self, frame_id: &str) -> NodeIndex {
        let frame_id_string = FrameIdString::from(frame_id).expect("Frame name too long");
        *self
            .frame_indices
            .entry(frame_id_string)
            .or_insert_with(|| self.graph.add_node(frame_id_string))
    }

    fn get_segment_buffer(
        &self,
        parent: &FrameIdString,
        child: &FrameIdString,
    ) -> TransformResult<TransformBuffer<T>> {
        self.transform_store
            .get_buffer(parent, child)
            .ok_or_else(|| TransformError::TransformNotFound {
                from: parent.to_string(),
                to: child.to_string(),
            })
    }

    /// add a transform to the tree.
    pub fn add_transform(&mut self, sft: &StampedFrameTransform<T>) -> TransformResult<()>
    where
        T: bincode::Encode + bincode::Decode<()>,
    {
        let transform_msg = sft.payload().ok_or_else(|| {
            TransformError::Unknown("Failed to get transform payload".to_string())
        })?;

        let timestamp = match sft.tov {
            Tov::Time(time) => time,
            Tov::Range(range) => range.start, // Use start of range
            _ => {
                return Err(TransformError::Unknown(
                    "Invalid Time of Validity".to_string(),
                ));
            }
        };

        // Ensure frames exist in the graph
        let parent_idx = self.ensure_frame(&transform_msg.parent_frame);
        let child_idx = self.ensure_frame(&transform_msg.child_frame);

        // Check for cycles
        if self.would_create_cycle(parent_idx, child_idx) {
            return Err(TransformError::CyclicTransformTree);
        }

        // Add edge if it doesn't exist
        if !self.graph.contains_edge(parent_idx, child_idx) {
            self.graph.add_edge(parent_idx, child_idx, ());
        }

        // Clear velocity cache since we're adding a transform that could change velocities
        self.velocity_cache.clear();

        // Create StampedTransform for the store (internal implementation detail)
        let stamped = StampedTransform {
            transform: transform_msg.transform,
            stamp: timestamp,
            parent_frame: transform_msg.parent_frame,
            child_frame: transform_msg.child_frame,
        };

        // Add transform to the store
        self.transform_store.add_transform(stamped);
        Ok(())
    }

    fn would_create_cycle(&self, parent: NodeIndex, child: NodeIndex) -> bool {
        if self.graph.contains_edge(parent, child) {
            return false;
        }

        matches!(dijkstra(&self.graph, child, Some(parent), |_| 1), result if result.contains_key(&parent))
    }

    pub fn find_path(
        &self,
        from_frame: &str,
        to_frame: &str,
    ) -> TransformResult<Vec<(FrameIdString, FrameIdString, bool)>> {
        // If frames are the same, return empty path (identity transform)
        if from_frame == to_frame {
            return Ok(Vec::new());
        }

        let from_frame_id = FrameIdString::from(from_frame).expect("Frame name too long");
        let from_idx = self
            .frame_indices
            .get(&from_frame_id)
            .ok_or(TransformError::FrameNotFound(from_frame.to_string()))?;

        let to_frame_id = FrameIdString::from(to_frame).expect("Frame name too long");
        let to_idx = self
            .frame_indices
            .get(&to_frame_id)
            .ok_or(TransformError::FrameNotFound(to_frame.to_string()))?;

        // Create an undirected version of the graph to find any path (forward or inverse)
        let mut undirected_graph = self.graph.clone();

        // Add reverse edges for every existing edge to make it undirected
        let edges: Vec<_> = self.graph.edge_indices().collect();
        for edge_idx in edges {
            let (a, b) = self.graph.edge_endpoints(edge_idx).unwrap();
            if !undirected_graph.contains_edge(b, a) {
                undirected_graph.add_edge(b, a, ());
            }
        }

        // Now find path in undirected graph
        let path = dijkstra(&undirected_graph, *from_idx, Some(*to_idx), |_| 1);

        if !path.contains_key(to_idx) {
            return Err(TransformError::TransformNotFound {
                from: from_frame.to_string(),
                to: to_frame.to_string(),
            });
        }

        // Reconstruct the path
        let mut current = *to_idx;
        let mut path_nodes = vec![current];

        while current != *from_idx {
            let mut found_next = false;

            // Try all neighbors in undirected graph
            for neighbor in undirected_graph.neighbors(current) {
                if path.contains_key(&neighbor) && path[&neighbor] < path[&current] {
                    current = neighbor;
                    path_nodes.push(current);
                    found_next = true;
                    break;
                }
            }

            if !found_next {
                return Err(TransformError::TransformNotFound {
                    from: from_frame.to_string(),
                    to: to_frame.to_string(),
                });
            }
        }

        path_nodes.reverse();

        // Convert node path to edge path with direction information
        let mut path_edges = Vec::new();
        for i in 0..path_nodes.len() - 1 {
            let parent_idx = path_nodes[i];
            let child_idx = path_nodes[i + 1];

            let parent_frame = self.graph[parent_idx];
            let child_frame = self.graph[child_idx];

            // Check if this is a forward edge in original directed graph
            let is_forward = self.graph.contains_edge(parent_idx, child_idx);

            if is_forward {
                // Forward edge: parent -> child
                path_edges.push((parent_frame, child_frame, false));
            } else {
                // Inverse edge: child <- parent (we need child -> parent)
                path_edges.push((child_frame, parent_frame, true));
            }
        }

        Ok(path_edges)
    }

    /// Compute a simple hash value for a path to use as a cache key
    fn compute_path_hash(path: &[(FrameIdString, FrameIdString, bool)]) -> u64 {
        use std::collections::hash_map::DefaultHasher;
        use std::hash::{Hash, Hasher};

        let mut hasher = DefaultHasher::new();

        for (parent, child, inverse) in path {
            parent.hash(&mut hasher);
            child.hash(&mut hasher);
            inverse.hash(&mut hasher);
        }

        hasher.finish()
    }

    pub fn lookup_transform(
        &self,
        from_frame: &str,
        to_frame: &str,
        time: CuTime,
        robot_clock: &RobotClock,
    ) -> TransformResult<Transform3D<T>> {
        // Identity case: same frame
        if from_frame == to_frame {
            return Ok(Self::create_identity_transform());
        }

        // Find the path between frames
        let path = self.find_path(from_frame, to_frame)?;

        if path.is_empty() {
            // Empty path is another case for identity transform
            return Ok(Self::create_identity_transform());
        }

        // Calculate a hash for the path (for cache lookups)
        let path_hash = Self::compute_path_hash(&path);

        // Try to get the transform from cache - concurrent map allows lock-free reads
        if let Some(cached_transform) =
            self.cache
                .get(from_frame, to_frame, time, path_hash, robot_clock)
        {
            return Ok(cached_transform);
        }

        // Check if it's time to clean up the cache
        if self.cache.should_cleanup(robot_clock) {
            self.cache.cleanup(robot_clock);
        }

        // Cache miss - compute the transform

        // Compose multiple transforms along the path
        let mut result = Self::create_identity_transform();

        // Iterate through each segment of the path
        for (parent, child, inverse) in &path {
            let buffer = self.get_segment_buffer(parent, child)?;

            let transform = buffer
                .get_closest_transform(time)
                .ok_or(TransformError::TransformTimeNotAvailable(time))?;

            // Note: In transform composition, the right-most transform is applied first.
            let transform_to_apply = if *inverse {
                transform.transform.inverse()
            } else {
                transform.transform
            };
            result = transform_to_apply * result;
        }

        // Cache the computed result
        self.cache
            .insert(from_frame, to_frame, result, time, path_hash, robot_clock);

        Ok(result)
    }

    /// Look up the velocity of a frame at a specific time
    ///
    /// This computes the velocity by differentiating transforms over time.
    /// Returns the velocity expressed in the target frame.
    ///
    /// Results are automatically cached for improved performance. The cache is
    /// invalidated when new transforms are added or when cache entries expire based
    /// on their age. The cache significantly improves performance for repeated lookups
    /// of the same frames and times.
    ///
    /// # Arguments
    /// * `from_frame` - The source frame
    /// * `to_frame` - The target frame
    /// * `time` - The time at which to compute the velocity
    ///
    /// # Returns
    /// * A VelocityTransform containing linear and angular velocity components
    /// * Error if the transform is not available or cannot be computed
    ///
    /// # Performance
    /// The first lookup of a specific frame pair and time will compute the velocity and
    /// cache the result. Subsequent lookups will use the cached result, which is much faster.
    /// For real-time or performance-critical applications, this caching is crucial.
    ///
    /// # Cache Management
    /// The cache is automatically cleared when new transforms are added. You can also
    /// manually clear the cache with `clear_cache()` or trigger cleanup with `cleanup_cache()`.
    pub fn lookup_velocity(
        &self,
        from_frame: &str,
        to_frame: &str,
        time: CuTime,
        robot_clock: &RobotClock,
    ) -> TransformResult<VelocityTransform<T>> {
        // Identity case: same frame
        if from_frame == to_frame {
            return Ok(VelocityTransform::default());
        }

        // Find the path between frames
        let path = self.find_path(from_frame, to_frame)?;

        if path.is_empty() {
            // Empty path means identity transform (zero velocity)
            return Ok(VelocityTransform::default());
        }

        // Calculate a hash for the path (for cache lookups)
        let path_hash = Self::compute_path_hash(&path);

        // Try to get the velocity from cache
        if let Some(cached_velocity) =
            self.velocity_cache
                .get(from_frame, to_frame, time, path_hash, robot_clock)
        {
            return Ok(cached_velocity);
        }

        // Check if it's time to clean up the cache
        if self.velocity_cache.should_cleanup(robot_clock) {
            self.velocity_cache.cleanup(robot_clock);
        }

        // Cache miss - compute the velocity

        // Initialize zero velocity
        let mut result = VelocityTransform::default();

        // Iterate through each segment of the path
        for (parent, child, inverse) in &path {
            let buffer = self.get_segment_buffer(parent, child)?;

            // Compute velocity for this segment
            let segment_velocity = buffer
                .compute_velocity_at_time(time)
                .ok_or(TransformError::TransformTimeNotAvailable(time))?;

            // Get the transform at the requested time
            let transform = buffer
                .get_closest_transform(time)
                .ok_or(TransformError::TransformTimeNotAvailable(time))?;

            // Apply the proper velocity transformation
            // We need the current position for proper velocity transformation
            let position = [T::default(); 3]; // Assume transformation at origin for simplicity
            // A more accurate implementation would track the position

            let transformed_velocity = if *inverse {
                let inverse_transform = transform.transform.inverse();
                crate::velocity::transform_velocity(
                    &segment_velocity.negate(),
                    &inverse_transform,
                    &position,
                )
            } else {
                crate::velocity::transform_velocity(
                    &segment_velocity,
                    &transform.transform,
                    &position,
                )
            };

            result.linear[0] += transformed_velocity.linear[0];
            result.linear[1] += transformed_velocity.linear[1];
            result.linear[2] += transformed_velocity.linear[2];

            result.angular[0] += transformed_velocity.angular[0];
            result.angular[1] += transformed_velocity.angular[1];
            result.angular[2] += transformed_velocity.angular[2];
        }

        // Cache the computed result
        self.velocity_cache.insert(
            from_frame,
            to_frame,
            result.clone(),
            time,
            path_hash,
            robot_clock,
        );

        Ok(result)
    }
}

impl<T: Copy + Debug + Default + One + Serialize + DeserializeOwned + 'static + Neg<Output = T>>
    Default for TransformTree<T>
where
    Transform3D<T>: Clone + HasInverse<T> + std::ops::Mul<Output = Transform3D<T>>,
    T: std::ops::Add<Output = T>
        + std::ops::Sub<Output = T>
        + std::ops::Mul<Output = T>
        + std::ops::Div<Output = T>
        + std::ops::AddAssign
        + std::ops::SubAssign
        + num_traits::NumCast,
{
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
#[allow(deprecated)] // We intentionally test deprecated APIs for backward compatibility
mod tests {
    use super::*;
    use crate::test_utils::get_translation;
    use crate::{FrameTransform, frame_id};
    use cu29::clock::{CuDuration, RobotClock};

    // Helper function to replace assert_relative_eq
    fn assert_approx_eq(actual: f32, expected: f32, epsilon: f32, message: &str) {
        let diff = (actual - expected).abs();
        assert!(
            diff <= epsilon,
            "{message}: expected {expected}, got {actual}, difference {diff} exceeds epsilon {epsilon}",
        );
    }

    fn make_stamped(
        parent: &str,
        child: &str,
        ts: CuDuration,
        tf: Transform3D<f32>,
    ) -> StampedFrameTransform<f32> {
        let inner = FrameTransform {
            transform: tf,
            parent_frame: frame_id!(parent),
            child_frame: frame_id!(child),
        };
        let mut stf = StampedFrameTransform::new(Some(inner));
        stf.tov = ts.into();
        stf
    }

    // Only use f32/f64 for tests since our inverse transform is only implemented for these types

    #[test]
    fn test_add_transform() {
        let mut tree = TransformTree::<f32>::new();

        let inner = FrameTransform {
            transform: Transform3D::default(),
            parent_frame: frame_id!("world"),
            child_frame: frame_id!("robot"),
        };
        let mut stf = StampedFrameTransform::new(Some(inner));
        stf.tov = CuDuration(1000).into();

        assert!(tree.add_transform(&stf).is_ok());
    }

    #[test]
    fn test_cyclic_transforms() {
        let mut tree = TransformTree::<f32>::new();

        let transform1 = make_stamped("world", "robot", 1000.into(), Transform3D::default());
        let transform2 = make_stamped("robot", "sensor", 1000.into(), Transform3D::default());
        let transform3 = make_stamped("sensor", "world", 1000.into(), Transform3D::default());

        assert!(tree.add_transform(&transform1).is_ok());
        assert!(tree.add_transform(&transform2).is_ok());

        let result = tree.add_transform(&transform3);
        assert!(result.is_err());
        if let Err(e) = result {
            assert!(matches!(e, TransformError::CyclicTransformTree));
        }
    }

    #[test]
    fn test_find_path() {
        let mut tree = TransformTree::<f32>::new();

        let transform1 = make_stamped("world", "robot", 1000.into(), Transform3D::default());
        let transform2 = make_stamped("robot", "sensor", 1000.into(), Transform3D::default());

        assert!(tree.add_transform(&transform1).is_ok());
        assert!(tree.add_transform(&transform2).is_ok());

        let path = tree.find_path("world", "sensor");
        assert!(path.is_ok());

        let path_vec = path.unwrap();
        assert_eq!(path_vec.len(), 2);
        assert_eq!(path_vec[0].0.as_str(), "world");
        assert_eq!(path_vec[0].1.as_str(), "robot");
        assert_eq!(path_vec[1].0.as_str(), "robot");
        assert_eq!(path_vec[1].1.as_str(), "sensor");
    }

    #[test]
    fn test_lookup_transform_with_inverse() {
        let mut tree = TransformTree::<f32>::new();

        let matrix = [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [2.0, 3.0, 4.0, 1.0],
        ];
        let tf = make_stamped(
            "world",
            "robot",
            CuDuration(1000),
            Transform3D::from_matrix(matrix),
        );

        assert!(tree.add_transform(&tf).is_ok());

        let clock = RobotClock::default();

        let forward = tree
            .lookup_transform("world", "robot", CuDuration(1000), &clock)
            .unwrap();
        assert_eq!(get_translation(&forward).0, 2.0);
        assert_eq!(get_translation(&forward).1, 3.0);
        assert_eq!(get_translation(&forward).2, 4.0);

        let inverse = tree
            .lookup_transform("robot", "world", CuDuration(1000), &clock)
            .unwrap();
        assert_eq!(get_translation(&inverse).0, -2.0);
        assert_eq!(get_translation(&inverse).1, -3.0);
        assert_eq!(get_translation(&inverse).2, -4.0);
    }

    #[test]
    fn test_multi_step_transform_composition() {
        let mut tree = TransformTree::<f32>::new();
        let ts = CuDuration(1000);

        let world_to_base = make_stamped(
            "world",
            "base",
            ts,
            Transform3D::from_matrix([
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [1.0, 0.0, 0.0, 1.0],
            ]),
        );

        let base_to_arm = make_stamped(
            "base",
            "arm",
            ts,
            Transform3D::from_matrix([
                [0.0, 1.0, 0.0, 0.0],
                [-1.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]),
        );

        let arm_to_gripper = make_stamped(
            "arm",
            "gripper",
            ts,
            Transform3D::from_matrix([
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 2.0, 0.0, 1.0],
            ]),
        );

        assert!(tree.add_transform(&world_to_base).is_ok());
        assert!(tree.add_transform(&base_to_arm).is_ok());
        assert!(tree.add_transform(&arm_to_gripper).is_ok());

        let clock = RobotClock::default();
        let transform = tree
            .lookup_transform("world", "gripper", ts, &clock)
            .unwrap();
        let epsilon = 1e-5;

        let mat = transform.to_matrix();
        assert_approx_eq(mat[0][0], 0.0, epsilon, "mat_0_0");
        assert_approx_eq(mat[1][0], -1.0, epsilon, "mat_1_0");
        assert_approx_eq(mat[0][1], 1.0, epsilon, "mat_0_1");
        assert_approx_eq(mat[1][1], 0.0, epsilon, "mat_1_1");

        assert_approx_eq(get_translation(&transform).0, 0.0, epsilon, "translation_x");
        assert_approx_eq(get_translation(&transform).1, 3.0, epsilon, "translation_y");
        assert_approx_eq(get_translation(&transform).2, 0.0, epsilon, "translation_z");

        let cached = tree
            .lookup_transform("world", "gripper", ts, &clock)
            .unwrap();
        for i in 0..4 {
            for j in 0..4 {
                assert_approx_eq(
                    transform.to_matrix()[i][j],
                    cached.to_matrix()[i][j],
                    epsilon,
                    "matrix_element",
                );
            }
        }

        let inverse = tree
            .lookup_transform("gripper", "world", ts, &clock)
            .unwrap();
        let inv_mat = inverse.to_matrix();
        assert_approx_eq(inv_mat[1][0], 1.0, epsilon, "inv_mat_1_0");
        assert_approx_eq(inv_mat[0][1], -1.0, epsilon, "inv_mat_0_1");
        assert_approx_eq(get_translation(&inverse).0, -3.0, epsilon, "translation_x");
        assert_approx_eq(get_translation(&inverse).1, 0.0, epsilon, "translation_y");

        let product = transform * inverse;
        for i in 0..4 {
            for j in 0..4 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert_approx_eq(
                    product.to_matrix()[i][j],
                    expected,
                    epsilon,
                    "matrix_element",
                );
            }
        }
    }

    #[test]
    fn test_cache_invalidation() {
        let mut tree = TransformTree::<f32>::with_cache_settings(5, Duration::from_millis(50));
        let ts = CuDuration(1000);

        let tf = make_stamped(
            "a",
            "b",
            ts,
            Transform3D::from_matrix([
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [1.0, 2.0, 3.0, 1.0],
            ]),
        );

        assert!(tree.add_transform(&tf).is_ok());

        let clock = RobotClock::default();
        let result1 = tree.lookup_transform("a", "b", ts, &clock);
        assert!(result1.is_ok());

        let transform1 = result1.unwrap();
        assert_eq!(get_translation(&transform1).0, 1.0);

        std::thread::sleep(Duration::from_millis(100));

        let result2 = tree.lookup_transform("a", "b", ts, &clock);
        assert!(result2.is_ok());

        tree.clear_cache();

        let result3 = tree.lookup_transform("a", "b", ts, &clock);
        assert!(result3.is_ok());
    }

    #[test]
    fn test_multi_step_transform_with_inverse() {
        let mut tree = TransformTree::<f32>::new();
        let ts = CuDuration(1000);

        let world_to_robot = make_stamped(
            "world",
            "robot",
            ts,
            Transform3D::from_matrix([
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [1.0, 2.0, 3.0, 1.0],
            ]),
        );

        let robot_to_camera = make_stamped(
            "robot",
            "camera",
            ts,
            Transform3D::from_matrix([
                [0.0, 1.0, 0.0, 0.0],
                [-1.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.5, 0.0, 0.2, 1.0],
            ]),
        );

        assert!(tree.add_transform(&world_to_robot).is_ok());
        assert!(tree.add_transform(&robot_to_camera).is_ok());

        let clock = RobotClock::default();
        let transform = tree
            .lookup_transform("world", "camera", ts, &clock)
            .unwrap();
        let epsilon = 1e-5;

        let mat = transform.to_matrix();
        assert_approx_eq(mat[0][0], 0.0, epsilon, "mat_0_0");
        assert_approx_eq(mat[1][0], -1.0, epsilon, "mat_1_0");
        assert_approx_eq(mat[0][1], 1.0, epsilon, "mat_0_1");
        assert_approx_eq(mat[1][1], 0.0, epsilon, "mat_1_1");

        assert_approx_eq(
            get_translation(&transform).0,
            -1.5,
            epsilon,
            "translation_x",
        );
        assert_approx_eq(get_translation(&transform).1, 1.0, epsilon, "translation_y");
        assert_approx_eq(get_translation(&transform).2, 3.2, epsilon, "translation_z");

        let inverse = tree.lookup_transform("camera", "world", ts, &clock);
        assert!(inverse.is_ok());
    }

    #[test]
    fn test_cache_cleanup() {
        let tree = TransformTree::<f32>::with_cache_settings(5, Duration::from_millis(10));

        // Explicitly trigger cache cleanup
        let clock = RobotClock::default();
        tree.cleanup_cache(&clock);
    }

    #[test]
    fn test_lookup_velocity() {
        let mut tree = TransformTree::<f32>::new();

        let w2b_1 = make_stamped(
            "world",
            "base",
            CuDuration(1_000_000_000),
            Transform3D::from_matrix([
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]),
        );

        let w2b_2 = make_stamped(
            "world",
            "base",
            CuDuration(2_000_000_000),
            Transform3D::from_matrix([
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [1.0, 0.0, 0.0, 1.0],
            ]),
        );

        let b2s_1 = make_stamped(
            "base",
            "sensor",
            CuDuration(1_000_000_000),
            Transform3D::from_matrix([
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]),
        );

        let b2s_2 = make_stamped(
            "base",
            "sensor",
            CuDuration(2_000_000_000),
            Transform3D::from_matrix([
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 2.0, 0.0, 1.0],
            ]),
        );

        tree.add_transform(&w2b_1).unwrap();
        tree.add_transform(&w2b_2).unwrap();
        tree.add_transform(&b2s_1).unwrap();
        tree.add_transform(&b2s_2).unwrap();

        let clock = RobotClock::default();
        let velocity = tree.lookup_velocity("world", "sensor", CuDuration(1_500_000_000), &clock);
        assert!(velocity.is_ok());

        let vel = velocity.unwrap();
        let epsilon = 0.1;
        assert_approx_eq(vel.linear[0], 1.0, epsilon, "linear_velocity_0");
        assert_approx_eq(vel.linear[1], 2.0, epsilon, "linear_velocity_1");
        assert_approx_eq(vel.linear[2], 0.0, epsilon, "linear_velocity_2");
    }

    #[test]
    fn test_velocity_with_rotation() {
        let mut tree = TransformTree::<f32>::new();

        let ts1 = CuDuration(1_000_000_000);
        let ts2 = CuDuration(2_000_000_000);

        let w2b_1 = make_stamped(
            "world",
            "base",
            ts1,
            Transform3D::from_matrix([
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]),
        );

        let b2s_1 = make_stamped(
            "base",
            "sensor",
            ts1,
            Transform3D::from_matrix([
                [0.0, 1.0, 0.0, 0.0],
                [-1.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [1.0, 0.0, 0.0, 1.0],
            ]),
        );

        let w2b_2 = make_stamped(
            "world",
            "base",
            ts2,
            Transform3D::from_matrix([
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [1.0, 0.0, 0.0, 1.0],
            ]),
        );

        let b2s_2 = make_stamped(
            "base",
            "sensor",
            ts2,
            Transform3D::from_matrix([
                [0.0, 1.0, 0.0, 0.0],
                [-1.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [1.0, 0.0, 0.0, 1.0],
            ]),
        );

        tree.add_transform(&w2b_1).unwrap();
        tree.add_transform(&w2b_2).unwrap();
        tree.add_transform(&b2s_1).unwrap();
        tree.add_transform(&b2s_2).unwrap();

        let clock = RobotClock::default();
        let mid_ts = CuDuration(1_500_000_000);

        let velocity = tree.lookup_velocity("world", "sensor", mid_ts, &clock);
        assert!(velocity.is_ok());
        let vel = velocity.unwrap();
        let epsilon = 0.2;
        assert_approx_eq(vel.linear[0], 1.0, epsilon, "linear_velocity_0");
        assert_approx_eq(vel.linear[1], 0.0, epsilon, "linear_velocity_1");
        assert_approx_eq(vel.linear[2], 0.0, epsilon, "linear_velocity_2");

        let reverse = tree.lookup_velocity("sensor", "world", mid_ts, &clock);
        assert!(reverse.is_ok());
        let rev_vel = reverse.unwrap();
        assert_approx_eq(rev_vel.linear[0], -1.0, epsilon, "linear_velocity_0");
        assert_approx_eq(rev_vel.linear[1], 0.0, epsilon, "linear_velocity_1");
        assert_approx_eq(rev_vel.linear[2], 0.0, epsilon, "linear_velocity_2");
    }

    #[test]
    fn test_velocity_with_angular_motion() {
        let mut tree = TransformTree::<f32>::new();
        let ts1 = CuDuration(1_000_000_000);
        let ts2 = CuDuration(2_000_000_000);

        let w2b_1 = make_stamped(
            "world",
            "base",
            ts1,
            Transform3D::from_matrix([
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]),
        );

        let w2b_2 = make_stamped(
            "world",
            "base",
            ts2,
            Transform3D::from_matrix([
                [0.0, 1.0, 0.0, 0.0],
                [-1.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]),
        );

        let b2s_1 = make_stamped(
            "base",
            "sensor",
            ts1,
            Transform3D::from_matrix([
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [1.0, 0.0, 0.0, 1.0],
            ]),
        );

        let b2s_2 = make_stamped(
            "base",
            "sensor",
            ts2,
            Transform3D::from_matrix([
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [1.0, 0.0, 0.0, 1.0],
            ]),
        );

        tree.add_transform(&w2b_1).unwrap();
        tree.add_transform(&w2b_2).unwrap();
        tree.add_transform(&b2s_1).unwrap();
        tree.add_transform(&b2s_2).unwrap();

        let clock = RobotClock::default();
        let vel = tree
            .lookup_velocity("world", "sensor", CuDuration(1_500_000_000), &clock)
            .unwrap();

        let epsilon = 0.1;
        assert_approx_eq(vel.angular[0], 0.0, epsilon, "angular_velocity_0");
        assert_approx_eq(vel.angular[1], 0.0, epsilon, "angular_velocity_1");
        assert_approx_eq(vel.angular[2], -1.0, epsilon, "angular_velocity_2");

        assert!(!vel.linear[0].is_nan());
        assert!(!vel.linear[1].is_nan());
        assert!(!vel.linear[2].is_nan());

        assert!(!vel.angular[0].is_nan());
        assert!(!vel.angular[1].is_nan());
        assert!(!vel.angular[2].is_nan());
    }

    #[test]
    fn test_velocity_cache() {
        let mut tree = TransformTree::<f32>::new();
        let ts1 = CuDuration(1_000_000_000);
        let ts2 = CuDuration(2_000_000_000);

        let tf1 = make_stamped(
            "world",
            "robot",
            ts1,
            Transform3D::from_matrix([
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]),
        );

        let tf2 = make_stamped(
            "world",
            "robot",
            ts2,
            Transform3D::from_matrix([
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [2.0, 0.0, 0.0, 1.0],
            ]),
        );

        tree.add_transform(&tf1).unwrap();
        tree.add_transform(&tf2).unwrap();

        let clock = RobotClock::default();

        let start_time = std::time::Instant::now();
        let velocity1 = tree.lookup_velocity("world", "robot", CuDuration(1_500_000_000), &clock);
        let first_lookup_time = start_time.elapsed();

        assert!(velocity1.is_ok());
        let vel1 = velocity1.unwrap();
        assert_approx_eq(vel1.linear[0], 2.0, 0.01, "linear_velocity_0");

        let start_time = std::time::Instant::now();
        let velocity2 = tree.lookup_velocity("world", "robot", CuDuration(1_500_000_000), &clock);
        let second_lookup_time = start_time.elapsed();

        assert!(velocity2.is_ok());
        let vel2 = velocity2.unwrap();
        assert_approx_eq(vel2.linear[0], 2.0, 0.01, "linear_velocity_0");

        tree.clear_cache();

        let start_time = std::time::Instant::now();
        let velocity3 = tree.lookup_velocity("world", "robot", CuDuration(1_500_000_000), &clock);
        let third_lookup_time = start_time.elapsed();

        assert!(velocity3.is_ok());

        println!("First lookup: {first_lookup_time:?}");
        println!("Second lookup (cached): {second_lookup_time:?}");
        println!("Third lookup (after cache clear): {third_lookup_time:?}");
    }

    #[test]
    fn test_velocity_cache_invalidation() {
        let mut tree = TransformTree::<f32>::new();
        let ts1 = CuDuration(1_000_000_000);
        let ts2 = CuDuration(2_000_000_000);
        let ts3 = CuDuration(3_000_000_000);

        let tf1 = make_stamped(
            "world",
            "robot",
            ts1,
            Transform3D::from_matrix([
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]),
        );

        let tf2 = make_stamped(
            "world",
            "robot",
            ts2,
            Transform3D::from_matrix([
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [1.0, 0.0, 0.0, 1.0],
            ]),
        );

        let tf3 = make_stamped(
            "world",
            "robot",
            ts3,
            Transform3D::from_matrix([
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [3.0, 0.0, 0.0, 1.0],
            ]),
        );

        tree.add_transform(&tf1).unwrap();
        tree.add_transform(&tf2).unwrap();

        let clock = RobotClock::default();
        let velocity1 = tree
            .lookup_velocity("world", "robot", CuDuration(1_500_000_000), &clock)
            .unwrap();
        assert_approx_eq(velocity1.linear[0], 1.0, 0.01, "linear_velocity_0");

        tree.add_transform(&tf3).unwrap();

        let velocity2 = tree
            .lookup_velocity("world", "robot", CuDuration(1_500_000_000), &clock)
            .unwrap();
        assert_approx_eq(velocity2.linear[0], 1.0, 0.01, "linear_velocity_0");

        let velocity3 = tree
            .lookup_velocity("world", "robot", CuDuration(2_500_000_000), &clock)
            .unwrap();
        assert_approx_eq(velocity3.linear[0], 2.0, 0.01, "linear_velocity_0");
    }
}
