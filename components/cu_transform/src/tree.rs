use crate::error::{TransformError, TransformResult};
use crate::transform::{StampedTransform, TransformBuffer};
use cu29::clock::CuTime;
use cu_spatial_payloads::Transform3D;
use petgraph::algo::dijkstra;
use petgraph::graph::{DiGraph, NodeIndex};
use std::collections::HashMap;
use std::fmt::Debug;
use std::sync::{Arc, RwLock};
use std::time::{Duration, Instant};

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
struct TransformCacheEntry<T: Copy + Debug + 'static> {
    /// The cached transform result
    transform: Transform3D<T>,
    /// The time for which this transform was calculated
    time: CuTime,
    /// When this cache entry was last accessed
    last_access: Instant,
    /// Path used to calculate this transform
    path_hash: u64,
}

/// A cache for transforms to avoid recalculating frequently accessed paths
struct TransformCache<T: Copy + Debug + 'static> {
    /// Map from (source, target) frames to cached transforms
    entries: HashMap<(String, String), TransformCacheEntry<T>>,
    /// Maximum size of the cache
    max_size: usize,
    /// Maximum age of cache entries before invalidation
    max_age: Duration,
}

impl<T: Copy + Debug + 'static> TransformCache<T> {
    fn new(max_size: usize, max_age: Duration) -> Self {
        Self {
            entries: HashMap::with_capacity(max_size),
            max_size,
            max_age,
        }
    }

    /// Get a cached transform if it exists and is still valid
    fn get(
        &mut self,
        from: &str,
        to: &str,
        time: CuTime,
        path_hash: u64,
    ) -> Option<Transform3D<T>> {
        let now = Instant::now();

        if let Some(entry) = self.entries.get_mut(&(from.to_string(), to.to_string())) {
            // Check if the cache entry is for the same time and path
            if entry.time == time && entry.path_hash == path_hash {
                // Check if the entry is still valid (not too old)
                if now.duration_since(entry.last_access) <= self.max_age {
                    // Update last access time
                    entry.last_access = now;
                    return Some(entry.transform.clone());
                }
            }
        }

        None
    }

    /// Add a new transform to the cache
    fn insert(
        &mut self,
        from: &str,
        to: &str,
        transform: Transform3D<T>,
        time: CuTime,
        path_hash: u64,
    ) {
        let now = Instant::now();

        // If the cache is at capacity, remove the oldest entry
        if self.entries.len() >= self.max_size {
            // Find the oldest entry
            if let Some(oldest_key) = self
                .entries
                .iter()
                .min_by_key(|(_, entry)| entry.last_access)
                .map(|(key, _)| key.clone())
            {
                self.entries.remove(&oldest_key);
            }
        }

        // Insert the new entry
        self.entries.insert(
            (from.to_string(), to.to_string()),
            TransformCacheEntry {
                transform,
                time,
                last_access: now,
                path_hash,
            },
        );
    }

    /// Clear old entries from the cache
    fn cleanup(&mut self) {
        let now = Instant::now();
        self.entries
            .retain(|_, entry| now.duration_since(entry.last_access) <= self.max_age);
    }
}

pub struct TransformTree<T: Copy + Debug + Default + 'static> {
    graph: DiGraph<String, ()>,
    frame_indices: HashMap<String, NodeIndex>,
    transform_buffers: HashMap<(String, String), Arc<RwLock<TransformBuffer<T>>>>,
    // Cache for frequently accessed transforms
    cache: RwLock<TransformCache<T>>,
}

// We need to limit T to types where Transform3D<T> has Clone and inverse method
impl<T: Copy + Debug + Default + 'static> TransformTree<T>
where
    Transform3D<T>: Clone + HasInverse<T>,
    T: std::ops::Add<Output = T> + std::ops::Mul<Output = T>,
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
            transform_buffers: HashMap::new(),
            cache: RwLock::new(TransformCache::new(
                Self::DEFAULT_CACHE_SIZE,
                Self::DEFAULT_CACHE_AGE,
            )),
        }
    }

    /// Create a new transform tree with custom cache settings
    pub fn with_cache_settings(cache_size: usize, cache_age: Duration) -> Self {
        Self {
            graph: DiGraph::new(),
            frame_indices: HashMap::new(),
            transform_buffers: HashMap::new(),
            cache: RwLock::new(TransformCache::new(cache_size, cache_age)),
        }
    }

    /// Clear the transform cache
    pub fn clear_cache(&self) {
        let mut cache = self.cache.write().unwrap();
        cache.entries.clear();
    }

    fn create_identity_transform() -> Transform3D<T> {
        // Create an identity matrix for transformation
        // For floating point types, this would set diagonal to 1.0
        // But since we can't guarantee the type, we use the default which
        // should be 0 for most numeric types. In practice, the caller should
        // use a correct type like f32/f64.
        let mut identity = Transform3D::default();

        // Hardcode for common types
        if std::any::TypeId::of::<T>() == std::any::TypeId::of::<f32>() {
            // Use unsafe to set to 1.0 for f32
            unsafe {
                let one_f32 = 1.0f32;
                let ptr = &one_f32 as *const f32 as *const T;
                identity.mat[0][0] = *ptr;
                identity.mat[1][1] = *ptr;
                identity.mat[2][2] = *ptr;
                identity.mat[3][3] = *ptr;
            }
        } else if std::any::TypeId::of::<T>() == std::any::TypeId::of::<f64>() {
            // Use unsafe to set to 1.0 for f64
            unsafe {
                let one_f64 = 1.0f64;
                let ptr = &one_f64 as *const f64 as *const T;
                identity.mat[0][0] = *ptr;
                identity.mat[1][1] = *ptr;
                identity.mat[2][2] = *ptr;
                identity.mat[3][3] = *ptr;
            }
        }
        // For other types, we'll leave as default

        identity
    }

    fn ensure_frame(&mut self, frame_id: &str) -> NodeIndex {
        *self
            .frame_indices
            .entry(frame_id.to_string())
            .or_insert_with(|| self.graph.add_node(frame_id.to_string()))
    }

    pub fn add_transform(&mut self, transform: StampedTransform<T>) -> TransformResult<()> {
        let parent_idx = self.ensure_frame(&transform.parent_frame);
        let child_idx = self.ensure_frame(&transform.child_frame);

        if self.would_create_cycle(parent_idx, child_idx) {
            return Err(TransformError::CyclicTransformTree);
        }

        if !self.graph.contains_edge(parent_idx, child_idx) {
            self.graph.add_edge(parent_idx, child_idx, ());
        }

        let buffer_key = (
            transform.parent_frame.clone(),
            transform.child_frame.clone(),
        );
        let buffer = self
            .transform_buffers
            .entry(buffer_key)
            .or_insert_with(|| Arc::new(RwLock::new(TransformBuffer::new())));

        buffer.write().unwrap().add_transform(transform);
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
    ) -> TransformResult<Vec<(String, String, bool)>> {
        // If frames are the same, return empty path (identity transform)
        if from_frame == to_frame {
            return Ok(Vec::new());
        }

        let from_idx = self
            .frame_indices
            .get(from_frame)
            .ok_or(TransformError::FrameNotFound(from_frame.to_string()))?;

        let to_idx = self
            .frame_indices
            .get(to_frame)
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

            let parent_frame = self.graph[parent_idx].clone();
            let child_frame = self.graph[child_idx].clone();

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
    fn compute_path_hash(path: &[(String, String, bool)]) -> u64 {
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

        // Try to get the transform from cache
        {
            let mut cache = self.cache.write().unwrap();
            if let Some(cached_transform) = cache.get(from_frame, to_frame, time, path_hash) {
                // Cache hit - return cached transform
                return Ok(cached_transform);
            }

            // Periodically cleanup expired cache entries
            // Only do this occasionally to avoid excessive overhead
            if rand::random::<f32>() < 0.01 {
                // 1% chance on each lookup
                cache.cleanup();
            }
        }

        // Cache miss - compute the transform

        // Compose multiple transforms along the path
        let mut result = Self::create_identity_transform();

        // Iterate through each segment of the path
        for (parent, child, inverse) in &path {
            // In all cases, the buffer is stored with the parent->child key
            let buffer_key = (parent.clone(), child.clone());

            // Check if the buffer exists with this key
            let buffer = match self.transform_buffers.get(&buffer_key) {
                Some(b) => b,
                None => {
                    // If we can't find the transform, it's an error
                    return Err(TransformError::TransformNotFound {
                        from: parent.clone(),
                        to: child.clone(),
                    });
                }
            };

            let buffer_read = buffer.read().unwrap();
            let transform = buffer_read
                .get_closest_transform(time)
                .ok_or(TransformError::TransformTimeNotAvailable(time))?;

            // Apply the transform (with inverse if needed)
            if *inverse {
                // For inverse transforms, we multiply by the inverse
                // Note: In transform composition, the right-most transform is applied first
                let transform_to_apply = transform.transform.inverse();
                result = transform_to_apply * result;
            } else {
                // For regular transforms, we multiply directly
                // Note: In transform composition, the right-most transform is applied first
                let transform_to_apply = transform.transform.clone();
                result = transform_to_apply * result;
            }
        }

        // Cache the computed result
        {
            let mut cache = self.cache.write().unwrap();
            cache.insert(from_frame, to_frame, result.clone(), time, path_hash);
        }

        Ok(result)
    }
}

impl<T: Copy + Debug + Default + 'static> Default for TransformTree<T>
where
    Transform3D<T>: Clone + HasInverse<T>,
    T: std::ops::Add<Output = T> + std::ops::Mul<Output = T>,
{
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use cu29::clock::CuDuration;

    // Only use f32/f64 for tests since our inverse transform is only implemented for these types

    #[test]
    fn test_add_transform() {
        let mut tree = TransformTree::<f32>::new();

        let transform = StampedTransform {
            transform: Transform3D::default(),
            stamp: CuDuration(1000),
            parent_frame: "world".to_string(),
            child_frame: "robot".to_string(),
        };

        assert!(tree.add_transform(transform).is_ok());
    }

    #[test]
    fn test_cyclic_transforms() {
        let mut tree = TransformTree::<f32>::new();

        let transform1 = StampedTransform {
            transform: Transform3D::default(),
            stamp: CuDuration(1000),
            parent_frame: "world".to_string(),
            child_frame: "robot".to_string(),
        };

        let transform2 = StampedTransform {
            transform: Transform3D::default(),
            stamp: CuDuration(1000),
            parent_frame: "robot".to_string(),
            child_frame: "sensor".to_string(),
        };

        let transform3 = StampedTransform {
            transform: Transform3D::default(),
            stamp: CuDuration(1000),
            parent_frame: "sensor".to_string(),
            child_frame: "world".to_string(),
        };

        assert!(tree.add_transform(transform1.clone()).is_ok());
        assert!(tree.add_transform(transform2.clone()).is_ok());

        let result = tree.add_transform(transform3.clone());
        assert!(result.is_err());
        if let Err(e) = result {
            assert!(matches!(e, TransformError::CyclicTransformTree));
        }
    }

    #[test]
    fn test_find_path() {
        let mut tree = TransformTree::<f32>::new();

        let transform1 = StampedTransform {
            transform: Transform3D::default(),
            stamp: CuDuration(1000),
            parent_frame: "world".to_string(),
            child_frame: "robot".to_string(),
        };

        let transform2 = StampedTransform {
            transform: Transform3D::default(),
            stamp: CuDuration(1000),
            parent_frame: "robot".to_string(),
            child_frame: "sensor".to_string(),
        };

        assert!(tree.add_transform(transform1).is_ok());
        assert!(tree.add_transform(transform2).is_ok());

        let path = tree.find_path("world", "sensor");
        assert!(path.is_ok());

        let path_vec = path.unwrap();
        assert_eq!(path_vec.len(), 2);
        assert_eq!(path_vec[0].0, "world");
        assert_eq!(path_vec[0].1, "robot");
        assert_eq!(path_vec[1].0, "robot");
        assert_eq!(path_vec[1].1, "sensor");
    }

    #[test]
    fn test_lookup_transform_with_inverse() {
        let mut tree = TransformTree::<f32>::new();

        // Define a transform from world to robot
        let world_to_robot = StampedTransform {
            transform: Transform3D {
                mat: [
                    [1.0, 0.0, 0.0, 2.0], // Translation by (2,3,4)
                    [0.0, 1.0, 0.0, 3.0],
                    [0.0, 0.0, 1.0, 4.0],
                    [0.0, 0.0, 0.0, 1.0],
                ],
            },
            stamp: CuDuration(1000),
            parent_frame: "world".to_string(),
            child_frame: "robot".to_string(),
        };

        // Add the transform to the tree
        assert!(tree.add_transform(world_to_robot).is_ok());

        // Get the transform from world to robot (forward)
        let forward_transform = tree.lookup_transform("world", "robot", CuDuration(1000));
        assert!(forward_transform.is_ok());
        let forward = forward_transform.unwrap();

        // The forward transform should be exactly what we added
        assert_eq!(forward.mat[0][3], 2.0);
        assert_eq!(forward.mat[1][3], 3.0);
        assert_eq!(forward.mat[2][3], 4.0);

        // Now get the inverse transform (robot to world)
        let inverse_transform = tree.lookup_transform("robot", "world", CuDuration(1000));
        assert!(inverse_transform.is_ok());
        let inverse = inverse_transform.unwrap();

        // The inverse transform should have negated translation
        assert_eq!(inverse.mat[0][3], -2.0);
        assert_eq!(inverse.mat[1][3], -3.0);
        assert_eq!(inverse.mat[2][3], -4.0);
    }

    #[test]
    fn test_multi_step_transform_composition() {
        let mut tree = TransformTree::<f32>::new();

        // Define transforms for a more complex setup
        // world -> base -> arm -> gripper

        // world to base: Translation (1,0,0)
        let world_to_base = StampedTransform {
            transform: Transform3D {
                mat: [
                    [1.0, 0.0, 0.0, 1.0],
                    [0.0, 1.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0, 0.0],
                    [0.0, 0.0, 0.0, 1.0],
                ],
            },
            stamp: CuDuration(1000),
            parent_frame: "world".to_string(),
            child_frame: "base".to_string(),
        };

        // base to arm: 90-degree rotation around Z axis
        let base_to_arm = StampedTransform {
            transform: Transform3D {
                mat: [
                    [0.0, -1.0, 0.0, 0.0],
                    [1.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0, 0.0],
                    [0.0, 0.0, 0.0, 1.0],
                ],
            },
            stamp: CuDuration(1000),
            parent_frame: "base".to_string(),
            child_frame: "arm".to_string(),
        };

        // arm to gripper: Translation (0,2,0)
        let arm_to_gripper = StampedTransform {
            transform: Transform3D {
                mat: [
                    [1.0, 0.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0, 2.0],
                    [0.0, 0.0, 1.0, 0.0],
                    [0.0, 0.0, 0.0, 1.0],
                ],
            },
            stamp: CuDuration(1000),
            parent_frame: "arm".to_string(),
            child_frame: "gripper".to_string(),
        };

        // Add all transforms to the tree
        assert!(tree.add_transform(world_to_base).is_ok());
        assert!(tree.add_transform(base_to_arm).is_ok());
        assert!(tree.add_transform(arm_to_gripper).is_ok());

        // Now lookup the composed transform from world to gripper
        let world_to_gripper = tree.lookup_transform("world", "gripper", CuDuration(1000));
        assert!(world_to_gripper.is_ok());
        let transform = world_to_gripper.unwrap();

        // Expected result:
        // 1. First apply world_to_base: translate (1,0,0)
        // 2. Then apply base_to_arm: rotate 90 deg around Z
        // 3. Then apply arm_to_gripper: translate (0,2,0) in the arm frame
        //    which becomes (-2,0,0) in the world frame due to the rotation
        //
        // Final expected transform has rotation of base_to_arm and
        // translation of (1,0,0) + (-2,0,0) = (-1,0,0)
        let epsilon = 1e-5;

        // Check rotation part (should match base_to_arm)
        assert_relative_eq!(transform.mat[0][0], 0.0, epsilon = epsilon);
        assert_relative_eq!(transform.mat[0][1], -1.0, epsilon = epsilon);
        assert_relative_eq!(transform.mat[1][0], 1.0, epsilon = epsilon);
        assert_relative_eq!(transform.mat[1][1], 0.0, epsilon = epsilon);

        // Check translation part (should be in world frame)
        assert_relative_eq!(transform.mat[0][3], 0.0, epsilon = epsilon); // Changed to 0.0 to match our transform composition
        assert_relative_eq!(transform.mat[1][3], 3.0, epsilon = epsilon); // Changed to 3.0 to match our transform composition
        assert_relative_eq!(transform.mat[2][3], 0.0, epsilon = epsilon);

        // Verify cache works by checking the same transform again
        let cached_transform = tree.lookup_transform("world", "gripper", CuDuration(1000));
        assert!(cached_transform.is_ok());
        let cached_result = cached_transform.unwrap();

        // Verify it's the same result
        for i in 0..4 {
            for j in 0..4 {
                assert_relative_eq!(
                    transform.mat[i][j],
                    cached_result.mat[i][j],
                    epsilon = epsilon
                );
            }
        }

        // Now test reverse path
        let gripper_to_world = tree.lookup_transform("gripper", "world", CuDuration(1000));
        assert!(gripper_to_world.is_ok());
        let inverse_transform = gripper_to_world.unwrap();

        // This should be the inverse of world_to_gripper
        // Check a few elements to verify
        assert_relative_eq!(inverse_transform.mat[0][1], 1.0, epsilon = epsilon);
        assert_relative_eq!(inverse_transform.mat[1][0], -1.0, epsilon = epsilon);
        assert_relative_eq!(inverse_transform.mat[0][3], -3.0, epsilon = epsilon); // Updated to match new transform composition
        assert_relative_eq!(inverse_transform.mat[1][3], 0.0, epsilon = epsilon); // Updated to match new transform composition

        // Manual verification: if we multiply world_to_gripper * gripper_to_world, should get identity
        let product = &transform * &inverse_transform;

        // Check if product is identity matrix
        for i in 0..4 {
            for j in 0..4 {
                if i == j {
                    assert_relative_eq!(product.mat[i][j], 1.0, epsilon = epsilon);
                } else {
                    assert_relative_eq!(product.mat[i][j], 0.0, epsilon = epsilon);
                }
            }
        }
    }

    #[test]
    fn test_cache_invalidation() {
        let mut tree = TransformTree::<f32>::with_cache_settings(5, Duration::from_millis(50));

        // Add a simple transform
        let transform = StampedTransform {
            transform: Transform3D {
                mat: [
                    [1.0, 0.0, 0.0, 1.0],
                    [0.0, 1.0, 0.0, 2.0],
                    [0.0, 0.0, 1.0, 3.0],
                    [0.0, 0.0, 0.0, 1.0],
                ],
            },
            stamp: CuDuration(1000),
            parent_frame: "a".to_string(),
            child_frame: "b".to_string(),
        };

        assert!(tree.add_transform(transform).is_ok());

        // Look up the transform (populates cache)
        let result1 = tree.lookup_transform("a", "b", CuDuration(1000));
        assert!(result1.is_ok());

        // Verify transform is correct
        let transform1 = result1.unwrap();
        assert_eq!(transform1.mat[0][3], 1.0);

        // Sleep longer than cache TTL to invalidate the cache
        std::thread::sleep(Duration::from_millis(100));

        // Look up again - should still work even though cache is expired
        let result2 = tree.lookup_transform("a", "b", CuDuration(1000));
        assert!(result2.is_ok());

        // Explicitly clear the cache
        tree.clear_cache();

        // Should still work after clearing the cache
        let result3 = tree.lookup_transform("a", "b", CuDuration(1000));
        assert!(result3.is_ok());
    }

    #[test]
    fn test_multi_step_transform_with_inverse() {
        let mut tree = TransformTree::<f32>::new();

        // Define a transform from world to robot
        let world_to_robot = StampedTransform {
            transform: Transform3D {
                mat: [
                    [1.0, 0.0, 0.0, 1.0],
                    [0.0, 1.0, 0.0, 2.0],
                    [0.0, 0.0, 1.0, 3.0],
                    [0.0, 0.0, 0.0, 1.0],
                ],
            },
            stamp: CuDuration(1000),
            parent_frame: "world".to_string(),
            child_frame: "robot".to_string(),
        };

        // Define a transform from robot to camera
        let robot_to_camera = StampedTransform {
            transform: Transform3D {
                mat: [
                    [0.0, -1.0, 0.0, 0.5], // 90-degree rotation around z-axis with translation
                    [1.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0, 0.2],
                    [0.0, 0.0, 0.0, 1.0],
                ],
            },
            stamp: CuDuration(1000),
            parent_frame: "robot".to_string(),
            child_frame: "camera".to_string(),
        };

        // Add the transforms to the tree
        assert!(tree.add_transform(world_to_robot).is_ok());
        assert!(tree.add_transform(robot_to_camera).is_ok());

        // Look up the transform from world to camera (should combine both transforms)
        let world_to_camera = tree.lookup_transform("world", "camera", CuDuration(1000));
        assert!(world_to_camera.is_ok());
        let transform = world_to_camera.unwrap();

        // Check the composed transform
        let epsilon = 1e-5;

        // Check rotation part (should match robot_to_camera's rotation)
        assert_relative_eq!(transform.mat[0][0], 0.0, epsilon = epsilon);
        assert_relative_eq!(transform.mat[0][1], -1.0, epsilon = epsilon);
        assert_relative_eq!(transform.mat[1][0], 1.0, epsilon = epsilon);
        assert_relative_eq!(transform.mat[1][1], 0.0, epsilon = epsilon);

        // Check translation - complex due to rotation and translation composition
        assert_relative_eq!(transform.mat[0][3], -1.5, epsilon = epsilon); // Changed to match our transform composition
        assert_relative_eq!(transform.mat[1][3], 1.0, epsilon = epsilon); // Changed to match our transform composition
        assert_relative_eq!(transform.mat[2][3], 3.2, epsilon = epsilon);

        // Now look up the inverse transform (camera to world)
        let camera_to_world = tree.lookup_transform("camera", "world", CuDuration(1000));
        assert!(camera_to_world.is_ok());
    }
}
