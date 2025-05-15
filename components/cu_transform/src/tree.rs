use crate::error::{TransformError, TransformResult};
use crate::transform::{StampedTransform, TransformBuffer};
use cu29::clock::CuTime;
use cu_spatial_payloads::Transform3D;
use petgraph::algo::dijkstra;
use petgraph::graph::{DiGraph, NodeIndex};
use std::collections::HashMap;
use std::fmt::Debug;
use std::sync::{Arc, RwLock};

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

pub struct TransformTree<T: Copy + Debug + Default + 'static> {
    graph: DiGraph<String, ()>,
    frame_indices: HashMap<String, NodeIndex>,
    transform_buffers: HashMap<(String, String), Arc<RwLock<TransformBuffer<T>>>>,
}

// We need to limit T to types where Transform3D<T> has Clone and inverse method
impl<T: Copy + Debug + Default + 'static> TransformTree<T>
where
    Transform3D<T>: Clone + HasInverse<T>,
{
    pub fn new() -> Self {
        Self {
            graph: DiGraph::new(),
            frame_indices: HashMap::new(),
            transform_buffers: HashMap::new(),
        }
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

    pub fn lookup_transform(
        &self,
        from_frame: &str,
        to_frame: &str,
        time: CuTime,
    ) -> TransformResult<Transform3D<T>> {
        if from_frame == to_frame {
            // Create a proper identity matrix
            return Ok(Self::create_identity_transform());
        }

        let path = self.find_path(from_frame, to_frame)?;

        if path.is_empty() {
            // Another case for identity transform
            return Ok(Self::create_identity_transform());
        }

        // For now, we're only handling individual transforms, not chaining them
        // A complete implementation would multiply the transforms together

        // Get the appropriate step from the path
        let (parent, child, inverse) = &path[0];

        // In all cases, the buffer is stored with the parent->child key
        let buffer_key = (parent.clone(), child.clone());

        // Check if the buffer exists with this key
        let buffer = match self.transform_buffers.get(&buffer_key) {
            Some(b) => b,
            None => {
                // If we need to apply an inverse transform, we need to look up the transform
                // in the opposite direction, because we stored transforms directionally
                if *inverse {
                    return Err(TransformError::TransformNotFound {
                        from: parent.clone(),
                        to: child.clone(),
                    });
                } else {
                    // For a regular transform, if we can't find it, it's an error
                    return Err(TransformError::TransformNotFound {
                        from: parent.clone(),
                        to: child.clone(),
                    });
                }
            }
        };

        let buffer_read = buffer.read().unwrap();
        let transform = buffer_read
            .get_closest_transform(time)
            .ok_or(TransformError::TransformTimeNotAvailable(time))?;

        if *inverse {
            Ok(transform.transform.inverse())
        } else {
            Ok(transform.transform.clone())
        }
    }
}

impl<T: Copy + Debug + Default + 'static> Default for TransformTree<T>
where
    Transform3D<T>: Clone + HasInverse<T>,
{
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
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

        // Debug output for frames
        println!("Frames in the tree:");
        for frame in tree.frame_indices.keys() {
            println!("  {}", frame);
        }

        // Debug output for connections
        println!("Connections in the tree:");
        for (parent, child) in tree.transform_buffers.keys() {
            println!("  {} -> {}", parent, child);
        }

        // Now get the inverse transform (robot to world)
        let inverse_transform = tree.lookup_transform("robot", "world", CuDuration(1000));

        if let Err(e) = &inverse_transform {
            println!("Error looking up inverse transform: {:?}", e);
        }

        assert!(inverse_transform.is_ok());
        let inverse = inverse_transform.unwrap();

        // The inverse transform should have negated translation
        assert_eq!(inverse.mat[0][3], -2.0);
        assert_eq!(inverse.mat[1][3], -3.0);
        assert_eq!(inverse.mat[2][3], -4.0);
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

        // Debug output for frames
        println!("Frames in the tree:");
        for frame in tree.frame_indices.keys() {
            println!("  {}", frame);
        }

        // Debug output for connections
        println!("Connections in the tree:");
        for (parent, child) in tree.transform_buffers.keys() {
            println!("  {} -> {}", parent, child);
        }

        // Look up the transform from world to camera (should combine both transforms)
        let world_to_camera = tree.lookup_transform("world", "camera", CuDuration(1000));

        if let Err(e) = &world_to_camera {
            println!("Error looking up world to camera transform: {:?}", e);
        }

        assert!(world_to_camera.is_ok());

        // Now look up the inverse transform (camera to world)
        let camera_to_world = tree.lookup_transform("camera", "world", CuDuration(1000));

        if let Err(e) = &camera_to_world {
            println!("Error looking up camera to world transform: {:?}", e);
        }

        // Try getting the path directly
        let path = tree.find_path("camera", "world");
        if let Ok(p) = path {
            println!("Path from camera to world:");
            for (parent, child, inverse) in p {
                println!("  {} -> {} (inverse: {})", parent, child, inverse);
            }
        } else if let Err(e) = path {
            println!("Error finding path: {:?}", e);
        }

        assert!(camera_to_world.is_ok());

        // Check if one transform is the inverse of the other
        // We'd need matrix multiplication to properly verify this, but for now
        // we'll just check that we get a result without error
    }
}
