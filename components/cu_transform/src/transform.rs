use cu29::clock::{CuTime, CuTimeRange};
use cu_spatial_payloads::Transform3D;
use dashmap::DashMap;
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;
use std::fmt::Debug;
use std::sync::{Arc, RwLock};

const DEFAULT_CACHE_SIZE: usize = 100;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct StampedTransform<T: Copy + Debug + 'static> {
    pub transform: Transform3D<T>,
    pub stamp: CuTime,
    pub parent_frame: String,
    pub child_frame: String,
}

/// Internal transform buffer that holds ordered transforms between two frames
#[derive(Clone, Debug, Serialize, Deserialize)]
struct TransformBufferInternal<T: Copy + Debug + 'static> {
    transforms: VecDeque<StampedTransform<T>>,
    max_capacity: usize,
}

/// Thread-safe wrapper around a transform buffer with concurrent access
#[derive(Clone)]
pub struct TransformBuffer<T: Copy + Debug + 'static> {
    buffer: Arc<RwLock<TransformBufferInternal<T>>>,
}

impl<T: Copy + Debug + 'static> TransformBufferInternal<T> {
    fn new() -> Self {
        Self::with_capacity(DEFAULT_CACHE_SIZE)
    }

    fn with_capacity(capacity: usize) -> Self {
        Self {
            transforms: VecDeque::with_capacity(capacity),
            max_capacity: capacity,
        }
    }

    fn add_transform(&mut self, transform: StampedTransform<T>) {
        let pos = self
            .transforms
            .partition_point(|t| t.stamp <= transform.stamp);

        self.transforms.insert(pos, transform);

        while self.transforms.len() > self.max_capacity {
            self.transforms.pop_front();
        }
    }

    fn get_latest_transform(&self) -> Option<&StampedTransform<T>> {
        self.transforms.back()
    }

    fn get_time_range(&self) -> Option<CuTimeRange> {
        if self.transforms.is_empty() {
            return None;
        }

        Some(CuTimeRange {
            start: self.transforms.front().unwrap().stamp,
            end: self.transforms.back().unwrap().stamp,
        })
    }

    fn get_transforms_in_range(
        &self,
        start_time: CuTime,
        end_time: CuTime,
    ) -> Vec<&StampedTransform<T>> {
        self.transforms
            .iter()
            .filter(|t| t.stamp >= start_time && t.stamp <= end_time)
            .collect()
    }

    fn get_closest_transform(&self, time: CuTime) -> Option<&StampedTransform<T>> {
        if self.transforms.is_empty() {
            return None;
        }

        let pos = self.transforms.partition_point(|t| t.stamp <= time);

        match pos {
            0 => self.transforms.front(),

            p if p == self.transforms.len() => self.transforms.back(),

            p => {
                let before = &self.transforms[p - 1];
                let after = &self.transforms[p];

                if time.as_nanos() - before.stamp.as_nanos()
                    < after.stamp.as_nanos() - time.as_nanos()
                {
                    Some(before)
                } else {
                    Some(after)
                }
            }
        }
    }
}

impl<T: Copy + Debug + 'static> TransformBuffer<T> {
    pub fn new() -> Self {
        Self {
            buffer: Arc::new(RwLock::new(TransformBufferInternal::new())),
        }
    }

    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            buffer: Arc::new(RwLock::new(TransformBufferInternal::with_capacity(
                capacity,
            ))),
        }
    }

    /// Add a transform to the buffer
    pub fn add_transform(&self, transform: StampedTransform<T>) {
        let mut buffer = self.buffer.write().unwrap();
        buffer.add_transform(transform);
    }

    /// Get the latest transform in the buffer
    pub fn get_latest_transform(&self) -> Option<StampedTransform<T>> {
        let buffer = self.buffer.read().unwrap();
        buffer.get_latest_transform().cloned()
    }

    /// Get the time range of transforms in this buffer
    pub fn get_time_range(&self) -> Option<CuTimeRange> {
        let buffer = self.buffer.read().unwrap();
        buffer.get_time_range()
    }

    /// Get transforms within a specific time range
    pub fn get_transforms_in_range(
        &self,
        start_time: CuTime,
        end_time: CuTime,
    ) -> Vec<StampedTransform<T>> {
        let buffer = self.buffer.read().unwrap();
        buffer
            .get_transforms_in_range(start_time, end_time)
            .into_iter()
            .cloned()
            .collect()
    }

    /// Get the transform closest to the specified time
    pub fn get_closest_transform(&self, time: CuTime) -> Option<StampedTransform<T>> {
        let buffer = self.buffer.read().unwrap();
        buffer.get_closest_transform(time).cloned()
    }
}

impl<T: Copy + std::fmt::Debug + 'static> Default for TransformBuffer<T> {
    fn default() -> Self {
        Self::new()
    }
}

/// A concurrent transform buffer store to reduce contention among multiple transform pairs
pub struct TransformStore<T: Copy + Debug + 'static> {
    buffers: DashMap<(String, String), TransformBuffer<T>>,
}

impl<T: Copy + Debug + 'static> TransformStore<T> {
    pub fn new() -> Self {
        Self {
            buffers: DashMap::new(),
        }
    }

    /// Get or create a transform buffer for a specific parent-child frame pair
    pub fn get_or_create_buffer(&self, parent: &str, child: &str) -> TransformBuffer<T> {
        self.buffers
            .entry((parent.to_string(), child.to_string()))
            .or_insert_with(|| TransformBuffer::new())
            .clone()
    }

    /// Add a transform to the appropriate buffer
    pub fn add_transform(&self, transform: StampedTransform<T>) {
        let buffer = self.get_or_create_buffer(&transform.parent_frame, &transform.child_frame);
        buffer.add_transform(transform);
    }

    /// Get a transform buffer if it exists
    pub fn get_buffer(&self, parent: &str, child: &str) -> Option<TransformBuffer<T>> {
        self.buffers
            .get(&(parent.to_string(), child.to_string()))
            .map(|entry| entry.clone())
    }
}

impl<T: Copy + Debug + 'static> Default for TransformStore<T> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cu29::clock::CuDuration;

    #[test]
    fn test_add_transform() {
        let buffer = TransformBuffer::<f32>::new();

        let transform = StampedTransform {
            transform: Transform3D::default(),
            stamp: CuDuration(1000),
            parent_frame: "world".to_string(),
            child_frame: "robot".to_string(),
        };

        buffer.add_transform(transform);

        let latest = buffer.get_latest_transform();
        assert!(latest.is_some());
    }

    #[test]
    fn test_time_ordering() {
        let buffer = TransformBuffer::<f32>::new();

        let transform1 = StampedTransform {
            transform: Transform3D::default(),
            stamp: CuDuration(2000),
            parent_frame: "world".to_string(),
            child_frame: "robot".to_string(),
        };

        let transform2 = StampedTransform {
            transform: Transform3D::default(),
            stamp: CuDuration(1000),
            parent_frame: "world".to_string(),
            child_frame: "robot".to_string(),
        };

        let transform3 = StampedTransform {
            transform: Transform3D::default(),
            stamp: CuDuration(3000),
            parent_frame: "world".to_string(),
            child_frame: "robot".to_string(),
        };

        buffer.add_transform(transform1);
        buffer.add_transform(transform2);
        buffer.add_transform(transform3);

        let range = buffer.get_time_range().unwrap();
        assert_eq!(range.start.as_nanos(), 1000);
        assert_eq!(range.end.as_nanos(), 3000);

        let transforms = buffer.get_transforms_in_range(CuDuration(1500), CuDuration(2500));
        assert_eq!(transforms.len(), 1);
        assert_eq!(transforms[0].stamp.as_nanos(), 2000);
    }

    #[test]
    fn test_capacity_limit() {
        let buffer = TransformBuffer::<f32>::with_capacity(2);

        let transform1 = StampedTransform {
            transform: Transform3D::default(),
            stamp: CuDuration(1000),
            parent_frame: "world".to_string(),
            child_frame: "robot".to_string(),
        };

        let transform2 = StampedTransform {
            transform: Transform3D::default(),
            stamp: CuDuration(2000),
            parent_frame: "world".to_string(),
            child_frame: "robot".to_string(),
        };

        let transform3 = StampedTransform {
            transform: Transform3D::default(),
            stamp: CuDuration(3000),
            parent_frame: "world".to_string(),
            child_frame: "robot".to_string(),
        };

        buffer.add_transform(transform1);
        buffer.add_transform(transform2);
        buffer.add_transform(transform3);

        let range = buffer.get_time_range().unwrap();
        assert_eq!(range.start.as_nanos(), 2000);
        assert_eq!(range.end.as_nanos(), 3000);
    }

    #[test]
    fn test_get_closest_transform() {
        let buffer = TransformBuffer::<f32>::new();

        let transform1 = StampedTransform {
            transform: Transform3D::default(),
            stamp: CuDuration(1000),
            parent_frame: "world".to_string(),
            child_frame: "robot".to_string(),
        };

        let transform2 = StampedTransform {
            transform: Transform3D::default(),
            stamp: CuDuration(3000),
            parent_frame: "world".to_string(),
            child_frame: "robot".to_string(),
        };

        buffer.add_transform(transform1);
        buffer.add_transform(transform2);

        let closest = buffer.get_closest_transform(CuDuration(1000));
        assert!(closest.is_some());
        assert_eq!(closest.unwrap().stamp.as_nanos(), 1000);

        let closest = buffer.get_closest_transform(CuDuration(500));
        assert!(closest.is_some());
        assert_eq!(closest.unwrap().stamp.as_nanos(), 1000);

        let closest = buffer.get_closest_transform(CuDuration(4000));
        assert!(closest.is_some());
        assert_eq!(closest.unwrap().stamp.as_nanos(), 3000);

        let closest = buffer.get_closest_transform(CuDuration(1600));
        assert!(closest.is_some());
        assert_eq!(closest.unwrap().stamp.as_nanos(), 1000);

        let closest = buffer.get_closest_transform(CuDuration(2600));
        assert!(closest.is_some());
        assert_eq!(closest.unwrap().stamp.as_nanos(), 3000);
    }

    #[test]
    fn test_transform_store() {
        let store = TransformStore::<f32>::new();

        let transform = StampedTransform {
            transform: Transform3D::default(),
            stamp: CuDuration(1000),
            parent_frame: "world".to_string(),
            child_frame: "robot".to_string(),
        };

        store.add_transform(transform.clone());

        let buffer = store.get_buffer("world", "robot").unwrap();
        let closest = buffer.get_closest_transform(CuDuration(1000));
        assert!(closest.is_some());

        // Non-existent buffer
        let missing = store.get_buffer("world", "camera");
        assert!(missing.is_none());

        // Get or create should create a new buffer
        let _ = store.get_or_create_buffer("world", "camera");
        assert!(store.get_buffer("world", "camera").is_some());
    }
}
