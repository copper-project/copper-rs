use cu29::clock::{CuTime, CuTimeRange};
use cu_spatial_payloads::Transform3D;
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;
use std::fmt::Debug;

const DEFAULT_CACHE_SIZE: usize = 100;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct StampedTransform<T: Copy + Debug + 'static> {
    pub transform: Transform3D<T>,
    pub stamp: CuTime,
    pub parent_frame: String,
    pub child_frame: String,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct TransformBuffer<T: Copy + Debug + 'static> {
    transforms: VecDeque<StampedTransform<T>>,
    max_capacity: usize,
}

impl<T: Copy + Debug + 'static> TransformBuffer<T> {
    pub fn new() -> Self {
        Self::with_capacity(DEFAULT_CACHE_SIZE)
    }

    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            transforms: VecDeque::with_capacity(capacity),
            max_capacity: capacity,
        }
    }

    pub fn add_transform(&mut self, transform: StampedTransform<T>) {
        let pos = self
            .transforms
            .partition_point(|t| t.stamp <= transform.stamp);

        self.transforms.insert(pos, transform);

        while self.transforms.len() > self.max_capacity {
            self.transforms.pop_front();
        }
    }

    pub fn get_latest_transform(&self) -> Option<&StampedTransform<T>> {
        self.transforms.back()
    }

    pub fn get_time_range(&self) -> Option<CuTimeRange> {
        if self.transforms.is_empty() {
            return None;
        }

        Some(CuTimeRange {
            start: self.transforms.front().unwrap().stamp,
            end: self.transforms.back().unwrap().stamp,
        })
    }

    pub fn get_transforms_in_range(
        &self,
        start_time: CuTime,
        end_time: CuTime,
    ) -> Vec<&StampedTransform<T>> {
        self.transforms
            .iter()
            .filter(|t| t.stamp >= start_time && t.stamp <= end_time)
            .collect()
    }

    pub fn get_closest_transform(&self, time: CuTime) -> Option<&StampedTransform<T>> {
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

impl<T: Copy + std::fmt::Debug + 'static> Default for TransformBuffer<T> {
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
        let mut buffer = TransformBuffer::<f32>::new();

        let transform = StampedTransform {
            transform: Transform3D::default(),
            stamp: CuDuration(1000),
            parent_frame: "world".to_string(),
            child_frame: "robot".to_string(),
        };

        buffer.add_transform(transform);

        assert_eq!(buffer.transforms.len(), 1);
    }

    #[test]
    fn test_time_ordering() {
        let mut buffer = TransformBuffer::<f32>::new();

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

        assert_eq!(buffer.transforms.len(), 3);

        assert_eq!(buffer.transforms[0].stamp.as_nanos(), 1000);
        assert_eq!(buffer.transforms[1].stamp.as_nanos(), 2000);
        assert_eq!(buffer.transforms[2].stamp.as_nanos(), 3000);
    }

    #[test]
    fn test_capacity_limit() {
        let mut buffer = TransformBuffer::<f32>::with_capacity(2);

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

        assert_eq!(buffer.transforms.len(), 2);
        assert_eq!(buffer.transforms[0].stamp.as_nanos(), 2000);
        assert_eq!(buffer.transforms[1].stamp.as_nanos(), 3000);
    }

    #[test]
    fn test_get_closest_transform() {
        let mut buffer = TransformBuffer::<f32>::new();

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
}
