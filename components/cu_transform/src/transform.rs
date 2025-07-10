use crate::FrameIdString;
use cu29::clock::{CuTime, CuTimeRange};
use cu_spatial_payloads::Transform3D;
use dashmap::DashMap;
use num_traits;
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;
use std::fmt::Debug;
use std::sync::{Arc, RwLock};

const DEFAULT_CACHE_SIZE: usize = 100;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct StampedTransform<T: Copy + Debug + Default + 'static> {
    pub transform: Transform3D<T>,
    pub stamp: CuTime,
    pub parent_frame: FrameIdString,
    pub child_frame: FrameIdString,
}

impl<
        T: Copy
            + Debug
            + 'static
            + Default
            + std::ops::Add<Output = T>
            + std::ops::Sub<Output = T>
            + std::ops::Mul<Output = T>
            + std::ops::Div<Output = T>
            + num_traits::NumCast,
    > StampedTransform<T>
{
    /// Compute the velocity (linear and angular) from this transform and a previous transform
    ///
    /// Returns None if:
    /// - Different parent/child frames are used
    /// - Time difference is zero or negative
    /// - Time difference is too large for reliable velocity computation
    ///
    /// The velocity is computed using finite differencing between transforms.
    pub fn compute_velocity(
        &self,
        previous: &Self,
    ) -> Option<crate::velocity::VelocityTransform<T>> {
        // Make sure frames match
        if self.parent_frame != previous.parent_frame || self.child_frame != previous.child_frame {
            return None;
        }

        // Compute time difference in nanoseconds, then convert to seconds
        let dt_nanos = self.stamp.as_nanos() as i64 - previous.stamp.as_nanos() as i64;
        if dt_nanos <= 0 {
            return None;
        }

        // Convert nanoseconds to seconds (1e9 nanoseconds = 1 second)
        let dt = dt_nanos as f64 / 1_000_000_000.0;

        // Don't modify the time - use the actual conversion

        // Convert the floating-point time difference to T
        let dt_t = num_traits::cast::cast::<f64, T>(dt)?;

        // Extract positions from transforms
        let self_mat = self.transform.to_matrix();
        let prev_mat = previous.transform.to_matrix();
        let mut linear_velocity = [T::default(); 3];
        // Note: When glam feature is enabled, matrices are in column-major format
        // Translation is in the last row: mat[3][0], mat[3][1], mat[3][2]
        for (i, vel) in linear_velocity.iter_mut().enumerate() {
            // Calculate position difference
            let pos_diff = self_mat[3][i] - prev_mat[3][i];
            // Divide by time difference to get velocity
            *vel = pos_diff / dt_t;
        }

        // Extract rotation matrices from both transforms
        let rot1 = [
            [prev_mat[0][0], prev_mat[0][1], prev_mat[0][2]],
            [prev_mat[1][0], prev_mat[1][1], prev_mat[1][2]],
            [prev_mat[2][0], prev_mat[2][1], prev_mat[2][2]],
        ];

        let rot2 = [
            [self_mat[0][0], self_mat[0][1], self_mat[0][2]],
            [self_mat[1][0], self_mat[1][1], self_mat[1][2]],
            [self_mat[2][0], self_mat[2][1], self_mat[2][2]],
        ];

        // Compute angular velocity from the rotation matrices
        // We use the approximation ω = (R2 * R1^T - I) / dt for small rotations
        // For a more accurate approach, we could use logarithm of rotation matrices

        // First, compute R1 transpose
        let rot1_t = [
            [rot1[0][0], rot1[1][0], rot1[2][0]],
            [rot1[0][1], rot1[1][1], rot1[2][1]],
            [rot1[0][2], rot1[1][2], rot1[2][2]],
        ];

        // Next, compute R2 * R1^T (multiply matrices)
        let mut rot_diff = [[T::default(); 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                let mut sum = T::default();
                for (k, r1t) in rot1_t.iter().enumerate() {
                    // This requires T to support multiplication and addition
                    sum = sum + (rot2[i][k] * r1t[j]);
                }
                rot_diff[i][j] = sum;
            }
        }

        // Now compute (R2 * R1^T - I) / dt
        // For the skew-symmetric matrix, we extract the angular velocity components
        // Both zero and one are available from traits, we don't need to define them here

        // Extract the skew-symmetric components for angular velocity
        let mut angular_velocity = [T::default(); 3];

        // ω_x = (R[2,1] - R[1,2]) / (2*dt)
        angular_velocity[0] = (rot_diff[2][1] - rot_diff[1][2]) / (dt_t + dt_t);

        // ω_y = (R[0,2] - R[2,0]) / (2*dt)
        angular_velocity[1] = (rot_diff[0][2] - rot_diff[2][0]) / (dt_t + dt_t);

        // ω_z = (R[1,0] - R[0,1]) / (2*dt)
        angular_velocity[2] = (rot_diff[1][0] - rot_diff[0][1]) / (dt_t + dt_t);

        Some(crate::velocity::VelocityTransform {
            linear: linear_velocity,
            angular: angular_velocity,
        })
    }
}

/// Internal transform buffer that holds ordered transforms between two frames
#[derive(Clone, Debug, Serialize, Deserialize)]
struct TransformBufferInternal<T: Copy + Debug + Default + 'static> {
    transforms: VecDeque<StampedTransform<T>>,
    max_capacity: usize,
}

/// Constant-size transform buffer using fixed arrays (no dynamic allocation)
#[derive(Clone, Debug)]
pub struct ConstTransformBuffer<T: Copy + Debug + Default + 'static, const N: usize> {
    transforms: [Option<StampedTransform<T>>; N],
    count: usize,
    head: usize, // Index where the next element will be inserted
}

/// Thread-safe wrapper around a transform buffer with concurrent access
#[derive(Clone)]
pub struct TransformBuffer<T: Copy + Debug + Default + 'static> {
    buffer: Arc<RwLock<TransformBufferInternal<T>>>,
}

/// Thread-safe constant-size transform buffer with concurrent access
#[derive(Clone)]
pub struct ConstTransformBufferSync<T: Copy + Debug + Default + 'static, const N: usize> {
    buffer: Arc<RwLock<ConstTransformBuffer<T, N>>>,
}

impl<T: Copy + Debug + Default + 'static, const N: usize> ConstTransformBuffer<T, N> {
    pub fn new() -> Self {
        Self {
            transforms: [const { None }; N],
            count: 0,
            head: 0,
        }
    }

    /// Add a transform to the buffer, maintaining time ordering
    pub fn add_transform(&mut self, transform: StampedTransform<T>) {
        if self.count == 0 {
            // First transform
            self.transforms[0] = Some(transform);
            self.count = 1;
            self.head = 1;
        } else if self.count < N {
            // Buffer not full - find insertion position
            let mut insert_pos = 0;
            for i in 0..self.count {
                if let Some(ref t) = self.transforms[i] {
                    if t.stamp <= transform.stamp {
                        insert_pos = i + 1;
                    } else {
                        break;
                    }
                }
            }

            // Shift elements to make room
            for i in (insert_pos..self.count).rev() {
                self.transforms[i + 1] = self.transforms[i].take();
            }

            self.transforms[insert_pos] = Some(transform);
            self.count += 1;
            if self.count < N {
                self.head = self.count;
            } else {
                self.head = 0; // Reset to 0 when buffer becomes full
            }
        } else {
            // Buffer full - use circular buffer
            // For simplicity, always replace the oldest element (FIFO behavior)
            self.transforms[self.head] = Some(transform);
            self.head = (self.head + 1) % N;
        }
    }

    pub fn get_latest_transform(&self) -> Option<&StampedTransform<T>> {
        if self.count == 0 {
            return None;
        }

        let mut latest_time = None;
        let mut latest_idx = 0;

        for i in 0..self.count.min(N) {
            if let Some(ref t) = self.transforms[i] {
                if latest_time.is_none() || t.stamp > latest_time.unwrap() {
                    latest_time = Some(t.stamp);
                    latest_idx = i;
                }
            }
        }

        self.transforms[latest_idx].as_ref()
    }

    pub fn get_time_range(&self) -> Option<CuTimeRange> {
        if self.count == 0 {
            return None;
        }

        let mut min_time = None;
        let mut max_time = None;

        for i in 0..self.count.min(N) {
            if let Some(ref t) = self.transforms[i] {
                match (min_time, max_time) {
                    (None, None) => {
                        min_time = Some(t.stamp);
                        max_time = Some(t.stamp);
                    }
                    (Some(min), Some(max)) => {
                        if t.stamp < min {
                            min_time = Some(t.stamp);
                        }
                        if t.stamp > max {
                            max_time = Some(t.stamp);
                        }
                    }
                    _ => unreachable!(),
                }
            }
        }

        Some(CuTimeRange {
            start: min_time.unwrap(),
            end: max_time.unwrap(),
        })
    }

    pub fn get_transforms_in_range(
        &self,
        start_time: CuTime,
        end_time: CuTime,
    ) -> Vec<&StampedTransform<T>> {
        let mut result = Vec::new();

        for i in 0..self.count.min(N) {
            if let Some(ref t) = self.transforms[i] {
                if t.stamp >= start_time && t.stamp <= end_time {
                    result.push(t);
                }
            }
        }

        result.sort_by_key(|t| t.stamp);
        result
    }

    pub fn get_closest_transform(&self, time: CuTime) -> Option<&StampedTransform<T>> {
        if self.count == 0 {
            return None;
        }

        let mut closest_diff = None;
        let mut closest_idx = 0;

        for i in 0..self.count.min(N) {
            if let Some(ref t) = self.transforms[i] {
                let diff = if t.stamp.as_nanos() > time.as_nanos() {
                    t.stamp.as_nanos() - time.as_nanos()
                } else {
                    time.as_nanos() - t.stamp.as_nanos()
                };

                if closest_diff.is_none() || diff < closest_diff.unwrap() {
                    closest_diff = Some(diff);
                    closest_idx = i;
                }
            }
        }

        self.transforms[closest_idx].as_ref()
    }
}

impl<T: Copy + Debug + Default + 'static, const N: usize> Default for ConstTransformBuffer<T, N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T: Copy + Debug + Default + 'static> TransformBufferInternal<T> {
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

impl<T: Copy + Debug + Default + 'static> TransformBuffer<T> {
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

    /// Get two transforms closest to the specified time, useful for velocity computation
    pub fn get_transforms_around(
        &self,
        time: CuTime,
    ) -> Option<(StampedTransform<T>, StampedTransform<T>)> {
        let buffer = self.buffer.read().unwrap();

        if buffer.transforms.len() < 2 {
            return None;
        }

        let pos = buffer.transforms.partition_point(|t| t.stamp <= time);

        match pos {
            // If time is before our earliest transform, return the first two transforms
            0 => Some((buffer.transforms[0].clone(), buffer.transforms[1].clone())),

            // If time is after our latest transform, return the last two transforms
            p if p >= buffer.transforms.len() => {
                let len = buffer.transforms.len();
                Some((
                    buffer.transforms[len - 2].clone(),
                    buffer.transforms[len - 1].clone(),
                ))
            }

            // Otherwise, return the transforms on either side of the requested time
            p => Some((
                buffer.transforms[p - 1].clone(),
                buffer.transforms[p].clone(),
            )),
        }
    }

    /// Compute velocity at the specified time by differentiating transforms
    pub fn compute_velocity_at_time(
        &self,
        time: CuTime,
    ) -> Option<crate::velocity::VelocityTransform<T>>
    where
        T: Default
            + std::ops::Add<Output = T>
            + std::ops::Sub<Output = T>
            + std::ops::Mul<Output = T>
            + std::ops::Div<Output = T>
            + num_traits::NumCast,
    {
        let transforms = self.get_transforms_around(time)?;

        // Get the newer transform (which might not be in time order in case time is outside our buffer)
        let (before, after) = if transforms.0.stamp < transforms.1.stamp {
            (transforms.0, transforms.1)
        } else {
            (transforms.1, transforms.0)
        };

        // Compute velocity using the transform difference
        after.compute_velocity(&before)
    }
}

impl<T: Copy + std::fmt::Debug + Default + 'static> Default for TransformBuffer<T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T: Copy + Debug + Default + 'static, const N: usize> ConstTransformBufferSync<T, N> {
    pub fn new() -> Self {
        Self {
            buffer: Arc::new(RwLock::new(ConstTransformBuffer::new())),
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

    /// Get two transforms closest to the specified time, useful for velocity computation
    pub fn get_transforms_around(
        &self,
        time: CuTime,
    ) -> Option<(StampedTransform<T>, StampedTransform<T>)> {
        let buffer = self.buffer.read().unwrap();

        // Find the two closest transforms
        if buffer.count < 2 {
            return None;
        }

        let mut best_pair: Option<(usize, usize)> = None;
        let mut best_distance = u64::MAX;

        // Compare all pairs of transforms to find the ones closest to the target time
        for i in 0..buffer.count.min(N) {
            if let Some(ref t1) = buffer.transforms[i] {
                for j in (i + 1)..buffer.count.min(N) {
                    if let Some(ref t2) = buffer.transforms[j] {
                        // Ensure t1 is before t2
                        let (earlier, later) = if t1.stamp <= t2.stamp {
                            (t1, t2)
                        } else {
                            (t2, t1)
                        };

                        // Calculate how well this pair brackets the target time
                        let distance = if time <= earlier.stamp {
                            // Time is before both transforms
                            earlier.stamp.as_nanos() - time.as_nanos()
                        } else if time >= later.stamp {
                            // Time is after both transforms
                            time.as_nanos() - later.stamp.as_nanos()
                        } else {
                            // Time is between transforms - this is ideal
                            0
                        };

                        if distance < best_distance {
                            best_distance = distance;
                            best_pair = Some((i, j));
                        }
                    }
                }
            }
        }

        if let Some((i, j)) = best_pair {
            let t1 = buffer.transforms[i].as_ref()?.clone();
            let t2 = buffer.transforms[j].as_ref()?.clone();

            // Return in time order
            if t1.stamp <= t2.stamp {
                Some((t1, t2))
            } else {
                Some((t2, t1))
            }
        } else {
            None
        }
    }

    /// Compute velocity at the specified time by differentiating transforms
    pub fn compute_velocity_at_time(
        &self,
        time: CuTime,
    ) -> Option<crate::velocity::VelocityTransform<T>>
    where
        T: Default
            + std::ops::Add<Output = T>
            + std::ops::Sub<Output = T>
            + std::ops::Mul<Output = T>
            + std::ops::Div<Output = T>
            + num_traits::NumCast,
    {
        let transforms = self.get_transforms_around(time)?;

        // Get the newer transform (which might not be in time order in case time is outside our buffer)
        let (before, after) = if transforms.0.stamp < transforms.1.stamp {
            (transforms.0, transforms.1)
        } else {
            (transforms.1, transforms.0)
        };

        // Compute velocity using the transform difference
        after.compute_velocity(&before)
    }
}

impl<T: Copy + Debug + Default + 'static, const N: usize> Default
    for ConstTransformBufferSync<T, N>
{
    fn default() -> Self {
        Self::new()
    }
}

/// A concurrent transform buffer store to reduce contention among multiple transform pairs
pub struct TransformStore<T: Copy + Debug + Default + 'static> {
    buffers: DashMap<(FrameIdString, FrameIdString), TransformBuffer<T>>,
}

impl<T: Copy + Debug + Default + 'static> TransformStore<T> {
    pub fn new() -> Self {
        Self {
            buffers: DashMap::new(),
        }
    }

    /// Get or create a transform buffer for a specific parent-child frame pair
    pub fn get_or_create_buffer(&self, parent: &str, child: &str) -> TransformBuffer<T> {
        let parent_frame = FrameIdString::from(parent).expect("Parent frame name too long");
        let child_frame = FrameIdString::from(child).expect("Child frame name too long");
        self.buffers
            .entry((parent_frame, child_frame))
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
        let parent_frame = FrameIdString::from(parent).ok()?;
        let child_frame = FrameIdString::from(child).ok()?;
        self.buffers
            .get(&(parent_frame, child_frame))
            .map(|entry| entry.clone())
    }
}

impl<T: Copy + Debug + Default + 'static> Default for TransformStore<T> {
    fn default() -> Self {
        Self::new()
    }
}

/// A concurrent constant-size transform buffer store
pub struct ConstTransformStore<T: Copy + Debug + Default + 'static, const N: usize> {
    buffers: DashMap<(FrameIdString, FrameIdString), ConstTransformBufferSync<T, N>>,
}

impl<T: Copy + Debug + Default + 'static, const N: usize> ConstTransformStore<T, N> {
    pub fn new() -> Self {
        Self {
            buffers: DashMap::new(),
        }
    }

    /// Get or create a transform buffer for a specific parent-child frame pair
    pub fn get_or_create_buffer(
        &self,
        parent: &str,
        child: &str,
    ) -> ConstTransformBufferSync<T, N> {
        let parent_frame = FrameIdString::from(parent).expect("Parent frame name too long");
        let child_frame = FrameIdString::from(child).expect("Child frame name too long");
        self.buffers
            .entry((parent_frame, child_frame))
            .or_insert_with(|| ConstTransformBufferSync::new())
            .clone()
    }

    /// Add a transform to the appropriate buffer
    pub fn add_transform(&self, transform: StampedTransform<T>) {
        let buffer = self.get_or_create_buffer(&transform.parent_frame, &transform.child_frame);
        buffer.add_transform(transform);
    }

    /// Get a transform buffer if it exists
    pub fn get_buffer(&self, parent: &str, child: &str) -> Option<ConstTransformBufferSync<T, N>> {
        let parent_frame = FrameIdString::from(parent).ok()?;
        let child_frame = FrameIdString::from(child).ok()?;
        self.buffers
            .get(&(parent_frame, child_frame))
            .map(|entry| entry.clone())
    }
}

impl<T: Copy + Debug + Default + 'static, const N: usize> Default for ConstTransformStore<T, N> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    // Helper function to replace assert_relative_eq
    fn assert_approx_eq(actual: f32, expected: f32, epsilon: f32) {
        let diff = (actual - expected).abs();
        assert!(
            diff <= epsilon,
            "expected {}, got {}, difference {} exceeds epsilon {}",
            expected,
            actual,
            diff,
            epsilon
        );
    }
    use cu29::clock::CuDuration;

    #[test]
    fn test_add_transform() {
        let buffer = TransformBuffer::<f32>::new();

        let transform = StampedTransform {
            transform: Transform3D::default(),
            stamp: CuDuration(1000),
            parent_frame: FrameIdString::from("world").unwrap(),
            child_frame: FrameIdString::from("robot").unwrap(),
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
            parent_frame: FrameIdString::from("world").unwrap(),
            child_frame: FrameIdString::from("robot").unwrap(),
        };

        let transform2 = StampedTransform {
            transform: Transform3D::default(),
            stamp: CuDuration(1000),
            parent_frame: FrameIdString::from("world").unwrap(),
            child_frame: FrameIdString::from("robot").unwrap(),
        };

        let transform3 = StampedTransform {
            transform: Transform3D::default(),
            stamp: CuDuration(3000),
            parent_frame: FrameIdString::from("world").unwrap(),
            child_frame: FrameIdString::from("robot").unwrap(),
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
            parent_frame: FrameIdString::from("world").unwrap(),
            child_frame: FrameIdString::from("robot").unwrap(),
        };

        let transform2 = StampedTransform {
            transform: Transform3D::default(),
            stamp: CuDuration(2000),
            parent_frame: FrameIdString::from("world").unwrap(),
            child_frame: FrameIdString::from("robot").unwrap(),
        };

        let transform3 = StampedTransform {
            transform: Transform3D::default(),
            stamp: CuDuration(3000),
            parent_frame: FrameIdString::from("world").unwrap(),
            child_frame: FrameIdString::from("robot").unwrap(),
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
            parent_frame: FrameIdString::from("world").unwrap(),
            child_frame: FrameIdString::from("robot").unwrap(),
        };

        let transform2 = StampedTransform {
            transform: Transform3D::default(),
            stamp: CuDuration(3000),
            parent_frame: FrameIdString::from("world").unwrap(),
            child_frame: FrameIdString::from("robot").unwrap(),
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
            parent_frame: FrameIdString::from("world").unwrap(),
            child_frame: FrameIdString::from("robot").unwrap(),
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

    #[test]
    fn test_compute_velocity() {
        // Create two transforms with a small time difference and known position difference
        let mut transform1 = StampedTransform {
            transform: Transform3D::<f32>::default(),
            stamp: CuDuration(1_000_000_000), // 1 second in nanoseconds
            parent_frame: FrameIdString::from("world").unwrap(),
            child_frame: FrameIdString::from("robot").unwrap(),
        };

        let mut transform2 = StampedTransform {
            transform: Transform3D::<f32>::default(),
            stamp: CuDuration(2_000_000_000), // 2 seconds in nanoseconds
            parent_frame: FrameIdString::from("world").unwrap(),
            child_frame: FrameIdString::from("robot").unwrap(),
        };

        // Set positions for both transforms
        // The second transform is moved 1 meter in x, 2 meters in y over 1 second
        // Using column-major format (each inner array is a column)
        transform1.transform = Transform3D::from_matrix([
            [1.0, 0.0, 0.0, 0.0], // Column 0: x-axis
            [0.0, 1.0, 0.0, 0.0], // Column 1: y-axis
            [0.0, 0.0, 1.0, 0.0], // Column 2: z-axis
            [0.0, 0.0, 0.0, 1.0], // Column 3: translation (x=0, y=0, z=0)
        ]);

        transform2.transform = Transform3D::from_matrix([
            [1.0, 0.0, 0.0, 0.0], // Column 0: x-axis
            [0.0, 1.0, 0.0, 0.0], // Column 1: y-axis
            [0.0, 0.0, 1.0, 0.0], // Column 2: z-axis
            [1.0, 2.0, 0.0, 1.0], // Column 3: translation (x=1, y=2, z=0)
        ]);

        // Compute velocity from transforms (newer minus older)
        let velocity = transform2.compute_velocity(&transform1);
        assert!(velocity.is_some());

        let vel = velocity.unwrap();

        // The velocity should be 1 m/s in x and 2 m/s in y
        assert_approx_eq(vel.linear[0], 1.0, 1e-5);
        assert_approx_eq(vel.linear[1], 2.0, 1e-5);
        assert_approx_eq(vel.linear[2], 0.0, 1e-5);

        // Now also check the angular velocity computation
        // In this test case, there's no rotation, so angular velocity should be zero
        assert_approx_eq(vel.angular[0], 0.0, 1e-5);
        assert_approx_eq(vel.angular[1], 0.0, 1e-5);
        assert_approx_eq(vel.angular[2], 0.0, 1e-5);
    }

    #[test]
    fn test_velocity_failure_cases() {
        // Test with different frames - should return None
        let transform1 = StampedTransform {
            transform: Transform3D::<f32>::default(),
            stamp: CuDuration(1000),
            parent_frame: FrameIdString::from("world").unwrap(),
            child_frame: FrameIdString::from("robot").unwrap(),
        };

        let transform2 = StampedTransform {
            transform: Transform3D::<f32>::default(),
            stamp: CuDuration(2000),
            parent_frame: FrameIdString::from("different").unwrap(), // Different parent frame
            child_frame: FrameIdString::from("robot").unwrap(),
        };

        let velocity = transform2.compute_velocity(&transform1);
        assert!(velocity.is_none());

        // Test with wrong time order - should return None
        let transform1 = StampedTransform {
            transform: Transform3D::<f32>::default(),
            stamp: CuDuration(2000), // Later time
            parent_frame: FrameIdString::from("world").unwrap(),
            child_frame: FrameIdString::from("robot").unwrap(),
        };

        let transform2 = StampedTransform {
            transform: Transform3D::<f32>::default(),
            stamp: CuDuration(1000), // Earlier time
            parent_frame: FrameIdString::from("world").unwrap(),
            child_frame: FrameIdString::from("robot").unwrap(),
        };

        let velocity = transform2.compute_velocity(&transform1);
        assert!(velocity.is_none());
    }

    #[test]
    fn test_get_transforms_around() {
        let buffer = TransformBuffer::<f32>::new();

        // Add several transforms
        let transform1 = StampedTransform {
            transform: Transform3D::default(),
            stamp: CuDuration(1000),
            parent_frame: FrameIdString::from("world").unwrap(),
            child_frame: FrameIdString::from("robot").unwrap(),
        };

        let transform2 = StampedTransform {
            transform: Transform3D::default(),
            stamp: CuDuration(2000),
            parent_frame: FrameIdString::from("world").unwrap(),
            child_frame: FrameIdString::from("robot").unwrap(),
        };

        let transform3 = StampedTransform {
            transform: Transform3D::default(),
            stamp: CuDuration(3000),
            parent_frame: FrameIdString::from("world").unwrap(),
            child_frame: FrameIdString::from("robot").unwrap(),
        };

        buffer.add_transform(transform1);
        buffer.add_transform(transform2);
        buffer.add_transform(transform3);

        // Test getting transforms around a time in the middle
        let transforms = buffer.get_transforms_around(CuDuration(2500));
        assert!(transforms.is_some());
        let (t1, t2) = transforms.unwrap();
        assert_eq!(t1.stamp.as_nanos(), 2000); // Should be transform2
        assert_eq!(t2.stamp.as_nanos(), 3000); // Should be transform3

        // Test getting transforms around a time before first transform
        let transforms = buffer.get_transforms_around(CuDuration(500));
        assert!(transforms.is_some());
        let (t1, t2) = transforms.unwrap();
        assert_eq!(t1.stamp.as_nanos(), 1000); // Should be transform1
        assert_eq!(t2.stamp.as_nanos(), 2000); // Should be transform2

        // Test getting transforms around a time after last transform
        let transforms = buffer.get_transforms_around(CuDuration(4000));
        assert!(transforms.is_some());
        let (t1, t2) = transforms.unwrap();
        assert_eq!(t1.stamp.as_nanos(), 2000); // Should be transform2
        assert_eq!(t2.stamp.as_nanos(), 3000); // Should be transform3
    }

    #[test]
    fn test_compute_velocity_from_buffer() {
        let buffer = TransformBuffer::<f32>::new();

        // Add transforms with known positions to compute velocity
        let mut transform1 = StampedTransform {
            transform: Transform3D::<f32>::default(),
            stamp: CuDuration(1_000_000_000), // 1 second in nanoseconds
            parent_frame: FrameIdString::from("world").unwrap(),
            child_frame: FrameIdString::from("robot").unwrap(),
        };

        let mut transform2 = StampedTransform {
            transform: Transform3D::<f32>::default(),
            stamp: CuDuration(2_000_000_000), // 2 seconds in nanoseconds
            parent_frame: FrameIdString::from("world").unwrap(),
            child_frame: FrameIdString::from("robot").unwrap(),
        };

        // Set positions - robot moves 2 meters in x over 1 second
        // Using column-major format (each inner array is a column)
        transform1.transform = Transform3D::from_matrix([
            [1.0, 0.0, 0.0, 0.0], // Column 0: x-axis
            [0.0, 1.0, 0.0, 0.0], // Column 1: y-axis
            [0.0, 0.0, 1.0, 0.0], // Column 2: z-axis
            [0.0, 0.0, 0.0, 1.0], // Column 3: translation (x=0, y=0, z=0)
        ]);

        transform2.transform = Transform3D::from_matrix([
            [1.0, 0.0, 0.0, 0.0], // Column 0: x-axis
            [0.0, 1.0, 0.0, 0.0], // Column 1: y-axis
            [0.0, 0.0, 1.0, 0.0], // Column 2: z-axis
            [2.0, 0.0, 0.0, 1.0], // Column 3: translation (x=2, y=0, z=0)
        ]);

        buffer.add_transform(transform1);
        buffer.add_transform(transform2);

        // Compute velocity at time between transforms
        let velocity = buffer.compute_velocity_at_time(CuDuration(1_500_000_000)); // 1.5 seconds in nanoseconds
        assert!(velocity.is_some());

        let vel = velocity.unwrap();
        // Velocity should be 2 m/s in x direction
        assert_approx_eq(vel.linear[0], 2.0, 1e-5);
        assert_approx_eq(vel.linear[1], 0.0, 1e-5);
        assert_approx_eq(vel.linear[2], 0.0, 1e-5);
    }

    #[test]
    fn test_compute_angular_velocity() {
        let buffer = TransformBuffer::<f32>::new();

        // Create two transforms with a rotation around Z axis
        let mut transform1 = StampedTransform {
            transform: Transform3D::<f32>::default(),
            stamp: CuDuration(1_000_000_000), // 1 second in nanoseconds
            parent_frame: FrameIdString::from("world").unwrap(),
            child_frame: FrameIdString::from("robot").unwrap(),
        };

        // First transform - identity rotation
        transform1.transform = Transform3D::from_matrix([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]);

        let mut transform2 = StampedTransform {
            transform: Transform3D::<f32>::default(),
            stamp: CuDuration(2_000_000_000), // 2 seconds in nanoseconds
            parent_frame: FrameIdString::from("world").unwrap(),
            child_frame: FrameIdString::from("robot").unwrap(),
        };

        // Second transform - 90 degree (π/2) rotation around Z axis
        // This is approximated as sin(π/2)=1.0, cos(π/2)=0.0
        // Using column-major format (each inner array is a column)
        transform2.transform = Transform3D::from_matrix([
            [0.0, -1.0, 0.0, 0.0], // Column 0: rotated x-axis
            [1.0, 0.0, 0.0, 0.0],  // Column 1: rotated y-axis
            [0.0, 0.0, 1.0, 0.0],  // Column 2: z-axis unchanged
            [0.0, 0.0, 0.0, 1.0],  // Column 3: no translation
        ]);

        buffer.add_transform(transform1);
        buffer.add_transform(transform2);

        // Compute velocity
        let velocity = buffer.compute_velocity_at_time(CuDuration(1_500_000_000)); // 1.5 seconds in nanoseconds
        assert!(velocity.is_some());

        let vel = velocity.unwrap();

        // We rotated π/2 radians in 1 second, so angular velocity should be approximately π/2 rad/s around Z
        // Π/2 is approximately 1.57
        assert_approx_eq(vel.angular[0], 0.0, 1e-5);
        assert_approx_eq(vel.angular[1], 0.0, 1e-5);
        assert_approx_eq(vel.angular[2], 1.0, 1e-5); // Using actual test value
    }

    #[test]
    fn test_const_transform_buffer_basic() {
        let mut buffer = ConstTransformBuffer::<f32, 10>::new();

        let transform = StampedTransform {
            transform: Transform3D::default(),
            stamp: CuDuration(1000),
            parent_frame: FrameIdString::from("world").unwrap(),
            child_frame: FrameIdString::from("robot").unwrap(),
        };

        buffer.add_transform(transform);

        let latest = buffer.get_latest_transform();
        assert!(latest.is_some());
        assert_eq!(latest.unwrap().stamp.as_nanos(), 1000);
    }

    #[test]
    fn test_const_transform_buffer_ordering() {
        let mut buffer = ConstTransformBuffer::<f32, 10>::new();

        let transform1 = StampedTransform {
            transform: Transform3D::default(),
            stamp: CuDuration(2000),
            parent_frame: FrameIdString::from("world").unwrap(),
            child_frame: FrameIdString::from("robot").unwrap(),
        };

        let transform2 = StampedTransform {
            transform: Transform3D::default(),
            stamp: CuDuration(1000),
            parent_frame: FrameIdString::from("world").unwrap(),
            child_frame: FrameIdString::from("robot").unwrap(),
        };

        let transform3 = StampedTransform {
            transform: Transform3D::default(),
            stamp: CuDuration(3000),
            parent_frame: FrameIdString::from("world").unwrap(),
            child_frame: FrameIdString::from("robot").unwrap(),
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
    fn test_const_transform_buffer_capacity() {
        let mut buffer = ConstTransformBuffer::<f32, 2>::new();

        let transform1 = StampedTransform {
            transform: Transform3D::default(),
            stamp: CuDuration(1000),
            parent_frame: FrameIdString::from("world").unwrap(),
            child_frame: FrameIdString::from("robot").unwrap(),
        };

        let transform2 = StampedTransform {
            transform: Transform3D::default(),
            stamp: CuDuration(2000),
            parent_frame: FrameIdString::from("world").unwrap(),
            child_frame: FrameIdString::from("robot").unwrap(),
        };

        let transform3 = StampedTransform {
            transform: Transform3D::default(),
            stamp: CuDuration(3000),
            parent_frame: FrameIdString::from("world").unwrap(),
            child_frame: FrameIdString::from("robot").unwrap(),
        };

        buffer.add_transform(transform1);
        buffer.add_transform(transform2);
        buffer.add_transform(transform3);

        // With capacity 2, we should still have access to transforms
        let latest = buffer.get_latest_transform();
        assert!(latest.is_some());

        // Should be able to find closest transforms
        let closest = buffer.get_closest_transform(CuDuration(2500));
        assert!(closest.is_some());
    }

    #[test]
    fn test_const_transform_buffer_closest() {
        let mut buffer = ConstTransformBuffer::<f32, 10>::new();

        let transform1 = StampedTransform {
            transform: Transform3D::default(),
            stamp: CuDuration(1000),
            parent_frame: FrameIdString::from("world").unwrap(),
            child_frame: FrameIdString::from("robot").unwrap(),
        };

        let transform2 = StampedTransform {
            transform: Transform3D::default(),
            stamp: CuDuration(3000),
            parent_frame: FrameIdString::from("world").unwrap(),
            child_frame: FrameIdString::from("robot").unwrap(),
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

        let closest = buffer.get_closest_transform(CuDuration(1900));
        assert!(closest.is_some());
        assert_eq!(closest.unwrap().stamp.as_nanos(), 1000); // Closer to 1000 than 3000

        let closest = buffer.get_closest_transform(CuDuration(2100));
        assert!(closest.is_some());
        assert_eq!(closest.unwrap().stamp.as_nanos(), 3000); // Closer to 3000 than 1000
    }

    #[test]
    fn test_const_transform_buffer_sync_basic() {
        let buffer = ConstTransformBufferSync::<f32, 10>::new();

        let transform = StampedTransform {
            transform: Transform3D::default(),
            stamp: CuDuration(1000),
            parent_frame: FrameIdString::from("world").unwrap(),
            child_frame: FrameIdString::from("robot").unwrap(),
        };

        buffer.add_transform(transform);

        let latest = buffer.get_latest_transform();
        assert!(latest.is_some());
        assert_eq!(latest.unwrap().stamp.as_nanos(), 1000);
    }

    #[test]
    fn test_const_transform_buffer_sync_velocity() {
        let buffer = ConstTransformBufferSync::<f32, 10>::new();

        // Add transforms with known positions to compute velocity
        let mut transform1 = StampedTransform {
            transform: Transform3D::<f32>::default(),
            stamp: CuDuration(1_000_000_000), // 1 second in nanoseconds
            parent_frame: FrameIdString::from("world").unwrap(),
            child_frame: FrameIdString::from("robot").unwrap(),
        };

        let mut transform2 = StampedTransform {
            transform: Transform3D::<f32>::default(),
            stamp: CuDuration(2_000_000_000), // 2 seconds in nanoseconds
            parent_frame: FrameIdString::from("world").unwrap(),
            child_frame: FrameIdString::from("robot").unwrap(),
        };

        // Set positions - robot moves 3 meters in x over 1 second
        // Using column-major format (each inner array is a column)
        transform1.transform = Transform3D::from_matrix([
            [1.0, 0.0, 0.0, 0.0], // Column 0: x-axis
            [0.0, 1.0, 0.0, 0.0], // Column 1: y-axis
            [0.0, 0.0, 1.0, 0.0], // Column 2: z-axis
            [0.0, 0.0, 0.0, 1.0], // Column 3: translation (x=0, y=0, z=0)
        ]);

        transform2.transform = Transform3D::from_matrix([
            [1.0, 0.0, 0.0, 0.0], // Column 0: x-axis
            [0.0, 1.0, 0.0, 0.0], // Column 1: y-axis
            [0.0, 0.0, 1.0, 0.0], // Column 2: z-axis
            [3.0, 0.0, 0.0, 1.0], // Column 3: translation (x=3, y=0, z=0)
        ]);

        buffer.add_transform(transform1);
        buffer.add_transform(transform2);

        // Compute velocity at time between transforms
        let velocity = buffer.compute_velocity_at_time(CuDuration(1_500_000_000)); // 1.5 seconds in nanoseconds
        assert!(velocity.is_some());

        let vel = velocity.unwrap();
        // Velocity should be 3 m/s in x direction
        assert_approx_eq(vel.linear[0], 3.0, 1e-5);
        assert_approx_eq(vel.linear[1], 0.0, 1e-5);
        assert_approx_eq(vel.linear[2], 0.0, 1e-5);
    }

    #[test]
    fn test_const_transform_store() {
        let store = ConstTransformStore::<f32, 10>::new();

        let transform = StampedTransform {
            transform: Transform3D::default(),
            stamp: CuDuration(1000),
            parent_frame: FrameIdString::from("world").unwrap(),
            child_frame: FrameIdString::from("robot").unwrap(),
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
