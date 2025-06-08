//! Transform message system using CuMsg and compile-time frame types
//! This replaces the StampedTransform approach with a more Copper-native design

use crate::frames::{FrameId, FramePair};
use crate::velocity::VelocityTransform;
use crate::FrameIdString;
use bincode::{Decode, Encode};
use cu29::clock::{CuTime, CuTimeRange, Tov};
use cu29::prelude::{CuMsg, CuMsgPayload};
use cu_spatial_payloads::Transform3D;
use num_traits;
use serde::{Deserialize, Serialize};
use std::fmt::Debug;

/// Transform message payload for use with CuMsg
/// This contains just the transform data without timestamps,
/// as timestamps are handled by CuMsg metadata
/// 
/// # Example
/// ```
/// use cu_transform::{TransformMsg, Transform3D};
/// use cu29::prelude::*;
/// use cu29::clock::{CuDuration, Tov};
/// 
/// // Create a transform message
/// let transform = Transform3D::<f32>::default();
/// let msg = TransformMsg::new(
///     transform,
///     "world",
///     "robot"
/// );
/// 
/// // Wrap in CuMsg for timestamp handling
/// let mut cu_msg = CuMsg::new(Some(msg));
/// cu_msg.metadata.tov = Tov::Time(CuDuration(1000));
/// ```
#[derive(Clone, Debug, Serialize, Deserialize, Default)]
pub struct TransformMsg<T: Copy + Debug + Default + 'static> {
    /// The actual transform
    pub transform: Transform3D<T>,
    /// Parent frame identifier
    pub parent_frame: FrameIdString,
    /// Child frame identifier
    pub child_frame: FrameIdString,
}

impl<T: Copy + Debug + Default + 'static> TransformMsg<T> {
    /// Create a new transform message
    pub fn new(transform: Transform3D<T>, parent_frame: impl AsRef<str>, child_frame: impl AsRef<str>) -> Self {
        Self {
            transform,
            parent_frame: FrameIdString::from(parent_frame.as_ref()).expect("Parent frame name too long (max 64 chars)"),
            child_frame: FrameIdString::from(child_frame.as_ref()).expect("Child frame name too long (max 64 chars)"),
        }
    }

    /// Create from a StampedTransform (for migration)
    pub fn from_stamped(stamped: &crate::transform::StampedTransform<T>) -> Self {
        Self {
            transform: stamped.transform,
            parent_frame: FrameIdString::from(stamped.parent_frame.as_str()).expect("Parent frame name too long"),
            child_frame: FrameIdString::from(stamped.child_frame.as_str()).expect("Child frame name too long"),
        }
    }
}

// Manual Encode/Decode implementations to work with Transform3D's specific implementations
impl<T: Copy + Debug + Default + 'static> Encode for TransformMsg<T>
where
    T: Encode,
{
    fn encode<E: bincode::enc::Encoder>(
        &self,
        encoder: &mut E,
    ) -> Result<(), bincode::error::EncodeError> {
        self.transform.encode(encoder)?;
        self.parent_frame.encode(encoder)?;
        self.child_frame.encode(encoder)?;
        Ok(())
    }
}

impl<T: Copy + Debug + Default + 'static> Decode<()> for TransformMsg<T>
where
    T: Decode<()>,
{
    fn decode<D: bincode::de::Decoder<Context = ()>>(
        decoder: &mut D,
    ) -> Result<Self, bincode::error::DecodeError> {
        let transform = Transform3D::decode(decoder)?;
        let parent_frame_str = String::decode(decoder)?;
        let child_frame_str = String::decode(decoder)?;
        let parent_frame = FrameIdString::from(&parent_frame_str)
            .map_err(|_| bincode::error::DecodeError::OtherString("Parent frame name too long".to_string()))?;
        let child_frame = FrameIdString::from(&child_frame_str)
            .map_err(|_| bincode::error::DecodeError::OtherString("Child frame name too long".to_string()))?;
        Ok(Self {
            transform,
            parent_frame,
            child_frame,
        })
    }
}

/// A typed transform message that carries frame relationship information at compile time
#[derive(Debug, Clone)]
pub struct TypedTransformMsg<T, Parent, Child>
where
    T: CuMsgPayload + Copy + Debug + 'static,
    Parent: FrameId,
    Child: FrameId,
{
    /// The actual transform message
    pub msg: CuMsg<Transform3D<T>>,
    /// Frame relationship (zero-sized at runtime)
    pub frames: FramePair<Parent, Child>,
}

impl<T, Parent, Child> TypedTransformMsg<T, Parent, Child>
where
    T: CuMsgPayload + Copy + Debug + 'static,
    Parent: FrameId,
    Child: FrameId,
{
    /// Create a new typed transform message
    pub fn new(transform: Transform3D<T>, time: CuTime) -> Self {
        let mut msg = CuMsg::new(Some(transform));
        msg.metadata.tov = Tov::Time(time);

        Self {
            msg,
            frames: FramePair::new(),
        }
    }

    /// Get the transform data
    pub fn transform(&self) -> Option<&Transform3D<T>> {
        self.msg.payload()
    }

    /// Get the timestamp from the message
    pub fn timestamp(&self) -> Option<CuTime> {
        match self.msg.metadata.tov {
            Tov::Time(time) => Some(time),
            _ => None,
        }
    }

    /// Get the parent frame ID
    pub fn parent_id(&self) -> u32 {
        Parent::ID
    }

    /// Get the child frame ID  
    pub fn child_id(&self) -> u32 {
        Child::ID
    }

    /// Get the parent frame name
    pub fn parent_name(&self) -> &'static str {
        Parent::NAME
    }

    /// Get the child frame name
    pub fn child_name(&self) -> &'static str {
        Child::NAME
    }
}

/// Fixed-size transform buffer using compile-time frame types
/// This replaces the dynamic Vec-based approach with a fixed-size array
#[derive(Debug)]
pub struct TypedTransformBuffer<T, Parent, Child, const N: usize>
where
    T: CuMsgPayload + Copy + Debug + 'static,
    Parent: FrameId,
    Child: FrameId,
{
    /// Fixed-size array of transform messages
    transforms: [Option<TypedTransformMsg<T, Parent, Child>>; N],
    /// Current number of transforms stored
    count: usize,
}

impl<T, Parent, Child, const N: usize> TypedTransformBuffer<T, Parent, Child, N>
where
    T: CuMsgPayload + Copy + Debug + 'static,
    Parent: FrameId,
    Child: FrameId,
{
    /// Create a new typed transform buffer
    pub fn new() -> Self {
        Self {
            transforms: std::array::from_fn(|_| None),
            count: 0,
        }
    }

    /// Add a transform to the buffer
    pub fn add_transform(&mut self, transform_msg: TypedTransformMsg<T, Parent, Child>) {
        if self.count < N {
            // Still have space, just add to the end
            self.transforms[self.count] = Some(transform_msg);
            self.count += 1;
        } else {
            // Buffer is full, shift everything and add to the end
            for i in 0..N - 1 {
                self.transforms[i] = self.transforms[i + 1].take();
            }
            self.transforms[N - 1] = Some(transform_msg);
        }

        // Sort to maintain time ordering
        self.sort_by_time();
    }

    /// Sort transforms by timestamp
    fn sort_by_time(&mut self) {
        // Create a temporary vector of (timestamp, index) pairs
        let mut time_indices: Vec<(CuTime, usize)> = Vec::new();

        for i in 0..self.count {
            if let Some(ref transform) = self.transforms[i] {
                if let Some(time) = transform.timestamp() {
                    time_indices.push((time, i));
                }
            }
        }

        // Sort by timestamp
        time_indices.sort_by_key(|(time, _)| *time);

        // Create a new ordered array
        let mut new_transforms: [Option<TypedTransformMsg<T, Parent, Child>>; N] =
            std::array::from_fn(|_| None);

        for (idx, (_, old_idx)) in time_indices.iter().enumerate() {
            new_transforms[idx] = self.transforms[*old_idx].take();
        }

        self.transforms = new_transforms;
    }

    /// Get the latest transform
    pub fn get_latest_transform(&self) -> Option<&TypedTransformMsg<T, Parent, Child>> {
        if self.count == 0 {
            return None;
        }

        // Since we maintain sorted order, the latest is the last one
        self.transforms[self.count - 1].as_ref()
    }

    /// Get transform closest to specified time
    pub fn get_closest_transform(
        &self,
        time: CuTime,
    ) -> Option<&TypedTransformMsg<T, Parent, Child>> {
        if self.count == 0 {
            return None;
        }

        let mut closest_idx = 0;
        let mut closest_diff = u64::MAX;

        for i in 0..self.count {
            if let Some(ref transform) = self.transforms[i] {
                if let Some(transform_time) = transform.timestamp() {
                    let diff = if time.as_nanos() > transform_time.as_nanos() {
                        time.as_nanos() - transform_time.as_nanos()
                    } else {
                        transform_time.as_nanos() - time.as_nanos()
                    };

                    if diff < closest_diff {
                        closest_diff = diff;
                        closest_idx = i;
                    }
                }
            }
        }

        self.transforms[closest_idx].as_ref()
    }

    /// Get time range of stored transforms
    pub fn get_time_range(&self) -> Option<CuTimeRange> {
        if self.count == 0 {
            return None;
        }

        // Since we maintain sorted order, first is min, last is max
        let start = self.transforms[0].as_ref()?.timestamp()?;
        let end = self.transforms[self.count - 1].as_ref()?.timestamp()?;

        Some(CuTimeRange { start, end })
    }

    /// Get two transforms around the specified time for velocity computation
    #[allow(clippy::type_complexity)]
    pub fn get_transforms_around(
        &self,
        time: CuTime,
    ) -> Option<(
        &TypedTransformMsg<T, Parent, Child>,
        &TypedTransformMsg<T, Parent, Child>,
    )> {
        if self.count < 2 {
            return None;
        }

        // Find transforms before and after the requested time
        let mut before_idx = None;
        let mut after_idx = None;

        for i in 0..self.count {
            if let Some(ref transform) = self.transforms[i] {
                if let Some(transform_time) = transform.timestamp() {
                    if transform_time <= time {
                        before_idx = Some(i);
                    } else if after_idx.is_none() {
                        after_idx = Some(i);
                        break;
                    }
                }
            }
        }

        match (before_idx, after_idx) {
            (Some(before), Some(after)) => Some((
                self.transforms[before].as_ref()?,
                self.transforms[after].as_ref()?,
            )),
            (Some(before), None) => {
                // Time is after all our transforms, use last two
                if before > 0 {
                    Some((
                        self.transforms[before - 1].as_ref()?,
                        self.transforms[before].as_ref()?,
                    ))
                } else {
                    None
                }
            }
            (None, Some(after)) => {
                // Time is before all our transforms, use first two
                if after + 1 < self.count {
                    Some((
                        self.transforms[after].as_ref()?,
                        self.transforms[after + 1].as_ref()?,
                    ))
                } else {
                    None
                }
            }
            _ => None,
        }
    }
}

impl<T, Parent, Child, const N: usize> Default for TypedTransformBuffer<T, Parent, Child, N>
where
    T: CuMsgPayload + Copy + Debug + 'static,
    Parent: FrameId,
    Child: FrameId,
{
    fn default() -> Self {
        Self::new()
    }
}

/// Velocity computation for typed transforms
impl<T, Parent, Child> TypedTransformMsg<T, Parent, Child>
where
    T: CuMsgPayload
        + Copy
        + Debug
        + Default
        + std::ops::Add<Output = T>
        + std::ops::Sub<Output = T>
        + std::ops::Mul<Output = T>
        + std::ops::Div<Output = T>
        + num_traits::NumCast
        + 'static,
    Parent: FrameId,
    Child: FrameId,
{
    /// Compute velocity from this transform and a previous transform
    pub fn compute_velocity(&self, previous: &Self) -> Option<VelocityTransform<T>> {
        let current_time = self.timestamp()?;
        let previous_time = previous.timestamp()?;
        let current_transform = self.transform()?;
        let previous_transform = previous.transform()?;

        // Compute time difference in nanoseconds, then convert to seconds
        let dt_nanos = current_time.as_nanos() as i64 - previous_time.as_nanos() as i64;
        if dt_nanos <= 0 {
            return None;
        }

        // Convert nanoseconds to seconds (1e9 nanoseconds = 1 second)
        let dt = dt_nanos as f64 / 1_000_000_000.0;

        let dt_t = num_traits::cast::cast::<f64, T>(dt)?;

        // Extract positions from transforms (column-major format)
        let current_mat = current_transform.to_matrix();
        let previous_mat = previous_transform.to_matrix();
        let mut linear_velocity = [T::default(); 3];
        for (i, vel) in linear_velocity.iter_mut().enumerate() {
            let pos_diff = current_mat[3][i] - previous_mat[3][i];
            *vel = pos_diff / dt_t;
        }

        // Compute angular velocity (simplified version for now)
        let angular_velocity = [T::default(); 3];

        Some(VelocityTransform {
            linear: linear_velocity,
            angular: angular_velocity,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frames::{RobotFrame, WorldFrame};
    use approx::assert_relative_eq;
    use cu29::clock::CuDuration;

    type WorldToRobotMsg = TypedTransformMsg<f32, WorldFrame, RobotFrame>;
    type WorldToRobotBuffer = TypedTransformBuffer<f32, WorldFrame, RobotFrame, 10>;

    #[test]
    fn test_typed_transform_msg_creation() {
        let transform = Transform3D::<f32>::default();
        let time = CuDuration(1000);

        let msg = WorldToRobotMsg::new(transform, time);

        assert_eq!(msg.parent_id(), WorldFrame::ID);
        assert_eq!(msg.child_id(), RobotFrame::ID);
        assert_eq!(msg.parent_name(), "world");
        assert_eq!(msg.child_name(), "robot");
        assert_eq!(msg.timestamp().unwrap().as_nanos(), 1000);
    }

    #[test]
    fn test_typed_transform_buffer() {
        let mut buffer = WorldToRobotBuffer::new();

        let transform1 = Transform3D::<f32>::default();
        let msg1 = WorldToRobotMsg::new(transform1, CuDuration(1000));

        let transform2 = Transform3D::<f32>::default();
        let msg2 = WorldToRobotMsg::new(transform2, CuDuration(2000));

        buffer.add_transform(msg1);
        buffer.add_transform(msg2);

        let latest = buffer.get_latest_transform().unwrap();
        assert_eq!(latest.timestamp().unwrap().as_nanos(), 2000);

        let range = buffer.get_time_range().unwrap();
        assert_eq!(range.start.as_nanos(), 1000);
        assert_eq!(range.end.as_nanos(), 2000);
    }

    #[test]
    fn test_closest_transform() {
        let mut buffer = WorldToRobotBuffer::new();

        let transform1 = Transform3D::<f32>::default();
        let msg1 = WorldToRobotMsg::new(transform1, CuDuration(1000));

        let transform2 = Transform3D::<f32>::default();
        let msg2 = WorldToRobotMsg::new(transform2, CuDuration(3000));

        buffer.add_transform(msg1);
        buffer.add_transform(msg2);

        let closest = buffer.get_closest_transform(CuDuration(1500));
        assert_eq!(closest.unwrap().timestamp().unwrap().as_nanos(), 1000);

        let closest = buffer.get_closest_transform(CuDuration(2500));
        assert_eq!(closest.unwrap().timestamp().unwrap().as_nanos(), 3000);
    }

    #[test]
    fn test_velocity_computation() {
        use crate::test_utils::translation_transform;
        
        let transform1 = translation_transform(0.0f32, 0.0, 0.0);
        let transform2 = translation_transform(1.0f32, 2.0, 0.0);

        let msg1 = WorldToRobotMsg::new(transform1, CuDuration(1_000_000_000)); // 1 second
        let msg2 = WorldToRobotMsg::new(transform2, CuDuration(2_000_000_000)); // 2 seconds

        let velocity = msg2.compute_velocity(&msg1).unwrap();

        assert_relative_eq!(velocity.linear[0], 1.0);
        assert_relative_eq!(velocity.linear[1], 2.0);
        assert_relative_eq!(velocity.linear[2], 0.0);
    }
}
