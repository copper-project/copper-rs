use crate::transform::StampedTransform;
use crate::transform_msg::TransformMsg;
use cu29::clock::{CuTime, Tov};
use cu29::prelude::*;
use cu_spatial_payloads::Transform3D;
use std::fmt::Debug;
use std::marker::PhantomData;

#[cfg(test)]
use crate::frame_id;

/// Maximum number of transforms in a single broadcast message
pub const MAX_TRANSFORMS_PER_BROADCAST: usize = 10;

/// Modern broadcast payload using TransformMsg
/// This is the preferred way to broadcast transforms using CuMsg
#[derive(Debug, Clone, Serialize)]
pub struct TransformBroadcast<T: Copy + Debug + Default + Serialize + 'static> {
    /// Fixed-size array of transform messages
    pub transforms: [Option<TransformMsg<T>>; MAX_TRANSFORMS_PER_BROADCAST],
    /// Number of valid transforms in the array
    pub count: usize,
}

impl<T: Copy + Debug + Default + Serialize + 'static> Default for TransformBroadcast<T> {
    fn default() -> Self {
        Self {
            transforms: [const { None }; MAX_TRANSFORMS_PER_BROADCAST],
            count: 0,
        }
    }
}

impl<T: Copy + Debug + Default + Serialize + 'static> TransformBroadcast<T> {
    /// Create a new broadcast payload
    pub fn new() -> Self {
        Self::default()
    }

    /// Add a transform to the broadcast
    /// Returns false if the broadcast is full
    pub fn add_transform(&mut self, transform: TransformMsg<T>) -> bool {
        if self.count >= MAX_TRANSFORMS_PER_BROADCAST {
            return false;
        }

        self.transforms[self.count] = Some(transform);
        self.count += 1;
        true
    }

    /// Clear all transforms from the broadcast
    pub fn clear(&mut self) {
        for i in 0..self.count {
            self.transforms[i] = None;
        }
        self.count = 0;
    }

    /// Get an iterator over the transforms
    pub fn iter(&self) -> impl Iterator<Item = &TransformMsg<T>> {
        self.transforms[..self.count]
            .iter()
            .filter_map(|t| t.as_ref())
    }
}

impl<
        T: Copy
            + Debug
            + Default
            + bincode::enc::Encode
            + bincode::de::Decode<()>
            + Serialize
            + 'static,
    > bincode::enc::Encode for TransformBroadcast<T>
{
    fn encode<E: bincode::enc::Encoder>(
        &self,
        encoder: &mut E,
    ) -> Result<(), bincode::error::EncodeError> {
        // Encode the count
        bincode::enc::Encode::encode(&self.count, encoder)?;

        // Encode each transform
        for i in 0..self.count {
            if let Some(ref transform) = self.transforms[i] {
                bincode::enc::Encode::encode(transform, encoder)?;
            }
        }

        Ok(())
    }
}

impl<
        T: Copy
            + Debug
            + Default
            + bincode::enc::Encode
            + bincode::de::Decode<()>
            + Serialize
            + 'static,
    > bincode::de::Decode<()> for TransformBroadcast<T>
{
    fn decode<D: bincode::de::Decoder<Context = ()>>(
        decoder: &mut D,
    ) -> Result<Self, bincode::error::DecodeError> {
        // Decode the count
        let count = <usize as bincode::de::Decode<()>>::decode(decoder)?;

        if count > MAX_TRANSFORMS_PER_BROADCAST {
            return Err(bincode::error::DecodeError::OtherString(format!(
                "Too many transforms in broadcast: {count} > {MAX_TRANSFORMS_PER_BROADCAST}"
            )));
        }

        let mut transforms = [const { None }; MAX_TRANSFORMS_PER_BROADCAST];

        // Decode each transform
        for transform in transforms.iter_mut().take(count) {
            *transform = Some(<TransformMsg<T> as bincode::de::Decode<()>>::decode(
                decoder,
            )?);
        }

        Ok(Self { transforms, count })
    }
}

#[derive(Debug, Clone, Serialize)]
#[deprecated(note = "Use TransformBroadcast with TransformMsg instead")]
pub struct TransformBroadcastPayload<
    T: Copy + Debug + Default + bincode::enc::Encode + bincode::de::Decode<()> + 'static,
> {
    /// Fixed-size array of transforms
    pub transforms: [Option<StampedTransform<T>>; MAX_TRANSFORMS_PER_BROADCAST],
    /// Number of valid transforms in the array
    pub count: usize,
}

#[allow(deprecated)]
impl<T: Copy + Debug + Default + bincode::enc::Encode + bincode::de::Decode<()> + 'static> Default
    for TransformBroadcastPayload<T>
{
    fn default() -> Self {
        Self {
            transforms: [const { None }; MAX_TRANSFORMS_PER_BROADCAST],
            count: 0,
        }
    }
}

#[allow(deprecated)]
impl<T: Copy + Debug + Default + bincode::enc::Encode + bincode::de::Decode<()> + 'static>
    bincode::enc::Encode for TransformBroadcastPayload<T>
{
    fn encode<E: bincode::enc::Encoder>(
        &self,
        encoder: &mut E,
    ) -> Result<(), bincode::error::EncodeError> {
        // Encode the number of valid transforms
        bincode::enc::Encode::encode(&self.count, encoder)?;

        // Encode only the valid transforms
        for i in 0..self.count {
            if let Some(transform) = &self.transforms[i] {
                // Encode transform.transform (Transform3D)
                bincode::enc::Encode::encode(&transform.transform, encoder)?;

                // Encode timestamp
                bincode::enc::Encode::encode(&transform.stamp, encoder)?;

                // Encode parent_frame as string
                bincode::enc::Encode::encode(&transform.parent_frame.as_str(), encoder)?;

                // Encode child_frame as string
                bincode::enc::Encode::encode(&transform.child_frame.as_str(), encoder)?
            }
        }

        Ok(())
    }
}

#[allow(deprecated)]
impl<T: Copy + Debug + Default + bincode::enc::Encode + bincode::de::Decode<()> + 'static>
    bincode::de::Decode<()> for TransformBroadcastPayload<T>
{
    fn decode<D: bincode::de::Decoder<Context = ()>>(
        decoder: &mut D,
    ) -> Result<Self, bincode::error::DecodeError> {
        // Decode the number of transforms
        let count = <usize as bincode::de::Decode<()>>::decode(decoder)?;

        if count > MAX_TRANSFORMS_PER_BROADCAST {
            return Err(bincode::error::DecodeError::Other(
                "Too many transforms in broadcast",
            ));
        }

        // Create array to hold the transforms
        let mut transforms = [const { None }; MAX_TRANSFORMS_PER_BROADCAST];

        // Decode each transform
        #[allow(clippy::needless_range_loop)]
        for i in 0..count {
            // Decode transform.transform (Transform3D)
            let transform_3d = <Transform3D<T> as bincode::de::Decode<()>>::decode(decoder)?;

            // Decode timestamp
            let stamp = <CuTime as bincode::de::Decode<()>>::decode(decoder)?;

            // Decode parent_frame string
            let parent_frame_str = <String as bincode::de::Decode<()>>::decode(decoder)?;
            let parent_frame = crate::FrameIdString::from(&parent_frame_str)
                .map_err(|_| bincode::error::DecodeError::Other("Parent frame name too long"))?;

            // Decode child_frame string
            let child_frame_str = <String as bincode::de::Decode<()>>::decode(decoder)?;
            let child_frame = crate::FrameIdString::from(&child_frame_str)
                .map_err(|_| bincode::error::DecodeError::Other("Child frame name too long"))?;

            // Create StampedTransform and add to array
            transforms[i] = Some(StampedTransform {
                transform: transform_3d,
                stamp,
                parent_frame,
                child_frame,
            });
        }

        Ok(Self { transforms, count })
    }
}

#[deprecated(note = "Use ModernTransformBroadcaster with TransformBroadcast instead")]
pub struct TransformBroadcaster<'cl, T> {
    _marker: PhantomData<&'cl T>,
}

#[allow(deprecated)]
impl<T> Freezable for TransformBroadcaster<'_, T> {
    fn freeze<E: bincode::enc::Encoder>(
        &self,
        _encoder: &mut E,
    ) -> Result<(), bincode::error::EncodeError> {
        Ok(())
    }

    fn thaw<D: bincode::de::Decoder>(
        &mut self,
        _decoder: &mut D,
    ) -> Result<(), bincode::error::DecodeError> {
        Ok(())
    }
}

#[allow(deprecated)]
impl<'cl, T> CuSrcTask<'cl> for TransformBroadcaster<'cl, T>
where
    T: CuMsgPayload + Copy + Debug + 'static,
{
    type Output = output_msg!('cl, TransformBroadcastPayload<T>);

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {
            _marker: PhantomData,
        })
    }

    fn process(&mut self, clock: &RobotClock, new_msg: Self::Output) -> CuResult<()> {
        new_msg.metadata.tov = Tov::Time(clock.now());
        Ok(())
    }
}

/// Modern transform broadcaster using TransformBroadcast
pub struct ModernTransformBroadcaster<'cl, T> {
    _marker: PhantomData<&'cl T>,
}

impl<T> Freezable for ModernTransformBroadcaster<'_, T> {
    fn freeze<E: bincode::enc::Encoder>(
        &self,
        _encoder: &mut E,
    ) -> Result<(), bincode::error::EncodeError> {
        Ok(())
    }

    fn thaw<D: bincode::de::Decoder>(
        &mut self,
        _decoder: &mut D,
    ) -> Result<(), bincode::error::DecodeError> {
        Ok(())
    }
}

impl<'cl, T> CuSrcTask<'cl> for ModernTransformBroadcaster<'cl, T>
where
    T: CuMsgPayload + Copy + Debug + 'static,
{
    type Output = output_msg!('cl, TransformBroadcast<T>);

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {
            _marker: PhantomData,
        })
    }

    fn process(&mut self, clock: &RobotClock, new_msg: Self::Output) -> CuResult<()> {
        new_msg.metadata.tov = Tov::Time(clock.now());
        Ok(())
    }
}

#[deprecated(note = "Use ModernTransformListener with TransformBroadcast instead")]
pub struct TransformListener<'cl, T> {
    _marker: PhantomData<&'cl T>,
}

/// Modern transform listener using TransformBroadcast
pub struct ModernTransformListener<'cl, T> {
    _marker: PhantomData<&'cl T>,
}

impl<T> Freezable for ModernTransformListener<'_, T> {
    fn freeze<E: bincode::enc::Encoder>(
        &self,
        _encoder: &mut E,
    ) -> Result<(), bincode::error::EncodeError> {
        Ok(())
    }

    fn thaw<D: bincode::de::Decoder>(
        &mut self,
        _decoder: &mut D,
    ) -> Result<(), bincode::error::DecodeError> {
        Ok(())
    }
}

impl<'cl, T> CuTask<'cl> for ModernTransformListener<'cl, T>
where
    T: CuMsgPayload + Copy + Debug + 'static,
{
    type Input = input_msg!('cl, TransformBroadcast<T>);
    type Output = output_msg!('cl, ()); // No output, just updates internal state

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {
            _marker: PhantomData,
        })
    }

    fn process(
        &mut self,
        _clock: &RobotClock,
        input: Self::Input,
        output: Self::Output,
    ) -> CuResult<()> {
        output.metadata.tov = input.metadata.tov;
        Ok(())
    }
}

#[allow(deprecated)]
impl<T> Freezable for TransformListener<'_, T> {
    fn freeze<E: bincode::enc::Encoder>(
        &self,
        _encoder: &mut E,
    ) -> Result<(), bincode::error::EncodeError> {
        Ok(())
    }

    fn thaw<D: bincode::de::Decoder>(
        &mut self,
        _decoder: &mut D,
    ) -> Result<(), bincode::error::DecodeError> {
        Ok(())
    }
}

#[allow(deprecated)]
impl<'cl, T> CuTask<'cl> for TransformListener<'cl, T>
where
    T: CuMsgPayload + Copy + Debug + 'static,
{
    type Input = input_msg!('cl, TransformBroadcastPayload<T>);
    type Output = output_msg!('cl, ()); // No output, just updates internal state

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {
            _marker: PhantomData,
        })
    }

    fn process(
        &mut self,
        _clock: &RobotClock,
        input: Self::Input,
        output: Self::Output,
    ) -> CuResult<()> {
        output.metadata.tov = input.metadata.tov;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cu29::clock::CuDuration;

    #[test]
    fn test_transform_broadcast_serialization() {
        // Create an empty broadcast
        let broadcast: TransformBroadcast<f32> = TransformBroadcast::default();

        // Check that the empty broadcast has no transforms
        assert_eq!(broadcast.count, 0);

        // Create a broadcast with transforms
        let mut broadcast = TransformBroadcast::new();

        // Add transforms using TransformMsg
        let transform1 = Transform3D::from_matrix([
            [1.0, 0.0, 0.0, 2.0],
            [0.0, 1.0, 0.0, 3.0],
            [0.0, 0.0, 1.0, 4.0],
            [0.0, 0.0, 0.0, 1.0],
        ]);
        let msg1 = TransformMsg::new(transform1, "world", "robot");

        let transform2 = Transform3D::from_matrix([
            [0.0, -1.0, 0.0, 5.0],
            [1.0, 0.0, 0.0, 6.0],
            [0.0, 0.0, 1.0, 7.0],
            [0.0, 0.0, 0.0, 1.0],
        ]);
        let msg2 = TransformMsg::new(transform2, "robot", "camera");

        assert!(broadcast.add_transform(msg1.clone()));
        assert!(broadcast.add_transform(msg2.clone()));

        // Verify transforms
        assert_eq!(broadcast.count, 2);
        let transforms: Vec<_> = broadcast.iter().collect();
        assert_eq!(transforms.len(), 2);
        assert_eq!(transforms[0].parent_frame.as_str(), "world");
        assert_eq!(transforms[0].child_frame.as_str(), "robot");
        assert_eq!(transforms[1].parent_frame.as_str(), "robot");
        assert_eq!(transforms[1].child_frame.as_str(), "camera");
    }

    #[test]
    #[allow(deprecated)]
    fn test_legacy_transform_broadcast_payload_serialization() {
        use crate::transform::StampedTransform;
        // Create an empty payload
        let payload: TransformBroadcastPayload<f32> = TransformBroadcastPayload::default();

        // Check that the empty payload has no transforms
        assert_eq!(payload.count, 0);

        // Create a payload with transforms
        let mut payload = TransformBroadcastPayload::default();

        // Add transforms
        let transform1 = StampedTransform {
            transform: Transform3D::from_matrix([
                [1.0, 0.0, 0.0, 2.0],
                [0.0, 1.0, 0.0, 3.0],
                [0.0, 0.0, 1.0, 4.0],
                [0.0, 0.0, 0.0, 1.0],
            ]),
            stamp: CuDuration(1000),
            parent_frame: frame_id!("world"),
            child_frame: frame_id!("robot"),
        };

        let transform2 = StampedTransform {
            transform: Transform3D::from_matrix([
                [0.0, -1.0, 0.0, 5.0],
                [1.0, 0.0, 0.0, 6.0],
                [0.0, 0.0, 1.0, 7.0],
                [0.0, 0.0, 0.0, 1.0],
            ]),
            stamp: CuDuration(2000),
            parent_frame: frame_id!("robot"),
            child_frame: frame_id!("camera"),
        };

        payload.transforms[0] = Some(transform1);
        payload.transforms[1] = Some(transform2);
        payload.count = 2;

        // Verify transforms
        assert_eq!(payload.count, 2);
        assert_eq!(
            payload.transforms[0].as_ref().unwrap().stamp.as_nanos(),
            1000
        );
        assert_eq!(
            payload.transforms[0]
                .as_ref()
                .unwrap()
                .parent_frame
                .as_str(),
            "world"
        );
        assert_eq!(
            payload.transforms[0].as_ref().unwrap().child_frame.as_str(),
            "robot"
        );

        assert_eq!(
            payload.transforms[1].as_ref().unwrap().stamp.as_nanos(),
            2000
        );
        assert_eq!(
            payload.transforms[1]
                .as_ref()
                .unwrap()
                .parent_frame
                .as_str(),
            "robot"
        );
        assert_eq!(
            payload.transforms[1].as_ref().unwrap().child_frame.as_str(),
            "camera"
        );
    }

    #[test]
    fn test_transform_broadcast_encode_decode() {
        // Create a broadcast with transforms
        let mut broadcast = TransformBroadcast::<f32>::new();

        // Add transforms using TransformMsg
        let transform1 = Transform3D::from_matrix([
            [1.0, 0.0, 0.0, 2.0],
            [0.0, 1.0, 0.0, 3.0],
            [0.0, 0.0, 1.0, 4.0],
            [0.0, 0.0, 0.0, 1.0],
        ]);
        let msg1 = TransformMsg::new(transform1, "world", "robot");
        broadcast.add_transform(msg1);

        // Encode to bytes
        let encoded = bincode::encode_to_vec(&broadcast, bincode::config::standard())
            .expect("Failed to encode");

        // Decode from bytes
        let (decoded, _): (TransformBroadcast<f32>, _) =
            bincode::decode_from_slice(&encoded, bincode::config::standard())
                .expect("Failed to decode");

        // Verify the decoded broadcast
        assert_eq!(decoded.count, 1);
        let transforms: Vec<_> = decoded.iter().collect();
        assert_eq!(transforms.len(), 1);
        assert_eq!(transforms[0].parent_frame.as_str(), "world");
        assert_eq!(transforms[0].child_frame.as_str(), "robot");

        // Check transform matrix
        let decoded_mat = transforms[0].transform.to_matrix();
        let original_mat = transform1.to_matrix();
        for i in 0..4 {
            for j in 0..4 {
                assert_eq!(decoded_mat[i][j], original_mat[i][j]);
            }
        }
    }
}
