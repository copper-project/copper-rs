use crate::transform_msg::TransformMsg;
use cu29::clock::Tov;
use cu29::prelude::*;
use std::fmt::Debug;
use std::marker::PhantomData;

/// Maximum number of transforms in a single broadcast message
pub const MAX_TRANSFORMS_PER_BROADCAST: usize = 10;

/// Modern broadcast payload using TransformMsg
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
        new_msg.tov = Tov::Time(clock.now());
        Ok(())
    }
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
        output.tov = input.tov;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cu_spatial_payloads::Transform3D;

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
