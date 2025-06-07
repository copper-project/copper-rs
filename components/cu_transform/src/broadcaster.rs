use crate::transform::StampedTransform;
use cu29::clock::CuTime;
use cu29::prelude::*;
use cu_spatial_payloads::Transform3D;
use std::fmt::Debug;
use std::marker::PhantomData;

/// Maximum number of transforms in a single broadcast message
pub const MAX_TRANSFORMS_PER_BROADCAST: usize = 10;

#[derive(Debug, Clone)]
pub struct TransformBroadcastPayload<
    T: Copy + Debug + Default + bincode::enc::Encode + bincode::de::Decode<()> + 'static,
> {
    /// Fixed-size array of transforms
    pub transforms: [Option<StampedTransform<T>>; MAX_TRANSFORMS_PER_BROADCAST],
    /// Number of valid transforms in the array
    pub count: usize,
}

impl<T: Copy + Debug + Default + bincode::enc::Encode + bincode::de::Decode<()> + 'static> Default for TransformBroadcastPayload<T> {
    fn default() -> Self {
        Self {
            transforms: [const { None }; MAX_TRANSFORMS_PER_BROADCAST],
            count: 0,
        }
    }
}

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

                // Encode parent_frame string
                bincode::enc::Encode::encode(&transform.parent_frame, encoder)?;

                // Encode child_frame string
                bincode::enc::Encode::encode(&transform.child_frame, encoder)?;
            }
        }

        Ok(())
    }
}

impl<T: Copy + Debug + Default + bincode::enc::Encode + bincode::de::Decode<()> + 'static>
    bincode::de::Decode<()> for TransformBroadcastPayload<T>
{
    fn decode<D: bincode::de::Decoder<Context = ()>>(
        decoder: &mut D,
    ) -> Result<Self, bincode::error::DecodeError> {
        // Decode the number of transforms
        let count = <usize as bincode::de::Decode<()>>::decode(decoder)?;
        
        if count > MAX_TRANSFORMS_PER_BROADCAST {
            return Err(bincode::error::DecodeError::Other("Too many transforms in broadcast"));
        }

        // Create array to hold the transforms
        let mut transforms = [const { None }; MAX_TRANSFORMS_PER_BROADCAST];

        // Decode each transform
        for i in 0..count {
            // Decode transform.transform (Transform3D)
            let transform_3d = <Transform3D<T> as bincode::de::Decode<()>>::decode(decoder)?;

            // Decode timestamp
            let stamp = <CuTime as bincode::de::Decode<()>>::decode(decoder)?;

            // Decode parent_frame string
            let parent_frame = <String as bincode::de::Decode<()>>::decode(decoder)?;

            // Decode child_frame string
            let child_frame = <String as bincode::de::Decode<()>>::decode(decoder)?;

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

pub struct TransformBroadcaster<'cl, T> {
    _marker: PhantomData<&'cl T>,
}

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

pub struct TransformListener<'cl, T> {
    _marker: PhantomData<&'cl T>,
}

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
    fn test_transform_broadcast_payload_serialization() {
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
            parent_frame: "world".to_string(),
            child_frame: "robot".to_string(),
        };

        let transform2 = StampedTransform {
            transform: Transform3D::from_matrix([
                    [0.0, -1.0, 0.0, 5.0],
                    [1.0, 0.0, 0.0, 6.0],
                    [0.0, 0.0, 1.0, 7.0],
                    [0.0, 0.0, 0.0, 1.0],
            ]),
            stamp: CuDuration(2000),
            parent_frame: "robot".to_string(),
            child_frame: "camera".to_string(),
        };

        payload.transforms[0] = Some(transform1);
        payload.transforms[1] = Some(transform2);
        payload.count = 2;

        // Verify transforms
        assert_eq!(payload.count, 2);
        assert_eq!(payload.transforms[0].as_ref().unwrap().stamp.as_nanos(), 1000);
        assert_eq!(payload.transforms[0].as_ref().unwrap().parent_frame, "world");
        assert_eq!(payload.transforms[0].as_ref().unwrap().child_frame, "robot");

        assert_eq!(payload.transforms[1].as_ref().unwrap().stamp.as_nanos(), 2000);
        assert_eq!(payload.transforms[1].as_ref().unwrap().parent_frame, "robot");
        assert_eq!(payload.transforms[1].as_ref().unwrap().child_frame, "camera");
    }

    #[test]
    fn test_transform_broadcast_payload_encode_decode() {
        // Create a payload with transforms
        let mut payload: TransformBroadcastPayload<f32> = TransformBroadcastPayload::default();

        // Add transforms with specific values for testing
        let transform1 = StampedTransform {
            transform: Transform3D::from_matrix([
                    [1.0, 0.0, 0.0, 2.0],
                    [0.0, 1.0, 0.0, 3.0],
                    [0.0, 0.0, 1.0, 4.0],
                    [0.0, 0.0, 0.0, 1.0],
            ]),
            stamp: CuDuration(1000),
            parent_frame: "world".to_string(),
            child_frame: "robot".to_string(),
        };

        payload.transforms[0] = Some(transform1);
        payload.count = 1;

        // Encode to bytes
        let encoded = bincode::encode_to_vec(&payload, bincode::config::standard())
            .expect("Failed to encode");

        // Decode from bytes
        let (decoded, _): (TransformBroadcastPayload<f32>, _) =
            bincode::decode_from_slice(&encoded, bincode::config::standard())
                .expect("Failed to decode");

        // Verify the decoded payload
        assert_eq!(decoded.count, 1);
        assert_eq!(decoded.transforms[0].as_ref().unwrap().stamp.as_nanos(), 1000);
        assert_eq!(decoded.transforms[0].as_ref().unwrap().parent_frame, "world");
        assert_eq!(decoded.transforms[0].as_ref().unwrap().child_frame, "robot");

        // Check transform matrix
        let decoded_mat = decoded.transforms[0].as_ref().unwrap().transform.to_matrix();
        let payload_mat = payload.transforms[0].as_ref().unwrap().transform.to_matrix();
        for i in 0..4 {
            for j in 0..4 {
                assert_eq!(
                    decoded_mat[i][j],
                    payload_mat[i][j]
                );
            }
        }
    }
}
