use crate::transform::StampedTransform;
use cu29::prelude::*;
use cu_spatial_payloads::Transform3D;
use std::fmt::Debug;
use std::marker::PhantomData;

#[derive(Debug, Default, Clone)]
pub struct TransformBroadcastPayload<T: Copy + Debug + Default + 'static> {
    pub transforms: Vec<StampedTransform<T>>,
}

impl<T: Copy + Debug + Default + 'static> Encode for TransformBroadcastPayload<T> {
    fn encode<E: bincode::enc::Encoder>(&self, encoder: &mut E) -> Result<(), bincode::error::EncodeError> {
        Ok(())
    }
}

impl<T: Copy + Debug + Default + 'static> Decode<()> for TransformBroadcastPayload<T> {
    fn decode<D: bincode::de::Decoder>(decoder: &mut D, _ctx: ()) -> Result<Self, bincode::error::DecodeError> {
        Ok(Self::default())
    }
}

impl<T: Copy + Debug + Default + 'static> CuMsgPayload for TransformBroadcastPayload<T> {}

pub struct TransformBroadcaster<'cl, T> {
    _marker: PhantomData<&'cl T>,
}

impl<'cl, T> Freezable for TransformBroadcaster<'cl, T> {
    fn freeze(&mut self) -> CuResult<()> {
        Ok(())
    }

    fn thaw(&mut self) -> CuResult<()> {
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

impl<'cl, T> Freezable for TransformListener<'cl, T> {
    fn freeze(&mut self) -> CuResult<()> {
        Ok(())
    }

    fn thaw(&mut self) -> CuResult<()> {
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
