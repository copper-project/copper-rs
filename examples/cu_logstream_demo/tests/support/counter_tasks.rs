use cu29::bincode::{
    Decode, Encode,
    de::Decoder,
    enc::Encoder,
    error::{DecodeError, EncodeError},
};
use cu29::prelude::*;
use serde::{Deserialize, Serialize};

#[derive(
    Clone, Copy, Debug, Default, PartialEq, Eq, Encode, Decode, Serialize, Deserialize, Reflect,
)]
#[bincode(crate = "cu29::bincode")]
pub struct Sample(pub u64);

#[derive(Default, Reflect)]
pub struct Counter {
    next: u64,
}

impl Freezable for Counter {
    fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.next, encoder)
    }
    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        self.next = Decode::decode(decoder)?;
        Ok(())
    }
}

impl CuSrcTask for Counter {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(Sample);
    fn new(_: Option<&ComponentConfig>, _: ()) -> CuResult<Self> {
        Ok(Self::default())
    }
    fn process(&mut self, ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        output.set_payload(Sample(self.next));
        output.tov = Tov::Time(ctx.now());
        self.next += 1;
        Ok(())
    }
}

#[derive(Default, Reflect)]
pub struct Accumulator {
    sum: u64,
}

impl Freezable for Accumulator {
    fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.sum, encoder)
    }
    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        self.sum = Decode::decode(decoder)?;
        Ok(())
    }
}

impl CuCrossPlatformDeterministic for Accumulator {
    const REPLAY_ABI: u32 = 1;
}

impl CuTask for Accumulator {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(Sample);
    type Output<'m> = output_msg!(Sample);
    fn new(_: Option<&ComponentConfig>, _: ()) -> CuResult<Self> {
        Ok(Self::default())
    }
    fn process(
        &mut self,
        _: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        self.sum += input
            .payload()
            .ok_or_else(|| CuError::from("Counter produced no sample"))?
            .0;
        output.set_payload(Sample(self.sum));
        output.tov = input.tov;
        Ok(())
    }
}

/// This task runs on both the robot and the generated ground-side replay runtime.
#[derive(Default, Reflect)]
pub struct Derived;
impl Freezable for Derived {}
impl CuCrossPlatformDeterministic for Derived {
    const REPLAY_ABI: u32 = 1;
}
impl CuTask for Derived {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(Sample);
    type Output<'m> = output_msg!(Sample);
    fn new(_: Option<&ComponentConfig>, _: ()) -> CuResult<Self> {
        Ok(Self)
    }
    fn process(
        &mut self,
        _: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        let sum = input
            .payload()
            .ok_or_else(|| CuError::from("Accumulator produced no sample"))?;
        output.set_payload(Sample(sum.0 % 256));
        output.tov = input.tov;
        Ok(())
    }
}
