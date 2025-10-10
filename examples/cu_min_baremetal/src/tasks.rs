#[cfg(not(feature = "std"))]
extern crate alloc;
use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{Decode, Encode};
use cu29::prelude::*;
use serde::Serializer;

#[cfg(not(feature = "std"))]
use alloc::vec::Vec;

#[cfg(not(feature = "std"))]
use core::prelude::rust_2024::*;

pub struct DoraSource<const S: usize> {}

impl<const S: usize> Freezable for DoraSource<S> {}

impl<const S: usize> CuSrcTask for DoraSource<S> {
    type Output<'m> = output_msg!(DoraPayload);

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {})
    }

    fn process(&mut self, clock: &RobotClock, new_msg: &mut Self::Output<'_>) -> CuResult<()> {
        new_msg.tov = Tov::Time(clock.now());
        let DoraPayload(ref mut v) = new_msg.payload_mut().as_mut().unwrap();
        v.resize(FORTY_K, 0);
        v[42] = 42;
        Ok(())
    }
}

pub struct DoraSink<const S: usize> {}

impl<const S: usize> Freezable for DoraSink<S> {}

impl<const S: usize> CuSinkTask for DoraSink<S> {
    type Input<'m> = input_msg!(DoraPayload);

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {})
    }

    fn process(&mut self, _clock: &RobotClock, input: &Self::Input<'_>) -> CuResult<()> {
        let incoming = input.payload().unwrap();
        if incoming.0[42] != 42 {
            return Err("Something is wrong: 42 expected".into());
        }
        Ok(())
    }
}

const FORTY_K: usize = 40 * 1024;

#[allow(dead_code)]
pub type FortyKSrc = DoraSource<FORTY_K>;

#[allow(dead_code)]
pub type FortyKSink = DoraSink<FORTY_K>;

#[derive(Default, Debug, Clone)]
pub struct DoraPayload(Vec<u8>);

impl Decode<()> for DoraPayload {
    fn decode<D: Decoder<Context = ()>>(decoder: &mut D) -> Result<Self, DecodeError> {
        Ok(DoraPayload(Vec::<u8>::decode(decoder)?))
    }
}

impl Encode for DoraPayload {
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        self.0.encode(encoder)
    }
}

impl Serialize for DoraPayload {
    fn serialize<S>(&self, _serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        // Not needed for this benchmark.
        _serializer.serialize_bytes(&self.0)
    }
}
