use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{Decode, Encode};
use cu29::prelude::*;
use serde::Serializer;
use std::sync::Arc;

pub struct DoraSource<const S: usize> {
    pub pool: Arc<CuHostMemoryPool<Vec<u8>>>,
}

impl<const S: usize> Freezable for DoraSource<S> {}

impl<const S: usize> CuSrcTask for DoraSource<S> {
    type Output<'m> = output_msg!(DoraPayload);

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let pool = CuHostMemoryPool::new("dummybuffers", 3, || vec![0u8; S])?;
        Ok(Self { pool })
    }

    fn process(&mut self, clock: &RobotClock, new_msg: &mut Self::Output<'_>) -> CuResult<()> {
        new_msg.tov = Tov::Time(clock.now());
        let buffer = self.pool.acquire().unwrap();
        buffer.lock().unwrap()[42] = 42;
        new_msg.set_payload(DoraPayload(buffer));
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
        assert_eq!(incoming.0.lock().unwrap()[42], 42);
        Ok(())
    }
}

// select a specific case
const FORTY_MEG: usize = 40 * 1024 * 1024;

#[allow(dead_code)]
pub type FortyMegSrc = DoraSource<FORTY_MEG>;

#[allow(dead_code)]
pub type FortyMegSink = DoraSink<FORTY_MEG>;

#[derive(Default, Debug, Clone)]
pub struct DoraPayload(CuHandle<Vec<u8>>);

impl Decode<()> for DoraPayload {
    fn decode<D: Decoder<Context = ()>>(decoder: &mut D) -> Result<Self, DecodeError> {
        let handle = CuHandle::<Vec<u8>>::decode(decoder)?;
        Ok(DoraPayload(handle))
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
        todo!()
    }
}
