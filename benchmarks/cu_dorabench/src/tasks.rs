use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{Decode, Encode};
use cu29::prelude::*;
use serde::{Deserialize, Serialize, Serializer};
use std::sync::Arc;

#[derive(Reflect)]
#[reflect(no_field_bounds, from_reflect = false)]
pub struct DoraSource<const S: usize> {
    #[reflect(ignore)]
    pub pool: Arc<CuHostMemoryPool<Vec<u8>>>,
}

impl<const S: usize> Freezable for DoraSource<S> {}

impl<const S: usize> CuSrcTask for DoraSource<S> {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(DoraPayload);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let pool = CuHostMemoryPool::new("dummybuffers", 3, || vec![0u8; S])?;
        Ok(Self { pool })
    }

    fn process(&mut self, ctx: &CuContext, new_msg: &mut Self::Output<'_>) -> CuResult<()> {
        new_msg.tov = Tov::Time(ctx.now());
        let buffer = self.pool.acquire().unwrap();
        buffer.lock().unwrap()[42] = 42;
        new_msg.set_payload(DoraPayload(buffer));
        Ok(())
    }
}

#[derive(Reflect)]
#[reflect(from_reflect = false)]
pub struct DoraSink<const S: usize> {}

impl<const S: usize> Freezable for DoraSink<S> {}

impl<const S: usize> CuSinkTask for DoraSink<S> {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(DoraPayload);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {})
    }

    fn process(&mut self, _ctx: &CuContext, input: &Self::Input<'_>) -> CuResult<()> {
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

#[derive(Default, Debug, Clone, Reflect)]
#[reflect(opaque, from_reflect = false)]
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
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let data = self.0.with_inner(|inner| inner.to_vec());
        serializer.serialize_bytes(&data)
    }
}

impl<'de> Deserialize<'de> for DoraPayload {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let data = Vec::<u8>::deserialize(deserializer)?;
        Ok(DoraPayload(CuHandle::new_detached(data)))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bincode::config::standard;
    use bincode::enc::EncoderImpl;
    use bincode::enc::write::SizeWriter;

    #[test]
    fn dora_payload_measurement_reports_wrapped_handle_bytes() {
        let payload = DoraPayload(CuHandle::new_detached(vec![0u8; 1024]));
        let io = payload_io_stats(&payload).expect("payload IO measurement should succeed");
        let mut encoder = EncoderImpl::<_, _>::new(SizeWriter::default(), standard());
        payload
            .encode(&mut encoder)
            .expect("size measurement encoder should not fail");
        assert_eq!(io.encoded_bytes, encoder.into_writer().bytes_written);
        assert_eq!(
            io.resident_bytes,
            core::mem::size_of::<DoraPayload>() + 1024
        );
        assert_eq!(io.handle_bytes, 1024);
    }
}
