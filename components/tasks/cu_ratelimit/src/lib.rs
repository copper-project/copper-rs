use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{Decode, Encode};
use cu29::prelude::*;
use std::marker::PhantomData;

pub struct CuRateLimit<T>
where
    T: for<'a> CuMsgPayload + 'static,
{
    _marker: PhantomData<T>,
    interval: CuDuration,
    last_tov: Option<CuTime>,
}

impl<T> Freezable for CuRateLimit<T>
where
    T: CuMsgPayload,
{
    fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.last_tov, encoder)
    }

    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        self.last_tov = Decode::decode(decoder)?;
        Ok(())
    }
}

impl<'cl, T> CuTask<'cl> for CuRateLimit<T>
where
    T: CuMsgPayload + 'cl,
{
    type Input = input_msg!('cl, T);
    type Output = output_msg!('cl, T);

    fn new(config: Option<&ComponentConfig>) -> CuResult<Self> {
        let hz = config
            .and_then(|cfg| cfg.get::<f64>("rate"))
            .ok_or("Missing required 'rate' config for CuRateLimiter")?;
        let interval_ns = (1e9 / hz) as u64;
        Ok(Self {
            _marker: PhantomData,
            interval: CuDuration::from(interval_ns),
            last_tov: None,
        })
    }

    fn process(
        &mut self,
        _clock: &RobotClock,
        input: Self::Input,
        output: Self::Output,
    ) -> CuResult<()> {
        let tov = match input.metadata.tov {
            Tov::Time(ts) => ts,
            _ => return Err("Expected single timestamp TOV".into()),
        };

        let allow = match self.last_tov {
            None => true,
            Some(last) => (tov - last) >= self.interval,
        };

        if allow {
            self.last_tov = Some(tov);
            if let Some(payload) = input.payload() {
                output.set_payload(payload.clone());
            } else {
                output.clear_payload();
            }
        } else {
            output.clear_payload();
        }

        Ok(())
    }
}
