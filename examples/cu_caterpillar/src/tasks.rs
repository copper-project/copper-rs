use cu_rp_gpio::RPGpioPayload;
use cu29::bincode::de::Decoder;
use cu29::bincode::enc::Encoder;
use cu29::bincode::error::{DecodeError, EncodeError};
use cu29::bincode::{Decode, Encode};
use cu29::prelude::*;

#[derive(Default)]
pub struct CaterpillarSource {
    state: bool,
}

impl Freezable for CaterpillarSource {
    fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.state, encoder)
    }

    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        self.state = Decode::decode(decoder)?;
        Ok(())
    }
}

impl CuSrcTask for CaterpillarSource {
    type Output<'m> = output_msg!(RPGpioPayload);

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self { state: true })
    }

    fn process(&mut self, clock: &RobotClock, output: &mut Self::Output<'_>) -> CuResult<()> {
        // forward the state to the next task
        self.state = !self.state;
        output.set_payload(RPGpioPayload { on: self.state });
        output.tov = Tov::Time(clock.now());
        output.metadata.set_status(self.state);
        Ok(())
    }
}

pub struct CaterpillarTask {}

impl Freezable for CaterpillarTask {}

impl CuTask for CaterpillarTask {
    type Input<'m> = input_msg!(RPGpioPayload);
    type Output<'m> = output_msg!(RPGpioPayload);

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {})
    }

    fn process(
        &mut self,
        _clock: &RobotClock,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        // forward the state to the next task
        let incoming = *input.payload().unwrap();
        output.set_payload(incoming);
        output.tov = input.tov;
        output.metadata.set_status(incoming.on);
        Ok(())
    }
}
