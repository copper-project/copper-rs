use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{Decode, Encode};
use cu29_prelude::bincode;
use cu29_prelude::cutask::*;
use cu29_prelude::*;
use cu_rp_gpio::RPGpioPayload;

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

impl<'cl> CuSrcTask<'cl> for CaterpillarSource {
    type Output = output_msg!('cl, RPGpioPayload);

    fn new(_config: Option<&config::ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self { state: true })
    }

    fn process(&mut self, _clock: &clock::RobotClock, output: Self::Output) -> CuResult<()> {
        // forward the state to the next task
        self.state = !self.state;
        output.set_payload(RPGpioPayload { on: self.state });
        output.metadata.set_status(self.state);
        Ok(())
    }
}

pub struct CaterpillarTask {}

impl Freezable for CaterpillarTask {}

impl<'cl> CuTask<'cl> for CaterpillarTask {
    type Input = input_msg!('cl, RPGpioPayload);
    type Output = output_msg!('cl, RPGpioPayload);

    fn new(_config: Option<&config::ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {})
    }

    fn process(
        &mut self,
        _clock: &clock::RobotClock,
        input: Self::Input,
        output: Self::Output,
    ) -> CuResult<()> {
        // forward the state to the next task
        let incoming = *input.payload().unwrap();
        output.set_payload(incoming);
        output.metadata.set_status(incoming.on);
        Ok(())
    }
}
