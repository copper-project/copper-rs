use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{Decode, Encode};
use cu29::clock::RobotClock;
use cu29::config::ComponentConfig;
use cu29::cutask::{CuMsg, CuSrcTask, CuTask, Freezable};
use cu29::{input_msg, output_msg, CuResult};
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

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self { state: true })
    }

    fn process(&mut self, _clock: &RobotClock, output: Self::Output) -> CuResult<()> {
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

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {})
    }

    fn process(
        &mut self,
        _clock: &RobotClock,
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
