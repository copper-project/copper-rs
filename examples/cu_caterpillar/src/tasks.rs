use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{Decode, Encode};
use cu29::clock::RobotClock;
use cu29::config::ComponentConfig;
use cu29::cutask::{CuMsg, CuSinkTask, CuSrcTask, CuTask, Freezable};
use cu29::{input_msg, output_msg, CuResult};
use cu_rp_gpio::RPGpioPayload;
use cu_sensor_payloads::CuImage;

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

////////////////////// IMG ///////////////

#[derive(Default)]
pub struct CameraSource {
    state: bool,
}

impl Freezable for CameraSource {
    fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.state, encoder)
    }

    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        self.state = Decode::decode(decoder)?;
        Ok(())
    }
}

impl<'cl> CuSrcTask<'cl> for CameraSource {
    type Output = output_msg!('cl, CuImage);

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self { state: true })
    }

    fn process(&mut self, _clock: &RobotClock, output: Self::Output) -> CuResult<()> {
        // forward the state to the next task
        self.state = !self.state;
        let img = image::open(
            std::env::current_dir()
                .unwrap()
                .join("Philips_PM5544_small.png"),
        )
        .unwrap();
        output.set_payload(CuImage::new(img));
        output.metadata.set_status(self.state);
        // println!("src");
        Ok(())
    }
}

pub struct ImageProc {}

impl Freezable for ImageProc {}

impl<'cl> CuTask<'cl> for ImageProc {
    type Input = input_msg!('cl, CuImage);
    type Output = output_msg!('cl, CuImage);

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
        // println!("proc");
        // forward the state to the next task
        let incoming = input.payload().unwrap().clone();
        output
            .metadata
            .set_status(format!("w: {} h: {}", incoming.width(), incoming.height()));
        output.set_payload(incoming);
        Ok(())
    }
}

pub struct ImageView {}

impl Freezable for ImageView {}

impl<'cl> CuSinkTask<'cl> for ImageView {
    type Input = input_msg!('cl, CuImage);

    fn new(config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        // let config = config.ok_or("RPGpio needs a config, None was passed as ComponentConfig")?;

        // let pin_nb: u8 = (*config.0.get("pin").expect(
        //     "RPGpio expects a pin config value pointing to output pin you want to address",
        // ))
        // .clone()
        // .into();

        // #[cfg(not(feature = "mock"))]
        // let pin = GPIO
        //     .get(pin_nb)
        //     .map_err(|e| CuError::new_with_cause("Could not get pin", e))?
        //     .into_output();
        // #[cfg(mock)]
        // let pin = pin_nb;
        // Ok(Self { pin })
        Ok(Self {})
    }

    fn process(&mut self, _clock: &RobotClock, msg: Self::Input) -> CuResult<()> {
        let img = msg.payload().unwrap();

        // img.save("img.png").unwrap();
        // println!("view");

        Ok(())
    }
}
