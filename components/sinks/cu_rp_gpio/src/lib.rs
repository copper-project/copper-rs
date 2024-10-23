use bincode::{Decode, Encode};
use cu29::config::ComponentConfig;
use cu29::cutask::{CuMsg, CuMsgPayload, CuSinkTask, CuTaskLifecycle, Freezable};
use cu29::CuResult;
use cu29::{clock, input_msg};
use cu29_log_derive::debug;

#[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
use {
    copper::CuError,
    lazy_static::lazy_static,
    rppal::gpio::{Gpio, Level, OutputPin},
};

#[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
lazy_static! {
    static ref GPIO: Gpio = Gpio::new().expect("Could not create GPIO bindings");
}

/// Example of a GPIO output driver for the Raspberry Pi
/// The config takes one config value: `pin` which is the pin you want to address
/// Gpio uses BCM pin numbering. For example: BCM GPIO 23 is tied to physical pin 16.
#[derive(Encode, Decode)]
pub struct RPGpio {
    #[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
    pin: OutputPin,
    #[cfg(target_arch = "x86_64")]
    pin: u8,
}

#[derive(Debug, Clone, Copy, Default, Encode, Decode, PartialEq)]
pub struct RPGpioPayload {
    pub on: bool,
    pub creation: clock::OptionCuTime,
    pub actuation: clock::OptionCuTime,
}

impl CuMsgPayload for RPGpioPayload {}

impl From<RPGpioPayload> for bool {
    fn from(msg: RPGpioPayload) -> Self {
        msg.on
    }
}

impl From<RPGpioPayload> for u8 {
    fn from(msg: RPGpioPayload) -> Self {
        if msg.on {
            1
        } else {
            0
        }
    }
}

#[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
impl From<RPGpioPayload> for Level {
    fn from(msg: RPGpioPayload) -> Self {
        if msg.on {
            Level::Low
        } else {
            Level::High
        }
    }
}

impl Freezable for RPGpio {
    // pin is derived from the config, so we keep the default implementation.
}

impl CuTaskLifecycle for RPGpio {
    fn new(config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let config = config.ok_or("RPGpio needs a config, None was passed as ComponentConfig")?;

        let pin_nb: u8 = (*config.0.get("pin").expect(
            "RPGpio expects a pin config value pointing to output pin you want to address",
        ))
        .clone()
        .into();

        #[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
        let pin = GPIO
            .get(pin_nb)
            .map_err(|e| CuError::new_with_cause("Could not get pin", e))?
            .into_output();
        #[cfg(target_arch = "x86_64")]
        let pin = pin_nb;
        Ok(Self { pin })
    }
}

impl<'cl> CuSinkTask<'cl> for RPGpio {
    type Input = input_msg!('cl, RPGpioPayload);

    fn process(&mut self, clock: &clock::RobotClock, msg: Self::Input) -> CuResult<()> {
        #[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
        self.pin.write(msg.payload.into());
        #[cfg(target_arch = "x86_64")]
        debug!(
            "Would write to pin {} the value {}. Creation to Actuation: {}",
            self.pin,
            msg.payload().unwrap().on,
            clock.now() - msg.payload().unwrap().creation.unwrap()
        );

        Ok(())
    }
}
