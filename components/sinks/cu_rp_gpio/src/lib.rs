use bincode::{Decode, Encode};
use cu29::config::ComponentConfig;
use cu29::cutask::{CuMsg, CuSinkTask, CuTaskLifecycle, Freezable};
use cu29::CuResult;
use cu29::{clock, input_msg};
use serde::{Deserialize, Serialize};

#[cfg(mock)]
use cu29_log_derive::debug;

#[cfg(hardware)]
use {
    cu29::CuError,
    lazy_static::lazy_static,
    rppal::gpio::{Gpio, Level, OutputPin},
};

#[cfg(hardware)]
lazy_static! {
    static ref GPIO: Gpio = Gpio::new().expect("Could not create GPIO bindings");
}

/// Example of a GPIO output driver for the Raspberry Pi
/// The config takes one config value: `pin` which is the pin you want to address
/// Gpio uses BCM pin numbering. For example: BCM GPIO 23 is tied to physical pin 16.
pub struct RPGpio {
    #[cfg(hardware)]
    pin: OutputPin,
    #[cfg(mock)]
    pin: u8,
}

#[derive(Debug, Clone, Copy, Encode, Decode, Default, PartialEq, Serialize, Deserialize)]
pub struct RPGpioPayload {
    pub on: bool,
}

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

#[cfg(hardware)]
impl From<RPGpioPayload> for Level {
    fn from(msg: RPGpioPayload) -> Self {
        if msg.on {
            Level::Low
        } else {
            Level::High
        }
    }
}

impl Freezable for RPGpio {}

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

        #[cfg(not(feature = "mock"))]
        let pin = GPIO
            .get(pin_nb)
            .map_err(|e| CuError::new_with_cause("Could not get pin", e))?
            .into_output();
        #[cfg(mock)]
        let pin = pin_nb;
        Ok(Self { pin })
    }
}

impl<'cl> CuSinkTask<'cl> for RPGpio {
    type Input = input_msg!('cl, RPGpioPayload);

    fn process(&mut self, _clock: &clock::RobotClock, msg: Self::Input) -> CuResult<()> {
        #[cfg(hardware)]
        self.pin.write((*msg.payload().unwrap()).into());

        #[cfg(mock)]
        debug!(
            "Would write to pin {} the value {:?}.",
            self.pin,
            msg.payload()
        );

        Ok(())
    }
}
