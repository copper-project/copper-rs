use bincode::{Decode, Encode};
use cu29::prelude::*;
use serde::{Deserialize, Serialize};

#[cfg(hardware)]
use {
    cu29::CuError,
    rppal::gpio::{Gpio, Level, OutputPin},
    std::sync::OnceLock,
};

#[cfg(hardware)]
static GPIO: OnceLock<Gpio> = OnceLock::new();

#[cfg(hardware)]
fn gpio() -> &'static Gpio {
    GPIO.get_or_init(|| Gpio::new().expect("Could not create GPIO bindings"))
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

impl<'cl> CuSinkTask<'cl> for RPGpio {
    type Input = input_msg!('cl, RPGpioPayload);

    fn new(config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let ComponentConfig(kv) =
            config.ok_or("RPGpio needs a config, None was passed as ComponentConfig")?;

        let pin_nb: u8 = kv
            .get("pin")
            .expect("RPGpio expects a pin config value pointing to output pin you want to address")
            .clone()
            .into();

        #[cfg(hardware)]
        let pin = gpio()
            .get(pin_nb)
            .map_err(|e| CuError::new_with_cause("Could not get pin", e))?
            .into_output();
        #[cfg(mock)]
        let pin = pin_nb;
        Ok(Self { pin })
    }

    fn process(&mut self, _clock: &RobotClock, msg: Self::Input) -> CuResult<()> {
        #[cfg(hardware)]
        self.pin.write((*msg.payload().unwrap()).into());

        #[cfg(mock)]
        debug!(
            "Would write to pin {} the value {}.",
            self.pin,
            msg.payload()
        );

        Ok(())
    }
}
