use copper::config::NodeInstanceConfig;
use copper::cutask::{CuMsg, CuSinkTask, CuTaskLifecycle};
use copper::CuResult;
use copper_log_derive::debug;

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
pub struct RPGpio {
    #[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
    pin: OutputPin,
    #[cfg(target_arch = "x86_64")]
    pin: u8,
}

#[derive(
    Debug, Clone, Copy, Default, bincode_derive::Encode, bincode_derive::Decode, PartialEq,
)]
pub struct RPGpioMsg {
    pub on: bool,
    pub creation: copper::clock::OptionCuTime,
    pub actuation: copper::clock::OptionCuTime,
}

impl From<RPGpioMsg> for bool {
    fn from(msg: RPGpioMsg) -> Self {
        msg.on
    }
}

impl From<RPGpioMsg> for u8 {
    fn from(msg: RPGpioMsg) -> Self {
        if msg.on {
            1
        } else {
            0
        }
    }
}

#[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
impl From<RPGpioMsg> for Level {
    fn from(msg: RPGpioMsg) -> Self {
        if msg.on {
            Level::Low
        } else {
            Level::High
        }
    }
}

impl CuTaskLifecycle for RPGpio {
    fn new(config: Option<&NodeInstanceConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let config =
            config.ok_or("RPGpio needs a config, None was passed as NodeInstanceConfig")?;

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

impl CuSinkTask for RPGpio {
    type Input = RPGpioMsg;

    fn process(
        &mut self,
        clock: &copper::clock::RobotClock,
        msg: &mut CuMsg<Self::Input>,
    ) -> CuResult<()> {
        msg.payload.actuation = clock.now().into();
        #[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
        self.pin.write(msg.payload.into());
        #[cfg(target_arch = "x86_64")]
        debug!(
            "Would write to pin {} the value {}. Creation to Actuation: {}",
            self.pin,
            msg.payload.on,
            msg.payload.actuation.unwrap() - msg.payload.creation.unwrap()
        );

        Ok(())
    }
}
