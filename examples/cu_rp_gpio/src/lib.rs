use std::thread::sleep;
use copper::config::NodeInstanceConfig;
use copper::cutask::{CuMsg, CuSinkTask, CuTaskLifecycle};
use copper::{CuError, CuResult};

use rppal::gpio::{Gpio, Level, OutputPin};
use lazy_static::lazy_static;

lazy_static! {
    static ref GPIO: Gpio = Gpio::new().expect("Could not create GPIO bindings");
}

/// Example of a GPIO output driver for the Raspberry Pi
/// The config takes one config value: `pin` which is the pin you want to address
/// Gpio uses BCM pin numbering. For example: BCM GPIO 23 is tied to physical pin 16.
pub struct RPGpio {
    #[cfg(target_arch = "arm")]
    pin: OutputPin,
    #[cfg(target_arch = "x86_64")]
    pin: u8,
}

#[derive(Debug, Clone, Copy, Default, serde::Serialize, serde::Deserialize, PartialEq)]
pub struct RPGpioMsg(pub bool);

impl From<RPGpioMsg> for bool {
    fn from(msg: RPGpioMsg) -> Self {
        msg.0
    }
}

impl From<RPGpioMsg> for u8 {
    fn from(msg: RPGpioMsg) -> Self {
        if msg.0 {
            1
        } else {
            0
        }
    }
}

impl From<RPGpioMsg> for Level {
    fn from(msg: RPGpioMsg) -> Self {
        if msg.0 {
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

        let pin_nb: u8 = (*config.get("pin").expect(
            "RPGpio expects a pin config value pointing to output pin you want to address",
        ))
        .clone()
        .into();

        #[cfg(target_arch = "arm")]
        let pin = GPIO.get(pin_nb)
            .map_err(|e| CuError::new_with_cause("Could not get pin", e))?
            .into_output();
        #[cfg(target_arch = "x86_64")]
        let pin = pin_nb;
        Ok(Self { pin })
    }
}

impl CuSinkTask for RPGpio {
    type Input = RPGpioMsg;

    fn process(&mut self, msg: &CuMsg<Self::Input>) -> CuResult<()> {
        #[cfg(target_arch = "arm")]
        self.pin.write(msg.payload.into());
        sleep(std::time::Duration::from_millis(1000));
        #[cfg(target_arch = "x86_64")]
        println!("Would write to pin {} the value {}", self.pin, msg.payload.0);
        Ok(())
    }
}
