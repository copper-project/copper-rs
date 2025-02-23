#[cfg(mock)]
mod mock;

use bincode::{Decode, Encode};
use cu29::clock::{CuDuration, RobotClock};
use cu29::config::ComponentConfig;
use cu29::cutask::{CuMsg, CuSrcTask, Freezable};
use cu29::output_msg;
use cu29::CuResult;
use serde::{Deserialize, Serialize};
use std::sync::{Arc, Mutex};

#[allow(unused_imports)]
use cu29_traits::CuError;

#[cfg(mock)]
use mock::{get_pin, InputPin};
#[cfg(hardware)]
use rppal::gpio::{Gpio, InputPin, Level, Trigger};

#[allow(dead_code)]
struct InterruptData {
    dat_pin: InputPin,
    ticks: i32,
    tov: CuDuration,
}

#[cfg(hardware)]
fn get_pin(pin_nb: u8) -> CuResult<InputPin> {
    // Gpio manages a singleton behind the scene.
    Ok(Gpio::new()
        .expect("Could not create GPIO bindings")
        .get(pin_nb)
        .map_err(|e| CuError::new_with_cause("Could not get pin", e))?
        .into_input())
}

#[derive(Default, Clone, Debug, Encode, Decode, Serialize, Deserialize)]
pub struct EncoderPayload {
    pub ticks: i32,
}

/// That allows the interfacing with the GenericPID
impl From<&EncoderPayload> for f32 {
    fn from(payload: &EncoderPayload) -> f32 {
        payload.ticks as f32
    }
}

#[allow(dead_code)]
pub struct Encoder {
    clk_pin: InputPin,
    data_from_interrupts: Arc<Mutex<InterruptData>>,
}

impl Freezable for Encoder {
    // pin is derived from the config, so we keep the default implementation.
}

impl<'cl> CuSrcTask<'cl> for Encoder {
    type Output = output_msg!('cl, EncoderPayload);

    fn new(config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let ComponentConfig(config) =
            config.ok_or("Encoder needs a config with clk_pin and dat_pin.")?;

        let clk_pin_nb_value = config.get("clk_pin").ok_or("Encoder needs a clk_pin")?;
        let clk_pin: u8 = clk_pin_nb_value.clone().into();

        let dat_pin_nb_value = config.get("dat_pin").ok_or("Encoder needs a dat_pin")?;
        let dat_pin: u8 = dat_pin_nb_value.clone().into();

        let clk_pin: InputPin = get_pin(clk_pin)?;
        let dat_pin: InputPin = get_pin(dat_pin)?;

        Ok(Self {
            clk_pin,
            data_from_interrupts: Arc::new(Mutex::new(InterruptData {
                dat_pin,
                ticks: 0,
                tov: CuDuration::default(),
            })),
        })
    }

    #[allow(unused_variables)]
    fn start(&mut self, clock: &RobotClock) -> CuResult<()> {
        let clock = clock.clone();
        let idata = Arc::clone(&self.data_from_interrupts);
        #[cfg(hardware)]
        self.clk_pin
            .set_async_interrupt(Trigger::FallingEdge, None, move |_| {
                let mut idata = idata.lock().unwrap();
                if idata.dat_pin.read() == Level::Low {
                    idata.ticks -= 1;
                } else {
                    idata.ticks += 1;
                }
                idata.tov = clock.now();
            })
            .map_err(|e| CuError::new_with_cause("Failed to set async interrupt", e))?;
        Ok(())
    }

    fn process(&mut self, clock: &RobotClock, new_msg: Self::Output) -> CuResult<()> {
        let idata = self.data_from_interrupts.lock().unwrap();
        new_msg.metadata.tov = Some(clock.now()).into();
        new_msg.metadata.set_status(idata.ticks);
        new_msg.set_payload(EncoderPayload { ticks: idata.ticks });
        Ok(())
    }
    fn stop(&mut self, _clock: &RobotClock) -> CuResult<()> {
        #[cfg(hardware)]
        self.clk_pin
            .clear_async_interrupt()
            .map_err(|e| CuError::new_with_cause("Failed to reset async interrupt", e))?;
        Ok(())
    }
}
