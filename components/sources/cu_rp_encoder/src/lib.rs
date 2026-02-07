#[cfg(mock)]
mod mock;

use bincode::{Decode, Encode};
use cu29::CuResult;
use cu29::prelude::*;
#[cfg(hardware)]
use cu29::resource::Owned;
use cu29::resource::{ResourceBindingMap, ResourceBindings, ResourceManager};
use std::sync::{Arc, Mutex};

#[allow(unused_imports)]
use cu29_traits::CuError;

#[cfg(hardware)]
use cu_linux_resources::LinuxInputPin;
#[cfg(mock)]
use mock::{InputPin, get_pin};
#[cfg(hardware)]
use rppal::gpio::{Gpio, Level, Trigger};
use serde::Deserialize;

#[cfg(hardware)]
type InputPin = LinuxInputPin;

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Binding {
    ClkPin,
    DatPin,
}

pub struct EncoderResources {
    #[cfg(hardware)]
    pub clk_pin: Option<Owned<InputPin>>,
    #[cfg(hardware)]
    pub dat_pin: Option<Owned<InputPin>>,
}

impl Default for EncoderResources {
    fn default() -> Self {
        Self {
            #[cfg(hardware)]
            clk_pin: None,
            #[cfg(hardware)]
            dat_pin: None,
        }
    }
}

impl<'r> ResourceBindings<'r> for EncoderResources {
    type Binding = Binding;

    fn from_bindings(
        manager: &'r mut ResourceManager,
        mapping: Option<&ResourceBindingMap<Self::Binding>>,
    ) -> CuResult<Self> {
        #[cfg(hardware)]
        {
            let clk_pin = match mapping.and_then(|m| m.get(Binding::ClkPin)) {
                Some(path) => Some(
                    manager
                        .take::<InputPin>(path.typed())
                        .map_err(|e| e.add_cause("Failed to fetch encoder clk pin resource"))?,
                ),
                None => None,
            };
            let dat_pin = match mapping.and_then(|m| m.get(Binding::DatPin)) {
                Some(path) => Some(
                    manager
                        .take::<InputPin>(path.typed())
                        .map_err(|e| e.add_cause("Failed to fetch encoder dat pin resource"))?,
                ),
                None => None,
            };
            Ok(Self { clk_pin, dat_pin })
        }
        #[cfg(mock)]
        {
            let _ = manager;
            let _ = mapping;
            Ok(Self {})
        }
    }
}

#[allow(dead_code)]
struct InterruptData {
    dat_pin: InputPin,
    ticks: i32,
    tov: CuDuration,
}

#[cfg(hardware)]
fn get_pin(pin_nb: u8) -> CuResult<InputPin> {
    // Gpio manages a singleton behind the scene.
    let pin = Gpio::new()
        .expect("Could not create GPIO bindings")
        .get(pin_nb)
        .map_err(|e| CuError::new_with_cause("Could not get pin", e))?
        .into_input();
    Ok(InputPin::new(pin))
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
    // pin is derived from the config/resources, so we keep the default implementation.
}

impl CuSrcTask for Encoder {
    type Resources<'r> = EncoderResources;
    type Output<'m> = output_msg!(EncoderPayload);

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        #[cfg(hardware)]
        let clk_pin: InputPin = match _resources.clk_pin {
            #[cfg(hardware)]
            Some(clk_pin) => clk_pin.0,
            _ => {
                let clk_pin = pin_from_config(config, "clk_pin")?;
                get_pin(clk_pin)?
            }
        };

        #[cfg(mock)]
        let clk_pin: InputPin = {
            let clk_pin = pin_from_config(config, "clk_pin")?;
            get_pin(clk_pin)?
        };

        #[cfg(hardware)]
        let dat_pin: InputPin = match _resources.dat_pin {
            #[cfg(hardware)]
            Some(dat_pin) => dat_pin.0,
            _ => {
                let dat_pin = pin_from_config(config, "dat_pin")?;
                get_pin(dat_pin)?
            }
        };

        #[cfg(mock)]
        let dat_pin: InputPin = {
            let dat_pin = pin_from_config(config, "dat_pin")?;
            get_pin(dat_pin)?
        };

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
            .get_mut()
            .set_async_interrupt(Trigger::FallingEdge, None, move |_| {
                let mut idata = idata.lock().unwrap();
                if idata.dat_pin.get_mut().read() == Level::Low {
                    idata.ticks -= 1;
                } else {
                    idata.ticks += 1;
                }
                idata.tov = clock.now();
            })
            .map_err(|e| CuError::new_with_cause("Failed to set async interrupt", e))?;
        Ok(())
    }

    fn process(&mut self, clock: &RobotClock, new_msg: &mut Self::Output<'_>) -> CuResult<()> {
        let idata = self.data_from_interrupts.lock().unwrap();
        new_msg.tov = Some(clock.now()).into();
        new_msg.metadata.set_status(idata.ticks);
        new_msg.set_payload(EncoderPayload { ticks: idata.ticks });
        Ok(())
    }
    fn stop(&mut self, _clock: &RobotClock) -> CuResult<()> {
        #[cfg(hardware)]
        self.clk_pin
            .get_mut()
            .clear_async_interrupt()
            .map_err(|e| CuError::new_with_cause("Failed to reset async interrupt", e))?;
        Ok(())
    }
}

fn pin_from_config(config: Option<&ComponentConfig>, key: &str) -> CuResult<u8> {
    let config = config.ok_or("Encoder needs a config with clk_pin and dat_pin.")?;
    config
        .get::<u8>(key)?
        .ok_or_else(|| CuError::from(format!("Encoder needs a {key}")))
}
