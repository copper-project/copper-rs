use bincode::{Decode, Encode};
use cu29::prelude::*;
#[cfg(hardware)]
use cu29::resource::Owned;
use cu29::resource::{ResourceBindingMap, ResourceBindings, ResourceManager};
use serde::{Deserialize, Serialize};

#[cfg(hardware)]
use cu_linux_resources::LinuxOutputPin;

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Binding {
    Pin,
}

pub struct GpioResources {
    #[cfg(hardware)]
    pub pin: Owned<LinuxOutputPin>,
}

impl<'r> ResourceBindings<'r> for GpioResources {
    type Binding = Binding;

    fn from_bindings(
        manager: &'r mut ResourceManager,
        mapping: Option<&ResourceBindingMap<Self::Binding>>,
    ) -> CuResult<Self> {
        #[cfg(hardware)]
        {
            let mapping = mapping.ok_or_else(|| {
                CuError::from("RPGpio requires a `pin` resource mapping in copperconfig")
            })?;
            let path = mapping.get(Binding::Pin).ok_or_else(|| {
                CuError::from("RPGpio resources must include `pin: <bundle.resource>`")
            })?;
            let pin = manager
                .take::<LinuxOutputPin>(path.typed())
                .map_err(|e| e.add_cause("Failed to fetch GPIO output resource"))?;
            Ok(Self { pin })
        }
        #[cfg(mock)]
        {
            let _ = manager;
            let _ = mapping;
            Ok(Self {})
        }
    }
}

/// Example of a GPIO output driver for the Raspberry Pi
/// On hardware, `pin` is provided by resource bindings (for example `<bundle>.gpio0`).
/// In `mock` mode, the config still accepts `pin` to select the simulated output.
#[derive(Reflect)]
#[reflect(from_reflect = false)]
pub struct RPGpio {
    #[cfg_attr(hardware, reflect(ignore))]
    #[cfg(hardware)]
    pin: LinuxOutputPin,
    #[cfg(mock)]
    pin: u8,
}

#[derive(
    Debug, Clone, Copy, Encode, Decode, Default, PartialEq, Serialize, Deserialize, Reflect,
)]
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
        if msg.on { 1 } else { 0 }
    }
}

impl Freezable for RPGpio {}

impl CuSinkTask for RPGpio {
    type Resources<'r> = GpioResources;
    type Input<'m> = input_msg!(RPGpioPayload);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        #[cfg(hardware)]
        let pin = _resources.pin.0;

        #[cfg(mock)]
        let pin = pin_from_config(_config)?;

        Ok(Self { pin })
    }

    fn process(&mut self, _clock: &RobotClock, msg: &Self::Input<'_>) -> CuResult<()> {
        #[cfg(hardware)]
        {
            // Keep historical active-low behavior for compatibility.
            if msg.payload().unwrap().on {
                self.pin.get_mut().set_low();
            } else {
                self.pin.get_mut().set_high();
            }
        }

        #[cfg(mock)]
        debug!(
            "Would write to pin {} the value {}.",
            self.pin,
            msg.payload()
        );

        Ok(())
    }
}

#[cfg(mock)]
fn pin_from_config(config: Option<&ComponentConfig>) -> CuResult<u8> {
    let ComponentConfig(kv) =
        config.ok_or("RPGpio needs a config, None was passed as ComponentConfig")?;

    let pin_nb: u8 = kv
        .get("pin")
        .ok_or("RPGpio expects a pin config value pointing to output pin you want to address")?
        .clone()
        .into();

    Ok(pin_nb)
}
