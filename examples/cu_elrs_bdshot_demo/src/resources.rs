use crate::{SerialResource, SerialResourceInner};
use cu29::resource::{ResourceBundle, ResourceManager};
use cu29::{CuResult, bundle_resources};

static SERIAL: spin::Mutex<Option<SerialResourceInner>> = spin::Mutex::new(None);

pub fn stash_crsf_serial(serial: SerialResourceInner) {
    SERIAL.lock().replace(serial);
}

pub struct RadioBundle;

bundle_resources!(RadioBundle: Serial);

impl ResourceBundle for RadioBundle {
    fn build(
        bundle: cu29::resource::BundleContext<Self>,
        _config: Option<&cu29::config::ComponentConfig>,
        manager: &mut ResourceManager,
    ) -> CuResult<()> {
        let serial = SERIAL
            .lock()
            .take()
            .ok_or_else(|| cu29::CuError::from("CRSF serial port not initialized"))?;
        let serial = SerialResource::new(serial);
        let key = bundle.key(RadioBundleId::Serial);
        manager.add_owned(key, serial)
    }
}
