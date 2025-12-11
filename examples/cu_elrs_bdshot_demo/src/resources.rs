use crate::{SerialResource, SerialResourceInner};
use alloc::format;
use cu29::resource::{ResourceBundle, ResourceManager};
use cu29::CuResult;

static SERIAL: spin::Mutex<Option<SerialResourceInner>> = spin::Mutex::new(None);

pub fn stash_crsf_serial(serial: SerialResourceInner) {
    SERIAL.lock().replace(serial);
}

pub struct RadioBundle;

impl ResourceBundle for RadioBundle {
    fn build(
        bundle_id: &str,
        _config: Option<&cu29::config::ComponentConfig>,
        manager: &mut ResourceManager,
    ) -> CuResult<()> {
        let serial = SERIAL
            .lock()
            .take()
            .ok_or_else(|| cu29::CuError::from("CRSF serial port not initialized"))?;
        let serial = SerialResource::new(serial);
        let path = format!("{bundle_id}.serial");
        manager.add_owned(path, serial)
    }
}
