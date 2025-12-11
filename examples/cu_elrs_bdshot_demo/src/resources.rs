use crate::{SerialResource, SerialResourceInner};
use cu29::resource::{ResourceBundle, ResourceDecl, ResourceManager};
use cu29::CuResult;

static SERIAL: spin::Mutex<Option<SerialResourceInner>> = spin::Mutex::new(None);

pub fn stash_crsf_serial(serial: SerialResourceInner) {
    SERIAL.lock().replace(serial);
}

pub struct RadioBundle;

impl ResourceBundle for RadioBundle {
    fn build(
        _bundle_id: &str,
        _config: Option<&cu29::config::ComponentConfig>,
        resources: &[ResourceDecl],
        manager: &mut ResourceManager,
    ) -> CuResult<()> {
        let serial = SERIAL
            .lock()
            .take()
            .ok_or_else(|| cu29::CuError::from("CRSF serial port not initialized"))?;
        let serial = SerialResource::new(serial);
        let key = resources
            .first()
            .ok_or_else(|| cu29::CuError::from("Radio bundle missing resource decl"))?
            .key;
        manager.add_owned(key.typed(), serial)
    }
}
