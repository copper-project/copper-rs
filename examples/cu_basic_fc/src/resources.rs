use crate::{SerialPort, SerialPortError};
use cu29::resource::{ResourceBundle, ResourceDecl, ResourceManager};
use cu29::{CuError, CuResult};

pub struct SerialBundle;

impl ResourceBundle for SerialBundle {
    fn build(
        _bundle_id: &str,
        _config: Option<&cu29::config::ComponentConfig>,
        resources: &[ResourceDecl],
        manager: &mut ResourceManager,
    ) -> CuResult<()> {
        let serial = cu_embedded_registry::take::<SerialPort, SerialPortError>(0)
            .ok_or_else(|| CuError::from("CRSF serial slot 0 not registered"))?;
        let key = resources
            .first()
            .ok_or_else(|| CuError::from("Serial bundle missing resource decl"))?
            .key;
        manager.add_owned(key.typed(), serial)
    }
}
