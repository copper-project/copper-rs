use crate::{GreenLed, SerialPort};
use alloc::string::String;
use cu29::resource::{ResourceBundle, ResourceDecl, ResourceKey, ResourceManager};
use cu29::{CuError, CuResult};
use spin::Mutex;

static SERIAL: Mutex<Option<SerialPort>> = Mutex::new(None);
static GREEN_LED: Mutex<Option<GreenLed>> = Mutex::new(None);

pub fn stash_serial(serial: SerialPort) {
    SERIAL.lock().replace(serial);
}

pub fn stash_green_led(led: GreenLed) {
    GREEN_LED.lock().replace(led);
}

fn lookup(resources: &[ResourceDecl], bundle: &str, name: &str) -> CuResult<ResourceKey> {
    let mut path = String::new();
    path.push_str(bundle);
    path.push('.');
    path.push_str(name);
    resources
        .iter()
        .find(|decl| decl.path == path)
        .map(|decl| decl.key)
        .ok_or_else(|| CuError::from("Resource not declared"))
}

pub struct MicoAirH743;

impl ResourceBundle for MicoAirH743 {
    fn build(
        bundle_id: &str,
        _config: Option<&cu29::config::ComponentConfig>,
        resources: &[ResourceDecl],
        manager: &mut ResourceManager,
    ) -> CuResult<()> {
        let serial = SERIAL
            .lock()
            .take()
            .ok_or_else(|| CuError::from("MicoAirH743 serial not initialized"))?;
        let led = GREEN_LED
            .lock()
            .take()
            .ok_or_else(|| CuError::from("MicoAirH743 green LED not initialized"))?;

        let serial_key = lookup(resources, bundle_id, "serial")?.typed();
        let led_key = lookup(resources, bundle_id, "green_led")?.typed();

        manager.add_owned(serial_key, serial)?;
        manager.add_owned(led_key, Mutex::new(led))?;
        Ok(())
    }
}
