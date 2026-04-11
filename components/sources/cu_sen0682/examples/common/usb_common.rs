use cu29::prelude::*;
use std::path::{Path, PathBuf};

pub fn change_to_manifest_dir() -> CuResult<()> {
    std::env::set_current_dir(env!("CARGO_MANIFEST_DIR")).map_err(|err| {
        CuError::new_with_cause("failed to switch to cu_sen0682 crate directory", err)
    })
}

pub fn resolve_usb_serial_port(port: Option<PathBuf>) -> CuResult<PathBuf> {
    match port {
        Some(port) => Ok(port),
        None => detect_usb_serial_port(),
    }
}

pub fn override_serial_resource(config: &mut cu29::config::CuConfig, port: &Path) {
    let Some(bundle) = config
        .resources
        .iter_mut()
        .find(|bundle| bundle.id == "linux")
    else {
        return;
    };
    let bundle_config = bundle.config.get_or_insert_with(ComponentConfig::new);
    bundle_config.set("serial0_dev", port.display().to_string());
    bundle_config.set("serial0_baudrate", 921_600u64);
    bundle_config.set("serial0_timeout_ms", 20u64);
}

fn detect_usb_serial_port() -> CuResult<PathBuf> {
    let dev_dir = Path::new("/dev");
    let mut candidates = std::fs::read_dir(dev_dir)
        .map_err(|err| CuError::new_with_cause("failed to scan /dev for USB serial devices", err))?
        .filter_map(|entry| entry.ok())
        .filter_map(|entry| {
            let name = entry.file_name();
            let name = name.to_string_lossy();
            if name.starts_with("ttyACM") || name.starts_with("ttyUSB") {
                Some(dev_dir.join(name.as_ref()))
            } else {
                None
            }
        })
        .collect::<Vec<_>>();

    candidates.sort();

    match candidates.len() {
        0 => Err(CuError::from(
            "no /dev/ttyACM* or /dev/ttyUSB* device found; plug the SEN0682 in or pass --port",
        )),
        1 => Ok(candidates.remove(0)),
        _ => Err(CuError::from(format!(
            "multiple USB serial devices found ({}); pass --port explicitly",
            candidates
                .iter()
                .map(|path| path.display().to_string())
                .collect::<Vec<_>>()
                .join(", ")
        ))),
    }
}
