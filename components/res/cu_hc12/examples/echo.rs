//! Run on one end of a radio pair; echoes incoming bytes to the other end.
#[cfg(unix)]
mod app {
    use cu_linux_resources::{LinuxNonblockingSerialPort, LinuxSerialRtsPin};
    use cu29::prelude::*;
    type RadioResources = cu_hc12::Hc12Resources<
        LinuxNonblockingSerialPort,
        LinuxSerialRtsPin,
        cu_hc12::host::StartupDelay,
    >;
    type Radio = cu_hc12::Hc12<LinuxNonblockingSerialPort, LinuxSerialRtsPin>;
    type RadioBridge = cu_serial_bridge::SerialBridge<Radio>;
    #[copper_runtime(config = "examples/echo.ron")]
    struct Echo {}
    pub fn run() -> CuResult<()> {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("logs/hc12-echo.copper");
        std::fs::create_dir_all(path.parent().unwrap())
            .map_err(|e| CuError::new_with_cause("Create example log directory", e))?;
        let app = Echo::builder()
            .with_log_path(&path, Some(1024 * 1024))?
            .build()?;
        app.run_until_shutdown()
            .map(|_| ())
            .map_err(|failure| failure.error)
    }
}
#[cfg(unix)]
fn main() -> cu29::CuResult<()> {
    app::run()
}
#[cfg(not(unix))]
fn main() {
    eprintln!("This wiring example uses Unix serial RTS resources.");
}
