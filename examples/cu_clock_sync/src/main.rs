//! A Copper application whose RobotClock follows an existing PTP reference.

#[cfg(target_os = "linux")]
use cu29::prelude::*;

#[cfg(target_os = "linux")]
mod tasks;

#[cfg(target_os = "linux")]
#[copper_runtime(config = "copperconfig.ron")]
struct App {}

#[cfg(target_os = "linux")]
fn main() {
    if let Err(error) = run() {
        eprintln!("PTP demo failed: {error}");
        eprintln!("Run `just` from this example to start the local PTP services and monitor.");
        std::process::exit(1);
    }
}

#[cfg(target_os = "linux")]
fn run() -> CuResult<()> {
    use std::io::IsTerminal;

    if !std::io::stdout().is_terminal() || !std::io::stdin().is_terminal() {
        return Err("The live clock demo needs an interactive terminal".into());
    }
    let log_dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("logs");
    std::fs::create_dir_all(&log_dir)
        .map_err(|error| CuError::new_with_cause("Cannot create example log directory", error))?;

    eprintln!("Connecting to the local PTP reference (up to 10 seconds)...");
    eprintln!("The live CLK panel opens when synchronized. Ctrl-C stops the demo.");
    eprintln!("Recording: {}", log_dir.join("ptp.copper").display());

    // The configured PTP reference supplies the clock for every task.
    let app = App::builder()
        .with_log_path(log_dir.join("ptp.copper"), None)?
        .build()?;
    if let Err(failure) = app.run_until_shutdown() {
        let error = failure.error;
        drop(failure.app);
        if error.message() != "Exiting..." {
            return Err(error);
        }
    }
    eprintln!(
        "Recording saved to {}",
        log_dir.join("ptp.copper").display()
    );
    Ok(())
}

#[cfg(not(target_os = "linux"))]
fn main() -> cu29::prelude::CuResult<()> {
    Err("This PTP example requires Linux".into())
}
