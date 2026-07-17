#[cfg(any(
    not(feature = "jetson-mipi"),
    all(target_os = "linux", target_arch = "aarch64")
))]
mod tasks;

#[cfg(all(
    feature = "jetson-mipi",
    not(all(target_os = "linux", target_arch = "aarch64"))
))]
compile_error!("feature `jetson-mipi` requires a Linux/aarch64 Jetson target");

#[cfg(any(
    not(feature = "jetson-mipi"),
    all(target_os = "linux", target_arch = "aarch64")
))]
mod app {
    use crate::tasks;
    use cu29::prelude::*;
    use std::path::PathBuf;

    #[copper_runtime(config = "copperconfig.ron")]
    struct CameraApp {}

    pub fn run() -> CuResult<()> {
        let log_path =
            PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("logs/feature_gated_camera.copper");
        if let Some(parent) = log_path.parent() {
            std::fs::create_dir_all(parent)
                .map_err(|error| CuError::new_with_cause("creating log directory", error))?;
        }

        let mut app = CameraApp::builder()
            .with_log_path(&log_path, Some(16 * 1024 * 1024))?
            .build()?;
        app.start_all_tasks()?;
        app.run_one_iteration()?;
        app.stop_all_tasks()
    }
}

#[cfg(any(
    not(feature = "jetson-mipi"),
    all(target_os = "linux", target_arch = "aarch64")
))]
fn main() {
    app::run().expect("feature-gated camera example failed");
}

#[cfg(all(
    feature = "jetson-mipi",
    not(all(target_os = "linux", target_arch = "aarch64"))
))]
fn main() {}
