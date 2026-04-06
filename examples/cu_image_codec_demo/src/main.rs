#[cfg(feature = "ffv1")]
pub mod tasks;

#[cfg(feature = "ffv1")]
mod real {
    use crate::tasks;
    use cu29::prelude::*;
    use std::path::PathBuf;

    #[copper_runtime(config = "copperconfig.ron")]
    struct ImageCodecDemoApp {}

    pub fn run() {
        let log_path = std::env::args_os()
            .nth(1)
            .map(PathBuf::from)
            .unwrap_or_else(|| PathBuf::from("logs/image_codec_demo.copper"));

        if let Some(parent) = log_path.parent() {
            std::fs::create_dir_all(parent).expect("Failed to create log directory");
        }

        let mut application = ImageCodecDemoApp::builder()
            .with_log_path(&log_path, Some(64 * 1024 * 1024))
            .expect("Failed to setup Copper log")
            .build()
            .expect("Failed to create application");
        application
            .start_all_tasks()
            .expect("Failed to start tasks");

        for _ in 0..180 {
            application
                .run_one_iteration()
                .expect("Failed to run iteration");
        }

        println!("Log written to: {}", log_path.display());
        println!(
            "Inspect with: cargo run -p cu-image-codec-demo --features logreader --bin cu-image-codec-demo-logreader -- {} fsck",
            log_path.display()
        );
    }
}

#[cfg(feature = "ffv1")]
fn main() {
    real::run();
}

#[cfg(not(feature = "ffv1"))]
fn main() {
    eprintln!("Enable the `ffv1` feature to run cu-image-codec-demo.");
    std::process::exit(1);
}
