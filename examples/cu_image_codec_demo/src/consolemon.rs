mod tasks;

use cu29::prelude::*;
use std::io::{IsTerminal, stdin, stdout};
use std::path::PathBuf;

#[copper_runtime(config = "copperconfig.consolemon.ron")]
struct ImageCodecDemoConsolemonApp {}

pub fn main() {
    if !stdout().is_terminal() || !stdin().is_terminal() {
        eprintln!(
            "cu-image-codec-demo-consolemon requires an interactive TTY so cu_consolemon can start its UI."
        );
        std::process::exit(1);
    }

    let log_path = std::env::args_os()
        .nth(1)
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("logs/image_codec_demo_consolemon.copper"));

    if let Some(parent) = log_path.parent() {
        std::fs::create_dir_all(parent).expect("Failed to create log directory");
    }

    let mut application = ImageCodecDemoConsolemonApp::builder()
        .with_log_path(&log_path, Some(64 * 1024 * 1024))
        .expect("Failed to setup Copper log")
        .build()
        .expect("Failed to create application");

    if let Err(err) = application.run() {
        if err.message() != "Exiting..." {
            error!("{err:?}");
        }
    }

    println!("Log written to: {}", log_path.display());
    println!(
        "Inspect with: cargo run -p cu-image-codec-demo --features logreader --bin cu-image-codec-demo-logreader -- {} fsck",
        log_path.display()
    );
}
