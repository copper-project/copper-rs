//! YOLOv8 Pose Estimation Demo for Copper
//!
//! This example demonstrates real-time pose estimation using:
//! - GStreamer (V4L2) camera input
//! - YOLOv8-pose model via Candle (HuggingFace ML framework)
//! - Rerun visualization for displaying results
//!
//! # Usage
//!
//! ```bash
//! # CPU inference (default)
//! cargo run -p cu-yolo-human-pose
//!
//! # CUDA inference (requires CUDA toolkit)
//! cargo run -p cu-yolo-human-pose --features cuda
//! ```

mod image;
mod payloads;
mod tasks;
mod yolo;

use std::fs;
use std::path::Path;

use cu29::prelude::*;

// Re-export for RON config visibility
pub use payloads::*;
pub use tasks::*;

const SLAB_SIZE: Option<usize> = Some(1024 * 1024 * 1024); // 1GB

#[copper_runtime(config = "copperconfig.ron")]
struct YoloPoseDemoApplication {}

fn main() {
    // Create temporary directory for Copper logs
    let logger_path = "logs/human-pose.copper";
    if let Some(parent) = Path::new(logger_path).parent()
        && !parent.exists()
    {
        fs::create_dir_all(parent).expect("Failed to create logs directory");
    }

    // Build the application from RON config
    let application = YoloPoseDemoApplication::builder()
        .with_log_path(logger_path, SLAB_SIZE)
        .expect("Failed to set up Copper logging")
        .build_app()
        .expect("Failed to build application");

    // `run` starts the tasks itself; the typestate wrapper rejects the manual
    // start_all_tasks call this example used to make before it.
    if let Err(e) = application.run() {
        error!("Error during iteration: {}", e.error.to_string());
    }
}
