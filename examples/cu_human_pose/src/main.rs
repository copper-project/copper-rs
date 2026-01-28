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
use std::path::PathBuf;

use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;

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

    // Initialize Copper context
    let copper_ctx = basic_copper_setup(&PathBuf::from(logger_path), SLAB_SIZE, true, None)
        .expect("Failed to set up Copper context");

    // Build the application from RON config
    let mut application = YoloPoseDemoApplicationBuilder::new()
        .with_context(&copper_ctx)
        .build()
        .expect("Failed to build application");

    // Start all tasks
    application
        .start_all_tasks()
        .expect("Failed to start tasks");

    if let Err(e) = application.run() {
        error!("Error during iteration: {}", e.to_string());
    }
}
