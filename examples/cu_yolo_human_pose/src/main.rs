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

mod payloads;
mod tasks;
mod yolo;

use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;

// Re-export for RON config visibility
pub use payloads::*;
pub use tasks::*;

const SLAB_SIZE: Option<usize> = Some(128 * 1024 * 1024); // 128 MiB

#[copper_runtime(config = "copperconfig.ron")]
struct YoloPoseDemoApplication {}

fn main() {
    // Set up logging
    env_logger::init();

    // Create temporary directory for Copper logs
    let tmp_dir = tempfile::TempDir::new().expect("Could not create temp directory");
    let logger_path = tmp_dir.path().join("yolo_pose.copper");

    println!("YOLOv8 Pose Estimation Demo");
    println!("===========================");
    println!();
    println!("Starting Copper runtime...");

    // Initialize Copper context
    let copper_ctx = basic_copper_setup(&logger_path, SLAB_SIZE, true, None)
        .expect("Failed to set up Copper context");

    println!("Building application...");

    // Build the application from RON config
    let mut application = YoloPoseDemoApplicationBuilder::new()
        .with_context(&copper_ctx)
        .build()
        .expect("Failed to build application");

    println!("Starting tasks...");

    // Start all tasks
    application
        .start_all_tasks()
        .expect("Failed to start tasks");

    println!();
    println!("Application running!");
    println!("Open Rerun viewer to see pose estimation results.");
    println!("Press Ctrl+C to stop.");
    println!();

    // Run the main loop
    loop {
        match application.run_one_iteration() {
            Ok(_) => {}
            Err(e) => {
                eprintln!("Error during iteration: {:?}", e);
                break;
            }
        }
    }
}
