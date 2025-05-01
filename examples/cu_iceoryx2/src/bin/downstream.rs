use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;
use iceoryx2_bb_log::{set_log_level, LogLevel};
use std::path::PathBuf;

#[copper_runtime(config = "downstream.ron")]
struct DownstreamApplication {}

const SLAB_SIZE: Option<usize> = Some(100 * 1024 * 1024);

fn main() {
    // the iceoryx logger does break if you attach a debugger to it.
    set_log_level(LogLevel::Fatal);
    let logger_path = "/tmp/downstream.copper";
    let copper_ctx = basic_copper_setup(
        &PathBuf::from(logger_path),
        SLAB_SIZE,
        false,
        None,
        Some("downstream.ron"),
    )
    .expect("Failed to setup logger.");
    let mut application = DownstreamApplicationBuilder::new()
        .with_context(&copper_ctx)
        .build()
        .expect("Failed to create application.");

    let outcome = application.run();
    match outcome {
        Ok(_result) => {}
        Err(error) => {
            debug!("Application Ended: {}", error)
        }
    }
}
