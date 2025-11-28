#![cfg(all(feature = "std", feature = "linux-embedded"))]

use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;
use cu_mpu9250::{app::ImuLogSink, ImuPayload, LinuxMpu9250Source};

#[copper_runtime(config = "copperconfig.ron")]
struct Mpu9250Logger {}

fn main() {
    use std::path::PathBuf;

    const SLAB_SIZE: Option<usize> = Some(8 * 1024 * 1024);
    let log_path = PathBuf::from("logs/mpu9250.copper");
    let copper_ctx =
        basic_copper_setup(&log_path, SLAB_SIZE, true, None).expect("Failed to setup logger.");

    let clock = copper_ctx.clock.clone();
    let mut app = Mpu9250Logger::new(clock, copper_ctx.unified_logger.clone(), None)
        .expect("Failed to create MPU9250 logger runtime");

    loop {
        app.run_one_iteration()
            .expect("Failed to run logger iteration");
    }
}
