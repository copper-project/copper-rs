//! Toyota Vehicle CAN Network Simulation
//!
//! Simulates a Toyota TSS2 ADAS radar ECU transmitting radar track messages
//! on a virtual CAN bus, using signal definitions from the DBC file.
//!
//! The simulation produces all 34 messages defined in `toyota_tss2_adas.dbc`:
//! - TRACK_A_0..15: Primary radar track data (distance, speed, lateral offset)
//! - TRACK_B_0..15: Secondary radar track data (acceleration, confidence score)
//! - NEW_MSG_1, NEW_MSG_2: Miscellaneous ADAS signals
//!
//! All messages include proper Toyota checksums and auto-incrementing counters.

#[allow(dead_code)]
mod dbc_generated;
mod signal_pack;
mod toyota_checksum;
mod ecu_radar;
mod bus_spy;

use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;
use std::fs;
use std::path::{Path, PathBuf};

#[copper_runtime(config = "copperconfig.ron")]
struct VehicleSimApplication {}

const SLAB_SIZE: Option<usize> = Some(64 * 1024 * 1024);

fn main() {
    let logger_path = "logs/vehicle_sim.copper";
    if let Some(parent) = Path::new(logger_path).parent()
        && !parent.exists()
    {
        fs::create_dir_all(parent).expect("Failed to create logs directory");
    }

    let copper_ctx = basic_copper_setup(&PathBuf::from(logger_path), SLAB_SIZE, true, None)
        .expect("Failed to setup logger.");
    let mut application = VehicleSimApplicationBuilder::new()
        .with_context(&copper_ctx)
        .build()
        .expect("Failed to create vehicle simulation application.");

    println!("╔══════════════════════════════════════════════════════════════╗");
    println!("║  Toyota TSS2 ADAS Vehicle CAN Network Simulation           ║");
    println!("║  DBC: toyota_tss2_adas.dbc (34 messages, 16 radar tracks)  ║");
    println!("║  Press Ctrl+C to stop                                      ║");
    println!("╚══════════════════════════════════════════════════════════════╝");

    if let Err(error) = application.run() {
        println!("Simulation ended: {error}");
    }
}
