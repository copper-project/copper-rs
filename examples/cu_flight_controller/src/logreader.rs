extern crate cu29 as bevy;

mod messages;

use cu29::prelude::*;
use cu29_export::run_cli;

// The simulator logs the MCU side of the closed-loop autonomy graph.
// Keep this schema aligned with `sim.rs` rather than the legacy mission config.
gen_cumsgs!("mcu_config.ron");

fn main() {
    run_cli::<CuStampedDataSet>().expect("Failed to run the export CLI");
}
