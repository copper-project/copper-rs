extern crate cu29 as bevy;

mod messages;

use cu29::prelude::*;
use cu29_export::run_cli;

// The simulator records the compute subsystem independently from the MCU.
// Keep this schema aligned with the compute side of `multi_copper_vitfly_sim.ron`.
gen_cumsgs!("compute_vitfly_config.ron");

fn main() {
    run_cli::<CuStampedDataSet>().expect("Failed to run the compute export CLI");
}
