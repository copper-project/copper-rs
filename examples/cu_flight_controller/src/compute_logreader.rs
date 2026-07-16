extern crate cu29 as bevy;

mod messages;

use cu29::prelude::*;
use cu29_export::run_cli;

// The simulator and deployed compute runtime use the same auto-flying graph.
gen_cumsgs!("compute_end2end_config.ron");

fn main() {
    run_cli::<CuStampedDataSet>().expect("Failed to run the compute export CLI");
}
