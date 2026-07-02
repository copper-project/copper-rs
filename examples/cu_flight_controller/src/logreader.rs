extern crate cu29 as bevy;

mod messages;

use cu29::prelude::*;
use cu29_export::run_cli;

gen_cumsgs!("copperconfig.ron");

fn main() {
    run_cli::<cumsgs::gnss::CuStampedDataSet>().expect("Failed to run the export CLI");
}
