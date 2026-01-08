pub mod tasks;

use cu29::prelude::*;
use cu29_export::run_cli;

// This will create the CuStampedDataSet that is specific to your copper project.
// It is used to instruct the log reader how to decode the logs.
gen_cumsgs!("copperconfig.ron");

#[cfg(feature = "logreader")]
fn main() {
    run_cli::<CuStampedDataSet>().expect("Failed to run the export CLI");
}
