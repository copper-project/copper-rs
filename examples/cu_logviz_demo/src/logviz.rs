pub mod tasks;

use cu29::prelude::*;
use cu29_logviz::run_cli;

// This will create the CuStampedDataSet that is specific to this example.
// It is used to instruct logviz how to decode the logs.
gen_cumsgs!("copperconfig.ron");

#[cfg(feature = "logviz")]
fn main() {
    run_cli::<CuStampedDataSet>().expect("Failed to run the logviz CLI");
}
