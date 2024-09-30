pub mod tasks;

use cu29::cutask::CuMsg as _CuMsg;
use cu29_derive::gen_cumsgs;
use cu29_export::run_cli;

/// This will create the CuMsgs that is specific to your copper project.
/// It is used to instruct the log reader how to decode the logs.
gen_cumsgs!("copperconfig.ron");

fn main() {
    run_cli::<CuMsgs>().expect("Failed to run the export CLI");
}
