//! Standard export CLI over this app's `.copper` logs: fsck, extract-copperlists,
//! extract-text-log, export-mcap.

use cu_autoware::payload;
use cu29::prelude::*;
use cu29_export::run_cli;

gen_cumsgs!("copperconfig.ron");

fn main() {
    run_cli::<CuMsgs>().expect("Failed to run the export CLI");
}
