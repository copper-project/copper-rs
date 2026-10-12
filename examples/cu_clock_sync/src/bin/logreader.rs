//! Extract the PTP-stamped messages recorded by the example.

use cu29::prelude::*;
gen_cumsgs!("copperconfig.ron");
fn main() {
    cu29_export::run_cli::<CuMsgs>().expect("Clock sync log export failed");
}
