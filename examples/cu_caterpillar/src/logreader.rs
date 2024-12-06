use cu29_export::run_cli;
use cu29_prelude::gen_cumsgs;

gen_cumsgs!("copperconfig.ron");

fn main() {
    run_cli::<CuMsgs>().expect("Failed to run the export CLI");
}
