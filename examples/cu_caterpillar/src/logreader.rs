use copper::cutask::CuMsg;
use copper_derive::gen_culist_payload;
use copper_export::run_cli;

type CuListPayload = gen_culist_payload!("copperconfig.ron");

fn main() {
    run_cli::<CuListPayload>().expect("Failed to run the export CLI");
}
