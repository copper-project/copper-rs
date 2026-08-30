use cu29::prelude::*;
use cu29_export::run_cli;

pub mod payloads;

gen_cumsgs!("copperconfig.ron");

fn main() {
    run_cli::<cumsgs::log_only::CuStampedDataSet>()
        .expect("Failed to run the Mandelbrot export CLI");
}
