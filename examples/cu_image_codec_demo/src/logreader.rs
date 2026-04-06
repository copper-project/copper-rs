#[cfg(feature = "logreader")]
mod real {
    use cu29::prelude::*;
    use cu29_export::run_cli;

    gen_cumsgs!("copperconfig.ron");

    pub fn run() {
        run_cli::<CuMsgs>().expect("Failed to run the export CLI");
    }
}

#[cfg(feature = "logreader")]
fn main() {
    real::run();
}

#[cfg(not(feature = "logreader"))]
fn main() {
    eprintln!("Enable the `logreader` feature to run the image codec demo logreader.");
    std::process::exit(1);
}
