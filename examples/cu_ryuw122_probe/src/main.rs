use cu_ryuw122_probe::run_cli;

fn main() {
    if let Err(err) = run_cli() {
        eprintln!("cu-ryuw122-probe failed: {err}");
        std::process::exit(1);
    }
}
