//! Standalone catalog-backed Copper log tooling.
fn main() {
    if let Err(error) = cu29_export::run_catalog_cli() {
        eprintln!("{error}");
        std::process::exit(1);
    }
}
