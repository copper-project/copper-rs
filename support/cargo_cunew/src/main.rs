use clap::Parser;

fn main() {
    let args = cargo_cunew::normalize_cargo_subcommand_args(std::env::args());
    let cli = cargo_cunew::Cli::parse_from(args);

    if let Err(error) = cargo_cunew::run(cli) {
        eprintln!("error: {error:#}");
        std::process::exit(1);
    }
}
