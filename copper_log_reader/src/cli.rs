use clap::{Parser, Subcommand};
use std::path::PathBuf;

#[derive(Parser)]
#[command(version, about, long_about = None)]
struct Opts {
    #[arg(short, long)]
    index: Option<PathBuf>,

    datalog: PathBuf,
}

fn main() {
    let opts: Opts = Opts::parse();

    let index = if let Some(index) = opts.index {
        index
    } else {
        // FIXME: This is a convienence for development, remove this
        PathBuf::from("../target/debug/copper_log_index")
    };

    // Now you can use the string index
    println!("Value for index: {}", index.to_str().unwrap());
}
