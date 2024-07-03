mod config;
use clap::{Parser, Subcommand};
use config::read_configuration;
pub use copper_traits::*;
use graphviz_rust::cmd::Format;
use graphviz_rust::exec;
use graphviz_rust::parse;
use graphviz_rust::printer::PrinterContext;
use std::io::Cursor;
use std::io::Write;
use std::path::PathBuf;
use std::process::Command;
use tempfile::NamedTempFile;

#[derive(Parser)]
#[clap(author, version, about, long_about = None)]
struct Args {
    /// Config file name
    #[clap(value_parser)]
    config: PathBuf,
    /// Open the SVG in the default system viewer
    #[clap(long)]
    open: bool,
}

fn main() -> std::io::Result<()> {
    // Parse command line arguments
    let args = Args::parse();

    let config = read_configuration(args.config.to_str().unwrap())
        .expect("Failed to read configuration file");
    let mut content = Vec::<u8>::new();
    {
        let mut cursor = Cursor::new(&mut content);
        config.render(&mut cursor);
    }
    // Parse the DOT content
    let graph = parse(String::from_utf8(content).unwrap().as_str()).unwrap();

    // Convert the graph to SVG content
    let graph_svg = exec(
        graph,
        &mut PrinterContext::default(),
        vec![Format::Svg.into()],
    )
    .unwrap();

    if args.open {
        // Create a temporary file to store the SVG
        let mut temp_file = NamedTempFile::new()?;
        temp_file.write_all(graph_svg.as_slice())?;

        // Open the SVG in the default system viewer
        Command::new("xdg-open")
            .arg(temp_file.path())
            .status()
            .expect("failed to open SVG file");
    } else {
        // Write the SVG content to a file
        let mut svg_file = std::fs::File::create("output.svg")?;
        svg_file.write_all(graph_svg.as_slice())?;
    }
    Ok(())
}
