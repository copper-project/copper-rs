mod config;
use clap::Parser;
use config::{read_configuration, ConfigGraphs};
pub use cu29_traits::*;
use std::io::Cursor;
use std::io::Write;
use std::path::PathBuf;
use std::process::{Command, Stdio};
use tempfile::Builder;

#[derive(Parser)]
#[clap(author, version, about, long_about = None)]
struct Args {
    /// Config file name
    #[clap(value_parser)]
    config: PathBuf,
    /// Mission id to render (omit to render every mission)
    #[clap(long)]
    mission: Option<String>,
    /// List missions contained in the configuration and exit
    #[clap(long, action)]
    list_missions: bool,
    /// Open the SVG in the default system viewer
    #[clap(long)]
    open: bool,
}

/// Render the configuration file to a dot file then convert it to an SVG and optionally opens it with inkscape.
fn main() -> std::io::Result<()> {
    // Parse command line arguments
    let args = Args::parse();

    let config = read_configuration(args.config.to_str().unwrap())
        .expect("Failed to read configuration file");

    if args.list_missions {
        print_mission_list(&config);
        return Ok(());
    }

    let mission = match validate_mission_arg(&config, args.mission.as_deref()) {
        Ok(mission) => mission,
        Err(err) => {
            eprintln!("{err}");
            std::process::exit(1);
        }
    };

    let mut content = Vec::<u8>::new();
    {
        let mut cursor = Cursor::new(&mut content);
        config.render(&mut cursor, mission.as_deref()).unwrap();
    }

    // Generate SVG from DOT
    let mut child = Command::new("dot")
        .arg("-Tsvg")
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .spawn()
        .expect("Failed to start dot process");

    {
        let stdin = child.stdin.as_mut().expect("Failed to open stdin");
        let result = stdin.write_all(&content);
        if let Err(e) = result {
            eprintln!("Failed to write to stdin of the dot process: {e}");
            std::process::exit(1);
        }
    }

    let output = child.wait_with_output().expect("Failed to read stdout");

    if !output.status.success() {
        std::process::exit(1);
    }

    let graph_svg = output.stdout;
    if args.open {
        // Create a temporary file to store the SVG
        let mut temp_file = Builder::new().suffix(".svg").tempfile()?;
        temp_file.write_all(graph_svg.as_slice())?;

        // Open the SVG in the default system viewer
        Command::new("inkscape") // xdg-open fails silently (while it works from a standard bash on the same file :shrug:)
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

fn validate_mission_arg(
    config: &config::CuConfig,
    requested: Option<&str>,
) -> CuResult<Option<String>> {
    match (&config.graphs, requested) {
        (ConfigGraphs::Simple(_), None) => Ok(None),
        (ConfigGraphs::Simple(_), Some("default")) => Ok(None),
        (ConfigGraphs::Simple(_), Some(id)) => Err(CuError::from(format!(
            "Config is not mission-based; remove --mission (received '{id}')"
        ))),
        (ConfigGraphs::Missions(graphs), Some(id)) => {
            if graphs.contains_key(id) {
                Ok(Some(id.to_string()))
            } else {
                Err(CuError::from(format!(
                    "Mission '{id}' not found. Available missions: {}",
                    format_mission_list(graphs)
                )))
            }
        }
        (ConfigGraphs::Missions(_), None) => Ok(None),
    }
}

fn print_mission_list(config: &config::CuConfig) {
    match &config.graphs {
        ConfigGraphs::Simple(_) => println!("default"),
        ConfigGraphs::Missions(graphs) => {
            let mut missions: Vec<_> = graphs.keys().cloned().collect();
            missions.sort();
            for mission in missions {
                println!("{mission}");
            }
        }
    }
}

fn format_mission_list(graphs: &hashbrown::HashMap<String, config::CuGraph>) -> String {
    let mut missions: Vec<_> = graphs.keys().cloned().collect();
    missions.sort();
    missions.join(", ")
}
