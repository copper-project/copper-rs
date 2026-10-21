use crate::load_logstats;
use crate::render_config_svg_with_stats;
use crate::render_multi_config_svg;
use crate::validate_mission_arg;
use clap::Parser;
use cu29_runtime::config;
use cu29_runtime::config::ConfigGraphs;
use cu29_runtime::config::{
    read_configuration, read_configuration_with_features, read_multi_configuration,
    read_multi_configuration_with_features,
};
use cu29_traits::CuError;
use cu29_traits::CuResult;
use std::collections::BTreeSet;
use std::fs;
use std::io::Write;
use std::path::Path;
use std::path::PathBuf;
use std::process::Command;

#[derive(Parser)]
#[clap(author, version, about, long_about = None)]
pub(super) struct Args {
    /// Config file name
    #[clap(value_parser)]
    pub(super) config: PathBuf,
    /// Resolve relative config, log, and output paths from this directory.
    #[clap(long)]
    pub(super) base_dir: Option<PathBuf>,
    /// Log statistics JSON file to enrich the DAG
    #[clap(long)]
    logstats: Option<PathBuf>,
    /// Copper log base path to convert to viewer statistics.
    #[clap(long, requires = "logreader", conflicts_with = "logstats")]
    pub(super) log: Option<PathBuf>,
    /// Application logreader binary used with --log.
    #[clap(long)]
    logreader: Option<String>,
    /// Cargo package containing the logreader; defaults to the binary name.
    #[clap(long)]
    logreader_package: Option<String>,
    /// Comma-separated Cargo features needed to build the logreader.
    #[clap(long, value_delimiter = ',')]
    logreader_features: Vec<String>,
    /// Mission id to render (omit to render every mission in a single-Copper config)
    #[clap(long)]
    mission: Option<String>,
    /// Comma-separated Cargo feature names used to resolve conditional config fragments
    #[clap(long, value_delimiter = ',')]
    features: Vec<String>,
    /// List missions contained in the configuration and exit
    #[clap(long, action)]
    list_missions: bool,
    /// Open the SVG in the default system viewer
    #[clap(long)]
    open: bool,
    /// Output SVG path.
    #[clap(long, default_value = "graph.svg")]
    pub(super) output: PathBuf,
}

pub(super) enum RenderInput {
    Single(Box<config::CuConfig>),
    Multi(config::MultiCopperConfig),
}

/// Render the configuration file to an SVG and optionally opens it with inkscape.
/// CLI entrypoint that parses args, renders SVG, and optionally opens it.
pub(super) fn run() -> std::io::Result<()> {
    // Parse command line arguments
    let mut args = Args::parse();
    if let Some(base) = &args.base_dir {
        args.config = resolve_path(base, &args.config);
        args.output = resolve_path(base, &args.output);
        args.log = args.log.take().map(|path| resolve_path(base, &path));
        args.logstats = args.logstats.take().map(|path| resolve_path(base, &path));
    }
    let mission = args.mission.clone().filter(|mission| !mission.is_empty());
    let features = args
        .features
        .iter()
        .filter(|feature| !feature.is_empty())
        .cloned()
        .collect::<Vec<_>>();
    let active_features = features.iter().map(String::as_str).collect::<Vec<_>>();
    let generated_logstats = prepare_logstats(&args, mission.as_deref(), &active_features)?;
    if let Some(tempdir) = &generated_logstats {
        args.logstats = Some(tempdir.path().join("logstats.json"));
    }
    let input = match load_render_input(&args.config, &active_features) {
        Ok(input) => input,
        Err(err) => {
            eprintln!("{err}");
            std::process::exit(1);
        }
    };

    let graph_svg = match input {
        RenderInput::Single(config) => {
            if args.list_missions {
                print_mission_list(&config);
                return Ok(());
            }

            let mission = match validate_mission_arg(&config, mission.as_deref()) {
                Ok(mission) => mission,
                Err(err) => {
                    eprintln!("{err}");
                    std::process::exit(1);
                }
            };

            let logstats = match args.logstats.as_deref() {
                Some(path) => match load_logstats(path, &config, mission.as_deref()) {
                    Ok(stats) => Some(stats),
                    Err(err) => {
                        eprintln!("{err}");
                        std::process::exit(1);
                    }
                },
                None => None,
            };

            match render_config_svg_with_stats(&config, mission.as_deref(), logstats.as_ref()) {
                Ok(svg) => svg.into_bytes(),
                Err(err) => {
                    eprintln!("{err}");
                    std::process::exit(1);
                }
            }
        }
        RenderInput::Multi(config) => {
            if args.list_missions {
                print_multi_mission_list(&config);
                return Ok(());
            }
            if args.logstats.is_some() {
                eprintln!("Multi-Copper DAG rendering does not support --logstats yet.");
                std::process::exit(1);
            }

            match render_multi_config_svg(&config, mission.as_deref()) {
                Ok(svg) => svg.into_bytes(),
                Err(err) => {
                    eprintln!("{err}");
                    std::process::exit(1);
                }
            }
        }
    };

    if let Some(parent) = args
        .output
        .parent()
        .filter(|parent| !parent.as_os_str().is_empty())
    {
        fs::create_dir_all(parent)?;
    }
    let mut svg_file = std::fs::File::create(&args.output)?;
    svg_file.write_all(graph_svg.as_slice())?;
    if args.open {
        open_svg(&args.output)?;
    }
    Ok(())
}

fn resolve_path(base: &Path, path: &Path) -> PathBuf {
    if path.is_absolute() {
        path.to_path_buf()
    } else {
        base.join(path)
    }
}

fn prepare_logstats(
    args: &Args,
    mission: Option<&str>,
    features: &[&str],
) -> std::io::Result<Option<tempfile::TempDir>> {
    let Some(log) = &args.log else {
        return Ok(None);
    };
    let logreader = args.logreader.as_deref().unwrap_or_default();
    let tempdir = tempfile::tempdir()?;
    let output = tempdir.path().join("logstats.json");
    let mut command = Command::new("cargo");
    if let Some(base) = &args.base_dir {
        command.current_dir(base);
    }
    command.arg("run");
    if let Some(package) = args
        .logreader_package
        .as_deref()
        .filter(|name| !name.is_empty())
    {
        command.args(["--package", package]);
    }
    let logreader_features = args
        .logreader_features
        .iter()
        .filter(|feature| !feature.is_empty())
        .cloned()
        .collect::<Vec<_>>();
    if !logreader_features.is_empty() {
        command.args(["--features", &logreader_features.join(",")]);
    }
    command.args(["--bin", logreader, "--"]);
    command.arg(log).args(["log-stats", "--config"]);
    command.arg(&args.config).args(["--output"]);
    command.arg(&output);
    if let Some(mission) = mission {
        command.args(["--mission", mission]);
    }
    if !features.is_empty() {
        command.args(["--features", &features.join(",")]);
    }
    let status = command.status()?;
    if !status.success() {
        return Err(std::io::Error::other(format!(
            "logreader exited with {status}"
        )));
    }
    Ok(Some(tempdir))
}

pub(super) fn load_render_input(path: &Path, active_features: &[&str]) -> CuResult<RenderInput> {
    let path_str = path.to_str().ok_or_else(|| {
        CuError::from(format!(
            "Config path '{}' is not valid UTF-8",
            path.display()
        ))
    })?;

    let multi_config = if active_features.is_empty() {
        read_multi_configuration(path_str)
    } else {
        read_multi_configuration_with_features(path_str, active_features)
    };

    match multi_config {
        Ok(config) => Ok(RenderInput::Multi(config)),
        Err(multi_err) => match if active_features.is_empty() {
            read_configuration(path_str)
        } else {
            read_configuration_with_features(path_str, active_features)
        } {
            Ok(config) => Ok(RenderInput::Single(Box::new(config))),
            Err(single_err) => Err(CuError::from(format!(
                "Failed to read '{}' as either a Copper config or a multi-Copper config.\nCopper config: {single_err}\nMulti-Copper config: {multi_err}",
                path.display()
            ))),
        },
    }
}

/// Hide platform-specific open commands behind a single helper.
fn open_svg(path: &std::path::Path) -> std::io::Result<()> {
    if cfg!(target_os = "windows") {
        Command::new("cmd")
            .args(["/C", "start", ""])
            .arg(path)
            .status()?;
        return Ok(());
    }

    let program = if cfg!(target_os = "macos") {
        "open"
    } else {
        "xdg-open"
    };
    Command::new(program).arg(path).status()?;
    Ok(())
}

/// Support a CLI mode that prints mission names and exits.
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

/// List deployment-wide mission ids accepted by every mission-based subsystem.
fn print_multi_mission_list(config: &config::MultiCopperConfig) {
    let mut common_missions: Option<BTreeSet<String>> = None;
    for subsystem in &config.subsystems {
        let ConfigGraphs::Missions(graphs) = &subsystem.config.graphs else {
            continue;
        };
        let missions = graphs.keys().cloned().collect::<BTreeSet<_>>();
        common_missions = Some(match common_missions {
            Some(common) => common.intersection(&missions).cloned().collect(),
            None => missions,
        });
    }

    match common_missions {
        Some(missions) => {
            for mission in missions {
                println!("{mission}");
            }
        }
        None => println!("default"),
    }
}
