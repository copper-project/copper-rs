//! Launches a Copper graph or schedule viewer for a workspace app.

use clap::{Parser, ValueEnum};
use std::path::{Path, PathBuf};
use std::process::{Command, ExitCode};

#[derive(Clone, Copy, Debug, ValueEnum)]
enum View {
    Graph,
    Schedule,
}

impl View {
    fn tool(self) -> &'static str {
        match self {
            Self::Graph => "cu29-graph-view",
            Self::Schedule => "cu29-schedule-view",
        }
    }

    fn output(self) -> &'static str {
        match self {
            Self::Graph => "graph.svg",
            Self::Schedule => "schedule.svg",
        }
    }
}

#[derive(Debug, Parser)]
#[command(about = "Render an application's Copper graph or execution schedule")]
struct Args {
    #[arg(value_enum)]
    view: View,
    #[arg(long, default_value = "cu_example_app")]
    app: String,
    #[arg(long)]
    config: Option<PathBuf>,
    #[arg(long)]
    output: Option<PathBuf>,
    #[arg(long)]
    mission: Option<String>,
    #[arg(long, value_delimiter = ',')]
    features: Vec<String>,
    #[arg(long)]
    log: Option<PathBuf>,
    #[arg(long)]
    candidate: Option<String>,
    #[arg(long)]
    predictions: Option<PathBuf>,
    #[arg(long)]
    measurements: Option<PathBuf>,
    #[arg(long)]
    open: bool,
}

fn main() -> ExitCode {
    match run(Args::parse()) {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("{error}");
            ExitCode::FAILURE
        }
    }
}

fn run(args: Args) -> Result<(), String> {
    let tool = args.view.tool();
    let app_dir = Path::new("apps").join(&args.app);
    let config = args
        .config
        .filter(|path| !path.as_os_str().is_empty())
        .unwrap_or_else(|| app_dir.join("copperconfig.ron"));
    let output = args
        .output
        .filter(|path| !path.as_os_str().is_empty())
        .unwrap_or_else(|| app_dir.join(args.view.output()));
    let mut command = viewer_command(tool)?;
    command.arg(config).arg("--output").arg(output);
    if args.open {
        command.arg("--open");
    }
    if let Some(mission) = args.mission {
        command.args(["--mission", &mission]);
    }
    if !args.features.is_empty() {
        command.args(["--features", &args.features.join(",")]);
    }
    if let Some(log) = args.log {
        command.arg("--log").arg(log);
        command.arg("--logreader").arg(format!("{}-logreader", args.app));
        command.arg("--logreader-package").arg(&args.app);
        command.args(["--logreader-features", "logreader"]);
    }
    if let Some(candidate) = args.candidate {
        command.args(["--candidate", &candidate]);
    }
    if let Some(predictions) = args.predictions {
        command.arg("--predictions").arg(predictions);
    }
    if let Some(measurements) = args.measurements {
        command.arg("--measurements").arg(measurements);
    }
    run_command(&mut command, tool)
}

fn viewer_command(tool: &str) -> Result<Command, String> {
    {% if copper_source == "local" %}
    let mut command = Command::new("cargo");
    command.args([
        "run",
        "--manifest-path",
        r#"{{copper_root_path}}/Cargo.toml"#,
        "-p",
        tool,
        "--",
    ]);
    Ok(command)
    {% else %}
    let installed = Path::new("target/viewer-tools/bin").join(tool);
    if !installed.is_file() {
        let mut command = Command::new("cargo");
        command.args(["install", "--locked", "--root", "target/viewer-tools"]);
        {% if copper_source == "crates.io" %}
        command.args([tool, "--version", "{{copper_version}}", "--bin", tool]);
        {% elsif copper_source == "git" %}
        command.args(["--git", r#"{{copper_git_url}}"#]);
        {% if copper_git_branch != "__none__" %}
        command.args(["--branch", r#"{{copper_git_branch}}"#]);
        {% elsif copper_git_tag != "__none__" %}
        command.args(["--tag", r#"{{copper_git_tag}}"#]);
        {% elsif copper_git_rev != "__none__" %}
        command.args(["--rev", r#"{{copper_git_rev}}"#]);
        {% endif %}
        command.args(["--bin", tool, tool]);
        {% endif %}
        run_command(&mut command, "cargo install viewer")?;
    }
    Ok(Command::new(installed))
    {% endif %}
}

fn run_command(command: &mut Command, description: &str) -> Result<(), String> {
    let status = command
        .status()
        .map_err(|error| format!("failed to start {description}: {error}"))?;
    status
        .success()
        .then_some(())
        .ok_or_else(|| format!("{description} exited with {status}"))
}
