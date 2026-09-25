//! Runs the Mandelbrot profile-guided scheduling fixture.

use clap::{Parser, Subcommand};
use std::fs;
use std::path::{Path, PathBuf};
use std::process::{Command, ExitCode};

const OUTPUT: &str = "target/pgs";
const CONTRACT: &str = "schedule.ron";
const LOGREADER: &str = "cu-parallel-mandelbrot-logreader";
const PACKAGE: &str = "cu-parallel-mandelbrot";

#[derive(Parser)]
struct Args {
    #[command(subcommand)]
    action: Action,
}

#[derive(Subcommand)]
enum Action {
    RenderPredictions {
        #[arg(long)]
        candidates: usize,
    },
    Select {
        #[arg(long)]
        candidate: usize,
    },
    RenderCandidate {
        #[arg(long)]
        candidate: usize,
    },
    Measure {
        #[arg(long)]
        candidates: String,
        #[arg(long)]
        baseline: PathBuf,
    },
}

fn main() -> ExitCode {
    match run(Args::parse().action) {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("{error}");
            ExitCode::FAILURE
        }
    }
}

fn run(action: Action) -> Result<(), String> {
    match action {
        Action::RenderPredictions { candidates } => {
            if candidates == 0 {
                return Err("candidate count must be greater than zero".into());
            }
            fs::write(
                format!("{OUTPUT}/candidate-count.txt"),
                candidates.to_string(),
            )
            .map_err(|error| format!("could not save candidate count: {error}"))?;
            for candidate in 1..=candidates {
                if candidate_config(candidate).is_ok() {
                    render_candidate(candidate, None)?;
                }
            }
        }
        Action::Select { candidate } => {
            let source = candidate_config(candidate)?;
            let selected = Path::new(OUTPUT).join("selected.config.ron");
            fs::copy(&source, &selected)
                .map_err(|error| format!("could not select '{}': {error}", source.display()))?;
        }
        Action::RenderCandidate { candidate } => {
            render_candidate(candidate, Some(candidate_log(candidate)))?;
        }
        Action::Measure {
            candidates,
            baseline,
        } => {
            let selected = parse_candidates(&candidates)?;
            let count: usize = fs::read_to_string(format!("{OUTPUT}/candidate-count.txt"))
                .map_err(|error| format!("run pgs-optimize first: {error}"))?
                .trim()
                .parse()
                .map_err(|error| format!("invalid saved candidate count: {error}"))?;
            let mut command = Command::new("cargo");
            command.args(["run", "-p", PACKAGE, "--bin", LOGREADER, "--"]);
            command.arg(&baseline).args([
                "optimize-schedule",
                "--contract",
                CONTRACT,
                "--output",
                OUTPUT,
                "--candidates",
                &count.to_string(),
            ]);
            for &candidate in &selected {
                candidate_config(candidate)?;
                command.arg("--measure").arg(format!(
                    "plan-{candidate}={}",
                    candidate_log(candidate).display()
                ));
            }
            run_command(&mut command, "PGS measurement")?;
            render_view(
                "schedule",
                Path::new("copperconfig.ron"),
                Path::new(OUTPUT).join("baseline.measured.schedule.svg"),
                Some(&baseline),
                Some("baseline"),
                true,
            )?;
            for candidate in selected {
                render_measured(candidate)?;
            }
        }
    }
    Ok(())
}

fn candidate_config(candidate: usize) -> Result<PathBuf, String> {
    if candidate == 0 {
        return Err("candidate numbers start at 1".into());
    }
    let path = Path::new(OUTPUT).join(format!("plan-{candidate}.config.ron"));
    path.is_file()
        .then_some(path.clone())
        .ok_or_else(|| format!("candidate configuration '{}' is missing", path.display()))
}

fn candidate_log(candidate: usize) -> PathBuf {
    Path::new("logs").join(format!("pgs-candidate-{candidate}.copper"))
}

fn parse_candidates(input: &str) -> Result<Vec<usize>, String> {
    let candidates = input
        .split(',')
        .map(|value| {
            value
                .trim()
                .parse::<usize>()
                .map_err(|error| error.to_string())
        })
        .collect::<Result<Vec<_>, _>>()?;
    if candidates.is_empty() || candidates.contains(&0) {
        return Err("candidate list must contain positive numbers".into());
    }
    Ok(candidates)
}

fn render_candidate(candidate: usize, log: Option<PathBuf>) -> Result<(), String> {
    let config = candidate_config(candidate)?;
    let name = format!("plan-{candidate}");
    render_view(
        "graph",
        &config,
        Path::new(OUTPUT).join(format!("{name}.graph.svg")),
        log.as_deref(),
        Some(&name),
        false,
    )?;
    render_view(
        "schedule",
        &config,
        Path::new(OUTPUT).join(format!("{name}.schedule.svg")),
        log.as_deref(),
        Some(&name),
        false,
    )
}

fn render_measured(candidate: usize) -> Result<(), String> {
    let config = candidate_config(candidate)?;
    let log = candidate_log(candidate);
    let name = format!("plan-{candidate}");
    render_view(
        "graph",
        &config,
        Path::new(OUTPUT).join(format!("{name}.measured.graph.svg")),
        Some(&log),
        Some(&name),
        true,
    )?;
    render_view(
        "schedule",
        &config,
        Path::new(OUTPUT).join(format!("{name}.measured.schedule.svg")),
        Some(&log),
        Some(&name),
        true,
    )
}

fn render_view(
    view: &str,
    config: &Path,
    output: PathBuf,
    log: Option<&Path>,
    candidate: Option<&str>,
    measured: bool,
) -> Result<(), String> {
    let mut command = Command::new("cargo");
    let package = if view == "graph" {
        "cu29-graph-view"
    } else {
        "cu29-schedule-view"
    };
    command.args(["run", "-p", package, "--"]);
    command
        .arg(config)
        .arg("--output")
        .arg(output)
        .args(["--mission", "log_only"]);
    if let Some(log) = log {
        command.arg("--log").arg(log).args([
            "--logreader",
            LOGREADER,
            "--logreader-package",
            PACKAGE,
        ]);
    }
    if view == "schedule"
        && let Some(candidate) = candidate
    {
        command.args(["--candidate", candidate]);
        if candidate != "baseline" {
            command.args(["--predictions", &format!("{OUTPUT}/predictions.ron")]);
        }
        if measured {
            command.args(["--measurements", &format!("{OUTPUT}/measurements.ron")]);
        }
    }
    run_command(&mut command, "PGS viewer")
}

fn run_command(command: &mut Command, description: &str) -> Result<(), String> {
    let status = command
        .status()
        .map_err(|error| format!("could not start {description}: {error}"))?;
    status
        .success()
        .then_some(())
        .ok_or_else(|| format!("{description} exited with {status}"))
}
