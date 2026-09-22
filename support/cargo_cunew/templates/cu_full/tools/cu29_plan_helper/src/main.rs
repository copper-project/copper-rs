//! Runs the process-plan renderer for an application in this Copper workspace.

use clap::Parser;
{% unless copper_source == "local" %}
use std::env;
{% endunless %}
use std::ffi::{OsStr, OsString};
use std::fs;
use std::path::{Path, PathBuf};
use std::process::{Command, ExitCode};

#[derive(Debug, Parser)]
#[command(about = "Render an application's generated Copper process schedule")]
struct Args {
    /// Application directory below apps/.
    #[arg(long, env = "APP_DIR", default_value = "cu_example_app")]
    app_dir: String,
    /// Cargo package name; defaults to the application directory name.
    #[arg(long, env = "APP_NAME")]
    app_name: Option<String>,
    /// Render only this mission.
    #[arg(long)]
    mission: Option<String>,
    /// Comma-separated Cargo features used by conditional config fragments.
    #[arg(long, value_delimiter = ',')]
    features: Vec<String>,
    /// Add observed timing from this Copper log.
    #[arg(long, conflicts_with = "default_log")]
    log: Option<PathBuf>,
    /// Add observed timing from the selected application's default log.
    #[arg(long)]
    default_log: bool,
}

struct AppPaths {
    name: String,
    config: PathBuf,
    output: PathBuf,
    log_stats: PathBuf,
    default_log: PathBuf,
}

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("{error}");
            ExitCode::FAILURE
        }
    }
}

fn run() -> Result<(), String> {
    let args = Args::parse();
    let app = app_paths(&args);
    let mut planner_args = vec![
        app.config.clone().into_os_string(),
        OsString::from("--output"),
        app.output.clone().into_os_string(),
        OsString::from("--open"),
    ];

    push_option(&mut planner_args, "--mission", args.mission.as_deref());
    if !args.features.is_empty() {
        planner_args.push(OsString::from("--features"));
        planner_args.push(OsString::from(args.features.join(",")));
    }

    let log = args
        .log
        .or_else(|| args.default_log.then(|| app.default_log.clone()));
    if let Some(log) = log {
        let parent = app
            .log_stats
            .parent()
            .ok_or_else(|| "log statistics path has no parent directory".to_owned())?;
        fs::create_dir_all(parent).map_err(|error| {
            format!(
                "failed to create log statistics directory '{}': {error}",
                parent.display()
            )
        })?;
        let log = normalize_log_path(log);
        run_logreader(&app, &log, args.mission.as_deref())?;
        planner_args.push(OsString::from("--logstats"));
        planner_args.push(app.log_stats.into_os_string());
    }

    run_planner(&planner_args)
}

fn app_paths(args: &Args) -> AppPaths {
    let name = args
        .app_name
        .clone()
        .unwrap_or_else(|| args.app_dir.clone());
    let root = PathBuf::from("apps").join(&args.app_dir);
    AppPaths {
        config: root.join("copperconfig.ron"),
        output: root.join("plan.svg"),
        log_stats: root.join("target/cu29_plan_logstats.json"),
        default_log: root.join("logs").join(format!("{name}.copper")),
        name,
    }
}

fn push_option(args: &mut Vec<OsString>, name: &str, value: Option<&str>) {
    if let Some(value) = value {
        args.push(OsString::from(name));
        args.push(OsString::from(value));
    }
}

fn normalize_log_path(mut path: PathBuf) -> PathBuf {
    let normalized_name = path
        .file_name()
        .and_then(OsStr::to_str)
        .and_then(|name| name.strip_suffix("_0.copper"))
        .map(|base| format!("{base}.copper"));
    if let Some(name) = normalized_name {
        path.set_file_name(name);
    }
    path
}

fn run_logreader(app: &AppPaths, log: &Path, mission: Option<&str>) -> Result<(), String> {
    let mut command = Command::new("cargo");
    command
        .args(["run", "-p"])
        .arg(&app.name)
        .args(["--features=logreader", "--bin"])
        .arg(format!("{}-logreader", app.name))
        .arg("--")
        .arg(log)
        .args(["log-stats", "--config"])
        .arg(&app.config)
        .args(["--output"])
        .arg(&app.log_stats)
        .args(["--features", "logreader"]);
    if let Some(mission) = mission {
        command.args(["--mission", mission]);
    }
    run_command(&mut command, "logreader")
}

fn run_planner(args: &[OsString]) -> Result<(), String> {
    {% if copper_source == "local" %}
    let mut command = Command::new("cargo");
    command.args([
        "run",
        "--manifest-path",
        r#"{{copper_root_path}}/Cargo.toml"#,
        "-p",
        "cu29-plan",
        "--",
    ]);
    {% else %}
    let planner = find_or_install_planner()?;
    let mut command = Command::new(planner);
    {% endif %}
    command.args(args);
    run_command(&mut command, "cu29-plan")
}

{% unless copper_source == "local" %}
fn find_or_install_planner() -> Result<PathBuf, String> {
    if let Some(planner) = find_on_path("cu29-plan") {
        return Ok(planner);
    }

    eprintln!("cu29-plan not found on PATH; installing it now...");
    let mut command = Command::new("cargo");
    command.args(["install", "--locked"]);
    {% if copper_source == "crates.io" %}
    command.args([
        "cu29-plan",
        "--version",
        "{{copper_version}}",
        "--bin",
        "cu29-plan",
    ]);
    {% elsif copper_source == "git" %}
    command.args(["--git", r#"{{copper_git_url}}"#]);
    {% if copper_git_branch != "__none__" %}
    command.args(["--branch", r#"{{copper_git_branch}}"#]);
    {% elsif copper_git_tag != "__none__" %}
    command.args(["--tag", r#"{{copper_git_tag}}"#]);
    {% elsif copper_git_rev != "__none__" %}
    command.args(["--rev", r#"{{copper_git_rev}}"#]);
    {% endif %}
    command.args(["--bin", "cu29-plan", "cu29-plan"]);
    {% endif %}
    run_command(&mut command, "cargo install cu29-plan")?;

    find_on_path("cu29-plan")
        .or_else(planner_in_cargo_home)
        .ok_or_else(|| "failed to find cu29-plan after installation".to_owned())
}

fn find_on_path(name: &str) -> Option<PathBuf> {
    let path = env::var_os("PATH")?;
    env::split_paths(&path)
        .map(|directory| directory.join(format!("{name}{}", env::consts::EXE_SUFFIX)))
        .find(|candidate| candidate.is_file())
}

fn planner_in_cargo_home() -> Option<PathBuf> {
    let cargo_home = env::var_os("CARGO_HOME")
        .map(PathBuf::from)
        .or_else(|| env::var_os("HOME").map(|home| PathBuf::from(home).join(".cargo")))?;
    let planner = cargo_home
        .join("bin")
        .join(format!("cu29-plan{}", env::consts::EXE_SUFFIX));
    planner.is_file().then_some(planner)
}
{% endunless %}

fn run_command(command: &mut Command, description: &str) -> Result<(), String> {
    let status = command
        .status()
        .map_err(|error| format!("failed to start {description}: {error}"))?;
    status
        .success()
        .then_some(())
        .ok_or_else(|| format!("{description} exited with {status}"))
}
