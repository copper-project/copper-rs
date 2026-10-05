mod templates;

use std::env;
use std::io::{self, IsTerminal, Write};
use std::path::{Path, PathBuf};
use std::time::Duration;

use anyhow::{Context, Result, anyhow, bail};
use clap::{Parser, ValueEnum};
use include_dir::{Dir, include_dir};
use pathdiff::diff_paths;
use semver::Version;
use serde::Deserialize;

static BUNDLED_TEMPLATES: Dir<'_> = include_dir!("$CARGO_MANIFEST_DIR/templates");

const DEFAULT_GIT_URL: &str = "https://github.com/copper-project/copper-rs.git";
const DEFAULT_COPPER_VERSION: &str = env!("CARGO_PKG_VERSION");
const CRATES_IO_API: &str = "https://crates.io/api/v1/crates";
const NO_TEMPLATE_GIT_REF: &str = "__none__";

#[derive(Debug, Clone, Parser)]
#[command(
    about = "Bootstrap a new Copper project",
    version,
    after_help = "Examples:\n  cargo cunew my_robot\n  cargo cunew --template workspace my_workspace\n  cargo cunew --source local --copper-root /path/to/copper-rs my_robot"
)]
pub struct Cli {
    /// Destination directory for the generated project.
    #[arg(value_name = "PATH")]
    pub path: PathBuf,

    /// Template to generate.
    #[arg(long, value_enum, default_value_t = TemplateKind::Project)]
    pub template: TemplateKind,

    /// Copper dependency source.
    #[arg(long, value_enum, default_value_t = SourceKind::CratesIo)]
    pub source: SourceKind,

    /// Override the generated Cargo package name.
    #[arg(long)]
    pub name: Option<String>,

    /// Path to a local Copper checkout when using --source local.
    #[arg(long)]
    pub copper_root: Option<PathBuf>,

    /// Git URL when using --source git.
    #[arg(long, default_value = DEFAULT_GIT_URL)]
    pub git_url: String,

    /// Git branch when using --source git.
    #[arg(long)]
    pub git_branch: Option<String>,

    /// Git tag when using --source git.
    #[arg(long)]
    pub git_tag: Option<String>,

    /// Git revision when using --source git.
    #[arg(long)]
    pub git_rev: Option<String>,

    /// Skip initializing a git repository in the generated project.
    #[arg(long)]
    pub no_vcs: bool,

    /// Print generated file paths.
    #[arg(long)]
    pub verbose: bool,

    /// Deployment target. Bare-metal projects omit host-only PGS scaffolding.
    #[arg(long, value_enum)]
    pub target: Option<TargetKind>,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, ValueEnum)]
pub enum TemplateKind {
    Project,
    #[value(alias = "full")]
    Workspace,
}

impl TemplateKind {
    fn subfolder(self) -> &'static str {
        match self {
            Self::Project => "cu_project",
            Self::Workspace => "cu_full",
        }
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, ValueEnum)]
pub enum SourceKind {
    #[value(name = "crates.io", alias = "crates-io")]
    CratesIo,
    Git,
    Local,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, ValueEnum)]
pub enum TargetKind {
    Host,
    #[value(name = "bare-metal", alias = "nostd", alias = "no-std")]
    BareMetal,
}

impl SourceKind {
    fn as_template_value(self) -> &'static str {
        match self {
            Self::CratesIo => "crates.io",
            Self::Git => "git",
            Self::Local => "local",
        }
    }
}

#[derive(Debug, Clone)]
struct CopperVersions {
    cu29: String,
    cu29_export: String,
    cu29_build: String,
    cu_memmon: String,
}

#[derive(Debug, Clone)]
struct ResolvedOptions {
    project_name: String,
    destination_dir: PathBuf,
    initialize_git: bool,
    defines: Vec<String>,
}

pub fn run(cli: Cli) -> Result<PathBuf> {
    run_with_versions(cli, None)
}

fn run_with_versions(cli: Cli, versions_override: Option<CopperVersions>) -> Result<PathBuf> {
    validate_git_options(&cli)?;
    let target = resolve_target(&cli)?;
    let resolved = resolve_options(&cli, target, versions_override)?;
    templates::generate(&cli, &resolved, target).context("failed to generate Copper project")
}

fn resolve_options(
    cli: &Cli,
    target: TargetKind,
    versions_override: Option<CopperVersions>,
) -> Result<ResolvedOptions> {
    let project_path = cli.path.clone();
    let project_name = cli
        .name
        .clone()
        .or_else(|| {
            project_path
                .file_name()
                .map(|value| value.to_string_lossy().into_owned())
        })
        .filter(|value| !value.is_empty() && value != ".")
        .ok_or_else(|| {
            anyhow!(
                "could not derive a project name from {}",
                project_path.display()
            )
        })?;

    let project_name = if heck::ToSnakeCase::to_snake_case(project_name.as_str()) == project_name {
        project_name
    } else {
        heck::ToKebabCase::to_kebab_case(project_name.as_str())
    };

    let destination_dir = project_path
        .parent()
        .filter(|parent| !parent.as_os_str().is_empty())
        .map(Path::to_path_buf)
        .unwrap_or_else(|| PathBuf::from("."));

    let destination_root = absolutize(&destination_dir)?;
    let generated_root = destination_root.join(&project_name);

    let versions = match versions_override {
        Some(versions) => versions,
        None if cli.source == SourceKind::CratesIo => fetch_stable_versions()?,
        None => CopperVersions {
            cu29: DEFAULT_COPPER_VERSION.to_owned(),
            cu29_export: DEFAULT_COPPER_VERSION.to_owned(),
            cu29_build: DEFAULT_COPPER_VERSION.to_owned(),
            cu_memmon: DEFAULT_COPPER_VERSION.to_owned(),
        },
    };

    let copper_root = match cli.source {
        SourceKind::Local => Some(resolve_copper_root(cli, &destination_root)?),
        _ => None,
    };

    let defines = build_defines(
        cli,
        target,
        &versions,
        copper_root.as_deref(),
        &generated_root,
    )?;

    Ok(ResolvedOptions {
        project_name,
        destination_dir,
        initialize_git: !cli.no_vcs,
        defines,
    })
}

fn resolve_target(cli: &Cli) -> Result<TargetKind> {
    if let Some(target) = cli.target {
        return Ok(target);
    }
    if !io::stdin().is_terminal() || !io::stdout().is_terminal() {
        return Ok(TargetKind::Host);
    }

    print!("Is this project targeting bare metal/no_std? [y/N] ");
    io::stdout()
        .flush()
        .context("failed to write target prompt")?;
    let mut answer = String::new();
    io::stdin()
        .read_line(&mut answer)
        .context("failed to read target selection")?;
    match answer.trim().to_ascii_lowercase().as_str() {
        "" | "n" | "no" => Ok(TargetKind::Host),
        "y" | "yes" => Ok(TargetKind::BareMetal),
        _ => bail!("answer yes or no, or pass --target host|bare-metal"),
    }
}

fn validate_git_options(cli: &Cli) -> Result<()> {
    let git_ref_count = [&cli.git_branch, &cli.git_tag, &cli.git_rev]
        .into_iter()
        .filter(|value| value.is_some())
        .count();

    if git_ref_count > 1 {
        bail!("use at most one of --git-branch, --git-tag, or --git-rev");
    }

    if cli.source != SourceKind::Git && git_ref_count > 0 {
        bail!("git ref options require --source git");
    }

    if cli.source != SourceKind::Local && cli.copper_root.is_some() {
        bail!("--copper-root requires --source local");
    }

    Ok(())
}

fn build_defines(
    cli: &Cli,
    target: TargetKind,
    versions: &CopperVersions,
    copper_root: Option<&Path>,
    generated_root: &Path,
) -> Result<Vec<String>> {
    let mut defines = vec![
        format!("pgs_enabled={}", target == TargetKind::Host),
        format!("copper_source={}", cli.source.as_template_value()),
        format!("copper_version={}", versions.cu29),
        format!("copper_export_version={}", versions.cu29_export),
        format!("copper_build_version={}", versions.cu29_build),
        format!("copper_memmon_version={}", versions.cu_memmon),
        format!("copper_git_url={}", cli.git_url),
        format!(
            "copper_git_branch={}",
            cli.git_branch.as_deref().unwrap_or(NO_TEMPLATE_GIT_REF)
        ),
        format!(
            "copper_git_tag={}",
            cli.git_tag.as_deref().unwrap_or(NO_TEMPLATE_GIT_REF)
        ),
        format!(
            "copper_git_rev={}",
            cli.git_rev.as_deref().unwrap_or(NO_TEMPLATE_GIT_REF)
        ),
        format!(
            "copper_git_ref_snippet={}",
            format_git_ref_snippet(
                cli.git_branch.as_deref(),
                cli.git_tag.as_deref(),
                cli.git_rev.as_deref()
            )
        ),
    ];

    let copper_root_path = match copper_root {
        Some(root) => relative_or_absolute_toml_path(root, generated_root)?,
        None => "../..".to_owned(),
    };
    defines.push(format!("copper_root_path={copper_root_path}"));

    Ok(defines)
}

fn resolve_copper_root(cli: &Cli, destination_root: &Path) -> Result<PathBuf> {
    if let Some(root) = &cli.copper_root {
        return validate_copper_root(root);
    }

    if let Some(root) =
        detect_copper_root(&env::current_dir().context("failed to read current directory")?)
    {
        return Ok(root);
    }

    if let Some(root) = detect_copper_root(destination_root) {
        return Ok(root);
    }

    bail!("could not detect a Copper checkout; pass --copper-root /path/to/copper-rs")
}

fn detect_copper_root(start: &Path) -> Option<PathBuf> {
    start
        .ancestors()
        .find(|dir| is_copper_root(dir))
        .map(Path::to_path_buf)
}

fn validate_copper_root(path: &Path) -> Result<PathBuf> {
    let absolute = absolutize(path)?;
    if is_copper_root(&absolute) {
        absolute
            .canonicalize()
            .with_context(|| format!("failed to canonicalize {}", absolute.display()))
    } else {
        bail!(
            "{} does not look like a Copper checkout (expected core/cu29/Cargo.toml and support/cargo_cunew/templates)",
            absolute.display()
        )
    }
}

fn is_copper_root(path: &Path) -> bool {
    path.join("core/cu29/Cargo.toml").is_file()
        && path.join("support/cargo_cunew/templates").is_dir()
}

fn fetch_stable_versions() -> Result<CopperVersions> {
    let client: ureq::Agent = ureq::Agent::config_builder()
        .timeout_global(Some(Duration::from_secs(10)))
        .user_agent(format!("cargo-cunew/{}", env!("CARGO_PKG_VERSION")))
        .build()
        .into();
    let release =
        Version::parse(DEFAULT_COPPER_VERSION).context("invalid cargo-cunew package version")?;
    Ok(CopperVersions {
        cu29: fetch_crate_version(&client, "cu29", &release)?,
        cu29_export: fetch_crate_version(&client, "cu29-export", &release)?,
        cu29_build: fetch_crate_version(&client, "cu29-build", &release)?,
        cu_memmon: fetch_crate_version(&client, "cu-memmon", &release)?,
    })
}

fn fetch_crate_version(
    client: &ureq::Agent,
    crate_name: &str,
    release: &Version,
) -> Result<String> {
    let payload: CratesIoResponse = client
        .get(format!("{CRATES_IO_API}/{crate_name}"))
        .call()
        .with_context(|| format!("failed to query crates.io for {crate_name}; check your network connection or use --source local|git"))?
        .body_mut()
        .read_json()
        .with_context(|| format!("failed to decode crates.io response for {crate_name}"))?;
    select_stable_version(&payload, crate_name, release)
}

fn select_stable_version(
    payload: &CratesIoResponse,
    crate_name: &str,
    release: &Version,
) -> Result<String> {
    payload.versions.iter()
        .filter(|version| !version.yanked)
        .filter_map(|version| Version::parse(&version.num).ok())
        .filter(|version| version.pre.is_empty() && version.major == release.major && version.minor == release.minor)
        .max()
        .map(|version| version.to_string())
        .ok_or_else(|| anyhow!("no stable {crate_name} version is published for Copper {}.{}; use --source local|git", release.major, release.minor))
}

#[derive(Debug, Deserialize)]
struct CratesIoResponse {
    versions: Vec<CratesIoVersion>,
}

#[derive(Debug, Deserialize)]
struct CratesIoVersion {
    num: String,
    yanked: bool,
}

fn format_git_ref_snippet(branch: Option<&str>, tag: Option<&str>, rev: Option<&str>) -> String {
    if let Some(branch) = branch {
        return format!(", branch = \"{branch}\"");
    }
    if let Some(tag) = tag {
        return format!(", tag = \"{tag}\"");
    }
    if let Some(rev) = rev {
        return format!(", rev = \"{rev}\"");
    }
    String::new()
}

fn relative_or_absolute_toml_path(target: &Path, from: &Path) -> Result<String> {
    let target = absolutize(target)?;
    let from = absolutize(from)?;
    let from_parent = from.parent().unwrap_or(&from);
    let target_parent = target.parent().unwrap_or(&target);

    let prefer_relative = from.starts_with(target_parent) || target.starts_with(from_parent);
    let path = if prefer_relative {
        diff_paths(&target, &from).unwrap_or(target)
    } else {
        target
    };
    normalize_toml_path(&path)
}

fn normalize_toml_path(path: &Path) -> Result<String> {
    let display = path
        .to_str()
        .ok_or_else(|| anyhow!("{} is not valid UTF-8", path.display()))?;
    Ok(display.replace('\\', "/"))
}

fn absolutize(path: &Path) -> Result<PathBuf> {
    if path.is_absolute() {
        Ok(path.to_path_buf())
    } else {
        Ok(env::current_dir()
            .context("failed to read current directory")?
            .join(path))
    }
}

pub fn normalize_cargo_subcommand_args<I>(args: I) -> Vec<String>
where
    I: IntoIterator,
    I::Item: Into<String>,
{
    let mut normalized: Vec<String> = args.into_iter().map(Into::into).collect();
    if normalized.get(1).is_some_and(|arg| arg == "cunew") {
        normalized.remove(1);
    }
    normalized
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;

    #[test]
    fn strips_cargo_subcommand_marker() {
        let args = normalize_cargo_subcommand_args(["cargo-cunew", "cunew", "robot"]);
        assert_eq!(args, vec!["cargo-cunew", "robot"]);
    }

    #[test]
    fn keeps_direct_binary_args() {
        let args = normalize_cargo_subcommand_args(["cargo-cunew", "robot"]);
        assert_eq!(args, vec!["cargo-cunew", "robot"]);
    }

    #[test]
    fn formats_git_ref_snippets() {
        assert_eq!(
            format_git_ref_snippet(Some("main"), None, None),
            ", branch = \"main\""
        );
        assert_eq!(
            format_git_ref_snippet(None, Some("v1.0.0"), None),
            ", tag = \"v1.0.0\""
        );
        assert_eq!(
            format_git_ref_snippet(None, None, Some("abc123")),
            ", rev = \"abc123\""
        );
        assert!(format_git_ref_snippet(None, None, None).is_empty());
    }

    #[test]
    fn selects_stable_versions_in_the_tools_release_line() {
        let payload = CratesIoResponse {
            versions: vec![
                CratesIoVersion {
                    num: "2.0.0".to_owned(),
                    yanked: false,
                },
                CratesIoVersion {
                    num: "1.3.0".to_owned(),
                    yanked: false,
                },
                CratesIoVersion {
                    num: "1.2.5".to_owned(),
                    yanked: true,
                },
                CratesIoVersion {
                    num: "1.2.4-dev".to_owned(),
                    yanked: false,
                },
                CratesIoVersion {
                    num: "1.2.1".to_owned(),
                    yanked: false,
                },
                CratesIoVersion {
                    num: "1.2.3".to_owned(),
                    yanked: false,
                },
            ],
        };
        let release = Version::parse("1.2.4").expect("version");
        assert_eq!(
            select_stable_version(&payload, "cu29", &release).expect("stable version"),
            "1.2.3"
        );
        let release = Version::parse("1.1.4").expect("version");
        assert!(select_stable_version(&payload, "cu29", &release).is_err());
    }

    #[test]
    fn generates_project_template_for_crates_io() {
        let tempdir = tempfile::tempdir().expect("tempdir");
        let project = tempdir.path().join("hello-copper");
        let cli = Cli {
            path: project.clone(),
            template: TemplateKind::Project,
            source: SourceKind::CratesIo,
            name: None,
            copper_root: None,
            git_url: DEFAULT_GIT_URL.to_owned(),
            git_branch: None,
            git_tag: None,
            git_rev: None,
            no_vcs: true,
            verbose: false,
            target: Some(TargetKind::Host),
        };

        run_with_versions(
            cli,
            Some(CopperVersions {
                cu29: "9.9.9".to_owned(),
                cu29_export: "9.9.8".to_owned(),
                cu29_build: "9.9.1".to_owned(),
                cu_memmon: "9.9.2".to_owned(),
            }),
        )
        .expect("generation should succeed");

        let manifest = fs::read_to_string(project.join("Cargo.toml")).expect("manifest");
        let justfile = fs::read_to_string(project.join("justfile")).expect("justfile");
        let viewer = fs::read_to_string(project.join("src/view.rs")).expect("viewer helper");

        assert!(manifest.contains("edition = \"2024\""));
        assert!(manifest.contains("version = \"~9.9.9\""));
        assert!(manifest.contains("version = \"~9.9.8\""));
        assert!(manifest.contains("cu29-export"));
        assert!(manifest.contains("cu29-build = {  version = \"~9.9.1\""));
        assert!(manifest.contains("cu-memmon = {  version = \"~9.9.2\""));
        assert!(project.join(".gitignore").is_file());
        assert!(!project.join("init.rhai").exists());
        assert!(manifest.contains("[profile.debug-optimized]"));
        assert!(manifest.contains("[workspace]"));
        assert!(justfile.contains("--profile debug-optimized"));
        assert!(justfile.contains("graph config="));
        assert!(justfile.contains("sched-log config="));
        assert!(justfile.contains("pgs-optimize candidates="));
        assert!(viewer.contains("cu29-graph-view"));
        assert!(viewer.contains("cu29-schedule-view"));
        assert!(viewer.contains("\"~9.9.9\""));
        assert!(project.join("schedule.ron").is_file());
        assert!(project.join("src/pgs_candidate.rs").is_file());
    }

    #[test]
    fn generates_workspace_with_independent_registry_versions() {
        let tempdir = tempfile::tempdir().expect("tempdir");
        let project = tempdir.path().join("hello-workspace");
        let cli = Cli {
            path: project.clone(),
            template: TemplateKind::Workspace,
            source: SourceKind::CratesIo,
            name: None,
            copper_root: None,
            git_url: DEFAULT_GIT_URL.to_owned(),
            git_branch: None,
            git_tag: None,
            git_rev: None,
            no_vcs: true,
            verbose: false,
            target: Some(TargetKind::Host),
        };
        run_with_versions(
            cli,
            Some(CopperVersions {
                cu29: "1.2.3".to_owned(),
                cu29_export: "1.2.1".to_owned(),
                cu29_build: "1.2.1".to_owned(),
                cu_memmon: "1.2.1".to_owned(),
            }),
        )
        .expect("generation");
        let manifest = fs::read_to_string(project.join("Cargo.toml")).expect("manifest");
        assert!(manifest.contains("cu29 = {  version = \"~1.2.3\""));
        assert!(manifest.contains("cu29-build = {  version = \"~1.2.1\""));
        assert!(manifest.contains("cu29-export = {  version = \"~1.2.1\""));
        assert!(project.join("apps/cu_example_app/logs/.keep").is_file());
    }

    #[test]
    fn generates_workspace_template_for_local_checkout() {
        let tempdir = tempfile::tempdir().expect("tempdir");
        let project = tempdir.path().join("hello-workspace");
        let cli = Cli {
            path: project.clone(),
            template: TemplateKind::Workspace,
            source: SourceKind::Local,
            name: None,
            copper_root: Some(PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..")),
            git_url: DEFAULT_GIT_URL.to_owned(),
            git_branch: None,
            git_tag: None,
            git_rev: None,
            no_vcs: true,
            verbose: false,
            target: Some(TargetKind::Host),
        };

        run_with_versions(
            cli,
            Some(CopperVersions {
                cu29: "0.0.0".to_owned(),
                cu29_export: "0.0.0".to_owned(),
                cu29_build: "9.9.1".to_owned(),
                cu_memmon: "9.9.2".to_owned(),
            }),
        )
        .expect("generation should succeed");

        let manifest = fs::read_to_string(project.join("Cargo.toml")).expect("manifest");
        let app_manifest = fs::read_to_string(project.join("apps/cu_example_app/Cargo.toml"))
            .expect("app manifest");
        let justfile = fs::read_to_string(project.join("justfile")).expect("justfile");
        let viewer = fs::read_to_string(project.join("tools/cu29_view_helper/src/main.rs"))
            .expect("viewer helper");

        assert!(manifest.contains("core/cu29"));
        assert!(manifest.contains("core/cu29_export"));
        assert!(manifest.contains("[profile.debug-optimized]"));
        assert!(app_manifest.contains("edition = \"2024\""));
        assert!(justfile.contains("--profile debug-optimized"));
        assert!(justfile.contains("graph app="));
        assert!(justfile.contains("sched app="));
        assert!(justfile.contains("sched-log app="));
        assert!(justfile.contains("pgs-measure candidates="));
        assert!(viewer.contains("#[derive(Debug, Parser)]"));
        assert!(!viewer.contains("env ="));
        assert!(project.join("apps/cu_example_app/schedule.ron").is_file());
        assert!(!project.join(".git").exists());
    }

    #[test]
    fn preserves_underscore_directory_names_and_existing_files() {
        let tempdir = tempfile::tempdir().expect("tempdir");
        let project = tempdir.path().join("hello_copper");
        let cli = Cli {
            path: project.clone(),
            template: TemplateKind::Project,
            source: SourceKind::Local,
            name: None,
            copper_root: Some(PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..")),
            git_url: DEFAULT_GIT_URL.to_owned(),
            git_branch: None,
            git_tag: None,
            git_rev: None,
            no_vcs: true,
            verbose: false,
            target: Some(TargetKind::Host),
        };

        run_with_versions(
            cli,
            Some(CopperVersions {
                cu29: "1.2.3".to_owned(),
                cu29_export: "1.2.4".to_owned(),
                cu29_build: "9.9.1".to_owned(),
                cu_memmon: "9.9.2".to_owned(),
            }),
        )
        .expect("generation should succeed");

        assert!(project.join("Cargo.toml").exists());
        fs::write(project.join("user-file"), "keep").expect("user file");
        let cli = Cli::parse_from([
            "cargo-cunew",
            project.to_str().expect("path"),
            "--source",
            "local",
            "--copper-root",
            env!("CARGO_MANIFEST_DIR"),
        ]);
        // Validate the existing destination before any files are replaced.
        let options = ResolvedOptions {
            project_name: "hello_copper".to_owned(),
            destination_dir: tempdir.path().to_path_buf(),
            initialize_git: false,
            defines: Vec::new(),
        };
        assert!(templates::generate(&cli, &options, TargetKind::Host).is_err());
        assert_eq!(
            fs::read_to_string(project.join("user-file")).expect("preserved file"),
            "keep"
        );
    }

    #[test]
    fn generates_project_template_for_git_source() {
        let tempdir = tempfile::tempdir().expect("tempdir");
        let project = tempdir.path().join("hello-git");
        let cli = Cli {
            path: project.clone(),
            template: TemplateKind::Project,
            source: SourceKind::Git,
            name: None,
            copper_root: None,
            git_url: DEFAULT_GIT_URL.to_owned(),
            git_branch: Some("main".to_owned()),
            git_tag: None,
            git_rev: None,
            no_vcs: true,
            verbose: false,
            target: Some(TargetKind::Host),
        };

        run_with_versions(
            cli,
            Some(CopperVersions {
                cu29: "1.2.3".to_owned(),
                cu29_export: "1.2.4".to_owned(),
                cu29_build: "9.9.1".to_owned(),
                cu_memmon: "9.9.2".to_owned(),
            }),
        )
        .expect("generation should succeed");

        let manifest = fs::read_to_string(project.join("Cargo.toml")).expect("manifest");
        let justfile = fs::read_to_string(project.join("justfile")).expect("justfile");
        let viewer = fs::read_to_string(project.join("src/view.rs")).expect("viewer helper");

        assert!(manifest.contains("git = \"https://github.com/copper-project/copper-rs.git\""));
        assert!(manifest.contains("branch = \"main\""));
        assert!(viewer.contains("https://github.com/copper-project/copper-rs.git"));
        assert!(viewer.contains("command.args([\"--branch\", r#\"main\"#])"));
        assert!(!justfile.contains("plan-log"));
    }

    #[test]
    fn bare_metal_target_omits_pgs_in_both_templates() {
        for template in [TemplateKind::Project, TemplateKind::Workspace] {
            let tempdir = tempfile::tempdir().expect("tempdir");
            let project = tempdir.path().join("bare-copper");
            let cli = Cli {
                path: project.clone(),
                template,
                source: SourceKind::Local,
                name: None,
                copper_root: Some(PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..")),
                git_url: DEFAULT_GIT_URL.to_owned(),
                git_branch: None,
                git_tag: None,
                git_rev: None,
                no_vcs: true,
                verbose: false,
                target: Some(TargetKind::BareMetal),
            };
            run_with_versions(cli, None).expect("generation should succeed");
            let justfile = fs::read_to_string(project.join("justfile")).expect("justfile");
            assert!(!justfile.contains("pgs-"));
            match template {
                TemplateKind::Project => {
                    let manifest = fs::read_to_string(project.join("Cargo.toml")).unwrap();
                    assert!(!manifest.contains("pgs-candidate"));
                    assert!(!project.join("schedule.ron").exists());
                    assert!(!project.join("src/pgs_candidate.rs").exists());
                }
                TemplateKind::Workspace => {
                    let manifest =
                        fs::read_to_string(project.join("apps/cu_example_app/Cargo.toml")).unwrap();
                    assert!(!manifest.contains("pgs-candidate"));
                    assert!(!project.join("apps/cu_example_app/schedule.ron").exists());
                    assert!(
                        !project
                            .join("apps/cu_example_app/src/pgs_candidate.rs")
                            .exists()
                    );
                }
            }
        }
    }
}
