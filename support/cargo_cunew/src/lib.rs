use std::env;
use std::fs;
use std::path::{Path, PathBuf};
use std::time::Duration;

use anyhow::{Context, Result, anyhow, bail};
use cargo_generate::{GenerateArgs, TemplatePath, Vcs, generate};
use clap::{Parser, ValueEnum};
use include_dir::{Dir, DirEntry, include_dir};
use pathdiff::diff_paths;
use serde::Deserialize;
use tempfile::TempDir;

static BUNDLED_TEMPLATES: Dir<'_> = include_dir!("$CARGO_MANIFEST_DIR/templates");

const DEFAULT_GIT_URL: &str = "https://github.com/copper-project/copper-rs.git";
const DEFAULT_COPPER_VERSION: &str = env!("CARGO_PKG_VERSION");
const CRATES_IO_API: &str = "https://crates.io/api/v1/crates";

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

    /// Increase cargo-generate verbosity.
    #[arg(long)]
    pub verbose: bool,
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
}

#[derive(Debug, Clone)]
struct ResolvedOptions {
    project_name: String,
    destination_dir: PathBuf,
    vcs: Vcs,
    defines: Vec<String>,
}

pub fn run(cli: Cli) -> Result<PathBuf> {
    run_with_versions(cli, None)
}

fn run_with_versions(cli: Cli, versions_override: Option<CopperVersions>) -> Result<PathBuf> {
    validate_git_options(&cli)?;

    let resolved = resolve_options(&cli, versions_override)?;
    let bundled = materialize_bundled_templates()?;

    let args = GenerateArgs {
        name: Some(resolved.project_name),
        destination: Some(resolved.destination_dir),
        allow_commands: true,
        define: resolved.defines,
        no_workspace: true,
        silent: true,
        verbose: cli.verbose,
        vcs: Some(resolved.vcs),
        template_path: TemplatePath {
            path: Some(
                bundled
                    .path()
                    .join(cli.template.subfolder())
                    .to_string_lossy()
                    .into_owned(),
            ),
            ..TemplatePath::default()
        },
        ..GenerateArgs::default()
    };

    generate(args).context("failed to generate Copper project")
}

fn resolve_options(
    cli: &Cli,
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

    let destination_dir = project_path
        .parent()
        .filter(|parent| !parent.as_os_str().is_empty())
        .map(Path::to_path_buf)
        .unwrap_or_else(|| PathBuf::from("."));

    let destination_root = absolutize(&destination_dir)?;
    let generated_root = destination_root.join(&project_name);

    let versions = versions_override.unwrap_or_else(|| match cli.source {
        SourceKind::CratesIo => resolve_crates_io_versions(),
        _ => CopperVersions {
            cu29: DEFAULT_COPPER_VERSION.to_owned(),
            cu29_export: DEFAULT_COPPER_VERSION.to_owned(),
        },
    });

    let copper_root = match cli.source {
        SourceKind::Local => Some(resolve_copper_root(cli, &destination_root)?),
        _ => None,
    };

    let defines = build_defines(cli, &versions, copper_root.as_deref(), &generated_root)?;

    Ok(ResolvedOptions {
        project_name,
        destination_dir,
        vcs: if cli.no_vcs { Vcs::None } else { Vcs::Git },
        defines,
    })
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
    versions: &CopperVersions,
    copper_root: Option<&Path>,
    generated_root: &Path,
) -> Result<Vec<String>> {
    let mut defines = vec![
        format!("copper_source={}", cli.source.as_template_value()),
        format!("copper_version={}", versions.cu29),
        format!("copper_export_version={}", versions.cu29_export),
        format!("copper_git_url={}", cli.git_url),
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

fn resolve_crates_io_versions() -> CopperVersions {
    fetch_latest_stable_versions().unwrap_or_else(|error| {
        eprintln!(
            "warning: failed to query crates.io for Copper versions ({error:#}); falling back to {DEFAULT_COPPER_VERSION}"
        );
        CopperVersions {
            cu29: DEFAULT_COPPER_VERSION.to_owned(),
            cu29_export: DEFAULT_COPPER_VERSION.to_owned(),
        }
    })
}

fn fetch_latest_stable_versions() -> Result<CopperVersions> {
    let client = reqwest::blocking::Client::builder()
        .timeout(Duration::from_secs(5))
        .user_agent(format!("cargo-cunew/{}", env!("CARGO_PKG_VERSION")))
        .build()
        .context("failed to build crates.io HTTP client")?;

    Ok(CopperVersions {
        cu29: fetch_crate_version(&client, "cu29")?,
        cu29_export: fetch_crate_version(&client, "cu29-export")?,
    })
}

fn fetch_crate_version(client: &reqwest::blocking::Client, crate_name: &str) -> Result<String> {
    let response = client
        .get(format!("{CRATES_IO_API}/{crate_name}"))
        .send()
        .with_context(|| format!("failed to query crates.io for {crate_name}"))?
        .error_for_status()
        .with_context(|| format!("crates.io returned an error for {crate_name}"))?;

    let payload: CratesIoResponse = response
        .json()
        .with_context(|| format!("failed to decode crates.io response for {crate_name}"))?;

    if payload.krate.max_stable_version.is_empty() {
        bail!("crates.io did not return a stable version for {crate_name}");
    }

    Ok(payload.krate.max_stable_version)
}

#[derive(Debug, Deserialize)]
struct CratesIoResponse {
    #[serde(rename = "crate")]
    krate: CratesIoCrate,
}

#[derive(Debug, Deserialize)]
struct CratesIoCrate {
    max_stable_version: String,
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

fn materialize_bundled_templates() -> Result<TempDir> {
    let tempdir =
        tempfile::tempdir().context("failed to create a temporary directory for templates")?;
    write_dir(&BUNDLED_TEMPLATES, tempdir.path())?;
    Ok(tempdir)
}

fn write_dir(dir: &Dir<'_>, destination: &Path) -> Result<()> {
    fs::create_dir_all(destination)
        .with_context(|| format!("failed to create {}", destination.display()))?;

    for entry in dir.entries() {
        match entry {
            DirEntry::Dir(child) => {
                let name = child
                    .path()
                    .file_name()
                    .ok_or_else(|| anyhow!("invalid embedded directory path"))?;
                write_dir(child, &destination.join(name))?;
            }
            DirEntry::File(file) => {
                let name = file
                    .path()
                    .file_name()
                    .ok_or_else(|| anyhow!("invalid embedded file path"))?;
                let target = destination.join(name);
                fs::write(&target, file.contents())
                    .with_context(|| format!("failed to write {}", target.display()))?;
            }
        }
    }

    Ok(())
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
    fn materializes_bundled_templates_with_hidden_files() {
        let templates = materialize_bundled_templates().expect("templates should materialize");
        assert!(templates.path().join(".cargo/config.toml").is_file());
        assert!(templates.path().join("cu_project/.gitignore").is_file());
        assert!(
            templates
                .path()
                .join("cu_full/apps/cu_example_app/logs/.keep")
                .is_file()
        );
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
        };

        run_with_versions(
            cli,
            Some(CopperVersions {
                cu29: "9.9.9".to_owned(),
                cu29_export: "9.9.8".to_owned(),
            }),
        )
        .expect("generation should succeed");

        let manifest = fs::read_to_string(project.join("Cargo.toml")).expect("manifest");
        let justfile = fs::read_to_string(project.join("justfile")).expect("justfile");

        assert!(manifest.contains("edition = \"2024\""));
        assert!(manifest.contains("version = \"9.9.9\""));
        assert!(manifest.contains("version = \"9.9.8\""));
        assert!(manifest.contains("cu29-export"));
        assert!(!manifest.contains("\n[workspace]\n"));
        assert!(justfile.contains("Re-run cargo cunew with --source local"));
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
        };

        run_with_versions(
            cli,
            Some(CopperVersions {
                cu29: "0.0.0".to_owned(),
                cu29_export: "0.0.0".to_owned(),
            }),
        )
        .expect("generation should succeed");

        let manifest = fs::read_to_string(project.join("Cargo.toml")).expect("manifest");
        let app_manifest = fs::read_to_string(project.join("apps/cu_example_app/Cargo.toml"))
            .expect("app manifest");
        let justfile = fs::read_to_string(project.join("justfile")).expect("justfile");

        assert!(manifest.contains("core/cu29"));
        assert!(manifest.contains("core/cu29_export"));
        assert!(app_manifest.contains("edition = \"2024\""));
        assert!(justfile.contains("cu29-rendercfg"));
        assert!(!project.join(".git").exists());
    }
}
