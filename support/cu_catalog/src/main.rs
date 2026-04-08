use std::{
    collections::{BTreeMap, BTreeSet},
    fs,
    path::{Path, PathBuf},
};

use anyhow::{Context, Result, anyhow, bail};
use clap::{Args, Parser, Subcommand};
use reqwest::blocking::Client;
use ron::{Options, extensions::Extensions};
use serde::{Deserialize, Serialize};
use toml::Value;

#[derive(Debug, Parser)]
#[command(author, version, about)]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Debug, Subcommand)]
enum Commands {
    Generate(GenerateArgs),
}

#[derive(Debug, Args)]
struct GenerateArgs {
    #[arg(long, default_value = "catalog/index.ron")]
    index: PathBuf,
    #[arg(long, default_value = "catalog/generated")]
    output_dir: PathBuf,
    #[arg(long)]
    local_root: Option<PathBuf>,
}

#[derive(Debug, Deserialize)]
struct CatalogIndex {
    version: u32,
    #[serde(default)]
    defaults: CatalogDefaults,
    entries: Vec<CatalogIndexEntry>,
}

#[derive(Debug, Default, Deserialize)]
struct CatalogDefaults {
    github_repo: Option<String>,
    github_rev: Option<String>,
}

#[derive(Debug, Deserialize)]
struct CatalogIndexEntry {
    name: String,
    source: CatalogSource,
    #[serde(default)]
    overrides: CatalogOverrides,
}

#[derive(Debug, Deserialize)]
enum CatalogSource {
    CratesIo {
        crate_name: String,
        version: Option<String>,
    },
    GitHub {
        repo: Option<String>,
        rev: Option<String>,
        path: String,
    },
}

#[derive(Debug, Clone, Default, Deserialize)]
struct CatalogOverrides {
    description: Option<String>,
    keywords: Option<Vec<String>>,
    categories: Option<Vec<String>>,
    repository: Option<String>,
    homepage: Option<String>,
    license: Option<String>,
    kind: Option<String>,
    domains: Option<Vec<String>>,
    environments: Option<Vec<String>>,
    host_os: Option<Vec<String>>,
    embedded_targets: Option<Vec<String>>,
}

#[derive(Debug, Clone, Default, Deserialize)]
struct CopperMetadataPartial {
    kind: Option<String>,
    domains: Option<Vec<String>>,
    environments: Option<Vec<String>>,
    host_os: Option<Vec<String>>,
    embedded_targets: Option<Vec<String>>,
}

#[derive(Debug, Clone, Serialize)]
struct CopperMetadataResolved {
    kind: String,
    domains: Vec<String>,
    environments: Vec<String>,
    host_os: Vec<String>,
    embedded_targets: Vec<String>,
}

#[derive(Debug, Clone, Serialize)]
struct ResolvedCatalogSource {
    source_type: String,
    repo: Option<String>,
    rev: Option<String>,
    path: Option<String>,
    crate_name: Option<String>,
    version: Option<String>,
    source_url: String,
    manifest_url: Option<String>,
    readme_url: Option<String>,
}

#[derive(Debug, Clone, Serialize)]
struct ResolvedCatalogEntry {
    catalog_name: String,
    package_name: String,
    description: String,
    keywords: Vec<String>,
    categories: Vec<String>,
    repository: Option<String>,
    homepage: Option<String>,
    license: Option<String>,
    copper: CopperMetadataResolved,
    source: ResolvedCatalogSource,
}

#[derive(Debug)]
struct SourceMetadata {
    package_name: String,
    description: Option<String>,
    keywords: Vec<String>,
    categories: Vec<String>,
    repository: Option<String>,
    homepage: Option<String>,
    license: Option<String>,
    copper: CopperMetadataPartial,
    source: ResolvedCatalogSource,
}

#[derive(Debug, Deserialize)]
struct CratesIoResponse {
    #[serde(rename = "crate")]
    krate: CratesIoCrate,
    versions: Vec<CratesIoVersion>,
}

#[derive(Debug, Deserialize)]
struct CratesIoCrate {
    name: String,
    description: Option<String>,
    homepage: Option<String>,
    repository: Option<String>,
    keywords: Vec<String>,
    categories: Vec<String>,
    newest_version: String,
    max_stable_version: Option<String>,
}

#[derive(Debug, Deserialize)]
struct CratesIoVersion {
    num: String,
    license: Option<String>,
}

struct Resolver<'a> {
    defaults: &'a CatalogDefaults,
    local_root: Option<&'a Path>,
    client: Client,
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    match cli.command {
        Commands::Generate(args) => generate(args),
    }
}

fn generate(args: GenerateArgs) -> Result<()> {
    let raw = fs::read_to_string(&args.index)
        .with_context(|| format!("failed to read index file {}", args.index.display()))?;
    let index: CatalogIndex = Options::default()
        .with_default_extension(Extensions::IMPLICIT_SOME)
        .from_str(&raw)
        .with_context(|| format!("failed to parse RON index {}", args.index.display()))?;
    if index.version != 1 {
        bail!("unsupported catalog index version {}", index.version);
    }

    let resolver = Resolver {
        defaults: &index.defaults,
        local_root: args.local_root.as_deref(),
        client: Client::builder()
            .user_agent("cu-catalog/0.1 (+https://github.com/copper-project/copper-rs)")
            .build()
            .context("failed to build HTTP client")?,
    };

    let mut entries = index
        .entries
        .iter()
        .map(|entry| resolve_entry(&resolver, entry))
        .collect::<Result<Vec<_>>>()?;

    entries.sort_by(|left, right| {
        left.copper
            .kind
            .cmp(&right.copper.kind)
            .then_with(|| left.package_name.cmp(&right.package_name))
    });

    fs::create_dir_all(&args.output_dir)
        .with_context(|| format!("failed to create output dir {}", args.output_dir.display()))?;

    let json_path = args.output_dir.join("catalog.json");
    let markdown_path = args.output_dir.join("catalog.md");
    let html_path = args.output_dir.join("index.html");

    fs::write(&json_path, serde_json::to_string_pretty(&entries)?)
        .with_context(|| format!("failed to write {}", json_path.display()))?;
    fs::write(&markdown_path, render_markdown(&entries))
        .with_context(|| format!("failed to write {}", markdown_path.display()))?;
    fs::write(&html_path, render_html(&entries)?)
        .with_context(|| format!("failed to write {}", html_path.display()))?;

    println!("wrote {}", json_path.display());
    println!("wrote {}", markdown_path.display());
    println!("wrote {}", html_path.display());

    Ok(())
}

fn resolve_entry(
    resolver: &Resolver<'_>,
    entry: &CatalogIndexEntry,
) -> Result<ResolvedCatalogEntry> {
    let source_metadata = match &entry.source {
        CatalogSource::GitHub { repo, rev, path } => {
            resolve_github_source(resolver, repo.as_deref(), rev.as_deref(), path)
                .with_context(|| format!("failed to resolve GitHub entry {}", entry.name))?
        }
        CatalogSource::CratesIo {
            crate_name,
            version,
        } => resolve_crates_io_source(resolver, crate_name, version.as_deref())
            .with_context(|| format!("failed to resolve crates.io entry {}", entry.name))?,
    };

    let description = entry
        .overrides
        .description
        .clone()
        .or(source_metadata.description)
        .unwrap_or_default();

    let keywords = normalize_string_vec(
        entry
            .overrides
            .keywords
            .clone()
            .unwrap_or(source_metadata.keywords),
    );
    let categories = normalize_string_vec(
        entry
            .overrides
            .categories
            .clone()
            .unwrap_or(source_metadata.categories),
    );

    let repository = entry
        .overrides
        .repository
        .clone()
        .or(source_metadata.repository);
    let homepage = entry
        .overrides
        .homepage
        .clone()
        .or(source_metadata.homepage);
    let license = entry.overrides.license.clone().or(source_metadata.license);

    let copper = merge_copper_metadata(source_metadata.copper, &entry.overrides)
        .with_context(|| format!("entry {} is missing Copper metadata", entry.name))?;

    Ok(ResolvedCatalogEntry {
        catalog_name: entry.name.clone(),
        package_name: source_metadata.package_name,
        description,
        keywords,
        categories,
        repository,
        homepage,
        license,
        copper,
        source: source_metadata.source,
    })
}

fn resolve_github_source(
    resolver: &Resolver<'_>,
    repo: Option<&str>,
    rev: Option<&str>,
    path: &str,
) -> Result<SourceMetadata> {
    let repo = repo
        .map(ToOwned::to_owned)
        .or_else(|| resolver.defaults.github_repo.clone())
        .ok_or_else(|| anyhow!("missing GitHub repo and no default github_repo configured"))?;
    let rev = rev
        .map(ToOwned::to_owned)
        .or_else(|| resolver.defaults.github_rev.clone())
        .ok_or_else(|| anyhow!("missing GitHub rev and no default github_rev configured"))?;

    let crate_manifest = if let Some(local_root) = resolver.local_root {
        fs::read_to_string(local_root.join(path).join("Cargo.toml")).with_context(|| {
            format!(
                "failed to read local manifest {}",
                local_root.join(path).join("Cargo.toml").display()
            )
        })?
    } else {
        fetch_text(
            &resolver.client,
            &github_raw_url(&repo, &rev, &format!("{path}/Cargo.toml")),
        )?
    };

    let root_manifest = if let Some(local_root) = resolver.local_root {
        fs::read_to_string(local_root.join("Cargo.toml")).with_context(|| {
            format!(
                "failed to read local workspace manifest {}",
                local_root.join("Cargo.toml").display()
            )
        })?
    } else {
        fetch_text(&resolver.client, &github_raw_url(&repo, &rev, "Cargo.toml"))?
    };

    let manifest: Value = toml::from_str(&crate_manifest)
        .with_context(|| format!("failed to parse crate manifest for {repo}:{path}"))?;
    let workspace_manifest: Value = toml::from_str(&root_manifest)
        .with_context(|| format!("failed to parse workspace manifest for repo {repo}"))?;

    let package_name = resolve_package_string(&manifest, Some(&workspace_manifest), "name")
        .with_context(|| format!("crate manifest at {repo}:{path} is missing package.name"))?;
    let description = resolve_package_string(&manifest, Some(&workspace_manifest), "description");
    let keywords = resolve_package_array(&manifest, Some(&workspace_manifest), "keywords");
    let categories = resolve_package_array(&manifest, Some(&workspace_manifest), "categories");
    let repository = resolve_package_string(&manifest, Some(&workspace_manifest), "repository");
    let homepage = resolve_package_string(&manifest, Some(&workspace_manifest), "homepage");
    let license = resolve_package_string(&manifest, Some(&workspace_manifest), "license");
    let copper = resolve_copper_metadata(&manifest)?;

    let readme_path = resolve_package_string(&manifest, Some(&workspace_manifest), "readme")
        .unwrap_or_else(|| "README.md".to_owned());

    Ok(SourceMetadata {
        package_name,
        description,
        keywords,
        categories,
        repository,
        homepage,
        license,
        copper,
        source: ResolvedCatalogSource {
            source_type: "github".to_owned(),
            repo: Some(repo.clone()),
            rev: Some(rev.clone()),
            path: Some(path.to_owned()),
            crate_name: None,
            version: Some(rev.clone()),
            source_url: github_tree_url(&repo, &rev, path),
            manifest_url: Some(github_blob_url(&repo, &rev, &format!("{path}/Cargo.toml"))),
            readme_url: Some(github_blob_url(
                &repo,
                &rev,
                &format!("{path}/{readme_path}"),
            )),
        },
    })
}

fn resolve_crates_io_source(
    resolver: &Resolver<'_>,
    crate_name: &str,
    requested_version: Option<&str>,
) -> Result<SourceMetadata> {
    let url = format!("https://crates.io/api/v1/crates/{crate_name}");
    let response = resolver
        .client
        .get(&url)
        .send()
        .with_context(|| format!("failed to fetch crates.io metadata for {crate_name}"))?
        .error_for_status()
        .with_context(|| format!("crates.io returned an error for {crate_name}"))?
        .json::<CratesIoResponse>()
        .with_context(|| format!("failed to decode crates.io response for {crate_name}"))?;

    let version = requested_version
        .map(ToOwned::to_owned)
        .or_else(|| response.krate.max_stable_version.clone())
        .unwrap_or_else(|| response.krate.newest_version.clone());

    let license = response
        .versions
        .iter()
        .find(|candidate| candidate.num == version)
        .and_then(|candidate| candidate.license.clone());

    Ok(SourceMetadata {
        package_name: response.krate.name.clone(),
        description: response.krate.description.clone(),
        keywords: response.krate.keywords.clone(),
        categories: response.krate.categories.clone(),
        repository: response.krate.repository.clone(),
        homepage: response.krate.homepage.clone(),
        license,
        copper: CopperMetadataPartial::default(),
        source: ResolvedCatalogSource {
            source_type: "crates.io".to_owned(),
            repo: None,
            rev: None,
            path: None,
            crate_name: Some(response.krate.name.clone()),
            version: Some(version.clone()),
            source_url: format!("https://crates.io/crates/{}", response.krate.name),
            manifest_url: None,
            readme_url: None,
        },
    })
}

fn fetch_text(client: &Client, url: &str) -> Result<String> {
    client
        .get(url)
        .send()
        .with_context(|| format!("failed to fetch {url}"))?
        .error_for_status()
        .with_context(|| format!("request returned an error for {url}"))?
        .text()
        .with_context(|| format!("failed to read response body from {url}"))
}

fn resolve_package_string(
    manifest: &Value,
    workspace_manifest: Option<&Value>,
    key: &str,
) -> Option<String> {
    let package = manifest.get("package")?.as_table()?;
    match package.get(key) {
        Some(Value::String(value)) => Some(value.clone()),
        Some(Value::Table(table))
            if table.get("workspace").and_then(Value::as_bool) == Some(true) =>
        {
            resolve_workspace_string(workspace_manifest, key)
        }
        _ => None,
    }
}

fn resolve_package_array(
    manifest: &Value,
    workspace_manifest: Option<&Value>,
    key: &str,
) -> Vec<String> {
    let Some(package) = manifest.get("package").and_then(Value::as_table) else {
        return Vec::new();
    };
    match package.get(key) {
        Some(Value::Array(values)) => values
            .iter()
            .filter_map(Value::as_str)
            .map(ToOwned::to_owned)
            .collect(),
        Some(Value::Table(table))
            if table.get("workspace").and_then(Value::as_bool) == Some(true) =>
        {
            resolve_workspace_array(workspace_manifest, key)
        }
        _ => Vec::new(),
    }
}

fn resolve_workspace_string(workspace_manifest: Option<&Value>, key: &str) -> Option<String> {
    workspace_manifest?
        .get("workspace")?
        .get("package")?
        .get(key)?
        .as_str()
        .map(ToOwned::to_owned)
}

fn resolve_workspace_array(workspace_manifest: Option<&Value>, key: &str) -> Vec<String> {
    workspace_manifest
        .and_then(|manifest| manifest.get("workspace"))
        .and_then(|workspace| workspace.get("package"))
        .and_then(|package| package.get(key))
        .and_then(Value::as_array)
        .map(|values| {
            values
                .iter()
                .filter_map(Value::as_str)
                .map(ToOwned::to_owned)
                .collect()
        })
        .unwrap_or_default()
}

fn resolve_copper_metadata(manifest: &Value) -> Result<CopperMetadataPartial> {
    let Some(package) = manifest.get("package") else {
        return Ok(CopperMetadataPartial::default());
    };
    let Some(metadata) = package.get("metadata") else {
        return Ok(CopperMetadataPartial::default());
    };
    let Some(copper) = metadata.get("copper") else {
        return Ok(CopperMetadataPartial::default());
    };

    copper
        .clone()
        .try_into()
        .context("failed to decode [package.metadata.copper]")
}

fn merge_copper_metadata(
    source: CopperMetadataPartial,
    overrides: &CatalogOverrides,
) -> Result<CopperMetadataResolved> {
    let kind = overrides
        .kind
        .clone()
        .or(source.kind)
        .context("missing kind")?;
    let domains = normalize_string_vec(
        overrides
            .domains
            .clone()
            .or(source.domains)
            .unwrap_or_default(),
    );
    if domains.is_empty() {
        bail!("missing domains");
    }

    let environments = normalize_string_vec(
        overrides
            .environments
            .clone()
            .or(source.environments)
            .unwrap_or_default(),
    );
    if environments.is_empty() {
        bail!("missing environments");
    }

    let environment_set: BTreeSet<_> = environments.iter().cloned().collect();
    let host_os = normalize_string_vec(
        overrides
            .host_os
            .clone()
            .or(source.host_os)
            .unwrap_or_default(),
    );
    let embedded_targets = normalize_string_vec(
        overrides
            .embedded_targets
            .clone()
            .or(source.embedded_targets)
            .unwrap_or_default(),
    );

    if !host_os.is_empty() && !environment_set.contains("host") {
        bail!("host_os requires the host environment");
    }
    if !embedded_targets.is_empty() && !environment_set.contains("embedded") {
        bail!("embedded_targets requires the embedded environment");
    }

    Ok(CopperMetadataResolved {
        kind,
        domains,
        environments,
        host_os,
        embedded_targets,
    })
}

fn normalize_string_vec(values: Vec<String>) -> Vec<String> {
    values
        .into_iter()
        .map(|value| value.trim().to_owned())
        .filter(|value| !value.is_empty())
        .collect::<BTreeSet<_>>()
        .into_iter()
        .collect()
}

fn render_markdown(entries: &[ResolvedCatalogEntry]) -> String {
    let mut counts: BTreeMap<&str, usize> = BTreeMap::new();
    for entry in entries {
        *counts.entry(&entry.copper.kind).or_default() += 1;
    }

    let mut out = String::new();
    out.push_str("# Copper Catalog\n\n");
    out.push_str("Generated from `catalog/index.ron` plus Cargo package metadata.\n\n");
    out.push_str("## Counts\n\n");
    for (kind, count) in counts {
        out.push_str(&format!("- `{kind}`: {count}\n"));
    }
    out.push_str("\n## Entries\n\n");
    out.push_str("| Crate | Kind | Domains | Environments | Description |\n");
    out.push_str("| --- | --- | --- | --- | --- |\n");

    for entry in entries {
        let crate_link = markdown_link(&entry.package_name, &entry.source.source_url);
        let domains = entry.copper.domains.join(", ");
        let environments = render_environment_summary(&entry.copper);
        let description = escape_markdown_cell(&entry.description);
        out.push_str(&format!(
            "| {crate_link} | `{}` | {} | {} | {} |\n",
            entry.copper.kind,
            escape_markdown_cell(&domains),
            escape_markdown_cell(&environments),
            description,
        ));
    }

    out
}

fn render_html(entries: &[ResolvedCatalogEntry]) -> Result<String> {
    let data =
        serde_json::to_string(entries).context("failed to serialize catalog JSON for HTML")?;
    Ok(HTML_TEMPLATE.replace("__CATALOG_DATA__", &data))
}

fn render_environment_summary(copper: &CopperMetadataResolved) -> String {
    let mut pieces = Vec::new();
    pieces.push(copper.environments.join(", "));
    if !copper.host_os.is_empty() {
        pieces.push(format!("host_os: {}", copper.host_os.join(", ")));
    }
    if !copper.embedded_targets.is_empty() {
        pieces.push(format!(
            "embedded_targets: {}",
            copper.embedded_targets.join(", ")
        ));
    }
    pieces.join(" | ")
}

fn markdown_link(label: &str, url: &str) -> String {
    format!(
        "[`{}`]({})",
        escape_markdown_cell(label),
        escape_markdown_cell(url)
    )
}

fn escape_markdown_cell(value: &str) -> String {
    value.replace('|', "\\|").replace('\n', " ")
}

fn github_raw_url(repo: &str, rev: &str, path: &str) -> String {
    format!("https://raw.githubusercontent.com/{repo}/{rev}/{path}")
}

fn github_tree_url(repo: &str, rev: &str, path: &str) -> String {
    format!("https://github.com/{repo}/tree/{rev}/{path}")
}

fn github_blob_url(repo: &str, rev: &str, path: &str) -> String {
    format!("https://github.com/{repo}/blob/{rev}/{path}")
}

const HTML_TEMPLATE: &str = r##"<!DOCTYPE html>
<html lang="en">
  <head>
    <meta charset="utf-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1" />
    <title>Copper Catalog</title>
    <style>
      :root {
        --bg: #120d0b;
        --panel: rgba(30, 20, 16, 0.88);
        --panel-strong: rgba(46, 29, 22, 0.96);
        --line: rgba(215, 154, 112, 0.22);
        --text: #f3e7dc;
        --muted: #c8b2a1;
        --accent: #d98c52;
        --accent-strong: #f3a86c;
        --chip: rgba(217, 140, 82, 0.14);
        --chip-text: #ffd7bb;
        --shadow: 0 18px 54px rgba(0, 0, 0, 0.32);
      }

      * {
        box-sizing: border-box;
      }

      body {
        margin: 0;
        min-height: 100vh;
        font-family: "Iowan Old Style", "Palatino Linotype", "Book Antiqua", Georgia, serif;
        color: var(--text);
        background:
          radial-gradient(circle at top left, rgba(217, 140, 82, 0.18), transparent 34%),
          radial-gradient(circle at top right, rgba(84, 134, 150, 0.16), transparent 28%),
          linear-gradient(180deg, #1b120f 0%, #120d0b 55%, #0b0908 100%);
      }

      body::before {
        content: "";
        position: fixed;
        inset: 0;
        pointer-events: none;
        background-image:
          linear-gradient(rgba(255, 255, 255, 0.02) 1px, transparent 1px),
          linear-gradient(90deg, rgba(255, 255, 255, 0.02) 1px, transparent 1px);
        background-size: 36px 36px;
        mask-image: radial-gradient(circle at center, black 52%, transparent 90%);
      }

      main {
        width: min(1320px, calc(100vw - 32px));
        margin: 0 auto;
        padding: 48px 0 72px;
      }

      .hero {
        padding: 28px;
        border: 1px solid var(--line);
        border-radius: 28px;
        background: linear-gradient(140deg, rgba(45, 28, 20, 0.98), rgba(20, 16, 14, 0.92));
        box-shadow: var(--shadow);
      }

      .eyebrow {
        margin: 0 0 12px;
        color: var(--accent-strong);
        text-transform: uppercase;
        letter-spacing: 0.16em;
        font-size: 0.76rem;
      }

      h1 {
        margin: 0;
        font-size: clamp(2.4rem, 4vw, 4.3rem);
        line-height: 0.98;
      }

      .lede {
        margin: 14px 0 0;
        max-width: 68ch;
        color: var(--muted);
        font-size: 1.06rem;
      }

      .toolbar {
        margin-top: 24px;
        display: grid;
        grid-template-columns: minmax(0, 1.5fr) repeat(3, minmax(0, 0.7fr));
        gap: 12px;
      }

      .field {
        display: flex;
        flex-direction: column;
        gap: 6px;
      }

      .field label {
        color: var(--muted);
        font-size: 0.8rem;
        letter-spacing: 0.06em;
        text-transform: uppercase;
      }

      .field input,
      .field select {
        width: 100%;
        padding: 14px 16px;
        border-radius: 16px;
        border: 1px solid rgba(255, 255, 255, 0.08);
        background: rgba(13, 10, 9, 0.86);
        color: var(--text);
        font: inherit;
      }

      .meta {
        margin-top: 18px;
        display: flex;
        flex-wrap: wrap;
        gap: 10px;
        align-items: center;
        color: var(--muted);
      }

      .count-pill {
        padding: 8px 12px;
        border: 1px solid var(--line);
        border-radius: 999px;
        background: rgba(0, 0, 0, 0.2);
      }

      .grid {
        margin-top: 28px;
        display: grid;
        grid-template-columns: repeat(auto-fill, minmax(290px, 1fr));
        gap: 16px;
      }

      .card {
        padding: 20px;
        border-radius: 24px;
        border: 1px solid var(--line);
        background: linear-gradient(180deg, var(--panel-strong), var(--panel));
        box-shadow: var(--shadow);
      }

      .card-header {
        display: flex;
        justify-content: space-between;
        gap: 12px;
        align-items: start;
      }

      .card h2 {
        margin: 0;
        font-size: 1.34rem;
      }

      .crate-link {
        color: var(--text);
        text-decoration: none;
      }

      .crate-link:hover {
        color: var(--accent-strong);
      }

      .kind {
        padding: 6px 10px;
        border-radius: 999px;
        background: rgba(243, 168, 108, 0.15);
        color: var(--accent-strong);
        font-family: "JetBrains Mono", "Fira Code", "SFMono-Regular", monospace;
        font-size: 0.78rem;
      }

      .description {
        margin: 14px 0 0;
        color: var(--muted);
        min-height: 3.4em;
      }

      .chips {
        margin-top: 16px;
        display: flex;
        flex-wrap: wrap;
        gap: 8px;
      }

      .chip {
        padding: 6px 10px;
        border-radius: 999px;
        background: var(--chip);
        color: var(--chip-text);
        border: 1px solid rgba(255, 255, 255, 0.06);
        font-size: 0.82rem;
      }

      .links {
        margin-top: 18px;
        display: flex;
        flex-wrap: wrap;
        gap: 12px;
      }

      .links a {
        color: var(--accent-strong);
        text-decoration: none;
      }

      .links a:hover {
        text-decoration: underline;
      }

      .empty {
        margin-top: 26px;
        padding: 26px;
        border-radius: 20px;
        border: 1px dashed var(--line);
        color: var(--muted);
        text-align: center;
        display: none;
      }

      @media (max-width: 920px) {
        .toolbar {
          grid-template-columns: 1fr;
        }
      }
    </style>
  </head>
  <body>
    <main>
      <section class="hero">
        <p class="eyebrow">Copper Component Catalog</p>
        <h1>Find drivers, tasks, bridges, payloads, and embedded pieces fast.</h1>
        <p class="lede">
          This page is generated from <code>catalog/index.ron</code> and Cargo metadata.
          Filter by kind, domain, runtime environment, or just search by crate name and description.
        </p>
        <div class="toolbar">
          <div class="field">
            <label for="search">Search</label>
            <input id="search" type="search" placeholder="lidar, zenoh, imu, safety..." />
          </div>
          <div class="field">
            <label for="kind">Kind</label>
            <select id="kind"></select>
          </div>
          <div class="field">
            <label for="domain">Domain</label>
            <select id="domain"></select>
          </div>
          <div class="field">
            <label for="environment">Environment</label>
            <select id="environment"></select>
          </div>
        </div>
        <div class="meta">
          <span class="count-pill" id="resultCount"></span>
        </div>
      </section>
      <section class="grid" id="grid"></section>
      <section class="empty" id="emptyState">No components match the current filters.</section>
    </main>

    <script>
      const CATALOG = __CATALOG_DATA__;

      const searchInput = document.getElementById("search");
      const kindSelect = document.getElementById("kind");
      const domainSelect = document.getElementById("domain");
      const environmentSelect = document.getElementById("environment");
      const grid = document.getElementById("grid");
      const resultCount = document.getElementById("resultCount");
      const emptyState = document.getElementById("emptyState");

      function uniqueValues(values) {
        return [...new Set(values)].sort((left, right) => left.localeCompare(right));
      }

      function fillSelect(select, label, values) {
        select.innerHTML = "";
        const allOption = document.createElement("option");
        allOption.value = "";
        allOption.textContent = `All ${label}`;
        select.appendChild(allOption);
        values.forEach((value) => {
          const option = document.createElement("option");
          option.value = value;
          option.textContent = value;
          select.appendChild(option);
        });
      }

      function renderLinks(entry) {
        const links = [];
        links.push(`<a href="${entry.source.source_url}" target="_blank" rel="noreferrer">source</a>`);
        if (entry.repository) {
          links.push(`<a href="${entry.repository}" target="_blank" rel="noreferrer">repo</a>`);
        }
        if (entry.homepage) {
          links.push(`<a href="${entry.homepage}" target="_blank" rel="noreferrer">homepage</a>`);
        }
        if (entry.source.readme_url) {
          links.push(`<a href="${entry.source.readme_url}" target="_blank" rel="noreferrer">readme</a>`);
        }
        return links.join(" · ");
      }

      function renderChips(values) {
        return values.map((value) => `<span class="chip">${value}</span>`).join("");
      }

      function renderEntries(entries) {
        grid.innerHTML = entries
          .map((entry) => {
            const envChips = [
              ...entry.copper.environments,
              ...entry.copper.host_os.map((value) => `host:${value}`),
              ...entry.copper.embedded_targets.map((value) => `target:${value}`),
            ];
            return `
              <article class="card">
                <div class="card-header">
                  <div>
                    <h2>
                      <a class="crate-link" href="${entry.source.source_url}" target="_blank" rel="noreferrer">${entry.package_name}</a>
                    </h2>
                  </div>
                  <span class="kind">${entry.copper.kind}</span>
                </div>
                <p class="description">${entry.description || "No description provided."}</p>
                <div class="chips">${renderChips(entry.copper.domains)}</div>
                <div class="chips">${renderChips(envChips)}</div>
                <div class="links">${renderLinks(entry)}</div>
              </article>
            `;
          })
          .join("");

        resultCount.textContent = `${entries.length} component${entries.length === 1 ? "" : "s"} shown`;
        emptyState.style.display = entries.length === 0 ? "block" : "none";
      }

      function matches(entry) {
        const query = searchInput.value.trim().toLowerCase();
        const kind = kindSelect.value;
        const domain = domainSelect.value;
        const environment = environmentSelect.value;

        if (kind && entry.copper.kind !== kind) {
          return false;
        }
        if (domain && !entry.copper.domains.includes(domain)) {
          return false;
        }
        if (environment && !entry.copper.environments.includes(environment)) {
          return false;
        }
        if (!query) {
          return true;
        }

        const haystack = [
          entry.package_name,
          entry.catalog_name,
          entry.description,
          ...(entry.keywords || []),
          ...(entry.categories || []),
          ...entry.copper.domains,
        ]
          .join(" ")
          .toLowerCase();

        return haystack.includes(query);
      }

      function applyFilters() {
        renderEntries(CATALOG.filter(matches));
      }

      fillSelect(kindSelect, "Kinds", uniqueValues(CATALOG.map((entry) => entry.copper.kind)));
      fillSelect(
        domainSelect,
        "Domains",
        uniqueValues(CATALOG.flatMap((entry) => entry.copper.domains))
      );
      fillSelect(
        environmentSelect,
        "Environments",
        uniqueValues(CATALOG.flatMap((entry) => entry.copper.environments))
      );

      [searchInput, kindSelect, domainSelect, environmentSelect].forEach((element) => {
        element.addEventListener("input", applyFilters);
        element.addEventListener("change", applyFilters);
      });

      applyFilters();
    </script>
  </body>
</html>
"##;
