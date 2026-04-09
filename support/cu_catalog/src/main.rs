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
    #[serde(default)]
    allow_missing_copper_metadata: bool,
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
    copper: Option<CopperMetadataResolved>,
    visual_url: Option<String>,
    metadata_warnings: Vec<String>,
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

const ASSET_EXTENSIONS: [&str; 5] = ["svg", "webp", "png", "jpg", "jpeg"];

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
    validate_sorted_entries(&index.entries)
        .context("catalog index entries must be sorted alphabetically by name")?;

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

    entries.sort_by(|left, right| left.catalog_name.cmp(&right.catalog_name));

    fs::create_dir_all(&args.output_dir)
        .with_context(|| format!("failed to create output dir {}", args.output_dir.display()))?;

    let assets_root = args
        .index
        .parent()
        .map(|parent| parent.join("assets"))
        .unwrap_or_else(|| PathBuf::from("assets"));
    attach_visual_urls(&mut entries, &assets_root, &args.output_dir)?;

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

fn validate_sorted_entries(entries: &[CatalogIndexEntry]) -> Result<()> {
    for pair in entries.windows(2) {
        let left = &pair[0];
        let right = &pair[1];
        if left.name > right.name {
            bail!(
                "entry {} is out of order; it must come after {}",
                left.name,
                right.name
            );
        }
    }

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

    let (copper, metadata_warnings) =
        resolve_catalog_copper(&source_metadata.copper, &entry.overrides)
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
        visual_url: None,
        metadata_warnings,
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

    let local_manifest_path = resolver
        .local_root
        .map(|local_root| local_root.join(path).join("Cargo.toml"));
    let use_local_checkout = local_manifest_path
        .as_ref()
        .is_some_and(|manifest_path| manifest_path.is_file());

    let crate_manifest = if use_local_checkout {
        let manifest_path = local_manifest_path
            .as_ref()
            .expect("local manifest path exists when use_local_checkout is true");
        fs::read_to_string(manifest_path)
            .with_context(|| format!("failed to read local manifest {}", manifest_path.display()))?
    } else {
        fetch_text(
            &resolver.client,
            &github_raw_url(&repo, &rev, &format!("{path}/Cargo.toml")),
        )?
    };

    let root_manifest = if use_local_checkout {
        let local_root = resolver
            .local_root
            .expect("local root exists when use_local_checkout is true");
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

    let readme_path = resolve_package_string(&manifest, Some(&workspace_manifest), "readme");
    let readme_url = if let Some(readme_path) = readme_path {
        Some(github_blob_url(
            &repo,
            &rev,
            &format!("{path}/{readme_path}"),
        ))
    } else if use_local_checkout
        && resolver
            .local_root
            .is_some_and(|local_root| local_root.join(path).join("README.md").is_file())
    {
        Some(github_blob_url(&repo, &rev, &format!("{path}/README.md")))
    } else {
        None
    };

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
            readme_url,
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
    source: &CopperMetadataPartial,
    overrides: &CatalogOverrides,
) -> Result<CopperMetadataResolved> {
    let kind = overrides
        .kind
        .clone()
        .or_else(|| source.kind.clone())
        .context("missing kind")?;
    let domains = normalize_string_vec(
        overrides
            .domains
            .clone()
            .or_else(|| source.domains.clone())
            .unwrap_or_default(),
    );
    if domains.is_empty() {
        bail!("missing domains");
    }

    let environments = normalize_string_vec(
        overrides
            .environments
            .clone()
            .or_else(|| source.environments.clone())
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
            .or_else(|| source.host_os.clone())
            .unwrap_or_default(),
    );
    let embedded_targets = normalize_string_vec(
        overrides
            .embedded_targets
            .clone()
            .or_else(|| source.embedded_targets.clone())
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

fn resolve_catalog_copper(
    source: &CopperMetadataPartial,
    overrides: &CatalogOverrides,
) -> Result<(Option<CopperMetadataResolved>, Vec<String>)> {
    match merge_copper_metadata(source, overrides) {
        Ok(copper) => Ok((Some(copper), Vec::new())),
        Err(_err) if overrides.allow_missing_copper_metadata => Ok((
            None,
            vec![summarize_copper_metadata_issue(source, overrides)],
        )),
        Err(err) => Err(err),
    }
}

fn summarize_copper_metadata_issue(
    source: &CopperMetadataPartial,
    overrides: &CatalogOverrides,
) -> String {
    let mut missing = Vec::new();

    if overrides.kind.as_ref().or(source.kind.as_ref()).is_none() {
        missing.push("kind");
    }

    let domains = normalize_string_vec(
        overrides
            .domains
            .clone()
            .or_else(|| source.domains.clone())
            .unwrap_or_default(),
    );
    if domains.is_empty() {
        missing.push("domains");
    }

    let environments = normalize_string_vec(
        overrides
            .environments
            .clone()
            .or_else(|| source.environments.clone())
            .unwrap_or_default(),
    );
    if environments.is_empty() {
        missing.push("environments");
    }

    if missing.is_empty() {
        "Copper metadata incomplete".to_owned()
    } else {
        format!("Copper metadata missing: {}", missing.join(", "))
    }
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

fn attach_visual_urls(
    entries: &mut [ResolvedCatalogEntry],
    assets_root: &Path,
    output_dir: &Path,
) -> Result<()> {
    let output_dir = fs::canonicalize(output_dir)
        .with_context(|| format!("failed to canonicalize output dir {}", output_dir.display()))?;
    let components_dir = assets_root.join("components");
    let domains_dir = assets_root.join("domains");
    let kinds_dir = assets_root.join("kinds");

    for entry in entries {
        entry.visual_url =
            resolve_visual_url(entry, &components_dir, &domains_dir, &kinds_dir, &output_dir)?;
    }

    Ok(())
}

fn resolve_visual_url(
    entry: &ResolvedCatalogEntry,
    components_dir: &Path,
    domains_dir: &Path,
    kinds_dir: &Path,
    output_dir: &Path,
) -> Result<Option<String>> {
    let Some(asset_path) = find_visual_asset(entry, components_dir, domains_dir, kinds_dir) else {
        return Ok(None);
    };

    let asset_path = fs::canonicalize(&asset_path)
        .with_context(|| format!("failed to canonicalize asset {}", asset_path.display()))?;
    let relative = relative_path(output_dir, &asset_path);

    Ok(Some(relative.to_string_lossy().replace('\\', "/")))
}

fn find_visual_asset(
    entry: &ResolvedCatalogEntry,
    components_dir: &Path,
    domains_dir: &Path,
    kinds_dir: &Path,
) -> Option<PathBuf> {
    find_asset_by_stem(components_dir, &entry.catalog_name)
        .or_else(|| {
            entry.copper.as_ref().and_then(|copper| {
                copper.domains.iter().find_map(|domain| {
                    let stem = domain.replace('/', "-");
                    find_asset_by_stem(domains_dir, &stem)
                })
            })
        })
        .or_else(|| {
            entry.copper
                .as_ref()
                .and_then(|copper| find_asset_by_stem(kinds_dir, &copper.kind))
        })
        .or_else(|| find_asset_by_stem(kinds_dir, "unclassified"))
}

fn find_asset_by_stem(dir: &Path, stem: &str) -> Option<PathBuf> {
    ASSET_EXTENSIONS
        .iter()
        .map(|extension| dir.join(format!("{stem}.{extension}")))
        .find(|candidate| candidate.is_file())
}

fn relative_path(from_dir: &Path, to_path: &Path) -> PathBuf {
    let from_components: Vec<_> = from_dir.components().collect();
    let to_components: Vec<_> = to_path.components().collect();
    let common_len = from_components
        .iter()
        .zip(to_components.iter())
        .take_while(|(left, right)| left == right)
        .count();

    let mut relative = PathBuf::new();
    for _ in common_len..from_components.len() {
        relative.push("..");
    }
    for component in &to_components[common_len..] {
        relative.push(component.as_os_str());
    }

    relative
}

fn render_markdown(entries: &[ResolvedCatalogEntry]) -> String {
    let mut counts: BTreeMap<&str, usize> = BTreeMap::new();
    for entry in entries {
        let kind = entry
            .copper
            .as_ref()
            .map(|copper| copper.kind.as_str())
            .unwrap_or("unclassified");
        *counts.entry(kind).or_default() += 1;
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
        let kind = entry
            .copper
            .as_ref()
            .map(|copper| copper.kind.as_str())
            .unwrap_or("unclassified");
        let domains = entry
            .copper
            .as_ref()
            .map(|copper| copper.domains.join(", "))
            .unwrap_or_else(|| entry.metadata_warnings.join("; "));
        let environments = entry
            .copper
            .as_ref()
            .map(render_environment_summary)
            .unwrap_or_else(|| "metadata needed".to_owned());
        let description = escape_markdown_cell(&entry.description);
        out.push_str(&format!(
            "| {crate_link} | `{}` | {} | {} | {} |\n",
            kind,
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
    <title>copper-rs component catalog</title>
    <style>
      :root {
        --bg: #171c27;
        --bg-deep: #0f141c;
        --panel: rgba(27, 34, 47, 0.9);
        --panel-strong: rgba(35, 43, 59, 0.96);
        --field-bg: rgba(11, 15, 24, 0.92);
        --line: rgba(111, 145, 215, 0.22);
        --line-strong: rgba(111, 145, 215, 0.44);
        --text: #eef3ff;
        --muted: #aab6cd;
        --accent: #73a8ff;
        --accent-strong: #9ac8ff;
        --chip: rgba(115, 168, 255, 0.12);
        --chip-text: #dbe8ff;
        --warn: #ffcc7a;
        --warn-bg: rgba(255, 204, 122, 0.14);
        --warn-line: rgba(255, 204, 122, 0.28);
        --shadow: 0 24px 72px rgba(4, 8, 17, 0.42);
      }

      * {
        box-sizing: border-box;
      }

      body {
        margin: 0;
        min-height: 100vh;
        font-family: "Avenir Next", "Segoe UI Variable", "Inter", "Helvetica Neue", Arial, sans-serif;
        color: var(--text);
        background:
          radial-gradient(circle at top left, rgba(115, 168, 255, 0.12), transparent 34%),
          radial-gradient(circle at 86% 14%, rgba(71, 95, 150, 0.18), transparent 24%),
          linear-gradient(180deg, #232938 0%, var(--bg) 28%, var(--bg-deep) 100%);
      }

      body::before {
        content: "";
        position: fixed;
        inset: 0;
        pointer-events: none;
        background-image:
          linear-gradient(rgba(136, 156, 204, 0.035) 1px, transparent 1px),
          linear-gradient(90deg, rgba(136, 156, 204, 0.035) 1px, transparent 1px);
        background-size: 34px 34px;
        mask-image: linear-gradient(180deg, black 0%, rgba(0, 0, 0, 0.88) 54%, transparent 96%);
      }

      main {
        width: min(1360px, calc(100vw - 32px));
        margin: 0 auto;
        padding: 36px 0 80px;
      }

      .hero {
        position: relative;
        overflow: hidden;
        padding: 24px 28px 24px;
        border: 1px solid var(--line);
        border-radius: 24px;
        background: linear-gradient(180deg, rgba(31, 39, 54, 0.96), rgba(20, 25, 36, 0.92));
        box-shadow: var(--shadow);
      }

      .hero-header {
        display: flex;
        align-items: center;
      }

      .hero-link {
        display: flex;
        align-items: center;
        gap: 14px;
        color: inherit;
        text-decoration: none;
      }

      .hero-link:hover h1 {
        color: var(--accent-strong);
      }

      .brand-logo {
        width: 52px;
        height: 52px;
        flex: 0 0 auto;
        background-image: url("data:image/png;base64,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");
        background-position: center;
        background-repeat: no-repeat;
        background-size: contain;
      }

      h1 {
        margin: 0;
        max-width: none;
        font-size: clamp(1.45rem, 1.9vw, 2.15rem);
        line-height: 0.98;
        letter-spacing: -0.045em;
        font-weight: 740;
      }

      .toolbar {
        margin-top: 26px;
        display: grid;
        grid-template-columns: minmax(0, 1.7fr) repeat(3, minmax(0, 0.8fr));
        gap: 14px;
      }

      .field {
        display: flex;
        flex-direction: column;
        gap: 8px;
      }

      .field label {
        color: var(--muted);
        font-size: 0.8rem;
        letter-spacing: 0.08em;
        text-transform: uppercase;
        font-weight: 650;
      }

      .field input,
      .field select {
        width: 100%;
        padding: 15px 16px;
        border-radius: 14px;
        border: 1px solid rgba(255, 255, 255, 0.07);
        background: var(--field-bg);
        color: var(--text);
        font: inherit;
        box-shadow: inset 0 1px 0 rgba(255, 255, 255, 0.02);
        transition:
          border-color 120ms ease,
          box-shadow 120ms ease,
          background 120ms ease;
      }

      .field input::placeholder {
        color: #7f8ca6;
      }

      .field input:focus,
      .field select:focus {
        outline: none;
        border-color: var(--line-strong);
        box-shadow: 0 0 0 4px rgba(115, 168, 255, 0.12);
      }

      .meta {
        margin-top: 20px;
        display: flex;
        flex-wrap: wrap;
        gap: 10px;
        align-items: center;
        color: var(--muted);
      }

      .count-pill {
        padding: 9px 14px;
        border: 1px solid var(--line);
        border-radius: 999px;
        background: rgba(9, 12, 19, 0.68);
      }

      .grid {
        margin-top: 30px;
        display: grid;
        grid-template-columns: repeat(auto-fill, minmax(290px, 1fr));
        gap: 18px;
      }

      .card {
        position: relative;
        overflow: hidden;
        padding: 22px;
        border-radius: 22px;
        border: 1px solid var(--line);
        background: linear-gradient(180deg, var(--panel-strong), var(--panel));
        box-shadow: var(--shadow);
        transition:
          transform 140ms ease,
          border-color 140ms ease,
          background 140ms ease;
      }

      .card:hover {
        transform: translateY(-2px);
        border-color: var(--line-strong);
        background: linear-gradient(180deg, rgba(39, 48, 66, 0.98), rgba(22, 28, 39, 0.94));
      }

      .card-media {
        margin: -22px -22px 18px;
        padding: 18px;
        aspect-ratio: 16 / 9;
        display: flex;
        align-items: center;
        justify-content: center;
        border-bottom: 1px solid rgba(255, 255, 255, 0.06);
        background:
          radial-gradient(circle at top left, rgba(115, 168, 255, 0.18), transparent 42%),
          linear-gradient(180deg, rgba(18, 24, 34, 0.98), rgba(10, 13, 19, 0.94));
      }

      .card-media img {
        width: 100%;
        height: 100%;
        display: block;
        object-fit: contain;
        filter: drop-shadow(0 12px 28px rgba(0, 0, 0, 0.32));
      }

      .card-header {
        display: flex;
        justify-content: space-between;
        gap: 12px;
        align-items: start;
      }

      .card h2 {
        margin: 0;
        font-size: 1.36rem;
        line-height: 1.15;
        font-weight: 720;
      }

      .crate-link {
        color: var(--text);
        text-decoration: none;
      }

      .crate-link:hover {
        color: var(--accent-strong);
      }

      .kind {
        --kind-border: rgba(115, 168, 255, 0.18);
        --kind-bg: rgba(115, 168, 255, 0.14);
        --kind-text: var(--accent-strong);
        padding: 6px 10px;
        border-radius: 999px;
        border: 1px solid var(--kind-border);
        background: var(--kind-bg);
        color: var(--kind-text);
        font-family: "JetBrains Mono", "Fira Code", "SFMono-Regular", monospace;
        font-size: 0.78rem;
        letter-spacing: 0.02em;
      }

      .kind-source {
        --kind-border: rgba(132, 211, 255, 0.3);
        --kind-bg: rgba(132, 211, 255, 0.12);
        --kind-text: #8edcff;
      }

      .kind-sink {
        --kind-border: rgba(255, 202, 147, 0.32);
        --kind-bg: rgba(255, 202, 147, 0.12);
        --kind-text: #ffd4a6;
      }

      .kind-bridge {
        --kind-border: rgba(138, 240, 228, 0.3);
        --kind-bg: rgba(138, 240, 228, 0.12);
        --kind-text: #93f4e8;
      }

      .kind-task {
        --kind-border: rgba(154, 239, 175, 0.3);
        --kind-bg: rgba(154, 239, 175, 0.12);
        --kind-text: #a8f5b9;
      }

      .kind-payload {
        --kind-border: rgba(255, 179, 154, 0.3);
        --kind-bg: rgba(255, 179, 154, 0.12);
        --kind-text: #ffc0ab;
      }

      .kind-resource {
        --kind-border: rgba(197, 229, 242, 0.3);
        --kind-bg: rgba(197, 229, 242, 0.12);
        --kind-text: #d7eff8;
      }

      .kind-monitor {
        --kind-border: rgba(136, 224, 255, 0.3);
        --kind-bg: rgba(136, 224, 255, 0.12);
        --kind-text: #95e5ff;
      }

      .kind-codec {
        --kind-border: rgba(255, 224, 139, 0.3);
        --kind-bg: rgba(255, 224, 139, 0.12);
        --kind-text: #ffe7a1;
      }

      .kind-lib {
        --kind-border: rgba(214, 230, 255, 0.3);
        --kind-bg: rgba(214, 230, 255, 0.12);
        --kind-text: #e0ecff;
      }

      .kind-testing {
        --kind-border: rgba(240, 176, 255, 0.3);
        --kind-bg: rgba(240, 176, 255, 0.12);
        --kind-text: #f3bcff;
      }

      .kind-unclassified {
        --kind-border: rgba(217, 223, 236, 0.3);
        --kind-bg: rgba(217, 223, 236, 0.12);
        --kind-text: #edf1fb;
      }

      .kind-missing {
        border-color: var(--warn-line);
        background: var(--warn-bg);
        color: var(--warn);
      }

      .description {
        margin: 14px 0 0;
        color: var(--muted);
        min-height: 3.9em;
        line-height: 1.55;
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

      .chip-warning {
        background: var(--warn-bg);
        color: #ffe2b2;
        border-color: var(--warn-line);
      }

      .links {
        margin-top: 18px;
        display: flex;
        flex-wrap: wrap;
        gap: 12px;
        font-weight: 600;
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

      .contribute {
        margin-top: 26px;
        padding: 22px 24px;
        border: 1px solid var(--line);
        border-radius: 20px;
        background: linear-gradient(180deg, rgba(24, 31, 43, 0.94), rgba(17, 22, 31, 0.94));
        box-shadow: var(--shadow);
      }

      .contribute p {
        margin: 0;
        display: flex;
        align-items: center;
        flex-wrap: wrap;
        gap: 12px;
        color: var(--muted);
        line-height: 1.6;
      }

      .contribute-icon {
        width: 22px;
        height: 22px;
        flex: 0 0 auto;
        color: var(--text);
      }

      .contribute-icon svg {
        display: block;
        width: 100%;
        height: 100%;
        fill: currentColor;
      }

      .contribute a {
        color: var(--accent-strong);
        font-weight: 650;
        text-decoration: none;
      }

      .contribute a:hover {
        text-decoration: underline;
      }

      @media (max-width: 920px) {
        .hero-header {
          align-items: flex-start;
        }

        .toolbar {
          grid-template-columns: 1fr;
        }
      }

      @media (max-width: 640px) {
        main {
          width: min(100%, calc(100vw - 20px));
          padding-top: 18px;
        }

        .hero {
          padding: 20px 18px 20px;
          border-radius: 20px;
        }

        .brand-logo {
          width: 42px;
          height: 42px;
        }

        h1 {
          font-size: clamp(1.25rem, 4.8vw, 1.7rem);
        }
      }
    </style>
  </head>
  <body>
    <main>
      <section class="hero">
        <div class="hero-header">
          <a
            class="hero-link"
            href="https://github.com/copper-project/copper-rs/blob/master/README.md"
            target="_blank"
            rel="noreferrer"
          >
            <div class="brand-logo" aria-hidden="true"></div>
            <h1>copper-rs component catalog</h1>
          </a>
        </div>
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
      <section class="contribute">
        <p>
          <span class="contribute-icon" aria-hidden="true">
            <svg viewBox="0 0 24 24" aria-hidden="true">
              <path
                d="M12 0C5.37 0 0 5.37 0 12c0 5.3 3.438 9.8 8.205 11.385.6.113.82-.258.82-.577
                0-.285-.01-1.04-.015-2.04-3.338.724-4.042-1.61-4.042-1.61-.546-1.385-1.333-1.755-1.333-1.755
                -1.089-.744.084-.729.084-.729 1.205.084 1.839 1.236 1.839 1.236 1.07 1.835 2.807 1.305 3.492.998
                .108-.776.418-1.305.762-1.605-2.665-.3-5.466-1.335-5.466-5.93 0-1.31.465-2.38 1.235-3.22
                -.135-.303-.54-1.523.105-3.176 0 0 1.005-.322 3.3 1.23a11.5 11.5 0 0 1 3.003-.404
                c1.02.005 2.045.138 3.005.404 2.295-1.552 3.295-1.23 3.295-1.23.645 1.653.24 2.873.12 3.176
                .765.84 1.23 1.91 1.23 3.22 0 4.61-2.805 5.625-5.475 5.92.43.37.825 1.102.825 2.222
                0 1.606-.015 2.896-.015 3.286 0 .315.21.69.825.57C20.565 21.795 24 17.295 24 12
                24 5.37 18.63 0 12 0Z"
              />
            </svg>
          </span>
          <span>You have a component to share? Open a PR and add yours to</span>
          <a
            href="https://github.com/copper-project/copper-rs/blob/master/catalog/index.ron"
            target="_blank"
            rel="noreferrer"
            >catalog/index.ron</a
          ><span>.</span>
        </p>
      </section>
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
        if (entry.source.manifest_url && !entry.source.readme_url) {
          links.push(`<a href="${entry.source.manifest_url}" target="_blank" rel="noreferrer">manifest</a>`);
        }
        if (entry.source.readme_url) {
          links.push(`<a href="${entry.source.readme_url}" target="_blank" rel="noreferrer">readme</a>`);
        }
        return links.join(" · ");
      }

      function renderChips(values) {
        return values.map((value) => `<span class="chip">${value}</span>`).join("");
      }

      function renderWarningChips(values) {
        return values.map((value) => `<span class="chip chip-warning">${value}</span>`).join("");
      }

      function renderVisual(entry) {
        if (!entry.visual_url) {
          return "";
        }
        return `
          <div class="card-media">
            <img src="${entry.visual_url}" alt="${entry.package_name} visual" loading="lazy" />
          </div>
        `;
      }

      function renderEntries(entries) {
        grid.innerHTML = entries
          .map((entry) => {
            const copper = entry.copper;
            const envChips = copper
              ? [
                  ...copper.environments,
                  ...copper.host_os.map((value) => `host:${value}`),
                  ...copper.embedded_targets.map((value) => `target:${value}`),
                ]
              : [];
            const kindLabel = copper ? copper.kind : "needs metadata";
            const kindClass = copper ? `kind kind-${copper.kind}` : "kind kind-missing";
            const domainChips = copper ? renderChips(copper.domains) : "";
            const warningChips = renderWarningChips(entry.metadata_warnings || []);
            return `
              <article class="card">
                ${renderVisual(entry)}
                <div class="card-header">
                  <div>
                    <h2>
                      <a class="crate-link" href="${entry.source.source_url}" target="_blank" rel="noreferrer">${entry.package_name}</a>
                    </h2>
                  </div>
                  <span class="${kindClass}">${kindLabel}</span>
                </div>
                <p class="description">${entry.description || "No description provided."}</p>
                ${domainChips ? `<div class="chips">${domainChips}</div>` : ""}
                ${envChips.length ? `<div class="chips">${renderChips(envChips)}</div>` : ""}
                ${warningChips ? `<div class="chips">${warningChips}</div>` : ""}
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
        const copper = entry.copper;

        if (kind && copper?.kind !== kind) {
          return false;
        }
        if (domain && !copper?.domains.includes(domain)) {
          return false;
        }
        if (environment && !copper?.environments.includes(environment)) {
          return false;
        }
        if (!query) {
          return true;
        }

        const haystack = [
          entry.package_name,
          entry.catalog_name,
          entry.description,
          ...(entry.metadata_warnings || []),
          ...(entry.keywords || []),
          ...(entry.categories || []),
          ...(copper?.domains || []),
        ]
          .join(" ")
          .toLowerCase();

        return haystack.includes(query);
      }

      function applyFilters() {
        renderEntries(CATALOG.filter(matches));
      }

      fillSelect(
        kindSelect,
        "Kinds",
        uniqueValues(CATALOG.flatMap((entry) => (entry.copper ? [entry.copper.kind] : [])))
      );
      fillSelect(
        domainSelect,
        "Domains",
        uniqueValues(CATALOG.flatMap((entry) => (entry.copper ? entry.copper.domains : [])))
      );
      fillSelect(
        environmentSelect,
        "Environments",
        uniqueValues(
          CATALOG.flatMap((entry) => (entry.copper ? entry.copper.environments : []))
        )
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
