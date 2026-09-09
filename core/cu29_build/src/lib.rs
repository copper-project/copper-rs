#![doc = include_str!("../README.md")]

use sha2::{Digest, Sha256};
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

/// Environment variable used to pass the consuming crate's Cargo features to
/// Copper's procedural macros and generated runtime.
pub const COPPER_CFG_FEATURES_ENV: &str = "COPPER_CFG_FEATURES";

/// Name of the private build artifact consumed by Copper's runtime generator.
pub const COPPER_BUILD_METADATA_FILE: &str = "copper-build-metadata-v1.txt";

/// Reproducibility metadata collected from Cargo and Git during an application build.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct CopperBuildMetadata {
    pub git_repository: Option<String>,
    pub git_commit: Option<String>,
    pub git_dirty: Option<bool>,
    pub source_archive_hash: Option<String>,
    pub cargo_manifest_path: Option<String>,
    pub cargo_features: Vec<String>,
    pub rust_target: Option<String>,
    pub rust_toolchain: Option<String>,
    pub dependency_resolution: Option<String>,
}

/// Emit the standard Cargo build-script configuration required by Copper.
pub fn setup() {
    println!(
        "cargo::rustc-env=LOG_INDEX_DIR={}",
        std::env::var("OUT_DIR").expect("Cargo must provide OUT_DIR")
    );

    let features = normalize_features(&std::env::var("CARGO_CFG_FEATURE").unwrap_or_default());

    println!(
        "cargo::rustc-env={COPPER_CFG_FEATURES_ENV}={}",
        features.join(",")
    );

    let metadata = collect_build_metadata(features);
    let out_dir = PathBuf::from(std::env::var("OUT_DIR").expect("Cargo must provide OUT_DIR"));
    fs::write(
        out_dir.join(COPPER_BUILD_METADATA_FILE),
        encode_build_metadata(&metadata),
    )
    .expect("failed to write Copper build metadata");
}

fn normalize_features(value: &str) -> Vec<String> {
    let mut features: Vec<_> = value
        .split(',')
        .filter(|feature| !feature.is_empty())
        .map(str::to_owned)
        .collect();
    features.sort_unstable();
    features.dedup();
    features
}

/// Reads metadata produced by [`setup`] from Cargo's current `OUT_DIR`.
pub fn read_build_metadata() -> Option<CopperBuildMetadata> {
    let out_dir = PathBuf::from(std::env::var("OUT_DIR").ok()?);
    let contents = fs::read_to_string(out_dir.join(COPPER_BUILD_METADATA_FILE)).ok()?;
    decode_build_metadata(&contents)
}

fn collect_build_metadata(cargo_features: Vec<String>) -> CopperBuildMetadata {
    let manifest_dir = std::env::var("CARGO_MANIFEST_DIR").ok().map(PathBuf::from);
    let repository_root = manifest_dir
        .as_deref()
        .and_then(|path| git_output(path, &["rev-parse", "--show-toplevel"]))
        .map(PathBuf::from);
    let git_commit = repository_root
        .as_deref()
        .and_then(|root| git_output(root, &["rev-parse", "HEAD"]));
    let git_status = repository_root
        .as_deref()
        .and_then(|root| git_output(root, &["status", "--porcelain", "--untracked-files=all"]));
    let git_dirty = git_status.as_ref().map(|status| !status.is_empty());
    let source_archive_hash = match (repository_root.as_deref(), git_dirty) {
        (Some(root), Some(true)) => hash_dirty_source_bundle(root, git_commit.as_deref()),
        _ => None,
    };
    let git_repository = repository_root.as_deref().and_then(|root| {
        git_output(root, &["config", "--get", "remote.origin.url"])
            .map(|url| sanitize_repository_url(&url))
    });
    let cargo_manifest_path = match (manifest_dir.as_deref(), repository_root.as_deref()) {
        (Some(manifest), Some(root)) => manifest
            .join("Cargo.toml")
            .strip_prefix(root)
            .ok()
            .map(path_with_forward_slashes),
        _ => None,
    };
    let rust_target = std::env::var("TARGET")
        .ok()
        .filter(|value| !value.is_empty());
    let rust_toolchain = rustc_toolchain_identity();
    let dependency_resolution = manifest_dir
        .as_deref()
        .and_then(find_cargo_lock)
        .and_then(|path| fs::read(path).ok())
        .map(|bytes| format!("sha256:{}", sha256_hex(&bytes)));

    CopperBuildMetadata {
        git_repository,
        git_commit,
        git_dirty,
        source_archive_hash,
        cargo_manifest_path,
        cargo_features,
        rust_target,
        rust_toolchain,
        dependency_resolution,
    }
}

fn git_output(root: &Path, args: &[&str]) -> Option<String> {
    let output = Command::new("git")
        .arg("-C")
        .arg(root)
        .args(args)
        .output()
        .ok()?;
    output
        .status
        .success()
        .then(|| String::from_utf8_lossy(&output.stdout).trim().to_string())
}

fn hash_dirty_source_bundle(root: &Path, commit: Option<&str>) -> Option<String> {
    let mut bytes = Vec::new();
    bytes.extend_from_slice(b"copper-source-bundle-v1\0");
    bytes.extend_from_slice(commit.unwrap_or_default().as_bytes());
    bytes.push(0);

    let diff = Command::new("git")
        .arg("-C")
        .arg(root)
        .args(["diff", "--binary", "HEAD", "--", "."])
        .output()
        .ok()?;
    if !diff.status.success() {
        return None;
    }
    bytes.extend_from_slice(&diff.stdout);

    let untracked = Command::new("git")
        .arg("-C")
        .arg(root)
        .args(["ls-files", "--others", "--exclude-standard", "-z"])
        .output()
        .ok()?;
    if !untracked.status.success() {
        return None;
    }
    let mut paths: Vec<_> = untracked
        .stdout
        .split(|byte| *byte == 0)
        .filter(|path| !path.is_empty())
        .map(Vec::from)
        .collect();
    paths.sort();
    for path in paths {
        bytes.extend_from_slice(&path);
        bytes.push(0);
        bytes
            .extend_from_slice(&fs::read(root.join(String::from_utf8_lossy(&path).as_ref())).ok()?);
        bytes.push(0);
    }
    Some(format!("sha256:{}", sha256_hex(&bytes)))
}

fn rustc_toolchain_identity() -> Option<String> {
    let rustc = std::env::var("RUSTC").unwrap_or_else(|_| "rustc".to_string());
    let output = Command::new(rustc).arg("-vV").output().ok()?;
    if !output.status.success() {
        return None;
    }
    let value = String::from_utf8_lossy(&output.stdout)
        .lines()
        .map(str::trim)
        .filter(|line| !line.is_empty())
        .collect::<Vec<_>>()
        .join("; ");
    (!value.is_empty()).then_some(value)
}

fn find_cargo_lock(start: &Path) -> Option<PathBuf> {
    start
        .ancestors()
        .map(|dir| dir.join("Cargo.lock"))
        .find(|path| path.is_file())
}

fn path_with_forward_slashes(path: &Path) -> String {
    path.components()
        .map(|component| component.as_os_str().to_string_lossy())
        .collect::<Vec<_>>()
        .join("/")
}

/// Removes HTTP authority user-info so secrets are never persisted in a log.
pub fn sanitize_repository_url(url: &str) -> String {
    let trimmed = url.trim().split(['?', '#']).next().unwrap_or_default();
    let Some((scheme, rest)) = trimmed.split_once("://") else {
        return trimmed.to_string();
    };
    let (authority, suffix) = rest.split_once('/').unwrap_or((rest, ""));
    let sanitized_authority = authority
        .rsplit_once('@')
        .map_or(authority, |(_, host)| host);
    if suffix.is_empty() {
        format!("{scheme}://{sanitized_authority}")
    } else {
        format!("{scheme}://{sanitized_authority}/{suffix}")
    }
}

fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    digest.iter().map(|byte| format!("{byte:02x}")).collect()
}

fn encode_build_metadata(metadata: &CopperBuildMetadata) -> String {
    let mut lines = Vec::new();
    let mut push = |key: &str, value: Option<&str>| {
        lines.push(format!(
            "{key}={}",
            value.map(hex_encode).unwrap_or_default()
        ));
    };
    push("git_repository", metadata.git_repository.as_deref());
    push("git_commit", metadata.git_commit.as_deref());
    push(
        "git_dirty",
        metadata
            .git_dirty
            .map(|dirty| if dirty { "true" } else { "false" }),
    );
    push(
        "source_archive_hash",
        metadata.source_archive_hash.as_deref(),
    );
    push(
        "cargo_manifest_path",
        metadata.cargo_manifest_path.as_deref(),
    );
    push("cargo_features", Some(&metadata.cargo_features.join("\0")));
    push("rust_target", metadata.rust_target.as_deref());
    push("rust_toolchain", metadata.rust_toolchain.as_deref());
    push(
        "dependency_resolution",
        metadata.dependency_resolution.as_deref(),
    );
    lines.join("\n")
}

fn decode_build_metadata(contents: &str) -> Option<CopperBuildMetadata> {
    let get = |key: &str| -> Option<Option<String>> {
        let encoded = contents
            .lines()
            .find_map(|line| line.strip_prefix(&format!("{key}=")))?;
        if encoded.is_empty() {
            Some(None)
        } else {
            Some(Some(hex_decode(encoded)?))
        }
    };
    let cargo_features = get("cargo_features")?
        .unwrap_or_default()
        .split('\0')
        .filter(|feature| !feature.is_empty())
        .map(str::to_string)
        .collect();
    Some(CopperBuildMetadata {
        git_repository: get("git_repository")?,
        git_commit: get("git_commit")?,
        git_dirty: get("git_dirty")?.map(|value| value == "true"),
        source_archive_hash: get("source_archive_hash")?,
        cargo_manifest_path: get("cargo_manifest_path")?,
        cargo_features,
        rust_target: get("rust_target")?,
        rust_toolchain: get("rust_toolchain")?,
        dependency_resolution: get("dependency_resolution")?,
    })
}

fn hex_encode(value: &str) -> String {
    value
        .as_bytes()
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect()
}

fn hex_decode(value: &str) -> Option<String> {
    if !value.len().is_multiple_of(2) {
        return None;
    }
    let bytes = (0..value.len())
        .step_by(2)
        .map(|index| u8::from_str_radix(&value[index..index + 2], 16).ok())
        .collect::<Option<Vec<_>>>()?;
    String::from_utf8(bytes).ok()
}

#[cfg(test)]
mod tests {
    use super::{
        COPPER_CFG_FEATURES_ENV, CopperBuildMetadata, decode_build_metadata, encode_build_metadata,
        normalize_features, sanitize_repository_url,
    };

    #[test]
    fn feature_environment_name_is_stable() {
        assert_eq!(COPPER_CFG_FEATURES_ENV, "COPPER_CFG_FEATURES");
    }

    #[test]
    fn metadata_encoding_round_trips_arbitrary_text() {
        let metadata = CopperBuildMetadata {
            git_repository: Some("ssh://git@example.test/a=b/repo.git".to_string()),
            cargo_features: vec!["alpha".to_string(), "beta".to_string()],
            rust_toolchain: Some("rustc 1.95; host: x86_64".to_string()),
            ..CopperBuildMetadata::default()
        };
        assert_eq!(
            decode_build_metadata(&encode_build_metadata(&metadata)),
            Some(metadata)
        );
    }

    #[test]
    fn repository_url_sanitization_removes_http_user_info() {
        assert_eq!(
            sanitize_repository_url("https://user:token@example.test/acme/robot.git"),
            "https://example.test/acme/robot.git"
        );
        assert_eq!(
            sanitize_repository_url("git@github.com:acme/robot.git"),
            "git@github.com:acme/robot.git"
        );
        assert_eq!(
            sanitize_repository_url("https://token@example.test?credential=secret"),
            "https://example.test"
        );
    }

    #[test]
    fn cargo_features_are_sorted_and_deduplicated() {
        assert_eq!(normalize_features("gpu,camera,gpu"), ["camera", "gpu"]);
    }
}
