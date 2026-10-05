//! Render the bundled Copper templates into a new project directory.

use crate::BUNDLED_TEMPLATES;
use crate::Cli;
use crate::ResolvedOptions;
use anyhow::Context;
use anyhow::Result;
use anyhow::bail;
use heck::ToKebabCase;
use heck::ToSnakeCase;
use heck::ToUpperCamelCase;
use include_dir::Dir;
use include_dir::DirEntry;
use liquid_core::Filter;
use liquid_core::Runtime;
use liquid_core::Value;
use liquid_core::ValueView;
use liquid_derive::Display_filter;
use liquid_derive::FilterReflection;
use liquid_derive::ParseFilter;
use std::fs;
use std::path::Path;
use std::path::PathBuf;
use std::process::Command;

macro_rules! case_filter {
    ($name:literal, $parser:ident, $filter:ident, $method:ident) => {
        #[derive(Clone, ParseFilter, FilterReflection)]
        #[filter(name = $name, description = "Convert a project name", parsed($filter))]
        struct $parser;

        #[derive(Debug, Default, Display_filter)]
        #[name = $name]
        struct $filter;

        impl Filter for $filter {
            fn evaluate(
                &self,
                input: &dyn ValueView,
                _runtime: &dyn Runtime,
            ) -> liquid_core::Result<Value> {
                Ok(Value::scalar(input.to_kstr().$method()))
            }
        }
    };
}

case_filter!("kebab_case", KebabCaseParser, KebabCase, to_kebab_case);
case_filter!("snake_case", SnakeCaseParser, SnakeCase, to_snake_case);
case_filter!(
    "upper_camel_case",
    UpperCamelCaseParser,
    UpperCamelCase,
    to_upper_camel_case
);

pub(super) fn generate(cli: &Cli, options: &ResolvedOptions) -> Result<PathBuf> {
    let name = &options.project_name;
    let package_name = name.clone();
    if package_name.is_empty()
        || !name
            .chars()
            .all(|c| c.is_ascii_alphanumeric() || c == '-' || c == '_')
        || !name.starts_with(|c: char| c.is_ascii_alphabetic())
    {
        bail!(
            "project names must start with a letter and contain only ASCII letters, digits, hyphens, or underscores"
        );
    }
    let destination = options.destination_dir.join(&package_name);
    if destination.exists() {
        bail!("{} already exists", destination.display());
    }
    fs::create_dir_all(&options.destination_dir)
        .with_context(|| format!("failed to create {}", options.destination_dir.display()))?;
    let staging = tempfile::Builder::new()
        .prefix(".cunew-")
        .tempdir_in(&options.destination_dir)
        .context("failed to create project staging directory")?;
    let parser = liquid::ParserBuilder::with_stdlib()
        .filter(KebabCaseParser)
        .filter(SnakeCaseParser)
        .filter(UpperCamelCaseParser)
        .build()
        .context("failed to build template parser")?;
    let mut values = liquid::Object::new();
    values.insert("project-name".into(), Value::scalar(package_name));
    values.insert("crate_name".into(), Value::scalar(name.to_snake_case()));
    for define in &options.defines {
        let (key, value) = define
            .split_once('=')
            .context("invalid template variable")?;
        values.insert(key.to_owned().into(), Value::scalar(value.to_owned()));
    }
    let template = BUNDLED_TEMPLATES
        .get_dir(cli.template.subfolder())
        .context("bundled template is missing")?;
    render_dir(template, staging.path(), &parser, &values, cli.verbose)?;
    if options.initialize_git {
        let output = Command::new("git")
            .args(["init", "--quiet"])
            .arg(staging.path())
            .output()
            .context("failed to initialize git; install git or pass --no-vcs")?;
        if !output.status.success() {
            bail!(
                "git init failed: {}",
                String::from_utf8_lossy(&output.stderr)
            );
        }
    }
    fs::rename(staging.path(), &destination)
        .with_context(|| format!("failed to create {}", destination.display()))?;
    Ok(destination)
}

fn render_dir(
    dir: &Dir<'_>,
    destination: &Path,
    parser: &liquid::Parser,
    values: &liquid::Object,
    verbose: bool,
) -> Result<()> {
    fs::create_dir_all(destination)?;
    for entry in dir.entries() {
        let name = entry
            .path()
            .file_name()
            .context("invalid bundled template path")?;
        let target = destination.join(name);
        match entry {
            DirEntry::Dir(child) => render_dir(child, &target, parser, values, verbose)?,
            DirEntry::File(file) => {
                if name == "cargo-generate.toml" || name == "init.rhai" {
                    continue;
                }
                let target = if name == "Cargo.toml.template" {
                    destination.join("Cargo.toml")
                } else {
                    target
                };
                let source = file
                    .contents_utf8()
                    .context("template is not valid UTF-8")?;
                let rendered = parser
                    .parse(source)
                    .with_context(|| format!("failed to parse {}", file.path().display()))?
                    .render(values)
                    .with_context(|| format!("failed to render {}", file.path().display()))?;
                fs::write(&target, rendered)
                    .with_context(|| format!("failed to write {}", target.display()))?;
                if verbose {
                    eprintln!("created {}", target.display());
                }
            }
        }
    }
    Ok(())
}
