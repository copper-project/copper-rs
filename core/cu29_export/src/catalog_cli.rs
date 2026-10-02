//! Shared catalog inspection, extraction and deep-validation CLI operations.

use crate::catalog::{copperlist_values_reader, decode_copperlist, load_catalog};
use crate::fsck::{CheckedCopperList, check_with};
use crate::runs;
use crate::{CatalogFormat, CopperListDecoder, ExportFormat};
use clap::{ColorChoice, Parser, Subcommand};
use cu29::prelude::{CuError, CuResult, OptionCuTime, ValueDecodeCatalog};
use serde::{Deserialize, Serialize};
use std::io::{IsTerminal, Write};
use std::path::{Path, PathBuf};

#[derive(Parser)]
#[command(
    name = "cu29-logextract",
    version,
    about = "Inspect Copper logs using their embedded decode catalogs"
)]
struct StandaloneCli {
    /// Slab base path, e.g. logs/robot.copper for robot_0.copper, robot_1.copper.
    unifiedlog_base: PathBuf,
    #[arg(long, global = true)]
    run: Option<usize>,
    #[arg(long, global = true, value_enum, default_value_t = ColorChoice::Auto)]
    color: ColorChoice,
    #[command(subcommand)]
    command: StandaloneCommand,
}
#[derive(Subcommand)]
enum StandaloneCommand {
    /// List recorded runs before choosing --run.
    ListRuns,
    /// Inspect or dump the complete embedded catalog.
    Catalog {
        #[arg(short, long, value_enum, default_value = "human")]
        export_format: CatalogFormat,
    },
    /// Decode recorded messages without an application-specific logreader.
    ExtractCopperlists {
        #[arg(short, long, default_value_t = ExportFormat::Json)]
        export_format: ExportFormat,
        #[arg(long, value_enum, default_value = "catalog")]
        decoder: CopperListDecoder,
    },
    /// Reconstruct text using the application's matching string index.
    ExtractTextLog { log_index: PathBuf },
    /// Check structure; --deep decodes every captured payload using the catalog.
    Fsck {
        #[arg(short, long, action = clap::ArgAction::Count)]
        verbose: u8,
        #[arg(long)]
        dump_runtime_lifecycle: bool,
        #[arg(long)]
        deep: bool,
    },
}

pub(crate) fn run_cli() -> CuResult<()> {
    run_with_args(StandaloneCli::parse())
}
fn run_with_args(args: StandaloneCli) -> CuResult<()> {
    let recorded = runs::discover(&args.unifiedlog_base)?;
    if matches!(args.command, StandaloneCommand::ListRuns) {
        crate::print_runs(&recorded);
        return Ok(());
    }
    let run = runs::select(&recorded, args.run)?;
    let mut output = std::io::stdout().lock();
    match args.command {
        StandaloneCommand::ListRuns => unreachable!(),
        StandaloneCommand::Catalog { export_format } => dump_catalog(
            run,
            &args.unifiedlog_base,
            export_format,
            args.color,
            &mut output,
        ),
        StandaloneCommand::ExtractCopperlists {
            export_format,
            decoder,
        } => {
            if decoder == CopperListDecoder::Typed {
                return Err(
                    "Typed decoding requires an application logreader; use --decoder catalog"
                        .into(),
                );
            }
            extract(
                &args.unifiedlog_base,
                Some(run.index),
                export_format,
                &mut output,
            )
        }
        StandaloneCommand::ExtractTextLog { log_index } => crate::textlog_dump(
            run.reader(&args.unifiedlog_base)?
                .stream(cu29::prelude::UnifiedLogType::StructuredLogLine),
            &log_index,
        ),
        StandaloneCommand::Fsck {
            verbose,
            dump_runtime_lifecycle,
            deep,
        } => {
            if deep {
                deep_check(run, &args.unifiedlog_base, verbose, dump_runtime_lifecycle)
            } else {
                check_with(
                    &mut run.reader(&args.unifiedlog_base)?,
                    verbose,
                    dump_runtime_lifecycle,
                    None,
                )
            }
        }
    }
}

#[derive(Serialize, Deserialize)]
struct CatalogDocument {
    catalog_version: u16,
    run: usize,
    catalog: ValueDecodeCatalog,
}

pub(crate) fn dump_catalog(
    run: &runs::RecordedRun,
    path: &Path,
    format: CatalogFormat,
    color: ColorChoice,
    output: &mut impl Write,
) -> CuResult<()> {
    let document = CatalogDocument {
        catalog_version: 1,
        run: run.index,
        catalog: load_catalog(run, path)?,
    };
    match format {
        CatalogFormat::Ron => {
            let pretty = ron::ser::PrettyConfig::default()
                .escape_strings(false)
                .struct_names(true)
                .enumerate_arrays(true);
            let text = ron::ser::to_string_pretty(&document, pretty).map_err(|error| {
                CuError::new_with_cause("Could not serialize catalog RON", error)
            })?;
            writeln!(output, "{text}").map_err(output_error)?;
        }
        CatalogFormat::Json => {
            serde_json::to_writer_pretty(&mut *output, &document).map_err(|error| {
                CuError::new_with_cause("Could not serialize catalog JSON", error)
            })?;
            writeln!(output).map_err(output_error)?;
        }
        CatalogFormat::Human => {
            let color = color == ColorChoice::Always
                || (color == ColorChoice::Auto && std::io::stdout().is_terminal());
            let title = if color { "\x1b[38;2;203;166;247m" } else { "" }; // Mocha mauve
            let detail = if color { "\x1b[38;2;166;227;161m" } else { "" }; // Mocha green
            let reset = if color { "\x1b[0m" } else { "" };
            let catalog = &document.catalog;
            writeln!(
                output,
                "{title}Catalog v1 · run {} · mission {} · {:?}{reset}",
                document.run, catalog.mission, catalog.layout
            )
            .map_err(output_error)?;
            writeln!(output, "slot  task  payload  binding").map_err(output_error)?;
            for (index, slot) in catalog.slots.iter().enumerate() {
                writeln!(
                    output,
                    "{index:<5} {}  {}  {}",
                    slot.task_id,
                    slot.msg_type,
                    slot.binding
                        .map_or_else(|| "uncaptured".into(), |id| id.to_string())
                )
                .map_err(output_error)?;
            }
            writeln!(
                output,
                "\n{} bindings · {} operations · {} schemas",
                catalog.description.bindings.len(),
                catalog.description.operations.len(),
                catalog.description.schemas.len()
            )
            .map_err(output_error)?;
            for (index, schema) in catalog.description.schemas.iter().enumerate() {
                writeln!(
                    output,
                    "\n{title}schema {index}: {}{reset}",
                    schema.type_path
                )
                .map_err(output_error)?;
                if let Some(quantity) = &schema.quantity {
                    writeln!(
                        output,
                        "  {detail}{} · {}{reset}",
                        quantity.quantity, quantity.storage_unit
                    )
                    .map_err(output_error)?;
                }
                for field in &schema.fields {
                    let child = &catalog.description.schemas[field.schema];
                    write!(
                        output,
                        "  {}: {}",
                        field
                            .name
                            .clone()
                            .unwrap_or_else(|| field.index.to_string()),
                        child.type_path
                    )
                    .map_err(output_error)?;
                    if let Some(quantity) = &child.quantity {
                        write!(
                            output,
                            " {detail}· {} · {}{reset}",
                            quantity.quantity, quantity.storage_unit
                        )
                        .map_err(output_error)?;
                    }
                    writeln!(output).map_err(output_error)?;
                }
                for variant in &schema.variants {
                    writeln!(
                        output,
                        "  variant {} ({} fields)",
                        variant.name,
                        variant.fields.len()
                    )
                    .map_err(output_error)?;
                }
            }
        }
    }
    Ok(())
}

pub(crate) fn extract(
    path: &Path,
    run: Option<usize>,
    format: ExportFormat,
    output: &mut impl Write,
) -> CuResult<()> {
    let mut iter = copperlist_values_reader(path, run)?;
    if format == ExportFormat::Csv {
        let catalog = iter.catalog();
        let mut columns = vec!["id".to_string()];
        for (index, slot) in catalog.slots.iter().enumerate() {
            let name = format!("{index}:{}", slot.task_id);
            columns.extend([
                format!("{name}_time"),
                format!("{name}_tov"),
                format!("{name}_original_present"),
                format!("{name}_captured"),
                name,
            ]);
        }
        csv_row(output, &columns)?;
    } else if format == ExportFormat::Json {
        write!(output, "[").map_err(output_error)?;
    }
    let mut first = true;
    for result in &mut iter {
        let entry = result?;
        match format {
            ExportFormat::Json | ExportFormat::Jsonl => {
                if format == ExportFormat::Json && !first {
                    write!(output, ",").map_err(output_error)?;
                }
                serde_json::to_writer(&mut *output, &entry).map_err(|error| {
                    CuError::new_with_cause("Could not serialize CopperList", error)
                })?;
                if format == ExportFormat::Jsonl {
                    writeln!(output).map_err(output_error)?;
                }
            }
            ExportFormat::Csv => {
                let mut cells = vec![entry.id.to_string()];
                for msg in &entry.msgs {
                    let payload = msg
                        .payload
                        .as_ref()
                        .map(|value| serde_json::to_string(&crate::value_export::PlainValue(value)))
                        .transpose()
                        .map_err(|error| {
                            CuError::new_with_cause("Could not serialize CSV payload", error)
                        })?
                        .unwrap_or_default();
                    cells.extend([
                        msg.metadata.process_time.to_string(),
                        msg.tov.to_string(),
                        msg.original_payload_present
                            .map_or_else(String::new, |value| value.to_string()),
                        msg.captured_payload_present.to_string(),
                        payload,
                    ]);
                }
                csv_row(output, &cells)?;
            }
        }
        first = false;
    }
    if format == ExportFormat::Json {
        writeln!(output, "]").map_err(output_error)?;
    }
    Ok(())
}

fn csv_row(output: &mut impl Write, cells: &[String]) -> CuResult<()> {
    for (index, cell) in cells.iter().enumerate() {
        if index != 0 {
            write!(output, ",").map_err(output_error)?;
        }
        if cell.contains([',', '"', '\n', '\r']) {
            write!(output, "\"{}\"", cell.replace('"', "\"\"")).map_err(output_error)?;
        } else {
            write!(output, "{cell}").map_err(output_error)?;
        }
    }
    writeln!(output).map_err(output_error)?;
    Ok(())
}

pub(crate) fn deep_check(
    run: &runs::RecordedRun,
    path: &Path,
    verbose: u8,
    lifecycle: bool,
) -> CuResult<()> {
    let catalog = load_catalog(run, path)?;
    let mut lists = 0usize;
    let mut payloads = 0usize;
    let mut decode = |bytes: &[u8]| {
        let (entry, used) = decode_copperlist(&catalog, bytes)?;
        lists += 1;
        payloads += entry
            .msgs
            .iter()
            .filter(|msg| msg.captured_payload_present)
            .count();
        Ok(CheckedCopperList {
            id: entry.id,
            used,
            start: entry
                .msgs
                .first()
                .map_or(OptionCuTime::none(), |msg| msg.metadata.process_time.start),
            end: entry
                .msgs
                .last()
                .map_or(OptionCuTime::none(), |msg| msg.metadata.process_time.end),
        })
    };
    check_with(
        &mut run.reader(path)?,
        verbose,
        lifecycle,
        Some(&mut decode),
    )
    .map_err(|error| CuError::from(format!("Run {} deep validation: {error}", run.index)))?;
    println!(
        "Deep validation: {lists} CopperLists, {payloads} captured payloads decoded completely; frozen task-state bytes are opaque."
    );
    Ok(())
}

fn output_error(error: std::io::Error) -> CuError {
    CuError::new_with_cause("Could not write export output", error)
}

#[cfg(test)]
mod tests {
    use super::*;
    fn with_fixture(test: impl FnOnce(&Path)) {
        let dir = tempfile::tempdir_in(env!("CARGO_MANIFEST_DIR")).unwrap();
        let path = dir.path().join("fixture.copper");
        crate::catalog_tests::fixture(&path, false, true);
        test(&path);
    }

    #[test]
    fn test_catalog_machine_roundtrips_and_color() {
        with_fixture(|path| {
            let recorded = runs::discover(path).unwrap();
            let run = runs::select(&recorded, None).unwrap();
            for format in [CatalogFormat::Ron, CatalogFormat::Json] {
                let mut output = Vec::new();
                dump_catalog(run, path, format, ColorChoice::Always, &mut output).unwrap();
                assert!(!output.contains(&0x1b));
                let text = std::str::from_utf8(&output).unwrap();
                let document: CatalogDocument = if format == CatalogFormat::Ron {
                    ron::from_str(text).unwrap()
                } else {
                    serde_json::from_str(text).unwrap()
                };
                assert_eq!(document.catalog_version, 1);
                assert_eq!(document.catalog.slots[0].task_id, "drive");
                document.catalog.description.validate().unwrap();
                if format == CatalogFormat::Ron {
                    assert!(text.contains("layout: Compact"));
                }
            }
            let mut output = Vec::new();
            dump_catalog(
                run,
                path,
                CatalogFormat::Human,
                ColorChoice::Always,
                &mut output,
            )
            .unwrap();
            assert!(output.contains(&0x1b));
        });
    }

    #[test]
    fn test_json_jsonl_and_csv_have_clean_data() {
        with_fixture(|path| {
            let mut output = Vec::new();
            extract(path, None, ExportFormat::Json, &mut output).unwrap();
            let entries: serde_json::Value = serde_json::from_slice(&output).unwrap();
            assert_eq!(entries.as_array().unwrap().len(), 2);
            assert_eq!(entries[0]["msgs"][0]["payload"], 300);
            let mut output = Vec::new();
            extract(path, None, ExportFormat::Jsonl, &mut output).unwrap();
            let text = std::str::from_utf8(&output).unwrap();
            assert_eq!(text.lines().count(), 2);
            for line in text.lines() {
                serde_json::from_str::<serde_json::Value>(line).unwrap();
            }
            let mut output = Vec::new();
            extract(path, None, ExportFormat::Csv, &mut output).unwrap();
            let text = std::str::from_utf8(&output).unwrap();
            assert_eq!(text.lines().count(), 3);
            assert!(text.starts_with("id,0:drive_time,"));
            let mut quoted = Vec::new();
            csv_row(
                &mut quoted,
                &["a,b".into(), "x\"y".into(), "line\nnext".into()],
            )
            .unwrap();
            assert_eq!(
                String::from_utf8(quoted).unwrap(),
                "\"a,b\",\"x\"\"y\",\"line\nnext\"\n"
            );
        });
    }

    #[test]
    fn test_deep_fsck_rejects_truncated_payload_and_legacy_log() {
        let dir = tempfile::tempdir_in(env!("CARGO_MANIFEST_DIR")).unwrap();
        for (name, corrupt, catalog) in [
            ("good", false, true),
            ("corrupt", true, true),
            ("legacy", false, false),
        ] {
            let path = dir.path().join(format!("{name}.copper"));
            crate::catalog_tests::fixture(&path, corrupt, catalog);
            let recorded = runs::discover(&path).unwrap();
            let run = runs::select(&recorded, None).unwrap();
            let result = deep_check(run, &path, 0, false);
            assert_eq!(result.is_ok(), name == "good");
            if name == "corrupt" {
                assert!(
                    result
                        .unwrap_err()
                        .to_string()
                        .contains("CopperList #1 slot 0")
                );
            }
            if name == "legacy" {
                check_with(&mut run.reader(&path).unwrap(), 0, false, None).unwrap();
            }
        }
    }

    #[test]
    fn test_shared_cli_mapping_and_decoder_defaults() {
        let app = crate::LogReaderCli::try_parse_from([
            "logreader",
            "app.copper",
            "extract-copperlists",
            "-e",
            "jsonl",
        ])
        .unwrap();
        assert!(matches!(
            app.command,
            crate::Command::ExtractCopperlists {
                decoder: CopperListDecoder::Typed,
                export_format: ExportFormat::Jsonl
            }
        ));
        let standalone = StandaloneCli::try_parse_from([
            "cu29-logextract",
            "app.copper",
            "extract-copperlists",
            "-e",
            "jsonl",
        ])
        .unwrap();
        assert!(matches!(
            standalone.command,
            StandaloneCommand::ExtractCopperlists {
                decoder: CopperListDecoder::Catalog,
                export_format: ExportFormat::Jsonl
            }
        ));
        for command in ["catalog", "fsck", "list-runs"] {
            crate::LogReaderCli::try_parse_from([
                "logreader",
                "app.copper",
                command,
                "--run",
                "1",
                "--color",
                "never",
            ])
            .unwrap();
            StandaloneCli::try_parse_from([
                "cu29-logextract",
                "app.copper",
                command,
                "--run",
                "1",
                "--color",
                "never",
            ])
            .unwrap();
        }
        assert!(
            StandaloneCli::try_parse_from(["cu29-logextract", "app.copper", "log-stats"]).is_err()
        );
    }
}
