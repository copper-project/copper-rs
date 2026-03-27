//! Log export helpers for Copper applications.
//!
//! This crate serves two related use cases:
//!
//! - Rust logreader binaries built with [`run_cli`]
//! - optional Python-facing iterators for offline analysis of structured logs,
//!   runtime lifecycle records, and app-specific CopperLists
//!
//! Python support here is intentionally offline. It reads data that Copper has
//! already recorded. That is very different from putting Python on the runtime
//! execution path, and it does not compromise realtime behavior during robot
//! execution.
//!
//! For runtime Python task prototyping, see `cu-python-task` instead.

mod fsck;
pub mod logstats;

#[cfg(feature = "mcap")]
pub mod mcap_export;

#[cfg(feature = "mcap")]
pub mod serde_to_jsonschema;

use bincode::Decode;
use bincode::config::standard;
use bincode::decode_from_std_read;
use bincode::error::DecodeError;
use clap::{Parser, Subcommand, ValueEnum};
use cu29::UnifiedLogType;
use cu29::prelude::*;
use cu29_intern_strs::read_interned_strings;
use fsck::check;
#[cfg(feature = "mcap")]
use indicatif::{ProgressBar, ProgressDrawTarget, ProgressStyle};
use logstats::{compute_logstats, write_logstats};
use serde::Serialize;
use std::fmt::{Display, Formatter};
#[cfg(feature = "mcap")]
use std::io::IsTerminal;
use std::io::Read;
use std::path::{Path, PathBuf};

#[cfg(feature = "mcap")]
pub use mcap_export::{
    McapExportStats, PayloadSchemas, export_to_mcap, export_to_mcap_with_schemas, mcap_info,
};

#[cfg(feature = "mcap")]
pub use serde_to_jsonschema::trace_type_to_jsonschema;

/// Registers the typed CopperList decoder used by the generic Python iterator.
///
/// Applications normally call this indirectly through
/// [`copperlist_iterator_unified_typed_py`].
#[cfg(feature = "python")]
pub use python::register_copperlist_python_type;

/// Creates a Python CopperList iterator for a specific CopperList tuple type.
///
/// This is intended for app-specific Python modules that know their generated
/// CopperList type at compile time. The helper registers the decoder and returns
/// an iterator object that yields Python objects built from the recorded
/// CopperLists.
#[cfg(feature = "python")]
pub fn copperlist_iterator_unified_typed_py<P>(
    unified_src_path: &str,
    py: pyo3::Python<'_>,
) -> pyo3::PyResult<pyo3::Py<pyo3::PyAny>>
where
    P: CopperListTuple,
{
    register_copperlist_python_type::<P>()
        .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
    let iter = python::copperlist_iterator_unified(unified_src_path)?;
    pyo3::Py::new(py, iter).map(|obj| obj.into())
}

/// Creates a Python `RuntimeLifecycleRecord` iterator from a unified log.
///
/// This is useful for offline analysis scripts that need to inspect mission
/// starts, stops, faults, and related runtime events.
#[cfg(feature = "python")]
pub fn runtime_lifecycle_iterator_unified_py(
    unified_src_path: &str,
    py: pyo3::Python<'_>,
) -> pyo3::PyResult<pyo3::Py<pyo3::PyAny>> {
    let iter = python::runtime_lifecycle_iterator_unified(unified_src_path)?;
    pyo3::Py::new(py, iter).map(|obj| obj.into())
}
#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord, ValueEnum)]
pub enum ExportFormat {
    Json,
    Csv,
}

impl Display for ExportFormat {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            ExportFormat::Json => write!(f, "json"),
            ExportFormat::Csv => write!(f, "csv"),
        }
    }
}

/// This is a generator for a main function to build a log extractor.
#[derive(Parser)]
#[command(author, version, about)]
pub struct LogReaderCli {
    /// The base path is the name with no _0 _1 et the end.
    /// for example for toto_0.copper, toto_1.copper ... the base name is toto.copper
    pub unifiedlog_base: PathBuf,

    #[command(subcommand)]
    pub command: Command,
}

#[derive(Subcommand)]
pub enum Command {
    /// Extract logs
    ExtractTextLog { log_index: PathBuf },
    /// Extract copperlists
    ExtractCopperlists {
        #[arg(short, long, default_value_t = ExportFormat::Json)]
        export_format: ExportFormat,
    },
    /// Check the log and dump info about it.
    Fsck {
        #[arg(short, long, action = clap::ArgAction::Count)]
        verbose: u8,
        /// Decode and print RuntimeLifecycle events.
        #[arg(long)]
        dump_runtime_lifecycle: bool,
    },
    /// Export log statistics to JSON for offline DAG rendering.
    LogStats {
        /// Output JSON file path
        #[arg(short, long, default_value = "cu29_logstats.json")]
        output: PathBuf,
        /// Config file used to map outputs to edges
        #[arg(long, default_value = "copperconfig.ron")]
        config: PathBuf,
        /// Mission id to use when reading the config
        #[arg(long)]
        mission: Option<String>,
    },
    /// Export copperlists to MCAP format (requires 'mcap' feature)
    #[cfg(feature = "mcap")]
    ExportMcap {
        /// Output MCAP file path
        #[arg(short, long)]
        output: PathBuf,
        /// Force progress bar even when stderr is not a TTY
        #[arg(long)]
        progress: bool,
        /// Suppress the progress bar
        #[arg(long)]
        quiet: bool,
    },
    /// Inspect an MCAP file and dump metadata, schemas, and stats (requires 'mcap' feature)
    #[cfg(feature = "mcap")]
    McapInfo {
        /// Path to the MCAP file to inspect
        mcap_file: PathBuf,
        /// Show full schema content
        #[arg(short, long)]
        schemas: bool,
        /// Show sample messages (first N messages per channel)
        #[arg(short = 'n', long, default_value_t = 0)]
        sample_messages: usize,
    },
}

fn write_json_pretty<T: Serialize + ?Sized>(value: &T) -> CuResult<()> {
    serde_json::to_writer_pretty(std::io::stdout(), value)
        .map_err(|e| CuError::new_with_cause("Failed to write JSON output", e))
}

fn write_json<T: Serialize + ?Sized>(value: &T) -> CuResult<()> {
    serde_json::to_writer(std::io::stdout(), value)
        .map_err(|e| CuError::new_with_cause("Failed to write JSON output", e))
}

fn build_read_logger(unifiedlog_base: &Path) -> CuResult<UnifiedLoggerRead> {
    let logger = UnifiedLoggerBuilder::new()
        .file_base_name(unifiedlog_base)
        .build()
        .map_err(|e| CuError::new_with_cause("Failed to create logger", e))?;
    match logger {
        UnifiedLogger::Read(dl) => Ok(dl),
        UnifiedLogger::Write(_) => Err(CuError::from(
            "Expected read-only unified logger in export CLI",
        )),
    }
}

/// This is a generator for a main function to build a log extractor.
/// It depends on the specific type of the CopperList payload that is determined at compile time from the configuration.
///
/// When the `mcap` feature is enabled, P must also implement `PayloadSchemas` for MCAP export support.
#[cfg(feature = "mcap")]
pub fn run_cli<P>() -> CuResult<()>
where
    P: CopperListTuple + CuPayloadRawBytes + mcap_export::PayloadSchemas,
{
    #[cfg(feature = "python")]
    let _ = python::register_copperlist_python_type::<P>();

    run_cli_inner::<P>()
}

/// This is a generator for a main function to build a log extractor.
/// It depends on the specific type of the CopperList payload that is determined at compile time from the configuration.
#[cfg(not(feature = "mcap"))]
pub fn run_cli<P>() -> CuResult<()>
where
    P: CopperListTuple + CuPayloadRawBytes,
{
    #[cfg(feature = "python")]
    let _ = python::register_copperlist_python_type::<P>();

    run_cli_inner::<P>()
}

#[cfg(feature = "mcap")]
fn run_cli_inner<P>() -> CuResult<()>
where
    P: CopperListTuple + CuPayloadRawBytes + mcap_export::PayloadSchemas,
{
    let args = LogReaderCli::parse();
    let unifiedlog_base = args.unifiedlog_base;

    let mut dl = build_read_logger(&unifiedlog_base)?;

    match args.command {
        Command::ExtractTextLog { log_index } => {
            let reader = UnifiedLoggerIOReader::new(dl, UnifiedLogType::StructuredLogLine);
            textlog_dump(reader, &log_index)?;
        }
        Command::ExtractCopperlists { export_format } => {
            println!("Extracting copperlists with format: {export_format}");
            let mut reader = UnifiedLoggerIOReader::new(dl, UnifiedLogType::CopperList);
            let iter = copperlists_reader::<P>(&mut reader);

            match export_format {
                ExportFormat::Json => {
                    for entry in iter {
                        write_json_pretty(&entry)?;
                    }
                }
                ExportFormat::Csv => {
                    let mut first = true;
                    for origin in P::get_all_task_ids() {
                        if !first {
                            print!(", ");
                        } else {
                            print!("id, ");
                        }
                        print!("{origin}_time, {origin}_tov, {origin},");
                        first = false;
                    }
                    println!();
                    for entry in iter {
                        let mut first = true;
                        for msg in entry.cumsgs() {
                            if let Some(payload) = msg.payload() {
                                if !first {
                                    print!(", ");
                                } else {
                                    print!("{}, ", entry.id);
                                }
                                let metadata = msg.metadata();
                                print!("{}, {}, ", metadata.process_time(), msg.tov());
                                write_json(payload)?; // TODO: escape for CSV
                                first = false;
                            }
                        }
                        println!();
                    }
                }
            }
        }
        Command::Fsck {
            verbose,
            dump_runtime_lifecycle,
        } => {
            if let Some(value) = check::<P>(&mut dl, verbose, dump_runtime_lifecycle) {
                return value;
            }
        }
        Command::LogStats {
            output,
            config,
            mission,
        } => {
            run_logstats::<P>(dl, output, config, mission)?;
        }
        #[cfg(feature = "mcap")]
        Command::ExportMcap {
            output,
            progress,
            quiet,
        } => {
            println!("Exporting copperlists to MCAP format: {}", output.display());

            let show_progress = should_show_progress(progress, quiet);
            let total_bytes = if show_progress {
                Some(copperlist_total_bytes(&unifiedlog_base)?)
            } else {
                None
            };

            let reader = UnifiedLoggerIOReader::new(dl, UnifiedLogType::CopperList);

            // Export to MCAP with schemas.
            // Note: P must implement PayloadSchemas and provide schemas for each CopperList slot.
            let stats = if let Some(total_bytes) = total_bytes {
                let progress_bar = make_progress_bar(total_bytes);
                let reader = ProgressReader::new(reader, progress_bar.clone());
                let result = export_to_mcap_impl::<P>(reader, &output);
                progress_bar.finish_and_clear();
                result?
            } else {
                export_to_mcap_impl::<P>(reader, &output)?
            };
            println!("{stats}");
        }
        #[cfg(feature = "mcap")]
        Command::McapInfo {
            mcap_file,
            schemas,
            sample_messages,
        } => {
            mcap_info(&mcap_file, schemas, sample_messages)?;
        }
    }

    Ok(())
}

#[cfg(not(feature = "mcap"))]
fn run_cli_inner<P>() -> CuResult<()>
where
    P: CopperListTuple + CuPayloadRawBytes,
{
    let args = LogReaderCli::parse();
    let unifiedlog_base = args.unifiedlog_base;

    let mut dl = build_read_logger(&unifiedlog_base)?;

    match args.command {
        Command::ExtractTextLog { log_index } => {
            let reader = UnifiedLoggerIOReader::new(dl, UnifiedLogType::StructuredLogLine);
            textlog_dump(reader, &log_index)?;
        }
        Command::ExtractCopperlists { export_format } => {
            println!("Extracting copperlists with format: {export_format}");
            let mut reader = UnifiedLoggerIOReader::new(dl, UnifiedLogType::CopperList);
            let iter = copperlists_reader::<P>(&mut reader);

            match export_format {
                ExportFormat::Json => {
                    for entry in iter {
                        write_json_pretty(&entry)?;
                    }
                }
                ExportFormat::Csv => {
                    let mut first = true;
                    for origin in P::get_all_task_ids() {
                        if !first {
                            print!(", ");
                        } else {
                            print!("id, ");
                        }
                        print!("{origin}_time, {origin}_tov, {origin},");
                        first = false;
                    }
                    println!();
                    for entry in iter {
                        let mut first = true;
                        for msg in entry.cumsgs() {
                            if let Some(payload) = msg.payload() {
                                if !first {
                                    print!(", ");
                                } else {
                                    print!("{}, ", entry.id);
                                }
                                let metadata = msg.metadata();
                                print!("{}, {}, ", metadata.process_time(), msg.tov());
                                write_json(payload)?;
                                first = false;
                            }
                        }
                        println!();
                    }
                }
            }
        }
        Command::Fsck {
            verbose,
            dump_runtime_lifecycle,
        } => {
            if let Some(value) = check::<P>(&mut dl, verbose, dump_runtime_lifecycle) {
                return value;
            }
        }
        Command::LogStats {
            output,
            config,
            mission,
        } => {
            run_logstats::<P>(dl, output, config, mission)?;
        }
    }

    Ok(())
}

fn run_logstats<P>(
    dl: UnifiedLoggerRead,
    output: PathBuf,
    config: PathBuf,
    mission: Option<String>,
) -> CuResult<()>
where
    P: CopperListTuple + CuPayloadRawBytes,
{
    let config_path = config
        .to_str()
        .ok_or_else(|| CuError::from("Config path is not valid UTF-8"))?;
    let cfg = read_configuration(config_path)
        .map_err(|e| CuError::new_with_cause("Failed to read configuration", e))?;
    let reader = UnifiedLoggerIOReader::new(dl, UnifiedLogType::CopperList);
    let stats = compute_logstats::<P>(reader, &cfg, mission.as_deref())?;
    write_logstats(&stats, &output)
}

/// Helper function for MCAP export.
///
/// Uses the PayloadSchemas trait to get per-slot payload schemas.
#[cfg(feature = "mcap")]
fn export_to_mcap_impl<P>(src: impl Read, output: &Path) -> CuResult<McapExportStats>
where
    P: CopperListTuple + mcap_export::PayloadSchemas,
{
    mcap_export::export_to_mcap::<P, _>(src, output)
}

#[cfg(feature = "mcap")]
struct ProgressReader<R> {
    inner: R,
    progress: ProgressBar,
}

#[cfg(feature = "mcap")]
impl<R> ProgressReader<R> {
    fn new(inner: R, progress: ProgressBar) -> Self {
        Self { inner, progress }
    }
}

#[cfg(feature = "mcap")]
impl<R: Read> Read for ProgressReader<R> {
    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        let read = self.inner.read(buf)?;
        if read > 0 {
            self.progress.inc(read as u64);
        }
        Ok(read)
    }
}

#[cfg(feature = "mcap")]
fn make_progress_bar(total_bytes: u64) -> ProgressBar {
    let progress_bar = ProgressBar::new(total_bytes);
    progress_bar.set_draw_target(ProgressDrawTarget::stderr_with_hz(5));

    let style = ProgressStyle::with_template(
        "[{elapsed_precise}] {bar:40} {bytes}/{total_bytes} ({bytes_per_sec}, ETA {eta})",
    )
    .unwrap_or_else(|_| ProgressStyle::default_bar());

    progress_bar.set_style(style.progress_chars("=>-"));
    progress_bar
}

#[cfg(feature = "mcap")]
fn should_show_progress(force_progress: bool, quiet: bool) -> bool {
    !quiet && (force_progress || std::io::stderr().is_terminal())
}

#[cfg(feature = "mcap")]
fn copperlist_total_bytes(log_base: &Path) -> CuResult<u64> {
    let mut reader = UnifiedLoggerRead::new(log_base)
        .map_err(|e| CuError::new_with_cause("Failed to open log for progress estimation", e))?;
    reader
        .scan_section_bytes(UnifiedLogType::CopperList)
        .map_err(|e| CuError::new_with_cause("Failed to scan log for progress estimation", e))
}

fn read_next_entry<T: Decode<()>>(src: &mut impl Read) -> Option<T> {
    let entry = decode_from_std_read::<T, _, _>(src, standard());
    match entry {
        Ok(entry) => Some(entry),
        Err(DecodeError::UnexpectedEnd { .. }) => None,
        Err(DecodeError::Io { inner, additional }) => {
            if inner.kind() == std::io::ErrorKind::UnexpectedEof {
                None
            } else {
                println!("Error {inner:?} additional:{additional}");
                None
            }
        }
        Err(e) => {
            println!("Error {e:?}");
            None
        }
    }
}

/// Extracts the copper lists from a binary representation.
/// P is the Payload determined by the configuration of the application.
pub fn copperlists_reader<P: CopperListTuple>(
    mut src: impl Read,
) -> impl Iterator<Item = CopperList<P>> {
    std::iter::from_fn(move || read_next_entry::<CopperList<P>>(&mut src))
}

/// Extracts the keyframes from the log.
pub fn keyframes_reader(mut src: impl Read) -> impl Iterator<Item = KeyFrame> {
    std::iter::from_fn(move || read_next_entry::<KeyFrame>(&mut src))
}

/// Extracts the runtime lifecycle records from the log.
pub fn runtime_lifecycle_reader(
    mut src: impl Read,
) -> impl Iterator<Item = RuntimeLifecycleRecord> {
    std::iter::from_fn(move || read_next_entry::<RuntimeLifecycleRecord>(&mut src))
}

/// Returns the first mission announced by the runtime lifecycle section, if any.
pub fn unified_log_mission(unifiedlog_base: &Path) -> CuResult<Option<String>> {
    let dl = build_read_logger(unifiedlog_base)?;
    let reader = UnifiedLoggerIOReader::new(dl, UnifiedLogType::RuntimeLifecycle);
    Ok(
        runtime_lifecycle_reader(reader).find_map(|entry| match entry.event {
            RuntimeLifecycleEvent::MissionStarted { mission } => Some(mission),
            _ => None,
        }),
    )
}

/// Ensures the unified log was recorded for the expected mission.
pub fn assert_unified_log_mission(unifiedlog_base: &Path, expected_mission: &str) -> CuResult<()> {
    match unified_log_mission(unifiedlog_base)? {
        Some(actual_mission) if actual_mission == expected_mission => Ok(()),
        Some(actual_mission) => Err(CuError::from(format!(
            "Mission mismatch: expected '{expected_mission}', found '{actual_mission}'"
        ))),
        None => Err(CuError::from(format!(
            "No MissionStarted runtime lifecycle event found while validating expected mission '{expected_mission}'"
        ))),
    }
}

pub fn structlog_reader(mut src: impl Read) -> impl Iterator<Item = CuResult<CuLogEntry>> {
    std::iter::from_fn(move || {
        let entry = decode_from_std_read::<CuLogEntry, _, _>(&mut src, standard());

        match entry {
            Err(DecodeError::UnexpectedEnd { .. }) => None,
            Err(DecodeError::Io {
                inner,
                additional: _,
            }) => {
                if inner.kind() == std::io::ErrorKind::UnexpectedEof {
                    None
                } else {
                    Some(Err(CuError::new_with_cause("Error reading log", inner)))
                }
            }
            Err(e) => Some(Err(CuError::new_with_cause("Error reading log", e))),
            Ok(entry) => {
                if entry.msg_index == 0 {
                    None
                } else {
                    Some(Ok(entry))
                }
            }
        }
    })
}

/// Full dump of the copper structured log from its binary representation.
/// This rebuilds a textual log.
/// src: the source of the log data
/// index: the path to the index file (containing the interned strings constructed at build time)
pub fn textlog_dump(src: impl Read, index: &Path) -> CuResult<()> {
    let all_strings = read_interned_strings(index).map_err(|e| {
        CuError::new_with_cause(
            "Failed to read interned strings from index",
            std::io::Error::other(e),
        )
    })?;

    for result in structlog_reader(src) {
        match result {
            Ok(entry) => match rebuild_logline(&all_strings, &entry) {
                Ok(line) => println!("{line}"),
                Err(e) => println!("Failed to rebuild log line: {e:?}"),
            },
            Err(e) => return Err(e),
        }
    }

    Ok(())
}

// Only compiled for users opting into the Python interface.
#[cfg(feature = "python")]
mod python {
    use bincode::config::standard;
    use bincode::decode_from_std_read;
    use bincode::error::DecodeError;
    use cu29::bevy_reflect::{PartialReflect, ReflectRef, VariantType};
    use cu29::prelude::*;
    use cu29_intern_strs::read_interned_strings;
    use pyo3::exceptions::{PyIOError, PyRuntimeError};
    use pyo3::prelude::*;
    use pyo3::types::{PyDelta, PyDict, PyList};
    use std::io::Read;
    use std::path::Path;
    use std::sync::OnceLock;

    type CopperListDecodeFn =
        for<'py> fn(&mut Box<dyn Read + Send + Sync>, Python<'py>) -> Option<PyResult<Py<PyAny>>>;
    static COPPERLIST_DECODER: OnceLock<CopperListDecodeFn> = OnceLock::new();

    /// Iterator over structured Copper log entries.
    #[pyclass]
    pub struct PyLogIterator {
        reader: Box<dyn Read + Send + Sync>,
    }

    /// Iterator over application-specific CopperLists decoded into Python values.
    #[pyclass]
    pub struct PyCopperListIterator {
        reader: Box<dyn Read + Send + Sync>,
        decode_next: CopperListDecodeFn,
    }

    /// Iterator over runtime lifecycle records stored in a unified log.
    #[pyclass]
    pub struct PyRuntimeLifecycleIterator {
        reader: Box<dyn Read + Send + Sync>,
    }

    /// Helper wrapper used when reflected unit-bearing values are exposed to Python.
    #[pyclass(get_all)]
    pub struct PyUnitValue {
        pub value: f64,
        pub unit: String,
    }

    /// Register the Python decoder for one concrete CopperList tuple type.
    ///
    /// App-specific extension modules call this once before constructing a
    /// `PyCopperListIterator`.
    pub fn register_copperlist_python_type<P>() -> CuResult<()>
    where
        P: CopperListTuple,
    {
        if COPPERLIST_DECODER.get().is_none() {
            COPPERLIST_DECODER
                .set(decode_next_copperlist::<P>)
                .map_err(|_| CuError::from("Failed to register CopperList Python decoder"))?;
        }
        Ok(())
    }
    #[pymethods]
    impl PyLogIterator {
        fn __iter__(slf: PyRefMut<Self>) -> PyRefMut<Self> {
            slf
        }

        fn __next__(mut slf: PyRefMut<Self>) -> Option<PyResult<PyCuLogEntry>> {
            match decode_from_std_read::<CuLogEntry, _, _>(&mut slf.reader, standard()) {
                Ok(entry) => {
                    if entry.msg_index == 0 {
                        None
                    } else {
                        Some(Ok(PyCuLogEntry { inner: entry }))
                    }
                }
                Err(DecodeError::UnexpectedEnd { .. }) => None,
                Err(DecodeError::Io { inner, .. })
                    if inner.kind() == std::io::ErrorKind::UnexpectedEof =>
                {
                    None
                }
                Err(e) => Some(Err(PyIOError::new_err(e.to_string()))),
            }
        }
    }

    #[pymethods]
    impl PyCopperListIterator {
        fn __iter__(slf: PyRefMut<Self>) -> PyRefMut<Self> {
            slf
        }

        fn __next__(mut slf: PyRefMut<Self>, py: Python<'_>) -> Option<PyResult<Py<PyAny>>> {
            (slf.decode_next)(&mut slf.reader, py)
        }
    }

    #[pymethods]
    impl PyRuntimeLifecycleIterator {
        fn __iter__(slf: PyRefMut<Self>) -> PyRefMut<Self> {
            slf
        }

        fn __next__(mut slf: PyRefMut<Self>, py: Python<'_>) -> Option<PyResult<Py<PyAny>>> {
            let entry = super::read_next_entry::<RuntimeLifecycleRecord>(&mut slf.reader)?;
            Some(runtime_lifecycle_record_to_py(&entry, py))
        }
    }
    /// Create an iterator over structured log entries from a bare structured log file.
    ///
    /// This is the non-unified-log path used by standalone structured log setups.
    /// The function returns the iterator together with the interned string table
    /// needed to format each message.
    #[pyfunction]
    pub fn struct_log_iterator_bare(
        bare_struct_src_path: &str,
        index_path: &str,
    ) -> PyResult<(PyLogIterator, Vec<String>)> {
        let file = std::fs::File::open(bare_struct_src_path)
            .map_err(|e| PyIOError::new_err(e.to_string()))?;
        let all_strings = read_interned_strings(Path::new(index_path))
            .map_err(|e| PyIOError::new_err(e.to_string()))?;
        Ok((
            PyLogIterator {
                reader: Box::new(file),
            },
            all_strings,
        ))
    }
    /// Create an iterator over structured log entries from a unified log file.
    ///
    /// The function returns the iterator together with the interned string table
    /// needed to rebuild the text messages.
    #[pyfunction]
    pub fn struct_log_iterator_unified(
        unified_src_path: &str,
        index_path: &str,
    ) -> PyResult<(PyLogIterator, Vec<String>)> {
        let all_strings = read_interned_strings(Path::new(index_path))
            .map_err(|e| PyIOError::new_err(e.to_string()))?;

        let logger = UnifiedLoggerBuilder::new()
            .file_base_name(Path::new(unified_src_path))
            .build()
            .map_err(|e| PyIOError::new_err(e.to_string()))?;
        let dl = match logger {
            UnifiedLogger::Read(dl) => dl,
            UnifiedLogger::Write(_) => {
                return Err(PyIOError::new_err(
                    "Expected read-only unified logger for Python export",
                ));
            }
        };

        let reader = UnifiedLoggerIOReader::new(dl, UnifiedLogType::StructuredLogLine);
        Ok((
            PyLogIterator {
                reader: Box::new(reader),
            },
            all_strings,
        ))
    }

    /// Create an iterator over CopperLists from a unified log file.
    ///
    /// The concrete CopperList tuple type must be registered from Rust first with
    /// `register_copperlist_python_type::<P>()`.
    #[pyfunction]
    pub fn copperlist_iterator_unified(unified_src_path: &str) -> PyResult<PyCopperListIterator> {
        let decode_next = *COPPERLIST_DECODER.get().ok_or_else(|| {
            PyRuntimeError::new_err(
                "CopperList decoder is not registered. \
Call register_copperlist_python_type::<P>() from Rust before using this function.",
            )
        })?;

        let logger = UnifiedLoggerBuilder::new()
            .file_base_name(Path::new(unified_src_path))
            .build()
            .map_err(|e| PyIOError::new_err(e.to_string()))?;
        let dl = match logger {
            UnifiedLogger::Read(dl) => dl,
            UnifiedLogger::Write(_) => {
                return Err(PyIOError::new_err(
                    "Expected read-only unified logger for Python export",
                ));
            }
        };

        let reader = UnifiedLoggerIOReader::new(dl, UnifiedLogType::CopperList);
        Ok(PyCopperListIterator {
            reader: Box::new(reader),
            decode_next,
        })
    }

    /// Create an iterator over runtime lifecycle records from a unified log file.
    #[pyfunction]
    pub fn runtime_lifecycle_iterator_unified(
        unified_src_path: &str,
    ) -> PyResult<PyRuntimeLifecycleIterator> {
        let logger = UnifiedLoggerBuilder::new()
            .file_base_name(Path::new(unified_src_path))
            .build()
            .map_err(|e| PyIOError::new_err(e.to_string()))?;
        let dl = match logger {
            UnifiedLogger::Read(dl) => dl,
            UnifiedLogger::Write(_) => {
                return Err(PyIOError::new_err(
                    "Expected read-only unified logger for Python export",
                ));
            }
        };

        let reader = UnifiedLoggerIOReader::new(dl, UnifiedLogType::RuntimeLifecycle);
        Ok(PyRuntimeLifecycleIterator {
            reader: Box::new(reader),
        })
    }
    /// Python wrapper for [`CuLogEntry`].
    #[pyclass]
    pub struct PyCuLogEntry {
        pub inner: CuLogEntry,
    }

    #[pymethods]
    impl PyCuLogEntry {
        /// Return the timestamp of the log entry as a `datetime.timedelta`.
        pub fn ts<'a>(&self, py: Python<'a>) -> PyResult<Bound<'a, PyDelta>> {
            let nanoseconds: u64 = self.inner.time.into();

            // Convert nanoseconds to seconds and microseconds
            let days = (nanoseconds / 86_400_000_000_000) as i32;
            let seconds = (nanoseconds / 1_000_000_000) as i32;
            let microseconds = ((nanoseconds % 1_000_000_000) / 1_000) as i32;

            PyDelta::new(py, days, seconds, microseconds, false)
        }

        /// Return the index of the message format string in the interned string table.
        pub fn msg_index(&self) -> u32 {
            self.inner.msg_index
        }

        /// Return the indexes of the parameter names in the interned string table.
        pub fn paramname_indexes(&self) -> Vec<u32> {
            self.inner.paramname_indexes.iter().copied().collect()
        }

        /// Return the structured parameters carried by this log line.
        pub fn params(&self, py: Python<'_>) -> PyResult<Vec<Py<PyAny>>> {
            self.inner
                .params
                .iter()
                .map(|value| value_to_py(value, py))
                .collect()
        }
    }

    /// This needs to match the name of the generated '.so'
    #[pymodule(name = "libcu29_export")]
    fn cu29_export(m: &Bound<'_, PyModule>) -> PyResult<()> {
        m.add_class::<PyCuLogEntry>()?;
        m.add_class::<PyLogIterator>()?;
        m.add_class::<PyCopperListIterator>()?;
        m.add_class::<PyRuntimeLifecycleIterator>()?;
        m.add_class::<PyUnitValue>()?;
        m.add_function(wrap_pyfunction!(struct_log_iterator_bare, m)?)?;
        m.add_function(wrap_pyfunction!(struct_log_iterator_unified, m)?)?;
        m.add_function(wrap_pyfunction!(copperlist_iterator_unified, m)?)?;
        m.add_function(wrap_pyfunction!(runtime_lifecycle_iterator_unified, m)?)?;
        Ok(())
    }

    fn decode_next_copperlist<P>(
        reader: &mut Box<dyn Read + Send + Sync>,
        py: Python<'_>,
    ) -> Option<PyResult<Py<PyAny>>>
    where
        P: CopperListTuple,
    {
        let entry = super::read_next_entry::<CopperList<P>>(reader)?;
        Some(copperlist_to_py::<P>(&entry, py))
    }

    fn copperlist_to_py<P>(entry: &CopperList<P>, py: Python<'_>) -> PyResult<Py<PyAny>>
    where
        P: CopperListTuple,
    {
        let task_ids = P::get_all_task_ids();
        let root = PyDict::new(py);
        root.set_item("id", entry.id)?;
        root.set_item("state", entry.get_state().to_string())?;

        let mut messages: Vec<Py<PyAny>> = Vec::new();
        for (idx, msg) in entry.cumsgs().into_iter().enumerate() {
            let message = PyDict::new(py);
            message.set_item("task_id", task_ids.get(idx).copied().unwrap_or("unknown"))?;
            message.set_item("tov", tov_to_py(msg.tov(), py)?)?;
            message.set_item("metadata", metadata_to_py(msg.metadata(), py)?)?;
            match msg.payload_reflect() {
                Some(payload) => message.set_item(
                    "payload",
                    partial_reflect_to_py(payload.as_partial_reflect(), py)?,
                )?,
                None => message.set_item("payload", py.None())?,
            }
            messages.push(dict_to_namespace(message, py)?);
        }

        root.set_item("messages", PyList::new(py, messages)?)?;
        dict_to_namespace(root, py)
    }

    fn runtime_lifecycle_record_to_py(
        entry: &RuntimeLifecycleRecord,
        py: Python<'_>,
    ) -> PyResult<Py<PyAny>> {
        let root = PyDict::new(py);
        root.set_item("timestamp_ns", entry.timestamp.as_nanos())?;
        root.set_item("event", runtime_lifecycle_event_to_py(&entry.event, py)?)?;
        dict_to_namespace(root, py)
    }

    fn runtime_lifecycle_event_to_py(
        event: &RuntimeLifecycleEvent,
        py: Python<'_>,
    ) -> PyResult<Py<PyAny>> {
        let root = PyDict::new(py);
        match event {
            RuntimeLifecycleEvent::Instantiated {
                config_source,
                effective_config_ron,
                stack,
            } => {
                root.set_item("kind", "instantiated")?;
                root.set_item("config_source", runtime_config_source_to_py(config_source))?;
                root.set_item("effective_config_ron", effective_config_ron)?;

                let stack_py = PyDict::new(py);
                stack_py.set_item("app_name", &stack.app_name)?;
                stack_py.set_item("app_version", &stack.app_version)?;
                stack_py.set_item("git_commit", &stack.git_commit)?;
                stack_py.set_item("git_dirty", stack.git_dirty)?;
                root.set_item("stack", dict_to_namespace(stack_py, py)?)?;
            }
            RuntimeLifecycleEvent::MissionStarted { mission } => {
                root.set_item("kind", "mission_started")?;
                root.set_item("mission", mission)?;
            }
            RuntimeLifecycleEvent::MissionStopped { mission, reason } => {
                root.set_item("kind", "mission_stopped")?;
                root.set_item("mission", mission)?;
                root.set_item("reason", reason)?;
            }
            RuntimeLifecycleEvent::Panic {
                message,
                file,
                line,
                column,
            } => {
                root.set_item("kind", "panic")?;
                root.set_item("message", message)?;
                root.set_item("file", file)?;
                root.set_item("line", line)?;
                root.set_item("column", column)?;
            }
            RuntimeLifecycleEvent::ShutdownCompleted => {
                root.set_item("kind", "shutdown_completed")?;
            }
        }

        dict_to_namespace(root, py)
    }

    fn runtime_config_source_to_py(source: &RuntimeLifecycleConfigSource) -> &'static str {
        match source {
            RuntimeLifecycleConfigSource::ProgrammaticOverride => "programmatic_override",
            RuntimeLifecycleConfigSource::ExternalFile => "external_file",
            RuntimeLifecycleConfigSource::BundledDefault => "bundled_default",
        }
    }

    fn metadata_to_py(metadata: &dyn CuMsgMetadataTrait, py: Python<'_>) -> PyResult<Py<PyAny>> {
        let process = metadata.process_time();
        let start: Option<CuTime> = process.start.into();
        let end: Option<CuTime> = process.end.into();

        let process_time = PyDict::new(py);
        process_time.set_item("start_ns", start.map(|t| t.as_nanos()))?;
        process_time.set_item("end_ns", end.map(|t| t.as_nanos()))?;

        let metadata_py = PyDict::new(py);
        metadata_py.set_item("process_time", dict_to_namespace(process_time, py)?)?;
        metadata_py.set_item("status_txt", metadata.status_txt().0.to_string())?;
        if let Some(origin) = metadata.origin() {
            let origin_py = PyDict::new(py);
            origin_py.set_item("subsystem_code", origin.subsystem_code)?;
            origin_py.set_item("instance_id", origin.instance_id)?;
            origin_py.set_item("cl_id", origin.cl_id)?;
            metadata_py.set_item("origin", dict_to_namespace(origin_py, py)?)?;
        } else {
            metadata_py.set_item("origin", py.None())?;
        }
        dict_to_namespace(metadata_py, py)
    }

    fn tov_to_py(tov: Tov, py: Python<'_>) -> PyResult<Py<PyAny>> {
        let tov_py = PyDict::new(py);
        match tov {
            Tov::None => {
                tov_py.set_item("kind", "none")?;
            }
            Tov::Time(t) => {
                tov_py.set_item("kind", "time")?;
                tov_py.set_item("time_ns", t.as_nanos())?;
            }
            Tov::Range(r) => {
                tov_py.set_item("kind", "range")?;
                tov_py.set_item("start_ns", r.start.as_nanos())?;
                tov_py.set_item("end_ns", r.end.as_nanos())?;
            }
        }
        dict_to_namespace(tov_py, py)
    }

    fn partial_reflect_to_py(value: &dyn PartialReflect, py: Python<'_>) -> PyResult<Py<PyAny>> {
        #[allow(unreachable_patterns)]
        match value.reflect_ref() {
            ReflectRef::Struct(s) => struct_to_py(s, py),
            ReflectRef::TupleStruct(ts) => tuple_struct_to_py(ts, py),
            ReflectRef::Tuple(t) => tuple_to_py(t, py),
            ReflectRef::List(list) => list_to_py(list, py),
            ReflectRef::Array(array) => array_to_py(array, py),
            ReflectRef::Map(map) => map_to_py(map, py),
            ReflectRef::Set(set) => set_to_py(set, py),
            ReflectRef::Enum(e) => enum_to_py(e, py),
            ReflectRef::Opaque(opaque) => opaque_to_py(opaque, py),
            _ => Ok(py.None()),
        }
    }

    fn struct_to_py(value: &dyn cu29::bevy_reflect::Struct, py: Python<'_>) -> PyResult<Py<PyAny>> {
        let dict = PyDict::new(py);
        for idx in 0..value.field_len() {
            if let Some(field) = value.field_at(idx) {
                let name = value
                    .name_at(idx)
                    .map(str::to_owned)
                    .unwrap_or_else(|| format!("field_{idx}"));
                dict.set_item(name, partial_reflect_to_py(field, py)?)?;
            }
        }

        if let Some(unit) = unit_abbrev_for_type_path(value.reflect_type_path())
            && let Some(raw_value) = dict.get_item("value")?
        {
            if let Ok(v) = raw_value.extract::<f64>() {
                let unit_value = PyUnitValue {
                    value: v,
                    unit: unit.to_string(),
                };
                return Ok(Py::new(py, unit_value)?.into());
            }
            if let Ok(v) = raw_value.extract::<f32>() {
                let unit_value = PyUnitValue {
                    value: v as f64,
                    unit: unit.to_string(),
                };
                return Ok(Py::new(py, unit_value)?.into());
            }
        }

        dict_to_namespace(dict, py)
    }

    fn tuple_struct_to_py(
        value: &dyn cu29::bevy_reflect::TupleStruct,
        py: Python<'_>,
    ) -> PyResult<Py<PyAny>> {
        let mut fields = Vec::with_capacity(value.field_len());
        for idx in 0..value.field_len() {
            if let Some(field) = value.field(idx) {
                fields.push(partial_reflect_to_py(field, py)?);
            } else {
                fields.push(py.None());
            }
        }
        Ok(PyList::new(py, fields)?.into_pyobject(py)?.into())
    }

    fn tuple_to_py(value: &dyn cu29::bevy_reflect::Tuple, py: Python<'_>) -> PyResult<Py<PyAny>> {
        let mut fields = Vec::with_capacity(value.field_len());
        for idx in 0..value.field_len() {
            if let Some(field) = value.field(idx) {
                fields.push(partial_reflect_to_py(field, py)?);
            } else {
                fields.push(py.None());
            }
        }
        Ok(PyList::new(py, fields)?.into_pyobject(py)?.into())
    }

    fn list_to_py(value: &dyn cu29::bevy_reflect::List, py: Python<'_>) -> PyResult<Py<PyAny>> {
        let mut items = Vec::with_capacity(value.len());
        for item in value.iter() {
            items.push(partial_reflect_to_py(item, py)?);
        }
        Ok(PyList::new(py, items)?.into_pyobject(py)?.into())
    }

    fn array_to_py(value: &dyn cu29::bevy_reflect::Array, py: Python<'_>) -> PyResult<Py<PyAny>> {
        let mut items = Vec::with_capacity(value.len());
        for item in value.iter() {
            items.push(partial_reflect_to_py(item, py)?);
        }
        Ok(PyList::new(py, items)?.into_pyobject(py)?.into())
    }

    fn map_to_py(value: &dyn cu29::bevy_reflect::Map, py: Python<'_>) -> PyResult<Py<PyAny>> {
        let dict = PyDict::new(py);
        for (key, val) in value.iter() {
            let key_str = reflect_key_to_string(key);
            dict.set_item(key_str, partial_reflect_to_py(val, py)?)?;
        }
        Ok(dict.into_pyobject(py)?.into())
    }

    fn set_to_py(value: &dyn cu29::bevy_reflect::Set, py: Python<'_>) -> PyResult<Py<PyAny>> {
        let mut items = Vec::with_capacity(value.len());
        for item in value.iter() {
            items.push(partial_reflect_to_py(item, py)?);
        }
        Ok(PyList::new(py, items)?.into_pyobject(py)?.into())
    }

    fn enum_to_py(value: &dyn cu29::bevy_reflect::Enum, py: Python<'_>) -> PyResult<Py<PyAny>> {
        let dict = PyDict::new(py);
        dict.set_item("variant", value.variant_name())?;

        match value.variant_type() {
            VariantType::Unit => {}
            VariantType::Tuple => {
                let mut fields = Vec::with_capacity(value.field_len());
                for idx in 0..value.field_len() {
                    if let Some(field) = value.field_at(idx) {
                        fields.push(partial_reflect_to_py(field, py)?);
                    } else {
                        fields.push(py.None());
                    }
                }
                dict.set_item("fields", PyList::new(py, fields)?)?;
            }
            VariantType::Struct => {
                let fields = PyDict::new(py);
                for idx in 0..value.field_len() {
                    if let Some(field) = value.field_at(idx) {
                        let name = value
                            .name_at(idx)
                            .map(str::to_owned)
                            .unwrap_or_else(|| format!("field_{idx}"));
                        fields.set_item(name, partial_reflect_to_py(field, py)?)?;
                    }
                }
                dict.set_item("fields", fields)?;
            }
        }

        dict_to_namespace(dict, py)
    }

    fn dict_to_namespace(dict: Bound<'_, PyDict>, py: Python<'_>) -> PyResult<Py<PyAny>> {
        let types = py.import("types")?;
        let namespace_ctor = types.getattr("SimpleNamespace")?;
        let namespace = namespace_ctor.call((), Some(&dict))?;
        Ok(namespace.into())
    }

    fn reflect_key_to_string(value: &dyn PartialReflect) -> String {
        if let Some(v) = value.try_downcast_ref::<String>() {
            return v.clone();
        }
        if let Some(v) = value.try_downcast_ref::<&'static str>() {
            return (*v).to_string();
        }
        if let Some(v) = value.try_downcast_ref::<char>() {
            return v.to_string();
        }
        if let Some(v) = value.try_downcast_ref::<bool>() {
            return v.to_string();
        }
        if let Some(v) = value.try_downcast_ref::<u64>() {
            return v.to_string();
        }
        if let Some(v) = value.try_downcast_ref::<i64>() {
            return v.to_string();
        }
        if let Some(v) = value.try_downcast_ref::<usize>() {
            return v.to_string();
        }
        if let Some(v) = value.try_downcast_ref::<isize>() {
            return v.to_string();
        }
        format!("{value:?}")
    }

    fn unit_abbrev_for_type_path(type_path: &str) -> Option<&'static str> {
        match type_path.rsplit("::").next()? {
            "Acceleration" => Some("m/s^2"),
            "Angle" => Some("rad"),
            "AngularVelocity" => Some("rad/s"),
            "ElectricPotential" => Some("V"),
            "Length" => Some("m"),
            "MagneticFluxDensity" => Some("T"),
            "Pressure" => Some("Pa"),
            "Ratio" => Some("1"),
            "ThermodynamicTemperature" => Some("K"),
            "Time" => Some("s"),
            "Velocity" => Some("m/s"),
            _ => None,
        }
    }

    fn opaque_to_py(value: &dyn PartialReflect, py: Python<'_>) -> PyResult<Py<PyAny>> {
        macro_rules! downcast_copy {
            ($ty:ty) => {
                if let Some(v) = value.try_downcast_ref::<$ty>() {
                    return Ok(v.into_pyobject(py)?.to_owned().into());
                }
            };
        }

        downcast_copy!(bool);
        downcast_copy!(u8);
        downcast_copy!(u16);
        downcast_copy!(u32);
        downcast_copy!(u64);
        downcast_copy!(u128);
        downcast_copy!(usize);
        downcast_copy!(i8);
        downcast_copy!(i16);
        downcast_copy!(i32);
        downcast_copy!(i64);
        downcast_copy!(i128);
        downcast_copy!(isize);
        downcast_copy!(f32);
        downcast_copy!(f64);
        downcast_copy!(char);

        if let Some(v) = value.try_downcast_ref::<String>() {
            return Ok(v.into_pyobject(py)?.into());
        }
        if let Some(v) = value.try_downcast_ref::<&'static str>() {
            return Ok(v.into_pyobject(py)?.into());
        }
        if let Some(v) = value.try_downcast_ref::<Vec<u8>>() {
            return Ok(v.into_pyobject(py)?.into());
        }

        let fallback = format!("{value:?}");
        Ok(fallback.into_pyobject(py)?.into())
    }
    fn value_to_py(value: &cu29::prelude::Value, py: Python<'_>) -> PyResult<Py<PyAny>> {
        match value {
            Value::String(s) => Ok(s.into_pyobject(py)?.into()),
            Value::U64(u) => Ok(u.into_pyobject(py)?.into()),
            Value::U128(u) => Ok(u.into_pyobject(py)?.into()),
            Value::I64(i) => Ok(i.into_pyobject(py)?.into()),
            Value::I128(i) => Ok(i.into_pyobject(py)?.into()),
            Value::F64(f) => Ok(f.into_pyobject(py)?.into()),
            Value::Bool(b) => Ok(b.into_pyobject(py)?.to_owned().into()),
            Value::CuTime(t) => Ok(t.0.into_pyobject(py)?.into()),
            Value::Bytes(b) => Ok(b.into_pyobject(py)?.into()),
            Value::Char(c) => Ok(c.into_pyobject(py)?.into()),
            Value::I8(i) => Ok(i.into_pyobject(py)?.into()),
            Value::U8(u) => Ok(u.into_pyobject(py)?.into()),
            Value::I16(i) => Ok(i.into_pyobject(py)?.into()),
            Value::U16(u) => Ok(u.into_pyobject(py)?.into()),
            Value::I32(i) => Ok(i.into_pyobject(py)?.into()),
            Value::U32(u) => Ok(u.into_pyobject(py)?.into()),
            Value::Map(m) => {
                let dict = PyDict::new(py);
                for (k, v) in m.iter() {
                    dict.set_item(value_to_py(k, py)?, value_to_py(v, py)?)?;
                }
                Ok(dict.into_pyobject(py)?.into())
            }
            Value::F32(f) => Ok(f.into_pyobject(py)?.into()),
            Value::Option(o) => match o.as_ref() {
                Some(value) => value_to_py(value, py),
                None => Ok(py.None()),
            },
            Value::Unit => Ok(py.None()),
            Value::Newtype(v) => value_to_py(v, py),
            Value::Seq(s) => {
                let items: Vec<Py<PyAny>> = s
                    .iter()
                    .map(|value| value_to_py(value, py))
                    .collect::<PyResult<_>>()?;
                let list = PyList::new(py, items)?;
                Ok(list.into_pyobject(py)?.into())
            }
        }
    }

    #[cfg(test)]
    mod tests {
        use super::*;

        #[test]
        fn value_to_py_preserves_128_bit_integers() {
            Python::initialize();
            Python::attach(|py| {
                let u128_value = u128::from(u64::MAX) + 99;
                let u128_py = value_to_py(&Value::U128(u128_value), py).unwrap();
                assert_eq!(u128_py.bind(py).extract::<u128>().unwrap(), u128_value);

                let i128_value = i128::from(i64::MIN) - 99;
                let i128_py = value_to_py(&Value::I128(i128_value), py).unwrap();
                assert_eq!(i128_py.bind(py).extract::<i128>().unwrap(), i128_value);
            });
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bincode::{Decode, Encode, encode_into_slice};
    use serde::Deserialize;
    use std::env;
    use std::fs;
    use std::io::Cursor;
    use std::path::PathBuf;
    use std::sync::{Arc, Mutex};
    use tempfile::{TempDir, tempdir};

    fn copy_stringindex_to_temp(tmpdir: &TempDir) -> PathBuf {
        // Build a minimal index on the fly so tests don't depend on build-time artifacts.
        let fake_out_dir = tmpdir.path().join("build").join("out").join("dir");
        fs::create_dir_all(&fake_out_dir).unwrap();
        // SAFETY: Tests run single-threaded here and we only read the variable after setting it.
        unsafe {
            env::set_var("LOG_INDEX_DIR", &fake_out_dir);
        }

        // Provide entries for the message indexes used in this test module.
        let _ = cu29_intern_strs::intern_string("unused to start counter");
        let _ = cu29_intern_strs::intern_string("Just a String {}");
        let _ = cu29_intern_strs::intern_string("Just a String (low level) {}");
        let _ = cu29_intern_strs::intern_string("Just a String (end to end) {}");

        let index_dir = cu29_intern_strs::default_log_index_dir();
        cu29_intern_strs::read_interned_strings(&index_dir).unwrap();
        index_dir
    }

    #[test]
    fn test_extract_low_level_cu29_log() {
        let temp_dir = TempDir::new().unwrap();
        let temp_path = copy_stringindex_to_temp(&temp_dir);
        let entry = CuLogEntry::new(3, CuLogLevel::Info);
        let bytes = bincode::encode_to_vec(&entry, standard()).unwrap();
        let reader = Cursor::new(bytes.as_slice());
        textlog_dump(reader, temp_path.as_path()).unwrap();
    }

    #[test]
    fn end_to_end_datalogger_and_structlog_test() {
        let dir = tempdir().expect("Failed to create temp dir");
        let path = dir
            .path()
            .join("end_to_end_datalogger_and_structlog_test.copper");
        {
            // Write a couple log entries
            let UnifiedLogger::Write(logger) = UnifiedLoggerBuilder::new()
                .write(true)
                .create(true)
                .file_base_name(&path)
                .preallocated_size(100000)
                .build()
                .expect("Failed to create logger")
            else {
                panic!("Failed to create logger")
            };
            let data_logger = Arc::new(Mutex::new(logger));
            let stream = stream_write(data_logger.clone(), UnifiedLogType::StructuredLogLine, 1024)
                .expect("Failed to create stream");
            let rt = LoggerRuntime::init(RobotClock::default(), stream, None::<NullLog>);

            let mut entry = CuLogEntry::new(4, CuLogLevel::Info); // this is a "Just a String {}" log line
            entry.add_param(0, Value::String("Parameter for the log line".into()));
            log(&mut entry).expect("Failed to log");
            let mut entry = CuLogEntry::new(2, CuLogLevel::Info); // this is a "Just a String {}" log line
            entry.add_param(0, Value::String("Parameter for the log line".into()));
            log(&mut entry).expect("Failed to log");

            // everything is dropped here
            drop(rt);
        }
        // Read back the log
        let UnifiedLogger::Read(logger) = UnifiedLoggerBuilder::new()
            .file_base_name(
                &dir.path()
                    .join("end_to_end_datalogger_and_structlog_test.copper"),
            )
            .build()
            .expect("Failed to create logger")
        else {
            panic!("Failed to create logger")
        };
        let reader = UnifiedLoggerIOReader::new(logger, UnifiedLogType::StructuredLogLine);
        let temp_dir = TempDir::new().unwrap();
        textlog_dump(
            reader,
            Path::new(copy_stringindex_to_temp(&temp_dir).as_path()),
        )
        .expect("Failed to dump log");
    }

    // This is normally generated at compile time in CuPayload.
    #[derive(Debug, PartialEq, Clone, Copy, Serialize, Deserialize, Encode, Decode, Default)]
    struct MyMsgs((u8, i32, f32));

    impl ErasedCuStampedDataSet for MyMsgs {
        fn cumsgs(&self) -> Vec<&dyn ErasedCuStampedData> {
            Vec::new()
        }
    }

    impl MatchingTasks for MyMsgs {
        fn get_all_task_ids() -> &'static [&'static str] {
            &[]
        }
    }

    /// Checks if we can recover the copper lists from a binary representation.
    #[test]
    fn test_copperlists_dump() {
        let mut data = vec![0u8; 10000];
        let mypls: [MyMsgs; 4] = [
            MyMsgs((1, 2, 3.0)),
            MyMsgs((2, 3, 4.0)),
            MyMsgs((3, 4, 5.0)),
            MyMsgs((4, 5, 6.0)),
        ];

        let mut offset: usize = 0;
        for pl in mypls.iter() {
            let cl = CopperList::<MyMsgs>::new(1, *pl);
            offset +=
                encode_into_slice(&cl, &mut data.as_mut_slice()[offset..], standard()).unwrap();
        }

        let reader = Cursor::new(data);

        let mut iter = copperlists_reader::<MyMsgs>(reader);
        assert_eq!(iter.next().unwrap().msgs, MyMsgs((1, 2, 3.0)));
        assert_eq!(iter.next().unwrap().msgs, MyMsgs((2, 3, 4.0)));
        assert_eq!(iter.next().unwrap().msgs, MyMsgs((3, 4, 5.0)));
        assert_eq!(iter.next().unwrap().msgs, MyMsgs((4, 5, 6.0)));
    }

    #[test]
    fn runtime_lifecycle_reader_extracts_started_mission() {
        let records = vec![
            RuntimeLifecycleRecord {
                timestamp: CuTime::default(),
                event: RuntimeLifecycleEvent::Instantiated {
                    config_source: RuntimeLifecycleConfigSource::BundledDefault,
                    effective_config_ron: "(missions: [])".to_string(),
                    stack: RuntimeLifecycleStackInfo {
                        app_name: "demo".to_string(),
                        app_version: "0.1.0".to_string(),
                        git_commit: None,
                        git_dirty: None,
                    },
                },
            },
            RuntimeLifecycleRecord {
                timestamp: CuTime::from_nanos(42),
                event: RuntimeLifecycleEvent::MissionStarted {
                    mission: "gnss".to_string(),
                },
            },
        ];
        let mut bytes = Vec::new();
        for record in &records {
            bytes.extend(bincode::encode_to_vec(record, standard()).unwrap());
        }

        let mission =
            runtime_lifecycle_reader(Cursor::new(bytes)).find_map(|entry| match entry.event {
                RuntimeLifecycleEvent::MissionStarted { mission } => Some(mission),
                _ => None,
            });
        assert_eq!(mission.as_deref(), Some("gnss"));
    }
}
