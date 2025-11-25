mod fsck;

use bincode::config::standard;
use bincode::decode_from_std_read;
use bincode::error::DecodeError;
use clap::{Parser, Subcommand, ValueEnum};
use cu29::prelude::*;
use cu29::UnifiedLogType;
use cu29_intern_strs::read_interned_strings;
use fsck::check;
use std::fmt::{Display, Formatter};
use std::io::Read;
use std::path::{Path, PathBuf};

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
    },
}

/// This is a generator for a main function to build a log extractor.
/// It depends on the specific type of the CopperList payload that is determined at compile time from the configuration.
pub fn run_cli<P>() -> CuResult<()>
where
    P: CopperListTuple,
{
    let args = LogReaderCli::parse();
    let unifiedlog_base = args.unifiedlog_base;

    let UnifiedLogger::Read(mut dl) = UnifiedLoggerBuilder::new()
        .file_base_name(&unifiedlog_base)
        .build()
        .expect("Failed to create logger")
    else {
        panic!("Failed to create logger");
    };

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
                        serde_json::to_writer_pretty(std::io::stdout(), &entry).unwrap();
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
                                serde_json::to_writer(std::io::stdout(), payload).unwrap(); // TODO: escape for CSV
                                first = false;
                            }
                        }
                        println!();
                    }
                }
            }
        }
        Command::Fsck { verbose } => {
            if let Some(value) = check::<P>(&mut dl, verbose) {
                return value;
            }
        }
    }

    Ok(())
}
/// Extracts the copper lists from a binary representation.
/// P is the Payload determined by the configuration of the application.
pub fn copperlists_reader<P: CopperListTuple>(
    mut src: impl Read,
) -> impl Iterator<Item = CopperList<P>> {
    std::iter::from_fn(move || {
        let entry = decode_from_std_read::<CopperList<P>, _, _>(&mut src, standard());
        match entry {
            Ok(entry) => Some(entry),
            Err(e) => match e {
                DecodeError::UnexpectedEnd { .. } => None,
                DecodeError::Io { inner, additional } => {
                    if inner.kind() == std::io::ErrorKind::UnexpectedEof {
                        None
                    } else {
                        println!("Error {inner:?} additional:{additional}");
                        None
                    }
                }
                _ => {
                    println!("Error {e:?}");
                    None
                }
            },
        }
    })
}

/// Extracts the keyframes from the log.
pub fn keyframes_reader(mut src: impl Read) -> impl Iterator<Item = KeyFrame> {
    std::iter::from_fn(move || {
        let entry = decode_from_std_read::<KeyFrame, _, _>(&mut src, standard());
        match entry {
            Ok(entry) => Some(entry),
            Err(e) => match e {
                DecodeError::UnexpectedEnd { .. } => None,
                DecodeError::Io { inner, additional } => {
                    if inner.kind() == std::io::ErrorKind::UnexpectedEof {
                        None
                    } else {
                        println!("Error {inner:?} additional:{additional}");
                        None
                    }
                }
                _ => {
                    println!("Error {e:?}");
                    None
                }
            },
        }
    })
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

// only for users opting into python interface, not supported on macOS at the moment
#[cfg(all(feature = "python", not(target_os = "macos")))]
mod python {
    use bincode::config::standard;
    use bincode::decode_from_std_read;
    use bincode::error::DecodeError;
    use cu29::prelude::*;
    use cu29_intern_strs::read_interned_strings;
    use pyo3::exceptions::PyIOError;
    use pyo3::prelude::*;
    use pyo3::types::{PyDelta, PyDict, PyList};
    use std::io::Read;
    use std::path::Path;

    #[pyclass]
    pub struct PyLogIterator {
        reader: Box<dyn Read + Send + Sync>,
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

    /// Creates an iterator of CuLogEntries from a bare binary structured log file (ie. not within a unified log).
    /// This is mainly used for using the structured logging out of the Copper framework.
    /// it returns a tuple with the iterator of log entries and the list of interned strings.
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
    /// Creates an iterator of CuLogEntries from a unified log file.
    /// This function allows you to easily use python to datamind Copper's structured text logs.
    /// it returns a tuple with the iterator of log entries and the list of interned strings.
    #[pyfunction]
    pub fn struct_log_iterator_unified(
        unified_src_path: &str,
        index_path: &str,
    ) -> PyResult<(PyLogIterator, Vec<String>)> {
        let all_strings = read_interned_strings(Path::new(index_path))
            .map_err(|e| PyIOError::new_err(e.to_string()))?;

        let UnifiedLogger::Read(dl) = UnifiedLoggerBuilder::new()
            .file_base_name(Path::new(unified_src_path))
            .build()
            .expect("Failed to create logger")
        else {
            panic!("Failed to create logger");
        };

        let reader = UnifiedLoggerIOReader::new(dl, UnifiedLogType::StructuredLogLine);
        Ok((
            PyLogIterator {
                reader: Box::new(reader),
            },
            all_strings,
        ))
    }

    /// This is a python wrapper for CuLogEntries.
    #[pyclass]
    pub struct PyCuLogEntry {
        pub inner: CuLogEntry,
    }

    #[pymethods]
    impl PyCuLogEntry {
        /// Returns the timestamp of the log entry.
        pub fn ts<'a>(&self, py: Python<'a>) -> Bound<'a, PyDelta> {
            let nanoseconds: u64 = self.inner.time.into();

            // Convert nanoseconds to seconds and microseconds
            let days = (nanoseconds / 86_400_000_000_000) as i32;
            let seconds = (nanoseconds / 1_000_000_000) as i32;
            let microseconds = ((nanoseconds % 1_000_000_000) / 1_000) as i32;

            PyDelta::new(py, days, seconds, microseconds, false).unwrap()
        }

        /// Returns the index of the message in the vector of interned strings.
        pub fn msg_index(&self) -> u32 {
            self.inner.msg_index
        }

        /// Returns the index of the parameter names in the vector of interned strings.
        pub fn paramname_indexes(&self) -> Vec<u32> {
            self.inner.paramname_indexes.iter().copied().collect()
        }

        /// Returns the parameters of this log line
        pub fn params(&self) -> Vec<Py<PyAny>> {
            self.inner.params.iter().map(value_to_py).collect()
        }
    }

    /// This needs to match the name of the generated '.so'
    #[pymodule(name = "libcu29_export")]
    fn cu29_export(m: &Bound<'_, PyModule>) -> PyResult<()> {
        m.add_class::<PyCuLogEntry>()?;
        m.add_class::<PyLogIterator>()?;
        m.add_function(wrap_pyfunction!(struct_log_iterator_bare, m)?)?;
        m.add_function(wrap_pyfunction!(struct_log_iterator_unified, m)?)?;
        Ok(())
    }

    fn value_to_py(value: &cu29::prelude::Value) -> Py<PyAny> {
        match value {
            Value::String(s) => Python::attach(|py| s.into_pyobject(py).unwrap().into()),
            Value::U64(u) => Python::attach(|py| u.into_pyobject(py).unwrap().into()),
            Value::I64(i) => Python::attach(|py| i.into_pyobject(py).unwrap().into()),
            Value::F64(f) => Python::attach(|py| f.into_pyobject(py).unwrap().into()),
            Value::Bool(b) => Python::attach(|py| b.into_pyobject(py).unwrap().to_owned().into()),
            Value::CuTime(t) => Python::attach(|py| t.0.into_pyobject(py).unwrap().into()),
            Value::Bytes(b) => Python::attach(|py| b.into_pyobject(py).unwrap().into()),
            Value::Char(c) => Python::attach(|py| c.into_pyobject(py).unwrap().into()),
            Value::I8(i) => Python::attach(|py| i.into_pyobject(py).unwrap().into()),
            Value::U8(u) => Python::attach(|py| u.into_pyobject(py).unwrap().into()),
            Value::I16(i) => Python::attach(|py| i.into_pyobject(py).unwrap().into()),
            Value::U16(u) => Python::attach(|py| u.into_pyobject(py).unwrap().into()),
            Value::I32(i) => Python::attach(|py| i.into_pyobject(py).unwrap().into()),
            Value::U32(u) => Python::attach(|py| u.into_pyobject(py).unwrap().into()),
            Value::Map(m) => Python::attach(|py| {
                let dict = PyDict::new(py);
                for (k, v) in m.iter() {
                    dict.set_item(value_to_py(k), value_to_py(v)).unwrap();
                }
                dict.into_pyobject(py).unwrap().into()
            }),
            Value::F32(f) => Python::attach(|py| f.into_pyobject(py).unwrap().into()),
            Value::Option(o) => Python::attach(|py| {
                if o.is_none() {
                    py.None()
                } else {
                    o.clone().map(|v| value_to_py(&v)).unwrap()
                }
            }),
            Value::Unit => Python::attach(|py| py.None()),
            Value::Newtype(v) => value_to_py(v),
            Value::Seq(s) => Python::attach(|py| {
                let list = PyList::new(py, s.iter().map(value_to_py)).unwrap();
                list.into_pyobject(py).unwrap().into()
            }),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bincode::{encode_into_slice, Decode, Encode};
    use std::env;
    use std::fs;
    use std::io::Cursor;
    use std::path::PathBuf;
    use std::sync::{Arc, Mutex};
    use tempfile::{tempdir, TempDir};

    fn copy_stringindex_to_temp(tmpdir: &TempDir) -> PathBuf {
        // Build a minimal index on the fly so tests don't depend on build-time artifacts.
        let fake_out_dir = tmpdir.path().join("build").join("out").join("dir");
        fs::create_dir_all(&fake_out_dir).unwrap();
        env::set_var("LOG_INDEX_DIR", &fake_out_dir);

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
    #[derive(Debug, PartialEq, Clone, Copy, Serialize, Encode, Decode, Default)]
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
}
