use std::fmt::{Display, Formatter};
use std::io::Read;
use std::path::{Path, PathBuf};

use bincode::config::standard;
use bincode::{decode_from_std_read, Decode, Encode};

use copper::copperlist::CopperList;
use copper_intern_strs::read_interned_strings;
use copper_log::{rebuild_logline, CuLogEntry};
use copper_traits::{CuResult, UnifiedLogType};

use clap::{Parser, Subcommand, ValueEnum};
use copper_traits::CopperListPayload;
use copper_unifiedlog::{UnifiedLogger, UnifiedLoggerBuilder, UnifiedLoggerIOReader};

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
    pub unifiedlog: PathBuf,

    #[command(subcommand)]
    pub command: Command,
}

#[derive(Subcommand)]
pub enum Command {
    /// Extract logs
    ExtractLog { log_index: PathBuf },
    /// Extract copperlists
    ExtractCopperlist {
        #[arg(short, long, default_value_t = ExportFormat::Json)]
        export_format: ExportFormat,
    },
}

/// This is a generator for a main function to build a log extractor.
/// It depends on the specific type of the CopperList payload that is determined at compile time from the configuration.
pub fn run_cli<P>() -> CuResult<()>
where
    P: CopperListPayload,
{
    let args = LogReaderCli::parse();
    let unifiedlog = args.unifiedlog;

    let UnifiedLogger::Read(dl) = UnifiedLoggerBuilder::new()
        .file_path(&unifiedlog)
        .build()
        .expect("Failed to create logger")
    else {
        panic!("Failed to create logger");
    };

    match args.command {
        Command::ExtractLog { log_index } => {
            let reader = UnifiedLoggerIOReader::new(dl, UnifiedLogType::StructuredLogLine);
            textlog_dump(reader, &log_index)?;
        }
        Command::ExtractCopperlist { export_format } => {
            println!("Extracting copperlists");
            let mut reader = UnifiedLoggerIOReader::new(dl, UnifiedLogType::CopperList);
            // let mut buf = [0u8; 8];
            // reader.read(&mut buf);
            // println!("Bytes ... {:x?}", buf);
            let iter = copperlists_dump::<P>(&mut reader);
            for entry in iter {
                println!("{:#?}", entry);
            }
            println!("The end.");
        }
    }

    Ok(())
}

/// Extracts the copper lists from a binary representation.
/// P is the Payload determined by the configuration of the application.
pub fn copperlists_dump<P: CopperListPayload>(
    mut src: impl Read,
) -> impl Iterator<Item = CopperList<P>> {
    std::iter::from_fn(move || {
        let entry = decode_from_std_read::<CopperList<P>, _, _>(&mut src, standard());
        match entry {
            Ok(entry) => Some(entry),
            Err(e) => {
                println!("Error {:?}", e);
                None
            }
        }
    })
}

/// Full dump of the copper structured log from its binary representation.
/// This rebuilds a textual log.
/// src: the source of the log data
/// index: the path to the index file (containing the interned strings constructed at build time)
pub fn textlog_dump(mut src: impl Read, index: &Path) -> CuResult<()> {
    let all_strings = read_interned_strings(index)?;
    println!("Copper: -> Beginning of log <-");
    loop {
        let entry = decode_from_std_read::<CuLogEntry, _, _>(&mut src, standard());
        if entry.is_err() {
            println!("Error {:?}", entry);
            break;
        }
        let entry = entry.unwrap();

        if entry.msg_index == 0 {
            println!("Copper: -> End of log <-");
            break;
        }

        let result = rebuild_logline(&all_strings, &entry);
        if result.is_err() {
            println!("Failed to rebuild log line: {:?}", result);
            continue;
        }
        println!("Culog: [{}] {}", entry.time, result.unwrap());
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use bincode::enc::write::Writer;
    use bincode::{encode_into_slice, encode_into_writer};
    use std::io::{Cursor, Write};
    use std::sync::{Arc, Mutex};
    use tempfile::tempdir;

    use copper_clock::RobotClock;
    use copper_log::value::Value;
    use copper_log_runtime::log;
    use copper_log_runtime::LoggerRuntime;
    use copper_traits::{UnifiedLogType, WriteStream};
    use copper_unifiedlog::{
        stream_write, UnifiedLogger, UnifiedLoggerBuilder, UnifiedLoggerIOReader,
    };

    use super::*;

    #[test]
    fn test_extract_low_level_copper_log() {
        let entry = CuLogEntry::new(3);
        let bytes = bincode::encode_to_vec(&entry, standard()).unwrap();
        let reader = Cursor::new(bytes.as_slice());
        textlog_dump(reader, Path::new("test/copper_log_index"));
    }

    #[test]
    fn end_to_end_datalogger_and_structlog_test() {
        let dir = tempdir().expect("Failed to create temp dir");
        let path = dir
            .path()
            .join("end_to_end_datalogger_and_structlog_test.coppper");
        {
            // Write a couple log entries
            let UnifiedLogger::Write(logger) = UnifiedLoggerBuilder::new()
                .write(true)
                .create(true)
                .file_path(&path)
                .preallocated_size(100000)
                .build()
                .expect("Failed to create logger")
            else {
                panic!("Failed to create logger")
            };
            let data_logger = Arc::new(Mutex::new(logger));
            let stream = stream_write(data_logger.clone(), UnifiedLogType::StructuredLogLine, 1024);
            let rt = LoggerRuntime::init(RobotClock::default(), stream, None);

            let mut entry = CuLogEntry::new(4); // this is a "Just a String {}" log line
            entry.add_param(0, Value::String("Parameter for the log line".into()));
            log(entry).expect("Failed to log");
            let mut entry = CuLogEntry::new(2); // this is a "Just a String {}" log line
            entry.add_param(0, Value::String("Parameter for the log line".into()));
            log(entry).expect("Failed to log");

            // everything is dropped here
        }
        // Read back the log
        let UnifiedLogger::Read(logger) = UnifiedLoggerBuilder::new()
            .file_path(&path)
            .build()
            .expect("Failed to create logger")
        else {
            panic!("Failed to create logger")
        };
        let reader = UnifiedLoggerIOReader::new(logger, UnifiedLogType::StructuredLogLine);
        textlog_dump(reader, Path::new("test/copper_log_index"));
    }

    // This is normally generated at compile time in CuPayload.
    type MyCuPayload = (u8, i32, f32);

    /// Checks if we can recover the copper lists from a binary representation.
    #[test]
    fn test_copperlists_dump() {
        let mut data = vec![0u8; 10000];
        let mypls: [MyCuPayload; 4] = [(1, 2, 3.0), (2, 3, 4.0), (3, 4, 5.0), (4, 5, 6.0)];

        let mut offset: usize = 0;
        for pl in mypls.iter() {
            let cl = CopperList::<MyCuPayload>::new(1, *pl);
            offset +=
                encode_into_slice(&cl, &mut data.as_mut_slice()[offset..], standard()).unwrap();
        }

        let reader = Cursor::new(data);

        let mut iter = copperlists_dump::<MyCuPayload>(reader);
        assert_eq!(iter.next().unwrap().payload, (1, 2, 3.0));
        assert_eq!(iter.next().unwrap().payload, (2, 3, 4.0));
        assert_eq!(iter.next().unwrap().payload, (3, 4, 5.0));
        assert_eq!(iter.next().unwrap().payload, (4, 5, 6.0));
    }
}
