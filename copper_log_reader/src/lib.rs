use bincode::config::standard;
use bincode::decode_from_std_read;
use byteorder::{ByteOrder, LittleEndian};
use copper_log::{rebuild_logline, CuLogEntry};
use copper_traits::{CuError, CuResult};
use rkv::backend::Lmdb;
use rkv::{Rkv, StoreOptions};
use std::io::Read;
use std::path::Path;

/// Full dump of the copper structured log from its binary representation.
/// src: the source of the log data
/// index: the path to the index file (containing the interned strings constructed at build time)
pub fn full_log_dump(mut src: impl Read, index: &Path) -> CuResult<()> {
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

/// Rebuild the interned string index in memory.
pub fn read_interned_strings(index: &Path) -> CuResult<Vec<String>> {
    let mut all_strings = Vec::<String>::new();
    let env = Rkv::new::<Lmdb>(index).map_err(|e| {
        CuError::from("Could not open the string index. Check the path.")
            .add_cause(e.to_string().as_str())
    })?;

    let index_to_string = env
        .open_single("index_to_string", StoreOptions::default())
        .expect("Failed to open index_to_string store");
    let db_reader = env.read().unwrap();
    let ri = index_to_string.iter_start(&db_reader);
    let mut i = ri.expect("Failed to start iterator");
    while let Some(Ok(v)) = i.next() {
        let (k, v) = v;
        let index = LittleEndian::read_u32(&k) as usize;

        if let rkv::Value::Str(s) = v {
            if all_strings.len() <= index as usize {
                all_strings.resize(index as usize + 1, String::new());
            }

            all_strings[index] = s.to_string();
        }
    }
    Ok(all_strings)
}

#[cfg(test)]
mod tests {

    use super::*;
    use copper_clock::RobotClock;
    use copper_datalogger::{stream_write, DataLogger, DataLoggerBuilder, DataLoggerIOReader};
    use copper_log::value::Value;
    use copper_log_runtime::log;
    use copper_log_runtime::LoggerRuntime;
    use copper_traits::{DataLogType, WriteStream};
    use std::io::{Cursor, Write};
    use std::sync::{Arc, Mutex};
    use tempfile::tempdir;

    #[test]
    fn test_extract_low_level_copper_log() {
        let entry = CuLogEntry::new(5);
        let bytes = bincode::encode_to_vec(&entry, standard()).unwrap();
        let reader = Cursor::new(bytes.as_slice());
        full_log_dump(reader, Path::new("test/copper_log_index"));
    }

    #[test]
    fn end_to_end_datalogger_and_structlog_test() {
        let dir = tempdir().expect("Failed to create temp dir");
        let path = dir
            .path()
            .join("end_to_end_datalogger_and_structlog_test.coppper");
        {
            // Write a couple log entries
            let DataLogger::Write(logger) = DataLoggerBuilder::new()
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
            let stream = stream_write(data_logger.clone(), DataLogType::StructuredLogLine, 1024);
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
        let DataLogger::Read(logger) = DataLoggerBuilder::new()
            .file_path(&path)
            .build()
            .expect("Failed to create logger")
        else {
            panic!("Failed to create logger")
        };
        let reader = DataLoggerIOReader::new(logger, DataLogType::StructuredLogLine);
        full_log_dump(reader, Path::new("test/copper_log_index"));
    }
}
