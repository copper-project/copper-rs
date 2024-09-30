use bincode::config::standard;
use bincode::decode_from_std_read;
use bincode::error::DecodeError;
use cu29::cutask::CuMsg as _CuMsg;
use cu29_derive::gen_cumsgs;
use cu29_intern_strs::read_interned_strings;
use cu29_log::CuLogEntry;
use cu29_traits::UnifiedLogType;
use cu29_unifiedlog::{UnifiedLogger, UnifiedLoggerBuilder, UnifiedLoggerIOReader};
use std::path::Path;

gen_cumsgs!("copperconfig.ron");

fn main() {
    // run_cli::<CuMsgs>().expect("Failed to run the export CLI");

    // extract PID values
    let UnifiedLogger::Read(dl) = UnifiedLoggerBuilder::new()
        .file_base_name(Path::new("logs/balance.copper"))
        .build()
        .expect("Failed to create logger")
    else {
        panic!("Failed to create logger");
    };
    let mut reader = UnifiedLoggerIOReader::new(dl, UnifiedLogType::StructuredLogLine);
    let all_strings = read_interned_strings(Path::new("cu29_log_index")).unwrap();
    println!("input, p, i, d, total");
    loop {
        let entry = decode_from_std_read::<CuLogEntry, _, _>(&mut reader, standard());

        match entry {
            Err(DecodeError::UnexpectedEnd { .. }) => break,
            Err(DecodeError::Io { inner, additional }) => {
                if inner.kind() == std::io::ErrorKind::UnexpectedEof {
                    break;
                } else {
                    println!("Error {:?} additional:{}", inner, additional);
                    break;
                }
            }
            Err(e) => {
                println!("Error {:?}", e);
                break;
            }
            Ok(entry) => {
                if entry.msg_index == 0 {
                    break;
                }
                if all_strings[entry.msg_index as usize].starts_with("PIDTask output") {
                    for param in entry.params {
                        print!("{},", param);
                    }
                    println!();
                }
            }
        };
    }
}
