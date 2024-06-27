use clap::{Parser, Subcommand};
use copper_log_reader::full_log_dump;
use copper_traits::DataLogType;
use copper_unifiedlog::{UnifiedLogger, UnifiedLoggerBuilder, UnifiedLoggerIOReader};
use std::io::Read;
use std::path::PathBuf;

#[derive(Parser)]
#[command(version, about, long_about = None)]
struct Opts {
    #[arg(short, long)]
    index: Option<PathBuf>, // Those are default to the test case for now.
    datalog: Option<PathBuf>,
}

fn main() {
    let opts: Opts = Opts::parse();

    let index = if let Some(index) = opts.index {
        index
    } else {
        PathBuf::from("test/copper_log_index")
    };

    let datalog = if let Some(datalog) = opts.datalog {
        datalog
    } else {
        PathBuf::from("test/test.copper")
    };

    let UnifiedLogger::Read(dl) = UnifiedLoggerBuilder::new()
        .file_path(&datalog)
        .build()
        .expect("Failed to create logger")
    else {
        panic!("Failed to create logger");
    };

    let reader = UnifiedLoggerIOReader::new(dl, DataLogType::StructuredLogLine);
    full_log_dump(reader, &index).expect("Failed to dump log");
}
