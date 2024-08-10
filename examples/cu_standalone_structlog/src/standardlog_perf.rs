use std::fs::File;
use std::path::Path;
use log::{info, LevelFilter};
use simplelog::{Config, WriteLogger};
use cu29_clock::RobotClock;

const LOG_FILE: &str = "./logfile.txt";

pub fn main() {
    let clock = RobotClock::new();

    WriteLogger::init(
        LevelFilter::Info,  // Set the desired log level
        Config::default(),
        File::create(Path::new(LOG_FILE)).unwrap()).unwrap();

    let bf = clock.now();
    for i in 0..1_000_000 {
        info!("This is the logline {} associated with the log Logging and some more = {}, {}", i , i+2, i+3);
    }
    log::logger().flush();
    let af = clock.now();
    println!("Total time: {} in {}", af - bf, LOG_FILE);
}