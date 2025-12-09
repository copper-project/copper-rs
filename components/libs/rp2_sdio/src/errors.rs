use defmt::Format;
use embedded_io::{Error as IoError, ErrorKind};

#[derive(Debug, Format)]
pub enum SdioError {
    CmdTimeout { cmd: u8, time_ms: u64 },
    WrongCmd { good_cmd: u8, bad_cmd: u8 },
    BadRxCrc16 { good_crc: u64, bad_crc: u64 },
    IllegalCommand {},
    BadCheck { good_check: u8, bad_check: u8 },
    BadVoltage { bad_volt: u8 },
    BadVoltRange { range: u32 },
}

impl IoError for SdioError {
    fn kind(&self) -> ErrorKind {
        // Map everything to Other for now; callers can inspect the variant for details.
        ErrorKind::Other
    }
}
