use alloc::string::{String, ToString};
use core::fmt::{Display, Formatter};

pub type Result<T> = core::result::Result<T, Error>;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Error {
    InvalidConfig(&'static str),
    ObjectTooLarge { actual: u64, maximum: u64 },
    TooManyRecords { actual: usize, maximum: usize },
    TooManySourceSymbols { actual: usize, maximum: usize },
    TooManyDatagrams { actual: usize, maximum: usize },
    BufferTooSmall { needed: usize, available: usize },
    TruncatedPacket,
    InvalidMagic,
    UnsupportedVersion(u8),
    InvalidHeaderLength(u8),
    UnknownLane(u8),
    UnknownRecordKind(u8),
    UnknownFecScheme(u8),
    UnknownSymbolKind(u8),
    PayloadLengthMismatch,
    CrcMismatch,
    InvalidFecMetadata(&'static str),
    InvalidFragment(&'static str),
    InconsistentObject,
    TruncatedRecord,
    RecordLengthMismatch,
    RecordDigestMismatch,
    Codec(String),
    Transport(&'static str),
}

impl Display for Error {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::InvalidConfig(message) => write!(formatter, "invalid configuration: {message}"),
            Self::ObjectTooLarge { actual, maximum } => {
                write!(formatter, "object has {actual} bytes; maximum is {maximum}")
            }
            Self::TooManyDatagrams { actual, maximum } => {
                write!(
                    formatter,
                    "object needs {actual} datagrams; maximum is {maximum}"
                )
            }
            Self::TooManyRecords { actual, maximum } => {
                write!(
                    formatter,
                    "stream has {actual} records; maximum is {maximum}"
                )
            }
            Self::TooManySourceSymbols { actual, maximum } => {
                write!(
                    formatter,
                    "stream has {actual} source symbols; maximum is {maximum}"
                )
            }
            Self::BufferTooSmall { needed, available } => {
                write!(
                    formatter,
                    "buffer needs {needed} bytes; only {available} available"
                )
            }
            Self::TruncatedPacket => formatter.write_str("truncated logstream packet"),
            Self::InvalidMagic => formatter.write_str("invalid logstream magic"),
            Self::UnsupportedVersion(version) => {
                write!(formatter, "unsupported logstream version {version}")
            }
            Self::InvalidHeaderLength(length) => {
                write!(formatter, "invalid logstream header length {length}")
            }
            Self::UnknownLane(value) => write!(formatter, "unknown logstream lane {value}"),
            Self::UnknownRecordKind(value) => write!(formatter, "unknown record kind {value}"),
            Self::UnknownFecScheme(value) => write!(formatter, "unknown FEC scheme {value}"),
            Self::UnknownSymbolKind(value) => write!(formatter, "unknown FEC symbol kind {value}"),
            Self::PayloadLengthMismatch => formatter.write_str("packet payload length mismatch"),
            Self::CrcMismatch => formatter.write_str("packet CRC32C mismatch"),
            Self::InvalidFecMetadata(message) => {
                write!(formatter, "invalid FEC metadata: {message}")
            }
            Self::InvalidFragment(message) => write!(formatter, "invalid fragment: {message}"),
            Self::InconsistentObject => formatter.write_str("packet belongs to a different object"),
            Self::TruncatedRecord => formatter.write_str("truncated semantic record"),
            Self::RecordLengthMismatch => formatter.write_str("semantic record length mismatch"),
            Self::RecordDigestMismatch => formatter.write_str("semantic record digest mismatch"),
            Self::Codec(message) => write!(formatter, "semantic codec failed: {message}"),
            Self::Transport(message) => write!(formatter, "datagram transport failed: {message}"),
        }
    }
}

impl From<cu_fec::Error> for Error {
    fn from(error: cu_fec::Error) -> Self {
        Self::Codec(error.to_string())
    }
}

impl From<cu_fec::WireError> for Error {
    fn from(error: cu_fec::WireError) -> Self {
        Self::Codec(error.to_string())
    }
}

#[cfg(feature = "std")]
impl std::error::Error for Error {}
