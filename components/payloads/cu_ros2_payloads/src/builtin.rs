use compact_str::CompactString;
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct Header {
    pub stamp: Time,
    pub frame_id: CompactString,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct Time {
    pub sec: i32,
    pub nanosec: u32,
}
