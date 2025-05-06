use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Header {
    pub stamp: Time,
    pub frame_id: std::string::String,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Time {
    pub sec: i32,
    pub nanosec: u32,
}
