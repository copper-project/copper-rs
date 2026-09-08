use compact_str::CompactString;
use serde::{Deserialize, Serialize};

/// `std_msgs/Header`.
///
/// `Default` is the ROS default: a zero stamp and an empty `frame_id`. Both are valid on the wire
/// and neither is useful — a zero stamp turns pipeline latency into apparent jitter downstream,
/// and an empty frame builds no tf tree — so fill them from the payload's own capture time and
/// frame rather than defaulting.
#[derive(Clone, Debug, Default, Serialize, Deserialize, PartialEq)]
pub struct Header {
    pub stamp: Time,
    pub frame_id: CompactString,
}

/// `builtin_interfaces/Time`.
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq)]
pub struct Time {
    pub sec: i32,
    pub nanosec: u32,
}
