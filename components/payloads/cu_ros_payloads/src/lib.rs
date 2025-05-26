pub mod builtin;
pub mod sensor_msgs;
pub mod std_msgs;

use serde::Serialize;
use std::convert::From;

// By default use Rust type as ROS type
#[macro_export]
macro_rules! ros_type_name {
    ($t:ty) => {{
        std::any::type_name::<$t>().rsplit("::").next().unwrap()
    }};
}

/// ROS adaptation trait to convert payload data to ROS compatible message.
/// The output type must match the structure of the related "msg" file.
/// The namespace relates to the ROS namespace (such as "std_msgs")
/// and the type name is the same as the message filename.
pub trait RosMsgAdapter<'a>: Sized {
    type Output: Serialize + for<'b> From<&'b Self>;

    fn namespace() -> &'a str;
    fn type_name() -> &'a str {
        ros_type_name!(Self::Output)
    }

    /// This hash is generated from an MD5 from the IDL.
    /// For example Int8 is "RIHS01_26525065a403d972cb672f0777e333f0c799ad444ae5fcd79e43d1e73bd0f440"
    fn type_hash() -> String;

    fn convert(&self) -> (Self::Output, String) {
        (self.into(), Self::type_hash())
    }
}
