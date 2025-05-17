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
pub trait RosMsgAdapter<'a>: std::marker::Sized {
    type Output: Serialize + for<'b> From<&'b Self>;

    fn namespace() -> &'a str;
    fn type_name() -> &'a str {
        ros_type_name!(Self::Output)
    }
    fn convert(&self) -> Self::Output {
        self.into()
    }
}
