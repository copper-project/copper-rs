pub mod builtin;
pub mod sensor_msgs;
pub mod std_msgs;

use serde::Serialize;

/// ROS adaptation trait to convert payload data to ROS compatible message.
/// The output type must match the structure of the related "msg" file.
/// The namespace relates to the ROS namespace (such as "std_msgs")
/// and the type name is the same as the message filename.
pub trait RosMsgAdapter {
    type Output: Serialize;

    fn namespace() -> &'static str;
    fn type_name() -> &'static str;
    fn convert(&self) -> Self::Output;
}
