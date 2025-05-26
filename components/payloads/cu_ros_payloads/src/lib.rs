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

    /// The namespace of the ROS message, such as "std_msgs" or "sensor_msgs".
    fn namespace() -> &'a str;

    /// The type name of the ROS message, such as "Int8" or "PointCloud2".
    fn type_name() -> &'a str {
        ros_type_name!(Self::Output)
    }

    /// This hash is generated from an SHA256 from the IDL.
    /// It is obscure.
    /// For example Int8 is "RIHS01_26525065a403d972cb672f0777e333f0c799ad444ae5fcd79e43d1e73bd0f440"
    /// This will only be used in ROS 2 Iron and later versions.
    fn type_hash() -> &'static str;

    /// Converts the current Copper type into the corresponding ROS message type.
    ///
    /// # Returns
    /// A tuple containing:
    /// - The converted ROS message type (`Self::Output`).
    /// - The type hash as a string, which is used to identify the message type in ROS.
    #[cfg(not(feature = "humble"))]
    fn convert(&self) -> (Self::Output, &'static str) {
        (self.into(), Self::type_hash())
    }

    #[cfg(feature = "humble")]
    fn convert(&self) -> (Self::Output, &str) {
        (self.into(), "TypeHashNotSupported")
    }
}
