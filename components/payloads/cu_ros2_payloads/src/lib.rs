pub mod builtin;
pub mod sensor_msgs;
pub mod std_msgs;

use core::fmt::Display;
use serde::Serialize;
use serde::de::DeserializeOwned;
use std::convert::From;

// By default use Rust type as ROS type
#[macro_export]
macro_rules! ros_type_name {
    ($t:ty) => {{ std::any::type_name::<$t>().rsplit("::").next().unwrap() }};
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
}

/// Bidirectional ROS adaptation trait used by transport bridges.
///
/// Implement this for Copper payloads that can be encoded to and decoded from a ROS message
/// representation.
pub trait RosBridgeAdapter: Sized + 'static {
    type RosMessage: Serialize + DeserializeOwned + 'static;

    fn namespace() -> &'static str;

    fn type_name() -> &'static str {
        ros_type_name!(Self::RosMessage)
    }

    fn type_hash() -> &'static str;

    fn to_ros_message(&self) -> Self::RosMessage;

    fn from_ros_message(msg: Self::RosMessage) -> Result<Self, String>;
}

impl<T> RosBridgeAdapter for T
where
    T: RosMsgAdapter<'static> + TryFrom<<T as RosMsgAdapter<'static>>::Output> + 'static,
    T::Output: Serialize + DeserializeOwned + 'static,
    <T as TryFrom<<T as RosMsgAdapter<'static>>::Output>>::Error: Display,
{
    type RosMessage = <T as RosMsgAdapter<'static>>::Output;

    fn namespace() -> &'static str {
        <T as RosMsgAdapter<'static>>::namespace()
    }

    fn type_name() -> &'static str {
        <T as RosMsgAdapter<'static>>::type_name()
    }

    #[cfg(not(feature = "humble"))]
    fn type_hash() -> &'static str {
        <T as RosMsgAdapter<'static>>::type_hash()
    }

    #[cfg(feature = "humble")]
    fn type_hash() -> &'static str {
        "TypeHashNotSupported"
    }

    fn to_ros_message(&self) -> Self::RosMessage {
        self.into()
    }

    fn from_ros_message(msg: Self::RosMessage) -> Result<Self, String> {
        T::try_from(msg).map_err(|e| e.to_string())
    }
}
