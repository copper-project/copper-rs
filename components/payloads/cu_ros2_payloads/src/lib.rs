pub mod builtin;
mod fixed_array;
pub mod geometry_msgs;
pub mod nav_msgs;
pub mod sensor_msgs;
pub mod std_msgs;

#[cfg(all(feature = "humble", feature = "jazzy"))]
compile_error!("features `humble` and `jazzy` are mutually exclusive");

use core::fmt::Display;
use serde::Serialize;
use serde::de::DeserializeOwned;
use std::convert::From;

// By default use Rust type as ROS type
#[macro_export]
macro_rules! ros_type_name {
    ($t:ty) => {{ std::any::type_name::<$t>().rsplit("::").next().unwrap() }};
}

/// A ROS 2 message type, identified the way rmw needs it on the wire.
///
/// The three constants are exactly what a bridge puts in an rmw_zenoh key expression
/// (`{domain}/{topic}/{namespace}::msg::dds_::{TYPE_NAME}_/{TYPE_HASH}`), so implementing this
/// keeps the identity next to the struct instead of copied into every adapter that publishes it.
///
/// [`TYPE_HASH`](RosMessage::TYPE_HASH) is the RIHS01 hash of the **Jazzy** IDL. Humble predates
/// type hashes entirely; the blanket [`RosBridgeAdapter`] impl already substitutes
/// `"TypeHashNotSupported"` under the `humble` feature, so this constant needs no distro `cfg`.
///
/// Only messages with a hash verified against a ROS 2 Jazzy install implement this. Sub-messages
/// that are never published on their own (`Point`, `Pose`, `Transform`, `RegionOfInterest`, ...)
/// deliberately do not, rather than carry a guessed literal.
pub trait RosMessage {
    /// The ROS namespace, such as `"geometry_msgs"`.
    const NAMESPACE: &'static str;
    /// The message name, such as `"TransformStamped"`.
    const TYPE_NAME: &'static str;
    /// The RIHS01 type hash of the Jazzy IDL.
    const TYPE_HASH: &'static str;
}

/// ROS adaptation trait to convert payload data to ROS compatible message.
/// The output type must match the structure of the related "msg" file.
/// The namespace relates to the ROS namespace (such as "std_msgs")
/// and the type name is the same as the message filename.
pub trait RosMsgAdapter<'a>: Sized {
    type Output: Serialize + for<'b> From<&'b Self>;

    /// Validates that this payload can be represented by the selected ROS compatibility profile.
    ///
    /// Most adapters are always representable. Payloads with distro-specific restrictions can
    /// override this hook so bridges fail before conversion and serialization.
    fn validate_ros_message(&self) -> Result<(), String> {
        Ok(())
    }

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

    /// Validates that this payload can be represented by the selected ROS compatibility profile.
    fn validate_ros_message(&self) -> Result<(), String> {
        Ok(())
    }

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

    fn validate_ros_message(&self) -> Result<(), String> {
        <T as RosMsgAdapter<'static>>::validate_ros_message(self)
    }

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

#[cfg(test)]
mod tests {
    use super::RosMessage;

    /// Every [`RosMessage::TYPE_HASH`] must be a well-formed RIHS01 literal, and no two types may
    /// share one. A wrong or duplicated hash is invisible at runtime: it goes into the rmw_zenoh
    /// key expression, so the subscriber simply never matches and neither side reports an error.
    #[test]
    fn type_hashes_are_well_formed_and_distinct() {
        fn entry<T: RosMessage>() -> (&'static str, &'static str, &'static str) {
            (T::NAMESPACE, T::TYPE_NAME, T::TYPE_HASH)
        }

        let entries = [
            entry::<crate::geometry_msgs::PoseStamped>(),
            entry::<crate::geometry_msgs::TransformStamped>(),
            entry::<crate::geometry_msgs::Twist>(),
            entry::<crate::nav_msgs::Odometry>(),
            entry::<crate::nav_msgs::Path>(),
            entry::<crate::sensor_msgs::CameraInfo>(),
            entry::<crate::sensor_msgs::CompressedImage>(),
            entry::<crate::sensor_msgs::Image>(),
            entry::<crate::sensor_msgs::Imu>(),
            entry::<crate::sensor_msgs::MagneticField>(),
            entry::<crate::sensor_msgs::PointCloud2>(),
            entry::<crate::sensor_msgs::PointField>(),
        ];

        for (namespace, type_name, hash) in entries {
            assert!(!namespace.is_empty(), "{type_name} has no namespace");
            let digest = hash
                .strip_prefix("RIHS01_")
                .unwrap_or_else(|| panic!("{namespace}/{type_name} hash lacks the RIHS01 prefix"));
            assert_eq!(
                digest.len(),
                64,
                "{namespace}/{type_name} hash is not a SHA-256 digest"
            );
            assert!(
                digest
                    .chars()
                    .all(|c| c.is_ascii_hexdigit() && !c.is_ascii_uppercase()),
                "{namespace}/{type_name} hash is not lowercase hex"
            );
        }

        let mut hashes: Vec<&str> = entries.iter().map(|(_, _, hash)| *hash).collect();
        hashes.sort_unstable();
        let count = hashes.len();
        hashes.dedup();
        assert_eq!(count, hashes.len(), "two message types share a type hash");
    }
}
