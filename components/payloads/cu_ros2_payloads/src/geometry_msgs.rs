//! `geometry_msgs` wire types.
//!
//! These mirror the ROS 2 `.msg` definitions field for field and in order: CDR is positional, so
//! a reordered or missing field decodes as garbage on the subscriber rather than as an error.
//!
//! [`Vector3`] and [`Quaternion`] are defined here, which is where ROS 2 puts them.
//! [`crate::sensor_msgs`] re-exports both so existing `sensor_msgs::Vector3` paths keep working.

use crate::builtin::Header;
use crate::{RosMessage, fixed_array};
use compact_str::CompactString;
use serde::{Deserialize, Serialize};

/// `geometry_msgs/Vector3`.
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq)]
pub struct Vector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

/// `geometry_msgs/Quaternion`.
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

impl Default for Quaternion {
    /// The identity rotation.
    ///
    /// Not all zeros: a zeroed quaternion has no norm and is not a rotation, so a `Default` that
    /// returned one would hand every consumer an invalid orientation.
    fn default() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        }
    }
}

/// `geometry_msgs/Point`.
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq)]
pub struct Point {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

/// `geometry_msgs/Pose`.
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq)]
pub struct Pose {
    pub position: Point,
    pub orientation: Quaternion,
}

/// `geometry_msgs/PoseStamped`.
#[derive(Clone, Debug, Default, Serialize, Deserialize, PartialEq)]
pub struct PoseStamped {
    pub header: Header,
    pub pose: Pose,
}

impl RosMessage for PoseStamped {
    const NAMESPACE: &'static str = "geometry_msgs";
    const TYPE_NAME: &'static str = "PoseStamped";
    const TYPE_HASH: &'static str =
        "RIHS01_10f3786d7d40fd2b54367835614bff85d4ad3b5dab62bf8bca0cc232d73b4cd8";
}

/// `geometry_msgs/Transform`.
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq)]
pub struct Transform {
    pub translation: Vector3,
    pub rotation: Quaternion,
}

/// `geometry_msgs/TransformStamped`.
///
/// The one message a tf2 listener consumes, so `child_frame_id` and `header.frame_id` are both
/// load-bearing: an empty pair publishes and decodes cleanly while building no tf tree at all.
#[derive(Clone, Debug, Default, Serialize, Deserialize, PartialEq)]
pub struct TransformStamped {
    pub header: Header,
    pub child_frame_id: CompactString,
    pub transform: Transform,
}

impl RosMessage for TransformStamped {
    const NAMESPACE: &'static str = "geometry_msgs";
    const TYPE_NAME: &'static str = "TransformStamped";
    const TYPE_HASH: &'static str =
        "RIHS01_0a241f87d04668d94099cbb5ba11691d5ad32c2f29682e4eb5653424bd275206";
}

/// `geometry_msgs/Twist`.
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq)]
pub struct Twist {
    pub linear: Vector3,
    pub angular: Vector3,
}

impl RosMessage for Twist {
    const NAMESPACE: &'static str = "geometry_msgs";
    const TYPE_NAME: &'static str = "Twist";
    const TYPE_HASH: &'static str =
        "RIHS01_9c45bf16fe0983d80e3cfe750d6835843d265a9a6c46bd2e609fcddde6fb8d2a";
}

/// `geometry_msgs/PoseWithCovariance`.
///
/// The covariance is a 6x6 row-major matrix stored as a FIXED 36-element array, so it carries no
/// length prefix on the wire — see [`crate::fixed_array`].
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq)]
pub struct PoseWithCovariance {
    pub pose: Pose,
    #[serde(with = "fixed_array")]
    pub covariance: [f64; 36],
}

impl Default for PoseWithCovariance {
    fn default() -> Self {
        Self {
            pose: Pose::default(),
            covariance: [0.0; 36],
        }
    }
}

/// `geometry_msgs/TwistWithCovariance`.
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq)]
pub struct TwistWithCovariance {
    pub twist: Twist,
    #[serde(with = "fixed_array")]
    pub covariance: [f64; 36],
}

impl Default for TwistWithCovariance {
    fn default() -> Self {
        Self {
            twist: Twist::default(),
            covariance: [0.0; 36],
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::builtin::Time;

    fn sample_header() -> Header {
        Header {
            stamp: Time {
                sec: 1_700_000_000,
                nanosec: 123_456_789,
            },
            frame_id: "odom".into(),
        }
    }

    fn roundtrip<T>(value: &T) -> (T, Vec<u8>)
    where
        T: Serialize + for<'de> Deserialize<'de>,
    {
        let bytes =
            cdr::serialize::<_, _, cdr::CdrLe>(value, cdr::Infinite).expect("cdr encode succeeds");
        let decoded = cdr::deserialize(bytes.as_slice()).expect("cdr decode succeeds");
        (decoded, bytes)
    }

    #[test]
    fn transform_stamped_roundtrips() {
        let value = TransformStamped {
            header: sample_header(),
            child_frame_id: "base_link".into(),
            transform: Transform {
                translation: Vector3 {
                    x: 1.0,
                    y: 2.0,
                    z: 3.0,
                },
                rotation: Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 0.707,
                    w: 0.707,
                },
            },
        };

        let (decoded, _) = roundtrip(&value);
        assert_eq!(decoded, value);
    }

    #[test]
    fn pose_stamped_roundtrips() {
        let value = PoseStamped {
            header: sample_header(),
            pose: Pose {
                position: Point {
                    x: -1.5,
                    y: 0.25,
                    z: 9.0,
                },
                orientation: Quaternion::default(),
            },
        };

        let (decoded, _) = roundtrip(&value);
        assert_eq!(decoded, value);
    }

    #[test]
    fn twist_roundtrips_as_six_float64() {
        let value = Twist {
            linear: Vector3 {
                x: 0.5,
                y: 0.0,
                z: 0.0,
            },
            angular: Vector3 {
                x: 0.0,
                y: 0.0,
                z: -0.25,
            },
        };

        let (decoded, bytes) = roundtrip(&value);
        assert_eq!(decoded, value);
        // 4-byte CDR encapsulation header then six f64, no padding: every member is 8-aligned and
        // the first one sits at body offset 0.
        assert_eq!(bytes.len(), 4 + 6 * 8);
    }

    #[test]
    fn pose_with_covariance_writes_no_length_prefix() {
        let value = PoseWithCovariance {
            pose: Pose::default(),
            covariance: core::array::from_fn(|i| i as f64),
        };

        let (decoded, bytes) = roundtrip(&value);
        assert_eq!(decoded, value);
        // Pose is 7 f64 and the covariance is a FIXED 36-element array. A `Vec` would add a
        // uint32 length here and shift the matrix, which is the bug this asserts against.
        assert_eq!(bytes.len(), 4 + 7 * 8 + 36 * 8);
    }

    #[test]
    fn twist_with_covariance_writes_no_length_prefix() {
        let value = TwistWithCovariance {
            twist: Twist::default(),
            covariance: [1.0; 36],
        };

        let (decoded, bytes) = roundtrip(&value);
        assert_eq!(decoded, value);
        assert_eq!(bytes.len(), 4 + 6 * 8 + 36 * 8);
    }

    #[test]
    fn default_quaternion_is_the_identity_rotation() {
        // A zeroed quaternion is not a rotation; every consumer normalizing it would divide by
        // zero, so the default must be the identity.
        assert_eq!(
            Quaternion::default(),
            Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0
            }
        );
    }
}
