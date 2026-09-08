//! `nav_msgs` wire types.

use crate::RosMessage;
use crate::builtin::Header;
use crate::geometry_msgs::{PoseStamped, PoseWithCovariance, TwistWithCovariance};
use compact_str::CompactString;
use serde::{Deserialize, Serialize};

/// `nav_msgs/Odometry`.
///
/// `header.frame_id` is the fixed frame the pose is expressed in and `child_frame_id` is the body
/// frame the twist is expressed in. They are different frames, and swapping them yields a message
/// that decodes fine and integrates wrongly.
#[derive(Clone, Debug, Default, Serialize, Deserialize, PartialEq)]
pub struct Odometry {
    pub header: Header,
    pub child_frame_id: CompactString,
    pub pose: PoseWithCovariance,
    pub twist: TwistWithCovariance,
}

impl RosMessage for Odometry {
    const NAMESPACE: &'static str = "nav_msgs";
    const TYPE_NAME: &'static str = "Odometry";
    const TYPE_HASH: &'static str =
        "RIHS01_3cc97dc7fb7502f8714462c526d369e35b603cfc34d946e3f2eda2766dfec6e0";
}

/// `nav_msgs/Path`.
#[derive(Clone, Debug, Default, Serialize, Deserialize, PartialEq)]
pub struct Path {
    pub header: Header,
    pub poses: Vec<PoseStamped>,
}

impl RosMessage for Path {
    const NAMESPACE: &'static str = "nav_msgs";
    const TYPE_NAME: &'static str = "Path";
    const TYPE_HASH: &'static str =
        "RIHS01_1957a5bb3cee5da65c4e52e52b65a93df227efce4c20f8458b36e73066ca334b";
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::builtin::Time;
    use crate::geometry_msgs::{Point, Pose, Quaternion, Twist, Vector3};

    fn sample_header(frame: &str) -> Header {
        Header {
            stamp: Time {
                sec: 42,
                nanosec: 7,
            },
            frame_id: frame.into(),
        }
    }

    #[test]
    fn odometry_roundtrips() {
        let value = Odometry {
            header: sample_header("odom"),
            child_frame_id: "base_link".into(),
            pose: PoseWithCovariance {
                pose: Pose {
                    position: Point {
                        x: 1.0,
                        y: 2.0,
                        z: 0.0,
                    },
                    orientation: Quaternion::default(),
                },
                covariance: core::array::from_fn(|i| i as f64),
            },
            twist: TwistWithCovariance {
                twist: Twist {
                    linear: Vector3 {
                        x: 0.3,
                        y: 0.0,
                        z: 0.0,
                    },
                    angular: Vector3 {
                        x: 0.0,
                        y: 0.0,
                        z: 0.1,
                    },
                },
                covariance: [0.5; 36],
            },
        };

        let bytes =
            cdr::serialize::<_, _, cdr::CdrLe>(&value, cdr::Infinite).expect("cdr encode succeeds");
        let decoded: Odometry = cdr::deserialize(bytes.as_slice()).expect("cdr decode succeeds");
        assert_eq!(decoded, value);
    }

    #[test]
    fn path_roundtrips_with_a_sequence_of_poses() {
        let value = Path {
            header: sample_header("map"),
            poses: vec![
                PoseStamped {
                    header: sample_header("map"),
                    pose: Pose::default(),
                },
                PoseStamped {
                    header: sample_header("map"),
                    pose: Pose {
                        position: Point {
                            x: 1.0,
                            y: 1.0,
                            z: 0.0,
                        },
                        orientation: Quaternion::default(),
                    },
                },
            ],
        };

        let bytes =
            cdr::serialize::<_, _, cdr::CdrLe>(&value, cdr::Infinite).expect("cdr encode succeeds");
        let decoded: Path = cdr::deserialize(bytes.as_slice()).expect("cdr decode succeeds");
        assert_eq!(decoded, value);
        assert_eq!(decoded.poses.len(), 2);
    }
}
