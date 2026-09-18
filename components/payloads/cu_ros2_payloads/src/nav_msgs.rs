//! `nav_msgs` wire types.

use crate::RosMessage;
use crate::builtin::{Header, Time};
use crate::geometry_msgs::{Pose, PoseStamped, PoseWithCovariance, TwistWithCovariance};
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

/// `nav_msgs/MapMetaData`, the `info` member of [`OccupancyGrid`].
///
/// `origin` is the pose of cell (0, 0)'s LOWER-LEFT CORNER in the grid's frame, not the centre of
/// the window — read as a centre, the whole map draws half a window off with nothing erroring.
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq)]
pub struct MapMetaData {
    pub map_load_time: Time,
    /// Metres per cell.
    pub resolution: f32,
    /// Cells across.
    pub width: u32,
    /// Cells down.
    pub height: u32,
    pub origin: Pose,
}

impl RosMessage for MapMetaData {
    const NAMESPACE: &'static str = "nav_msgs";
    const TYPE_NAME: &'static str = "MapMetaData";
    const TYPE_HASH: &'static str =
        "RIHS01_2772d4b2000ef2b35dbaeb80fd3946c1369f817fb4f75677d916d27c17d763c8";
}

/// `nav_msgs/OccupancyGrid`.
///
/// `data` is row-major from `info.origin`, one `int8` per cell: `0..=100` occupancy percent and
/// `-1` NEVER-OBSERVED. A consumer that clamps the sentinel to `0` plans through unmapped space.
///
/// `data` is deliberately NOT `serde_bytes`: that takes `u8` only, so an `int8[]` encodes element
/// by element rather than as the single octet sequence `CompressedImage` gets. The types differ in
/// signedness, not width, and a `u8` reinterpretation would turn every unknown cell (`-1`) into
/// `255` — decodable, and wrong in the direction that plans a robot through a wall.
#[derive(Clone, Debug, Default, Serialize, Deserialize, PartialEq)]
pub struct OccupancyGrid {
    pub header: Header,
    pub info: MapMetaData,
    pub data: Vec<i8>,
}

impl RosMessage for OccupancyGrid {
    const NAMESPACE: &'static str = "nav_msgs";
    const TYPE_NAME: &'static str = "OccupancyGrid";
    const TYPE_HASH: &'static str =
        "RIHS01_8d348150c12913a31ee0ec170fbf25089e4745d17035792a1ba94d6f0bc0cfc7";
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
    fn occupancy_grid_roundtrips() {
        let value = OccupancyGrid {
            header: sample_header("map"),
            info: MapMetaData {
                map_load_time: Time {
                    sec: 11,
                    nanosec: 12,
                },
                resolution: 0.05,
                width: 3,
                height: 2,
                origin: Pose {
                    position: Point {
                        x: -1.0,
                        y: -0.5,
                        z: 0.0,
                    },
                    orientation: Quaternion::default(),
                },
            },
            // Every class of cell: unknown, free, partial, full.
            data: vec![-1, 0, 50, 100, -1, 0],
        };

        let bytes =
            cdr::serialize::<_, _, cdr::CdrLe>(&value, cdr::Infinite).expect("cdr encode succeeds");
        let decoded: OccupancyGrid = cdr::deserialize(bytes.as_slice()).expect("cdr decode succeeds");
        assert_eq!(decoded, value);
    }

    /// `-1` is NEVER-OBSERVED and must survive the wire as `-1`.
    ///
    /// The failure this pins is not a decode error: reinterpreting `int8[]` as `u8` round-trips
    /// cleanly and silently turns every unknown cell into `255`, which a consumer clamping to
    /// `0..=100` reads as "certainly occupied" — or, clamped the other way, as free space that was
    /// never seen.
    #[test]
    fn the_unknown_cell_sentinel_survives_the_wire() {
        let value = OccupancyGrid {
            header: sample_header("map"),
            info: MapMetaData::default(),
            data: vec![-1; 8],
        };

        let bytes =
            cdr::serialize::<_, _, cdr::CdrLe>(&value, cdr::Infinite).expect("cdr encode succeeds");
        let decoded: OccupancyGrid = cdr::deserialize(bytes.as_slice()).expect("cdr decode succeeds");
        assert!(
            decoded.data.iter().all(|&c| c == -1),
            "unknown cells decoded as {:?}",
            decoded.data
        );
    }

    /// `data.len()` must equal `width * height`; nothing in the type enforces it, so a consumer
    /// indexing row-major walks off the end or renders a torn map.
    #[test]
    fn the_cell_count_is_the_products_of_the_dimensions() {
        let value = OccupancyGrid {
            header: sample_header("map"),
            info: MapMetaData {
                width: 4,
                height: 3,
                ..MapMetaData::default()
            },
            data: vec![0; 12],
        };
        assert_eq!(
            value.data.len(),
            value.info.width as usize * value.info.height as usize
        );
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
