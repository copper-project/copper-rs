//! `geometry_msgs` wire types.
//!
//! These mirror the ROS 2 `.msg` definitions field for field and in order: CDR is positional, so
//! a reordered or missing field decodes as garbage on the subscriber rather than as an error.
//!
//! [`Vector3`] and [`Quaternion`] are defined here, which is where ROS 2 puts them.
//! [`crate::sensor_msgs`] re-exports both so existing `sensor_msgs::Vector3` paths keep working.

use crate::builtin::Header;
use crate::{RosMessage, RosMsgAdapter, fixed_array};
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

impl RosMessage for Vector3 {
    const NAMESPACE: &'static str = "geometry_msgs";
    const TYPE_NAME: &'static str = "Vector3";
    const TYPE_HASH: &'static str =
        "RIHS01_cc12fe83e4c02719f1ce8070bfd14aecd40f75a96696a67a2a1f37f7dbb0765d";
}

impl RosMessage for Quaternion {
    const NAMESPACE: &'static str = "geometry_msgs";
    const TYPE_NAME: &'static str = "Quaternion";
    const TYPE_HASH: &'static str =
        "RIHS01_8a765f66778c8ff7c8ab94afcc590a2ed5325a1d9a076ffff38fbce36f458684";
}

impl RosMessage for Point {
    const NAMESPACE: &'static str = "geometry_msgs";
    const TYPE_NAME: &'static str = "Point";
    const TYPE_HASH: &'static str =
        "RIHS01_6963084842a9b04494d6b2941d11444708d892da2f4b09843b9c43f42a7f6881";
}

impl RosMessage for Pose {
    const NAMESPACE: &'static str = "geometry_msgs";
    const TYPE_NAME: &'static str = "Pose";
    const TYPE_HASH: &'static str =
        "RIHS01_d501954e9476cea2996984e812054b68026ae0bfae789d9a10b23daf35cc90fa";
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

impl RosMessage for Transform {
    const NAMESPACE: &'static str = "geometry_msgs";
    const TYPE_NAME: &'static str = "Transform";
    const TYPE_HASH: &'static str =
        "RIHS01_beb83fbe698636351461f6f35d1abb20010c43d55374d81bd041f1ba2581fddc";
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

impl RosMessage for PoseWithCovariance {
    const NAMESPACE: &'static str = "geometry_msgs";
    const TYPE_NAME: &'static str = "PoseWithCovariance";
    const TYPE_HASH: &'static str =
        "RIHS01_9a7c0fd234b7f45c6098745ecccd773ca1085670e64107135397aee31c02e1bb";
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

// ── cu_spatial_payloads conversions ───────────────────────────────────────────────────────────
//
// The crate's established shape, per the module docs on `RosMsgAdapter`:
//
//   - `From<&CopperPayload> for RosType`   outbound
//   - `TryFrom<RosType> for CopperPayload` inbound
//   - `RosMsgAdapter` on the payload       identity
//   - the blanket impl then supplies `RosBridgeAdapter` for the transport
//
// Stamped messages (`PoseStamped`, `TransformStamped`) are deliberately NOT bound here. None of
// these payloads carries a capture time or a frame id, so an adapter could only invent both, and
// a header invented at publish time is worse than no message: a zero stamp turns pipeline latency
// into apparent jitter for every consumer, and an empty `frame_id` silently builds no tf tree.
// Binding them needs a deliberate source for the two fields first.

use cu_spatial_payloads::{Point3d, Point3f, Transform3D};
use cu29::units::si::f32::Length as Length32;
use cu29::units::si::f64::Length as Length64;
use cu29::units::si::length::meter;

/// How far a rotation block may stray from orthonormality before it is rejected.
///
/// A rotation composed repeatedly drifts, so the check cannot demand exactness; these are set
/// well above the accumulated error of a long chain and well below any real scale factor.
const RIGID_TOLERANCE_F64: f64 = 1e-9;
const RIGID_TOLERANCE_F32: f64 = 1e-5;

/// Reject a rotation block that is not a pure rotation.
///
/// `geometry_msgs/Transform` is a translation plus a unit quaternion: it can represent a rigid
/// motion and nothing else. A `Transform3D` is a full 4x4 affine and may carry scale, shear or a
/// reflection, none of which survive the conversion. Dropping them silently would publish a
/// transform that looks plausible and places everything downstream wrongly, so this is an error
/// rather than a best-effort projection onto the nearest rotation.
fn ensure_rigid(rotation: [[f64; 3]; 3], tolerance: f64) -> Result<(), String> {
    // R^T R == I says the columns are orthonormal: no scale on any axis and no shear between them.
    for i in 0..3 {
        for j in i..3 {
            let dot: f64 = (0..3).map(|k| rotation[k][i] * rotation[k][j]).sum();
            let expected = if i == j { 1.0 } else { 0.0 };
            let deviation = (dot - expected).abs();
            if deviation > tolerance {
                return Err(format!(
                    "Transform3D is not rigid: R^T R deviates from the identity at ({i},{j}) by \
                     {deviation:.3e} (tolerance {tolerance:.1e}). geometry_msgs/Transform carries \
                     a unit quaternion and cannot represent scale or shear."
                ));
            }
        }
    }

    // An orthonormal matrix with determinant -1 is a reflection, which is also not a rotation.
    let determinant = rotation[0][0]
        * (rotation[1][1] * rotation[2][2] - rotation[1][2] * rotation[2][1])
        - rotation[0][1] * (rotation[1][0] * rotation[2][2] - rotation[1][2] * rotation[2][0])
        + rotation[0][2] * (rotation[1][0] * rotation[2][1] - rotation[1][1] * rotation[2][0]);
    if (determinant - 1.0).abs() > tolerance {
        return Err(format!(
            "Transform3D is not a rotation: determinant is {determinant:.6}, not 1. A determinant \
             of -1 is a reflection, which geometry_msgs/Transform cannot represent."
        ));
    }

    Ok(())
}

/// Shepperd's method: pick the branch whose divisor is largest so the square root never
/// approaches zero, which is where the naive trace-only formula loses all its precision.
fn quaternion_from_rotation(r: [[f64; 3]; 3]) -> Quaternion {
    let trace = r[0][0] + r[1][1] + r[2][2];
    if trace > 0.0 {
        let s = (trace + 1.0).sqrt() * 2.0;
        Quaternion {
            x: (r[2][1] - r[1][2]) / s,
            y: (r[0][2] - r[2][0]) / s,
            z: (r[1][0] - r[0][1]) / s,
            w: 0.25 * s,
        }
    } else if r[0][0] > r[1][1] && r[0][0] > r[2][2] {
        let s = (1.0 + r[0][0] - r[1][1] - r[2][2]).sqrt() * 2.0;
        Quaternion {
            x: 0.25 * s,
            y: (r[0][1] + r[1][0]) / s,
            z: (r[0][2] + r[2][0]) / s,
            w: (r[2][1] - r[1][2]) / s,
        }
    } else if r[1][1] > r[2][2] {
        let s = (1.0 + r[1][1] - r[0][0] - r[2][2]).sqrt() * 2.0;
        Quaternion {
            x: (r[0][1] + r[1][0]) / s,
            y: 0.25 * s,
            z: (r[1][2] + r[2][1]) / s,
            w: (r[0][2] - r[2][0]) / s,
        }
    } else {
        let s = (1.0 + r[2][2] - r[0][0] - r[1][1]).sqrt() * 2.0;
        Quaternion {
            x: (r[0][2] + r[2][0]) / s,
            y: (r[1][2] + r[2][1]) / s,
            z: 0.25 * s,
            w: (r[1][0] - r[0][1]) / s,
        }
    }
}

/// The inverse of [`quaternion_from_rotation`], for a quaternion already checked to be a unit.
fn rotation_from_quaternion(q: &Quaternion) -> [[f64; 3]; 3] {
    let (x, y, z, w) = (q.x, q.y, q.z, q.w);
    [
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
        ],
        [
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
        ],
        [
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        ],
    ]
}

/// A quaternion off the unit sphere is not a rotation, and normalizing one silently would hide a
/// sender's bug, so an out-of-tolerance norm is rejected.
fn ensure_unit_quaternion(q: &Quaternion) -> Result<(), String> {
    let norm_squared = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (norm_squared - 1.0).abs() > 1e-6 {
        return Err(format!(
            "geometry_msgs/Quaternion is not a unit quaternion: norm is {:.6}, not 1",
            norm_squared.sqrt()
        ));
    }
    Ok(())
}

/// Build the 4x4 the `Transform3D` constructor expects: `mat[row][column]`, translation in the
/// last column, bottom row `[0, 0, 0, 1]`.
fn homogeneous(rotation: [[f64; 3]; 3], translation: [f64; 3]) -> [[f64; 4]; 4] {
    let mut mat = [[0.0f64; 4]; 4];
    for row in 0..3 {
        mat[row][..3].copy_from_slice(&rotation[row]);
        mat[row][3] = translation[row];
    }
    mat[3][3] = 1.0;
    mat
}

/// `f64 -> f32` silently saturates to infinity, so a coordinate that does not fit is an error
/// rather than a point placed at infinity.
fn narrow_to_f32(value: f64, field: &str) -> Result<f32, String> {
    if !value.is_finite() {
        return Err(format!("{field} is not finite: {value}"));
    }
    let narrowed = value as f32;
    if !narrowed.is_finite() {
        return Err(format!("{field} = {value} overflows f32"));
    }
    Ok(narrowed)
}

/// The `f64` counterpart of [`narrow_to_f32`]: nothing to narrow, but a non-finite coordinate is
/// still not a position.
fn keep_f64(value: f64, field: &str) -> Result<f64, String> {
    if value.is_finite() {
        Ok(value)
    } else {
        Err(format!("{field} is not finite: {value}"))
    }
}

impl From<&Point3d> for Point {
    fn from(point: &Point3d) -> Self {
        Self {
            x: point.x.get::<meter>(),
            y: point.y.get::<meter>(),
            z: point.z.get::<meter>(),
        }
    }
}

impl TryFrom<Point> for Point3d {
    type Error = String;

    fn try_from(point: Point) -> Result<Self, Self::Error> {
        Ok(Self {
            x: Length64::new::<meter>(point.x),
            y: Length64::new::<meter>(point.y),
            z: Length64::new::<meter>(point.z),
        })
    }
}

impl RosMsgAdapter<'static> for Point3d {
    type Output = Point;

    fn namespace() -> &'static str {
        Point::NAMESPACE
    }

    fn type_name() -> &'static str {
        Point::TYPE_NAME
    }

    fn type_hash() -> &'static str {
        Point::TYPE_HASH
    }
}

impl From<&Point3f> for Point {
    fn from(point: &Point3f) -> Self {
        // Widening f32 to f64 is exact.
        Self {
            x: point.x.get::<meter>() as f64,
            y: point.y.get::<meter>() as f64,
            z: point.z.get::<meter>() as f64,
        }
    }
}

impl TryFrom<Point> for Point3f {
    type Error = String;

    fn try_from(point: Point) -> Result<Self, Self::Error> {
        Ok(Self {
            x: Length32::new::<meter>(narrow_to_f32(point.x, "geometry_msgs/Point.x")?),
            y: Length32::new::<meter>(narrow_to_f32(point.y, "geometry_msgs/Point.y")?),
            z: Length32::new::<meter>(narrow_to_f32(point.z, "geometry_msgs/Point.z")?),
        })
    }
}

impl RosMsgAdapter<'static> for Point3f {
    type Output = Point;

    fn namespace() -> &'static str {
        Point::NAMESPACE
    }

    fn type_name() -> &'static str {
        Point::TYPE_NAME
    }

    fn type_hash() -> &'static str {
        Point::TYPE_HASH
    }
}

/// `Transform3D` is a full 4x4 affine, so both ROS targets need the same rigidity guarantee and
/// the same quaternion extraction. Only the scalar precision differs.
macro_rules! impl_transform3d_conversions {
    ($scalar:ty, $tolerance:expr, $narrow:path) => {
        impl From<&Transform3D<$scalar>> for Transform {
            fn from(transform: &Transform3D<$scalar>) -> Self {
                let translation = transform.translation();
                let rotation = transform.rotation();
                let rotation: [[f64; 3]; 3] =
                    core::array::from_fn(|i| core::array::from_fn(|j| f64::from(rotation[i][j])));

                Self {
                    translation: Vector3 {
                        x: f64::from(translation[0].get::<meter>()),
                        y: f64::from(translation[1].get::<meter>()),
                        z: f64::from(translation[2].get::<meter>()),
                    },
                    rotation: quaternion_from_rotation(rotation),
                }
            }
        }

        #[allow(clippy::unnecessary_cast)]
        impl TryFrom<Transform> for Transform3D<$scalar> {
            type Error = String;

            fn try_from(transform: Transform) -> Result<Self, Self::Error> {
                ensure_unit_quaternion(&transform.rotation)?;
                let translation = [
                    $narrow(
                        transform.translation.x,
                        "geometry_msgs/Transform.translation.x",
                    )?,
                    $narrow(
                        transform.translation.y,
                        "geometry_msgs/Transform.translation.y",
                    )?,
                    $narrow(
                        transform.translation.z,
                        "geometry_msgs/Transform.translation.z",
                    )?,
                ];
                let mat = homogeneous(
                    rotation_from_quaternion(&transform.rotation),
                    [
                        f64::from(translation[0]),
                        f64::from(translation[1]),
                        f64::from(translation[2]),
                    ],
                );

                Ok(Self::from_matrix(core::array::from_fn(|i| {
                    core::array::from_fn(|j| mat[i][j] as $scalar)
                })))
            }
        }

        impl RosMsgAdapter<'static> for Transform3D<$scalar> {
            type Output = Transform;

            /// Refuse a transform that is not rigid.
            ///
            /// The bridge calls this before conversion, so a `Transform3D` carrying scale, shear
            /// or a reflection fails loudly here instead of publishing a quaternion that silently
            /// discarded it.
            fn validate_ros_message(&self) -> Result<(), String> {
                let rotation = self.rotation();
                ensure_rigid(
                    core::array::from_fn(|i| core::array::from_fn(|j| f64::from(rotation[i][j]))),
                    $tolerance,
                )
            }

            fn namespace() -> &'static str {
                Transform::NAMESPACE
            }

            fn type_name() -> &'static str {
                Transform::TYPE_NAME
            }

            fn type_hash() -> &'static str {
                Transform::TYPE_HASH
            }
        }

        // `Pose` carries the same information under a different name, and `cu_spatial_payloads`
        // itself aliases `Pose<T> = Transform3D<T>`. It cannot have its own `RosMsgAdapter`,
        // because `Output` is an associated type and the adapter above already spends it on
        // `Transform` — the type ROS names the same thing this payload does. These conversions
        // stay available for a graph that wants the pose spelling.
        impl From<&Transform3D<$scalar>> for Pose {
            fn from(transform: &Transform3D<$scalar>) -> Self {
                let transform: Transform = transform.into();
                Self {
                    position: Point {
                        x: transform.translation.x,
                        y: transform.translation.y,
                        z: transform.translation.z,
                    },
                    orientation: transform.rotation,
                }
            }
        }

        impl TryFrom<Pose> for Transform3D<$scalar> {
            type Error = String;

            fn try_from(pose: Pose) -> Result<Self, Self::Error> {
                Self::try_from(Transform {
                    translation: Vector3 {
                        x: pose.position.x,
                        y: pose.position.y,
                        z: pose.position.z,
                    },
                    rotation: pose.orientation,
                })
            }
        }
    };
}

impl_transform3d_conversions!(f64, RIGID_TOLERANCE_F64, keep_f64);
impl_transform3d_conversions!(f32, RIGID_TOLERANCE_F32, narrow_to_f32);

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

impl RosMessage for TwistWithCovariance {
    const NAMESPACE: &'static str = "geometry_msgs";
    const TYPE_NAME: &'static str = "TwistWithCovariance";
    const TYPE_HASH: &'static str =
        "RIHS01_49f574f033f095d8b6cd1beaca5ca7925e296e84af1716d16c89d38b059c8c18";
}

#[cfg(test)]
mod spatial_tests {
    use super::*;

    /// A quarter turn about z, translated by (1, 2, 3). `mat[row][column]`, translation in the
    /// last column.
    fn quarter_turn_about_z() -> [[f64; 4]; 4] {
        [
            [0.0, -1.0, 0.0, 1.0],
            [1.0, 0.0, 0.0, 2.0],
            [0.0, 0.0, 1.0, 3.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    }

    fn assert_close(actual: f64, expected: f64, what: &str) {
        assert!(
            (actual - expected).abs() < 1e-9,
            "{what}: expected {expected}, got {actual}"
        );
    }

    #[test]
    fn point3d_roundtrips_through_geometry_msgs_point() {
        let original = Point3d {
            x: Length64::new::<meter>(1.5),
            y: Length64::new::<meter>(-2.25),
            z: Length64::new::<meter>(0.0),
        };

        let ros: Point = (&original).into();
        assert_close(ros.x, 1.5, "x");
        assert_close(ros.y, -2.25, "y");

        let recovered = Point3d::try_from(ros).expect("finite point converts back");
        assert_close(recovered.x.get::<meter>(), 1.5, "recovered x");
        assert_close(recovered.y.get::<meter>(), -2.25, "recovered y");
    }

    #[test]
    fn point3f_rejects_a_coordinate_that_does_not_fit() {
        // f64 -> f32 saturates to infinity rather than failing, which would put the point at
        // infinity instead of reporting the problem.
        let error = Point3f::try_from(Point {
            x: 1e300,
            y: 0.0,
            z: 0.0,
        })
        .expect_err("1e300 does not fit in an f32");
        assert!(error.contains("overflows f32"), "unexpected error: {error}");

        let error = Point3f::try_from(Point {
            x: f64::NAN,
            y: 0.0,
            z: 0.0,
        })
        .expect_err("NaN is not a position");
        assert!(error.contains("not finite"), "unexpected error: {error}");
    }

    #[test]
    fn rigid_transform_roundtrips_through_geometry_msgs_transform() {
        let original = Transform3D::<f64>::from_matrix(quarter_turn_about_z());

        original
            .validate_ros_message()
            .expect("a quarter turn is rigid");

        let ros: Transform = (&original).into();
        assert_close(ros.translation.x, 1.0, "translation x");
        assert_close(ros.translation.y, 2.0, "translation y");
        assert_close(ros.translation.z, 3.0, "translation z");
        // A quarter turn about z is (0, 0, sin(45 deg), cos(45 deg)).
        assert_close(ros.rotation.z, std::f64::consts::FRAC_1_SQRT_2, "quat z");
        assert_close(ros.rotation.w, std::f64::consts::FRAC_1_SQRT_2, "quat w");

        let recovered = Transform3D::<f64>::try_from(ros).expect("unit quaternion converts back");
        let expected = quarter_turn_about_z();
        let actual = recovered.to_matrix();
        for row in 0..4 {
            for column in 0..4 {
                assert_close(
                    actual[row][column],
                    expected[row][column],
                    &format!("mat[{row}][{column}]"),
                );
            }
        }
    }

    #[test]
    fn scaled_transform_is_refused_rather_than_silently_flattened() {
        // Doubling x is representable as a Transform3D and NOT as a geometry_msgs/Transform.
        // Converting anyway would publish a plausible-looking rigid transform with the scale
        // quietly dropped, which nothing downstream could detect.
        let mut mat = quarter_turn_about_z();
        mat[0][0] = 0.0;
        mat[0][1] = -2.0;
        mat[1][0] = 2.0;

        let error = Transform3D::<f64>::from_matrix(mat)
            .validate_ros_message()
            .expect_err("a scaled transform is not rigid");
        assert!(
            error.contains("not rigid"),
            "expected a rigidity error, got: {error}"
        );
    }

    #[test]
    fn reflection_is_refused() {
        // Orthonormal, so R^T R == I passes, but the determinant is -1: a mirror, not a rotation.
        let mut mat = [[0.0f64; 4]; 4];
        mat[0][0] = -1.0;
        mat[1][1] = 1.0;
        mat[2][2] = 1.0;
        mat[3][3] = 1.0;

        let error = Transform3D::<f64>::from_matrix(mat)
            .validate_ros_message()
            .expect_err("a reflection is not a rotation");
        assert!(
            error.contains("determinant"),
            "expected a determinant error, got: {error}"
        );
    }

    #[test]
    fn non_unit_quaternion_is_refused_on_the_way_in() {
        let error = Transform3D::<f64>::try_from(Transform {
            translation: Vector3::default(),
            rotation: Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 2.0,
            },
        })
        .expect_err("a non-unit quaternion is not a rotation");
        assert!(
            error.contains("unit quaternion"),
            "unexpected error: {error}"
        );
    }

    #[test]
    fn pose_and_transform_spellings_agree() {
        let value = Transform3D::<f32>::from_matrix(core::array::from_fn(|i| {
            core::array::from_fn(|j| quarter_turn_about_z()[i][j] as f32)
        }));

        let as_transform: Transform = (&value).into();
        let as_pose: Pose = (&value).into();

        assert_close(as_pose.position.x, as_transform.translation.x, "position x");
        assert_close(as_pose.position.y, as_transform.translation.y, "position y");
        assert_close(as_pose.position.z, as_transform.translation.z, "position z");
        assert_eq!(as_pose.orientation, as_transform.rotation);

        let recovered = Transform3D::<f32>::try_from(as_pose).expect("pose converts back");
        assert!((recovered.translation()[0].get::<meter>() - 1.0).abs() < 1e-6);
    }

    #[test]
    fn adapters_publish_the_types_they_claim() {
        assert_eq!(<Point3d as RosMsgAdapter>::type_name(), "Point");
        assert_eq!(<Point3f as RosMsgAdapter>::namespace(), "geometry_msgs");
        assert_eq!(
            <Transform3D<f64> as RosMsgAdapter>::type_hash(),
            Transform::TYPE_HASH
        );
    }
}
