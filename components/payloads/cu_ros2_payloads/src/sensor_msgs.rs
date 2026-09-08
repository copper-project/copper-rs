use compact_str::CompactString;
use cu_sensor_payloads::{
    CuImage, CuImageBufferFormat, ImuPayload, MagnetometerPayload, PointCloud, PointCloudSoa,
    PointCloudSoaHandle,
};
use cu29::prelude::CuHandle;
use cu29::units::si::acceleration::meter_per_second_squared;
use cu29::units::si::angular_velocity::radian_per_second;
use cu29::units::si::length::meter;
use cu29::units::si::magnetic_flux_density::microtesla;
use cu29::units::si::ratio::percent;
use serde::{Deserialize, Serialize};

use crate::{RosMessage, RosMsgAdapter, builtin::Header};

const DATATYPE_UINT32: u8 = 6;
const DATATYPE_FLOAT32: u8 = 7;

const UT_TO_TESLA: f64 = 1e-6;
const TESLA_TO_UT: f64 = 1e6;

#[cfg(feature = "humble")]
const ROS2_IMAGE_COMPAT_DISTRO: &str = "Humble";
#[cfg(all(not(feature = "humble"), feature = "jazzy"))]
const ROS2_IMAGE_COMPAT_DISTRO: &str = "Jazzy";

// sensor_msgs/PointField
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct PointField {
    pub name: CompactString,
    pub offset: u32,
    pub datatype: u8,
    pub count: u32,
}

// sensor_msgs/PointCloud2
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct PointCloud2 {
    pub header: Header,
    pub height: u32,
    pub width: u32,
    pub fields: Vec<PointField>,
    pub is_bigendian: bool,
    pub point_step: u32,
    pub row_step: u32,
    #[serde(with = "serde_bytes")]
    pub data: Vec<u8>,
    pub is_dense: bool,
}

// sensor_msgs/Image
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct Image {
    pub header: Header,
    pub height: u32,
    pub width: u32,
    pub encoding: String,
    pub is_bigendian: u8,
    pub step: u32,
    #[serde(with = "serde_bytes")]
    pub data: Vec<u8>,
}

// Both live in `geometry_msgs` in ROS 2, and that is where they are defined now. Re-exported here
// because `sensor_msgs::Imu` is built from them and because these paths were public API.
pub use crate::geometry_msgs::{Quaternion, Vector3};

// sensor_msgs/Imu
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct Imu {
    pub header: Header,
    pub orientation: Quaternion,
    pub orientation_covariance: [f64; 9],
    pub angular_velocity: Vector3,
    pub angular_velocity_covariance: [f64; 9],
    pub linear_acceleration: Vector3,
    pub linear_acceleration_covariance: [f64; 9],
}

// sensor_msgs/MagneticField
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct MagneticField {
    pub header: Header,
    pub magnetic_field: Vector3,
    pub magnetic_field_covariance: [f64; 9],
}

// sensor_msgs/Temperature
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct Temperature {
    pub header: Header,
    pub temperature: f64,
    pub variance: f64,
}

// sensor_msgs/CompressedImage
//
// Distinct from `Image`, and the difference is the point: this carries the encoder's own bytes
// (`format` names them, e.g. "jpeg", "png", "h264"), so a hardware-encoded stream can be
// published without being expanded and re-encoded first.
#[derive(Clone, Debug, Default, Serialize, Deserialize, PartialEq)]
pub struct CompressedImage {
    pub header: Header,
    pub format: String,
    #[serde(with = "serde_bytes")]
    pub data: Vec<u8>,
}

impl RosMessage for CompressedImage {
    const NAMESPACE: &'static str = "sensor_msgs";
    const TYPE_NAME: &'static str = "CompressedImage";
    const TYPE_HASH: &'static str =
        "RIHS01_15640771531571185e2efc8a100baf923961a4d15d5569652e6cb6691e8e371a";
}

// sensor_msgs/RegionOfInterest
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq)]
pub struct RegionOfInterest {
    pub x_offset: u32,
    pub y_offset: u32,
    pub height: u32,
    pub width: u32,
    pub do_rectify: bool,
}

// sensor_msgs/CameraInfo
//
// `d` is a variable-length sequence (its length depends on `distortion_model`), while `k`, `r`
// and `p` are fixed 3x3, 3x3 and 3x4 row-major matrices. That distinction is on the wire: only
// `d` carries a length prefix.
#[derive(Clone, Debug, Default, Serialize, Deserialize, PartialEq)]
pub struct CameraInfo {
    pub header: Header,
    pub height: u32,
    pub width: u32,
    pub distortion_model: String,
    pub d: Vec<f64>,
    pub k: [f64; 9],
    pub r: [f64; 9],
    pub p: [f64; 12],
    pub binning_x: u32,
    pub binning_y: u32,
    pub roi: RegionOfInterest,
}

impl RosMessage for CameraInfo {
    const NAMESPACE: &'static str = "sensor_msgs";
    const TYPE_NAME: &'static str = "CameraInfo";
    const TYPE_HASH: &'static str =
        "RIHS01_b3dfd68ff46c9d56c80fd3bd4ed22c7a4ddce8c8348f2f59c299e73118e7e275";
}

impl RosMessage for RegionOfInterest {
    const NAMESPACE: &'static str = "sensor_msgs";
    const TYPE_NAME: &'static str = "RegionOfInterest";
    const TYPE_HASH: &'static str =
        "RIHS01_ad16bcba5f9131dcdba6fbded19f726f5440e3c513b4fb586dd3027eeed8abb1";
}

// The hashes the existing adapters already publish, kept next to the types they identify.
impl RosMessage for PointField {
    const NAMESPACE: &'static str = "sensor_msgs";
    const TYPE_NAME: &'static str = "PointField";
    const TYPE_HASH: &'static str =
        "RIHS01_5c6a4750728c2bcfbbf7037225b20b02d4429634732146b742dee1726637ef01";
}

impl RosMessage for PointCloud2 {
    const NAMESPACE: &'static str = "sensor_msgs";
    const TYPE_NAME: &'static str = "PointCloud2";
    const TYPE_HASH: &'static str =
        "RIHS01_9198cabf7da3796ae6fe19c4cb3bdd3525492988c70522628af5daa124bae2b5";
}

impl RosMessage for Image {
    const NAMESPACE: &'static str = "sensor_msgs";
    const TYPE_NAME: &'static str = "Image";
    const TYPE_HASH: &'static str =
        "RIHS01_d31d41a9a4c4bc8eae9be757b0beed306564f7526c88ea6a4588fb9582527d47";
}

impl RosMessage for Imu {
    const NAMESPACE: &'static str = "sensor_msgs";
    const TYPE_NAME: &'static str = "Imu";
    const TYPE_HASH: &'static str =
        "RIHS01_7d9a00ff131080897a5ec7e26e315954b8eae3353c3f995c55faf71574000b5b";
}

impl RosMessage for MagneticField {
    const NAMESPACE: &'static str = "sensor_msgs";
    const TYPE_NAME: &'static str = "MagneticField";
    const TYPE_HASH: &'static str =
        "RIHS01_e80f32f56a20486c9923008fc1a1db07bbb273cbbf6a5b3bfa00835ee00e4dff";
}

// sensor_msgs/JointState
//
// The arrays are parallel to `name` and each one may be empty, which the message defines as "not
// reported" rather than "zero": a robot that publishes positions but measures no effort sends an
// empty `effort`, not a run of zeros. Anything else must have the same length as `name`, since
// that is the only thing associating a value with a joint.
//
// `effort` is newtons or newton-metres. A servo that reports a unitless load or PWM duty has not
// measured effort, and putting that number here is indistinguishable downstream from a real
// torque measurement.
#[derive(Clone, Debug, Default, Serialize, Deserialize, PartialEq)]
pub struct JointState {
    pub header: Header,
    pub name: Vec<CompactString>,
    pub position: Vec<f64>,
    pub velocity: Vec<f64>,
    pub effort: Vec<f64>,
}

impl RosMessage for JointState {
    const NAMESPACE: &'static str = "sensor_msgs";
    const TYPE_NAME: &'static str = "JointState";
    const TYPE_HASH: &'static str =
        "RIHS01_a13ee3a330e346c9d87b5aa18d24e11690752bd33a0350f11c5882bc9179260e";
}

// sensor_msgs/BatteryState
//
// Every field after `voltage` documents NaN as its "unmeasured" value, so a driver that reads
// only a bus voltage fills the rest with NaN. Zero is a measurement — "flat battery", "0 degrees
// C", "0% charge" — and no consumer can tell an invented zero from a real one. For the same
// reason this derives no `Default`: a zeroed BatteryState is a lie in seven fields at once, and
// the three `power_supply_*` enums have an explicit `_UNKNOWN = 0` precisely so that unknown is
// stated rather than defaulted into.
//
// The wire layout is worth stating because it is easy to "tidy" wrongly: the three enums are
// bytes and `present` is one byte (CDR has no packed bools), so the four of them occupy four
// consecutive bytes between `percentage` and `cell_voltage`. Reordering any two same-width
// fields here still round-trips through this struct, still passes the type hash, and decodes
// into the wrong field on every ROS consumer.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct BatteryState {
    pub header: Header,
    /// Volts.
    pub voltage: f32,
    /// Degrees Celsius; NaN if unmeasured.
    pub temperature: f32,
    /// Amperes, negative when discharging; NaN if unmeasured.
    pub current: f32,
    /// Current charge in Ah; NaN if unmeasured.
    pub charge: f32,
    /// Last full capacity in Ah; NaN if unmeasured.
    pub capacity: f32,
    /// Design capacity in Ah; NaN if unmeasured.
    pub design_capacity: f32,
    /// Charge fraction on a 0 to 1 range; NaN if unmeasured.
    pub percentage: f32,
    /// One of the `POWER_SUPPLY_STATUS_*` constants.
    pub power_supply_status: u8,
    /// One of the `POWER_SUPPLY_HEALTH_*` constants.
    pub power_supply_health: u8,
    /// One of the `POWER_SUPPLY_TECHNOLOGY_*` constants.
    pub power_supply_technology: u8,
    /// True if the battery is present.
    pub present: bool,
    /// Per-cell voltages; NaN per cell if the count is known but the voltages are not.
    pub cell_voltage: Vec<f32>,
    /// Per-cell temperatures, same convention as `cell_voltage`.
    pub cell_temperature: Vec<f32>,
    /// Where the battery is inserted (slot number or plug).
    pub location: CompactString,
    pub serial_number: CompactString,
}

/// The enumerations `sensor_msgs/BatteryState` defines for its three `u8` fields.
///
/// They are part of the message definition, not a convenience: the fields are bare `u8`s, so
/// without them every publisher writes a magic number and every consumer compares against one.
impl BatteryState {
    pub const POWER_SUPPLY_STATUS_UNKNOWN: u8 = 0;
    pub const POWER_SUPPLY_STATUS_CHARGING: u8 = 1;
    pub const POWER_SUPPLY_STATUS_DISCHARGING: u8 = 2;
    pub const POWER_SUPPLY_STATUS_NOT_CHARGING: u8 = 3;
    pub const POWER_SUPPLY_STATUS_FULL: u8 = 4;

    pub const POWER_SUPPLY_HEALTH_UNKNOWN: u8 = 0;
    pub const POWER_SUPPLY_HEALTH_GOOD: u8 = 1;
    pub const POWER_SUPPLY_HEALTH_OVERHEAT: u8 = 2;
    pub const POWER_SUPPLY_HEALTH_DEAD: u8 = 3;
    pub const POWER_SUPPLY_HEALTH_OVERVOLTAGE: u8 = 4;
    pub const POWER_SUPPLY_HEALTH_UNSPEC_FAILURE: u8 = 5;
    pub const POWER_SUPPLY_HEALTH_COLD: u8 = 6;
    pub const POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE: u8 = 7;
    pub const POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE: u8 = 8;

    pub const POWER_SUPPLY_TECHNOLOGY_UNKNOWN: u8 = 0;
    pub const POWER_SUPPLY_TECHNOLOGY_NIMH: u8 = 1;
    pub const POWER_SUPPLY_TECHNOLOGY_LION: u8 = 2;
    pub const POWER_SUPPLY_TECHNOLOGY_LIPO: u8 = 3;
    pub const POWER_SUPPLY_TECHNOLOGY_LIFE: u8 = 4;
    pub const POWER_SUPPLY_TECHNOLOGY_NICD: u8 = 5;
    pub const POWER_SUPPLY_TECHNOLOGY_LIMN: u8 = 6;
    pub const POWER_SUPPLY_TECHNOLOGY_TERNARY: u8 = 7;
    pub const POWER_SUPPLY_TECHNOLOGY_VRLA: u8 = 8;
}

impl RosMessage for BatteryState {
    const NAMESPACE: &'static str = "sensor_msgs";
    const TYPE_NAME: &'static str = "BatteryState";
    const TYPE_HASH: &'static str =
        "RIHS01_4bee5dfce981c98faa6828b868307a0a73f992ed0789f374ee96c8f840e69741";
}

impl<const N: usize> RosMsgAdapter<'static> for PointCloudSoa<N> {
    type Output = PointCloud2;

    fn namespace() -> &'static str {
        "sensor_msgs"
    }

    fn type_name() -> &'static str {
        PointCloud2::TYPE_NAME
    }

    fn type_hash() -> &'static str {
        PointCloud2::TYPE_HASH
    }
}

impl<const N: usize> RosMsgAdapter<'static> for PointCloudSoaHandle<N> {
    type Output = PointCloud2;

    fn namespace() -> &'static str {
        "sensor_msgs"
    }

    fn type_name() -> &'static str {
        PointCloud2::TYPE_NAME
    }

    fn type_hash() -> &'static str {
        PointCloud2::TYPE_HASH
    }
}

impl RosMsgAdapter<'static> for CuImage<Vec<u8>> {
    type Output = Image;

    #[cfg(any(feature = "humble", feature = "jazzy"))]
    fn validate_ros_message(&self) -> Result<(), String> {
        match &self.format.pixel_format {
            b"GRAY" | b"Y800" | b"RGB3" | b"RGB " | b"BGR3" | b"BGR " | b"RGBA" | b"BGRA"
            | b"YUYV" | b"UYVY" => Ok(()),
            pixel_format => Err(format!(
                "CuImage pixel format '{}' is not supported by ROS 2 {} cv_bridge",
                String::from_utf8_lossy(pixel_format),
                ROS2_IMAGE_COMPAT_DISTRO
            )),
        }
    }

    fn namespace() -> &'static str {
        "sensor_msgs"
    }

    fn type_name() -> &'static str {
        Image::TYPE_NAME
    }

    fn type_hash() -> &'static str {
        Image::TYPE_HASH
    }
}

impl RosMsgAdapter<'static> for ImuPayload {
    type Output = Imu;

    fn namespace() -> &'static str {
        "sensor_msgs"
    }

    fn type_name() -> &'static str {
        Imu::TYPE_NAME
    }

    fn type_hash() -> &'static str {
        Imu::TYPE_HASH
    }
}

impl RosMsgAdapter<'static> for MagnetometerPayload {
    type Output = MagneticField;

    fn namespace() -> &'static str {
        "sensor_msgs"
    }

    fn type_name() -> &'static str {
        MagneticField::TYPE_NAME
    }

    fn type_hash() -> &'static str {
        MagneticField::TYPE_HASH
    }
}

impl<const N: usize> From<&PointCloudSoa<N>> for PointCloud2 {
    fn from(pointcloud: &PointCloudSoa<N>) -> Self {
        let len = pointcloud.len;

        let fields = vec![
            PointField {
                name: "x".into(),
                offset: 0,
                datatype: DATATYPE_FLOAT32,
                count: 1,
            },
            PointField {
                name: "y".into(),
                offset: 4,
                datatype: DATATYPE_FLOAT32,
                count: 1,
            },
            PointField {
                name: "z".into(),
                offset: 8,
                datatype: DATATYPE_FLOAT32,
                count: 1,
            },
            PointField {
                name: "intensity".into(),
                offset: 12,
                datatype: DATATYPE_FLOAT32,
                count: 1,
            },
            PointField {
                name: "tov_sec".into(),
                offset: 16,
                datatype: DATATYPE_UINT32,
                count: 1,
            },
            PointField {
                name: "tov_nsec".into(),
                offset: 20,
                datatype: DATATYPE_UINT32,
                count: 1,
            },
        ];

        let point_step = 24u32;
        let width = len as u32;
        let row_step = point_step * width;
        let mut data = Vec::with_capacity(row_step as usize);

        for idx in 0..len {
            let x = pointcloud.x[idx].get::<meter>();
            let y = pointcloud.y[idx].get::<meter>();
            let z = pointcloud.z[idx].get::<meter>();
            let intensity = pointcloud.i[idx].get::<percent>();
            let tov_nanos = pointcloud.tov[idx].as_nanos();
            let tov_sec = (tov_nanos / 1_000_000_000) as u32;
            let tov_nsec = (tov_nanos % 1_000_000_000) as u32;

            data.extend_from_slice(&x.to_le_bytes());
            data.extend_from_slice(&y.to_le_bytes());
            data.extend_from_slice(&z.to_le_bytes());
            data.extend_from_slice(&intensity.to_le_bytes());
            data.extend_from_slice(&tov_sec.to_le_bytes());
            data.extend_from_slice(&tov_nsec.to_le_bytes());
        }

        PointCloud2 {
            header: default_header(),
            height: 1,
            width,
            fields,
            is_bigendian: false,
            point_step,
            row_step,
            data,
            is_dense: true,
        }
    }
}

impl<const N: usize> From<&PointCloudSoaHandle<N>> for PointCloud2 {
    fn from(pointcloud: &PointCloudSoaHandle<N>) -> Self {
        pointcloud.with_inner(|inner| PointCloud2::from(inner))
    }
}

impl From<&CuImage<Vec<u8>>> for Image {
    fn from(image: &CuImage<Vec<u8>>) -> Self {
        let data = image.buffer_handle.with_inner(|inner| inner.to_vec());
        Self {
            header: default_header(),
            height: image.format.height,
            width: image.format.width,
            encoding: pixel_format_to_encoding(image.format.pixel_format),
            is_bigendian: 0,
            step: image.format.stride,
            data,
        }
    }
}

impl From<&ImuPayload> for Imu {
    fn from(value: &ImuPayload) -> Self {
        let mut orientation_covariance = [0.0_f64; 9];
        orientation_covariance[0] = -1.0;

        Self {
            header: default_header(),
            orientation: Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0,
            },
            orientation_covariance,
            angular_velocity: Vector3 {
                x: value.gyro_x.get::<radian_per_second>() as f64,
                y: value.gyro_y.get::<radian_per_second>() as f64,
                z: value.gyro_z.get::<radian_per_second>() as f64,
            },
            angular_velocity_covariance: [0.0; 9],
            linear_acceleration: Vector3 {
                x: value.accel_x.get::<meter_per_second_squared>() as f64,
                y: value.accel_y.get::<meter_per_second_squared>() as f64,
                z: value.accel_z.get::<meter_per_second_squared>() as f64,
            },
            linear_acceleration_covariance: [0.0; 9],
        }
    }
}

impl From<&MagnetometerPayload> for MagneticField {
    fn from(value: &MagnetometerPayload) -> Self {
        let x_t = (value.mag_x.get::<microtesla>() as f64) * UT_TO_TESLA;
        let y_t = (value.mag_y.get::<microtesla>() as f64) * UT_TO_TESLA;
        let z_t = (value.mag_z.get::<microtesla>() as f64) * UT_TO_TESLA;

        Self {
            header: default_header(),
            magnetic_field: Vector3 {
                x: x_t,
                y: y_t,
                z: z_t,
            },
            magnetic_field_covariance: [0.0; 9],
        }
    }
}

impl<const N: usize> TryFrom<PointCloud2> for PointCloudSoa<N> {
    type Error = String;

    fn try_from(value: PointCloud2) -> Result<Self, Self::Error> {
        let mut out = PointCloudSoa::<N>::default();
        decode_pointcloud2_into_soa(&mut out, &value)?;
        Ok(out)
    }
}

impl<const N: usize> TryFrom<PointCloud2> for PointCloudSoaHandle<N> {
    type Error = String;

    fn try_from(value: PointCloud2) -> Result<Self, Self::Error> {
        let mut out = PointCloudSoaHandle::<N>::default();
        out.with_inner_mut(|inner| decode_pointcloud2_into_soa(inner, &value))?;
        Ok(out)
    }
}

impl TryFrom<Image> for CuImage<Vec<u8>> {
    type Error = String;

    fn try_from(value: Image) -> Result<Self, Self::Error> {
        let format = CuImageBufferFormat {
            width: value.width,
            height: value.height,
            stride: value.step,
            pixel_format: encoding_to_pixel_format(&value.encoding),
        };
        if !format.is_valid() {
            return Err(format!(
                "Image: encoding '{}' is not valid for width={}, height={}, step={}",
                value.encoding, value.width, value.height, value.step
            ));
        }

        let required = format.required_bytes();
        if value.data.len() < required {
            return Err(format!(
                "Image: data length {} < expected {}",
                value.data.len(),
                required
            ));
        }
        Ok(CuImage::new(format, CuHandle::new_detached(value.data)))
    }
}

impl TryFrom<Imu> for ImuPayload {
    type Error = String;

    fn try_from(value: Imu) -> Result<Self, Self::Error> {
        // sensor_msgs/Imu has no temperature field, default to 0°C on import.
        Ok(ImuPayload::from_raw(
            [
                value.linear_acceleration.x as f32,
                value.linear_acceleration.y as f32,
                value.linear_acceleration.z as f32,
            ],
            [
                value.angular_velocity.x as f32,
                value.angular_velocity.y as f32,
                value.angular_velocity.z as f32,
            ],
            0.0,
        ))
    }
}

impl TryFrom<MagneticField> for MagnetometerPayload {
    type Error = String;

    fn try_from(value: MagneticField) -> Result<Self, Self::Error> {
        Ok(MagnetometerPayload::from_raw([
            (value.magnetic_field.x * TESLA_TO_UT) as f32,
            (value.magnetic_field.y * TESLA_TO_UT) as f32,
            (value.magnetic_field.z * TESLA_TO_UT) as f32,
        ]))
    }
}

fn validate_pointcloud2_buffer_len(
    height: u32,
    row_step: u32,
    data_len: usize,
) -> Result<(), String> {
    let required_bytes = (height as usize)
        .checked_mul(row_step as usize)
        .ok_or_else(|| "PointCloud2: row_step*height overflow".to_string())?;
    if data_len < required_bytes {
        return Err(format!(
            "PointCloud2: data length {} < expected {}",
            data_len, required_bytes
        ));
    }
    Ok(())
}

fn decode_pointcloud2_into_soa<const N: usize>(
    out: &mut PointCloudSoa<N>,
    value: &PointCloud2,
) -> Result<(), String> {
    let width_usize = value.width as usize;
    let height_usize = value.height as usize;
    let count = width_usize
        .checked_mul(height_usize)
        .ok_or_else(|| "PointCloud2: width*height overflow".to_string())?;
    if count > N {
        return Err(format!(
            "PointCloud2: {} points exceed PointCloudSoa capacity {}",
            count, N
        ));
    }

    out.len = 0;
    if count == 0 {
        return Ok(());
    }

    let point_step = value.point_step as usize;
    if point_step < 24 {
        return Err("PointCloud2: point_step too small for x/y/z/intensity/tov".to_string());
    }

    let row_step = value.row_step as usize;
    let min_row_step = width_usize
        .checked_mul(point_step)
        .ok_or_else(|| "PointCloud2: width*point_step overflow".to_string())?;
    if row_step < min_row_step {
        return Err(format!(
            "PointCloud2: row_step {} < width*point_step {}",
            row_step, min_row_step
        ));
    }

    let offsets = field_offsets(&value.fields)?;
    validate_pointcloud2_buffer_len(value.height, value.row_step, value.data.len())?;

    for row in 0..height_usize {
        let row_base = row
            .checked_mul(row_step)
            .ok_or_else(|| "PointCloud2: row offset overflow".to_string())?;
        for col in 0..width_usize {
            let base = row_base
                .checked_add(
                    col.checked_mul(point_step)
                        .ok_or_else(|| "PointCloud2: point offset overflow".to_string())?,
                )
                .ok_or_else(|| "PointCloud2: point offset overflow".to_string())?;

            let x = read_f32(&value.data, base + offsets.x)?;
            let y = read_f32(&value.data, base + offsets.y)?;
            let z = read_f32(&value.data, base + offsets.z)?;
            let intensity = read_f32(&value.data, base + offsets.intensity)?;
            let tov_sec = read_u32(&value.data, base + offsets.tov_sec)? as u64;
            let tov_nsec = read_u32(&value.data, base + offsets.tov_nsec)? as u64;

            let tov = tov_sec
                .checked_mul(1_000_000_000)
                .and_then(|s| s.checked_add(tov_nsec))
                .ok_or_else(|| "PointCloud2: timestamp overflow".to_string())?;
            out.push(PointCloud::new(tov.into(), x, y, z, intensity, None));
        }
    }

    Ok(())
}

#[derive(Clone, Copy)]
struct PointOffsets {
    x: usize,
    y: usize,
    z: usize,
    intensity: usize,
    tov_sec: usize,
    tov_nsec: usize,
}

fn default_header() -> Header {
    Header {
        stamp: crate::builtin::Time { sec: 0, nanosec: 0 },
        frame_id: "".into(),
    }
}

fn field_offsets(fields: &[PointField]) -> Result<PointOffsets, String> {
    let find = |name: &str, datatype: u8| -> Result<usize, String> {
        fields
            .iter()
            .find(|f| f.name.as_str() == name)
            .ok_or_else(|| format!("PointCloud2: missing field '{name}'"))
            .and_then(|field| {
                if field.datatype != datatype {
                    Err(format!(
                        "PointCloud2: field '{}' has datatype {}, expected {}",
                        name, field.datatype, datatype
                    ))
                } else {
                    Ok(field.offset as usize)
                }
            })
    };

    let intensity_offset = fields
        .iter()
        .find(|f| f.name.as_str() == "intensity" || f.name.as_str() == "i")
        .ok_or_else(|| "PointCloud2: missing field 'intensity' (or 'i')".to_string())
        .and_then(|field| {
            if field.datatype != DATATYPE_FLOAT32 {
                Err(format!(
                    "PointCloud2: field '{}' has datatype {}, expected {}",
                    field.name, field.datatype, DATATYPE_FLOAT32
                ))
            } else {
                Ok(field.offset as usize)
            }
        })?;

    Ok(PointOffsets {
        x: find("x", DATATYPE_FLOAT32)?,
        y: find("y", DATATYPE_FLOAT32)?,
        z: find("z", DATATYPE_FLOAT32)?,
        intensity: intensity_offset,
        tov_sec: find("tov_sec", DATATYPE_UINT32)?,
        tov_nsec: find("tov_nsec", DATATYPE_UINT32)?,
    })
}

fn read_f32(data: &[u8], offset: usize) -> Result<f32, String> {
    let bytes = data
        .get(offset..offset + 4)
        .ok_or_else(|| format!("PointCloud2: missing f32 at offset {offset}"))?;
    Ok(f32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]))
}

fn read_u32(data: &[u8], offset: usize) -> Result<u32, String> {
    let bytes = data
        .get(offset..offset + 4)
        .ok_or_else(|| format!("PointCloud2: missing u32 at offset {offset}"))?;
    Ok(u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]))
}

fn pixel_format_to_encoding(pixel_format: [u8; 4]) -> String {
    match &pixel_format {
        b"GRAY" | b"Y800" => "mono8".to_string(),
        b"RGB3" | b"RGB " => "rgb8".to_string(),
        b"BGR3" | b"BGR " => "bgr8".to_string(),
        b"RGBA" => "rgba8".to_string(),
        b"BGRA" => "bgra8".to_string(),
        b"NV12" => "nv12".to_string(),
        b"NV21" => "nv21".to_string(),
        b"I420" => "i420".to_string(),
        b"YV12" => "yv12".to_string(),
        #[cfg(any(feature = "humble", feature = "jazzy"))]
        b"YUYV" => "yuv422_yuy2".to_string(),
        #[cfg(not(any(feature = "humble", feature = "jazzy")))]
        b"YUYV" => "yuyv".to_string(),
        #[cfg(any(feature = "humble", feature = "jazzy"))]
        b"UYVY" => "yuv422".to_string(),
        #[cfg(not(any(feature = "humble", feature = "jazzy")))]
        b"UYVY" => "uyvy".to_string(),
        _ => {
            let end = pixel_format
                .iter()
                .position(|byte| *byte == 0)
                .unwrap_or(pixel_format.len());
            String::from_utf8_lossy(&pixel_format[..end]).to_string()
        }
    }
}

fn encoding_to_pixel_format(encoding: &str) -> [u8; 4] {
    match encoding {
        "mono8" => *b"GRAY",
        "rgb8" => *b"RGB3",
        "bgr8" => *b"BGR3",
        "rgba8" => *b"RGBA",
        "bgra8" => *b"BGRA",
        "nv12" | "NV12" => *b"NV12",
        "nv21" | "NV21" => *b"NV21",
        "i420" | "I420" => *b"I420",
        "yv12" | "YV12" => *b"YV12",
        "yuyv" | "YUYV" | "yuv422_yuy2" | "YUV422_YUY2" => *b"YUYV",
        "uyvy" | "UYVY" | "yuv422" | "YUV422" => *b"UYVY",
        _ => {
            let mut out = [0u8; 4];
            let bytes = encoding.as_bytes();
            let copy_len = bytes.len().min(out.len());
            out[..copy_len].copy_from_slice(&bytes[..copy_len]);
            out
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::RosBridgeAdapter;

    fn sample_pointcloud() -> PointCloudSoa<4> {
        let mut cloud = PointCloudSoa::<4>::default();
        cloud.push(PointCloud::new(
            1_500_000_111u64.into(),
            1.0,
            2.0,
            3.0,
            4.0,
            None,
        ));
        cloud.push(PointCloud::new(
            2_500_000_222u64.into(),
            5.0,
            6.0,
            7.0,
            8.0,
            None,
        ));
        cloud
    }

    fn assert_same_points<const N: usize>(expected: &PointCloudSoa<N>, actual: &PointCloudSoa<N>) {
        assert_eq!(expected.len, actual.len);
        for i in 0..expected.len {
            assert_eq!(expected.tov[i].as_nanos(), actual.tov[i].as_nanos());
            assert!((expected.x[i].value - actual.x[i].value).abs() < 1e-6);
            assert!((expected.y[i].value - actual.y[i].value).abs() < 1e-6);
            assert!((expected.z[i].value - actual.z[i].value).abs() < 1e-6);
            assert!((expected.i[i].value - actual.i[i].value).abs() < 1e-6);
        }
    }

    #[test]
    fn pointcloud_soa_roundtrip() {
        let cloud = sample_pointcloud();

        let ros_value = cloud.to_ros_message();
        let bytes = cdr::serialize::<_, _, cdr::CdrLe>(&ros_value, cdr::Infinite)
            .expect("cdr encode should succeed");
        let decoded_ros: <PointCloudSoa<4> as RosBridgeAdapter>::RosMessage =
            cdr::deserialize(bytes.as_slice()).expect("cdr decode should succeed");
        let recovered =
            PointCloudSoa::<4>::from_ros_message(decoded_ros).expect("adapter decode should work");

        assert_same_points(&cloud, &recovered);
    }

    #[test]
    fn pointcloud_soa_handle_roundtrip() {
        let expected = sample_pointcloud();
        let cloud = PointCloudSoaHandle::from_box(Box::new(sample_pointcloud()));
        let ros_value = cloud.to_ros_message();
        let bytes = cdr::serialize::<_, _, cdr::CdrLe>(&ros_value, cdr::Infinite)
            .expect("cdr encode should succeed");
        let decoded_ros: <PointCloudSoaHandle<4> as RosBridgeAdapter>::RosMessage =
            cdr::deserialize(bytes.as_slice()).expect("cdr decode should succeed");
        let recovered = PointCloudSoaHandle::<4>::from_ros_message(decoded_ros)
            .expect("adapter decode should work");

        recovered.with_inner(|actual| assert_same_points(&expected, actual));
    }

    #[test]
    fn image_roundtrip() {
        let format = CuImageBufferFormat {
            width: 4,
            height: 2,
            stride: 8,
            pixel_format: *b"GRAY",
        };
        let bytes = (0u8..16).collect::<Vec<_>>();
        let mut image = CuImage::new(format, CuHandle::new_detached(bytes.clone()));
        image.seq = 0;

        let ros_value = image.to_ros_message();
        let encoded = cdr::serialize::<_, _, cdr::CdrLe>(&ros_value, cdr::Infinite)
            .expect("cdr encode should succeed");
        let decoded_ros: <CuImage<Vec<u8>> as RosBridgeAdapter>::RosMessage =
            cdr::deserialize(encoded.as_slice()).expect("cdr decode should succeed");
        let recovered =
            CuImage::<Vec<u8>>::from_ros_message(decoded_ros).expect("adapter decode should work");

        assert_eq!(recovered.format.width, format.width);
        assert_eq!(recovered.format.height, format.height);
        assert_eq!(recovered.format.stride, format.stride);
        assert_eq!(recovered.format.pixel_format, format.pixel_format);
        let recovered_bytes = recovered.buffer_handle.with_inner(|inner| inner.to_vec());
        assert_eq!(recovered_bytes, bytes);
    }

    #[test]
    fn image_roundtrip_nv12_uses_layout_aware_size() {
        let format = CuImageBufferFormat {
            width: 4,
            height: 2,
            stride: 4,
            pixel_format: *b"NV12",
        };
        let bytes = (0u8..12).collect::<Vec<_>>();
        let image = CuImage::new(format, CuHandle::new_detached(bytes.clone()));

        let ros_value = image.to_ros_message();
        assert_eq!(ros_value.encoding, "nv12");
        assert_eq!(ros_value.step, 4);

        let recovered =
            CuImage::<Vec<u8>>::from_ros_message(ros_value).expect("adapter decode should work");
        assert_eq!(recovered.format.pixel_format, *b"NV12");
        assert_eq!(recovered.format.required_bytes(), 12);
        let recovered_bytes = recovered.buffer_handle.with_inner(|inner| inner.to_vec());
        assert_eq!(recovered_bytes, bytes);
    }

    #[test]
    fn image_packed_aliases_use_standard_ros_encodings() {
        let cases = [
            (*b"GRAY", "mono8"),
            (*b"Y800", "mono8"),
            (*b"RGB3", "rgb8"),
            (*b"RGB ", "rgb8"),
            (*b"BGR3", "bgr8"),
            (*b"BGR ", "bgr8"),
            (*b"RGBA", "rgba8"),
            (*b"BGRA", "bgra8"),
        ];

        for (pixel_format, expected) in cases {
            assert_eq!(pixel_format_to_encoding(pixel_format), expected);
        }
    }

    #[test]
    fn image_yuv422_encoding_matches_ros_profile() {
        #[cfg(any(feature = "humble", feature = "jazzy"))]
        let expected = [(b"YUYV", "yuv422_yuy2"), (b"UYVY", "yuv422")];
        #[cfg(not(any(feature = "humble", feature = "jazzy")))]
        let expected = [(b"YUYV", "yuyv"), (b"UYVY", "uyvy")];

        for (pixel_format, encoding) in expected {
            assert_eq!(pixel_format_to_encoding(*pixel_format), encoding);
        }
    }

    #[test]
    fn image_yuv422_input_accepts_modern_and_legacy_encodings() {
        for encoding in ["yuyv", "YUYV", "yuv422_yuy2", "YUV422_YUY2"] {
            assert_eq!(encoding_to_pixel_format(encoding), *b"YUYV");
        }
        for encoding in ["uyvy", "UYVY", "yuv422", "YUV422"] {
            assert_eq!(encoding_to_pixel_format(encoding), *b"UYVY");
        }
    }

    #[test]
    fn image_type_hash_matches_ros_profile() {
        #[cfg(feature = "humble")]
        let expected = "TypeHashNotSupported";
        #[cfg(not(feature = "humble"))]
        let expected = "RIHS01_d31d41a9a4c4bc8eae9be757b0beed306564f7526c88ea6a4588fb9582527d47";

        assert_eq!(
            <CuImage<Vec<u8>> as RosBridgeAdapter>::type_hash(),
            expected
        );
    }

    #[cfg(any(feature = "humble", feature = "jazzy"))]
    #[test]
    fn image_validation_rejects_formats_unsupported_by_legacy_cv_bridge() {
        for pixel_format in [b"NV12", b"NV21", b"I420", b"YV12", b"MJPG"] {
            let image = CuImage::new(
                CuImageBufferFormat {
                    width: 4,
                    height: 2,
                    stride: 4,
                    pixel_format: *pixel_format,
                },
                CuHandle::new_detached(vec![0; 64]),
            );

            let error = <CuImage<Vec<u8>> as RosBridgeAdapter>::validate_ros_message(&image)
                .expect_err("unsupported image format should be rejected");
            assert!(error.contains(String::from_utf8_lossy(pixel_format).as_ref()));
        }
    }

    #[cfg(any(feature = "humble", feature = "jazzy"))]
    #[test]
    fn image_validation_accepts_legacy_cv_bridge_formats() {
        for pixel_format in [
            b"GRAY", b"Y800", b"RGB3", b"RGB ", b"BGR3", b"BGR ", b"RGBA", b"BGRA", b"YUYV",
            b"UYVY",
        ] {
            let image = CuImage::new(
                CuImageBufferFormat {
                    width: 4,
                    height: 2,
                    stride: 16,
                    pixel_format: *pixel_format,
                },
                CuHandle::new_detached(vec![0; 64]),
            );

            <CuImage<Vec<u8>> as RosBridgeAdapter>::validate_ros_message(&image)
                .expect("supported image format should pass validation");
        }
    }

    #[test]
    fn imu_roundtrip_accel_and_gyro() {
        let imu = ImuPayload::from_raw([9.8, -0.2, 0.5], [0.1, -0.2, 1.5], 0.0);

        let ros_value = imu.to_ros_message();
        let encoded = cdr::serialize::<_, _, cdr::CdrLe>(&ros_value, cdr::Infinite)
            .expect("cdr encode should succeed");
        let decoded_ros: <ImuPayload as RosBridgeAdapter>::RosMessage =
            cdr::deserialize(encoded.as_slice()).expect("cdr decode should succeed");
        let recovered =
            ImuPayload::from_ros_message(decoded_ros).expect("adapter decode should work");

        assert!((imu.accel_x.value - recovered.accel_x.value).abs() < 1e-6);
        assert!((imu.accel_y.value - recovered.accel_y.value).abs() < 1e-6);
        assert!((imu.accel_z.value - recovered.accel_z.value).abs() < 1e-6);
        assert!((imu.gyro_x.value - recovered.gyro_x.value).abs() < 1e-6);
        assert!((imu.gyro_y.value - recovered.gyro_y.value).abs() < 1e-6);
        assert!((imu.gyro_z.value - recovered.gyro_z.value).abs() < 1e-6);
        assert_eq!(
            recovered
                .temperature
                .get::<cu29::units::si::thermodynamic_temperature::degree_celsius>(),
            0.0
        );
    }

    #[test]
    fn magnetic_field_roundtrip() {
        let mag = MagnetometerPayload::from_raw([42.0, -13.0, 8.0]);

        let ros_value = mag.to_ros_message();
        let encoded = cdr::serialize::<_, _, cdr::CdrLe>(&ros_value, cdr::Infinite)
            .expect("cdr encode should succeed");
        let decoded_ros: <MagnetometerPayload as RosBridgeAdapter>::RosMessage =
            cdr::deserialize(encoded.as_slice()).expect("cdr decode should succeed");
        let recovered =
            MagnetometerPayload::from_ros_message(decoded_ros).expect("adapter decode should work");

        assert!((mag.mag_x.get::<microtesla>() - recovered.mag_x.get::<microtesla>()).abs() < 1e-3);
        assert!((mag.mag_y.get::<microtesla>() - recovered.mag_y.get::<microtesla>()).abs() < 1e-3);
        assert!((mag.mag_z.get::<microtesla>() - recovered.mag_z.get::<microtesla>()).abs() < 1e-3);
    }
}

#[cfg(test)]
mod message_tests {
    use super::*;
    use crate::builtin::Time;

    fn sample_header() -> Header {
        Header {
            stamp: Time {
                sec: 1_700_000_000,
                nanosec: 500,
            },
            frame_id: "camera_left".into(),
        }
    }

    #[test]
    fn compressed_image_roundtrips_and_keeps_its_bytes_opaque() {
        let value = CompressedImage {
            header: sample_header(),
            format: "h264".into(),
            data: (0u8..64).collect(),
        };

        let bytes =
            cdr::serialize::<_, _, cdr::CdrLe>(&value, cdr::Infinite).expect("cdr encode succeeds");
        let decoded: CompressedImage =
            cdr::deserialize(bytes.as_slice()).expect("cdr decode succeeds");
        assert_eq!(decoded, value);
        // The encoder's payload must survive untouched: this type exists so a hardware-encoded
        // stream is published as-is rather than expanded into an `Image` and re-encoded.
        assert_eq!(decoded.data, (0u8..64).collect::<Vec<_>>());
    }

    #[test]
    fn camera_info_roundtrips() {
        let value = CameraInfo {
            header: sample_header(),
            height: 480,
            width: 640,
            distortion_model: "plumb_bob".into(),
            d: vec![0.0; 5],
            k: [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0],
            r: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            p: [
                500.0, 0.0, 320.0, -35.0, 0.0, 500.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0,
            ],
            binning_x: 0,
            binning_y: 0,
            roi: RegionOfInterest::default(),
        };

        let bytes =
            cdr::serialize::<_, _, cdr::CdrLe>(&value, cdr::Infinite).expect("cdr encode succeeds");
        let decoded: CameraInfo = cdr::deserialize(bytes.as_slice()).expect("cdr decode succeeds");
        assert_eq!(decoded, value);
    }

    #[test]
    fn camera_info_d_is_a_sequence_while_k_r_p_are_fixed() {
        let encode = |info: &CameraInfo| {
            cdr::serialize::<_, _, cdr::CdrLe>(info, cdr::Infinite)
                .expect("cdr encode succeeds")
                .len()
        };

        let mut base = CameraInfo {
            header: sample_header(),
            distortion_model: "plumb_bob".into(),
            d: vec![0.0; 4],
            ..Default::default()
        };
        let four = encode(&base);
        base.d.push(0.0);
        let five = encode(&base);
        // `d` is `float64[]`: one more element is 8 more bytes of content.
        assert_eq!(five - four, 8);

        // `k`, `r` and `p` are fixed matrices, so changing their VALUES cannot change the size.
        // If any of them encoded as a sequence there would be a uint32 length here too, and every
        // field after it would be shifted for a ROS subscriber.
        let mut changed = base.clone();
        changed.k = [1.0; 9];
        changed.r = [2.0; 9];
        changed.p = [3.0; 12];
        assert_eq!(encode(&changed), five);
    }

    #[test]
    fn fixed_matrices_carry_no_length_prefix() {
        #[derive(Serialize, Deserialize)]
        struct ProjectionOnly {
            p: [f64; 12],
        }

        let bytes =
            cdr::serialize::<_, _, cdr::CdrLe>(&ProjectionOnly { p: [1.0; 12] }, cdr::Infinite)
                .expect("cdr encode succeeds");
        // 4-byte encapsulation header then twelve bare f64 — no uint32 count.
        assert_eq!(bytes.len(), 4 + 12 * 8);
    }

    #[test]
    fn joint_state_roundtrips_with_an_empty_optional_array() {
        // A robot that measures position and velocity but no torque leaves `effort` empty; the
        // message defines that as "not reported", and the parallel arrays are still associated
        // with `name` by index.
        let value = JointState {
            header: Header {
                stamp: crate::builtin::Time {
                    sec: 1_700_000_000,
                    nanosec: 500_000_000,
                },
                frame_id: "base_link".into(),
            },
            name: vec!["left_wheel".into(), "right_wheel".into()],
            position: vec![0.25, -0.5],
            velocity: vec![1.0, -1.0],
            effort: Vec::new(),
        };

        let bytes =
            cdr::serialize::<_, _, cdr::CdrLe>(&value, cdr::Infinite).expect("cdr encode succeeds");
        let decoded: JointState = cdr::deserialize(bytes.as_slice()).expect("cdr decode succeeds");
        assert_eq!(decoded, value);
        assert_eq!(decoded.name.len(), 2);
        // An empty sequence still writes its uint32 length. Dropping it would shift every field
        // after it, and there are none here only because `effort` is last.
        assert_eq!(bytes[bytes.len() - 4..], 0u32.to_le_bytes());
    }

    #[test]
    fn battery_state_roundtrips_and_preserves_unmeasured_nan() {
        // Compared field by field rather than with `assert_eq!` on the struct: the honest value
        // of an unmeasured field is NaN, and NaN != NaN.
        let value = BatteryState {
            header: Header {
                stamp: crate::builtin::Time { sec: 7, nanosec: 8 },
                frame_id: "battery".into(),
            },
            voltage: 12.1,
            temperature: f32::NAN,
            current: f32::NAN,
            charge: f32::NAN,
            capacity: f32::NAN,
            design_capacity: f32::NAN,
            percentage: f32::NAN,
            power_supply_status: BatteryState::POWER_SUPPLY_STATUS_DISCHARGING,
            power_supply_health: BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN,
            power_supply_technology: BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO,
            present: true,
            cell_voltage: vec![4.05, 4.1, 3.95],
            cell_temperature: Vec::new(),
            location: "slot0".into(),
            serial_number: "".into(),
        };

        let bytes =
            cdr::serialize::<_, _, cdr::CdrLe>(&value, cdr::Infinite).expect("cdr encode succeeds");
        let decoded: BatteryState =
            cdr::deserialize(bytes.as_slice()).expect("cdr decode succeeds");

        assert_eq!(decoded.header, value.header);
        assert_eq!(decoded.voltage, 12.1);
        assert!(decoded.temperature.is_nan(), "NaN must survive the wire");
        assert!(decoded.percentage.is_nan());
        assert_eq!(decoded.power_supply_status, 2);
        assert_eq!(decoded.power_supply_technology, 3);
        assert!(decoded.present);
        assert_eq!(decoded.cell_voltage, vec![4.05, 4.1, 3.95]);
        assert!(decoded.cell_temperature.is_empty());
        assert_eq!(decoded.location, "slot0");
        assert_eq!(decoded.serial_number, "");
    }

    #[test]
    fn battery_state_writes_the_enums_and_present_as_four_bytes() {
        // A round trip through the same struct cannot detect a field order that disagrees with
        // the .msg: swapping `charge` and `capacity`, or `present` and `power_supply_status`, is
        // still symmetric here, still the same length, still the same type hash — and decodes
        // into the wrong field on every ROS consumer. So the bytes are checked directly.
        let value = BatteryState {
            header: Header::default(),
            voltage: 1.0,
            temperature: 2.0,
            current: 3.0,
            charge: 4.0,
            capacity: 5.0,
            design_capacity: 6.0,
            percentage: 7.0,
            power_supply_status: BatteryState::POWER_SUPPLY_STATUS_FULL,
            power_supply_health: BatteryState::POWER_SUPPLY_HEALTH_GOOD,
            power_supply_technology: BatteryState::POWER_SUPPLY_TECHNOLOGY_LION,
            present: true,
            cell_voltage: Vec::new(),
            cell_temperature: Vec::new(),
            location: "".into(),
            serial_number: "".into(),
        };

        let bytes =
            cdr::serialize::<_, _, cdr::CdrLe>(&value, cdr::Infinite).expect("cdr encode succeeds");
        // 4 encapsulation + 8 stamp + 4 length + 1 for the empty frame_id "\0" = 17, padded to
        // 20 for the first f32.
        let floats = 20;
        let mut expected = Vec::new();
        for value in [1.0f32, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0] {
            expected.extend_from_slice(&value.to_le_bytes());
        }
        // status, health, technology, present: four bytes, no padding and no widening. CDR has
        // no packed bool, so `present` is a whole byte of its own.
        expected.extend_from_slice(&[4, 1, 2, 1]);
        assert_eq!(&bytes[floats..floats + 32], &expected[..]);
    }
}
