use clap::Parser;
use cu_sensor_payloads::{CuImage, ImuPayload, PointCloud, PointCloudSoa};
use cu_spatial_payloads::Transform3D as CuTransform3D;
use cu29::clock::Tov;
use cu29::prelude::{
    ArrayLike, CopperListTuple, CuError, CuResult, UnifiedLogType, UnifiedLogger,
    UnifiedLoggerBuilder, UnifiedLoggerIOReader, UnifiedLoggerRead,
};
use cu29_export::copperlists_reader;
pub use rerun::RecordingStream;
use rerun::components::{ImageBuffer, ImageFormat, TransformMat3x3};
use rerun::datatypes::{Blob, ImageFormat as RerunImageFormat, Mat3x3};
use rerun::{Image, Points3D, Scalars, TextDocument, Transform3D as RerunTransform3D};
use serde::Serialize;
use std::any::{Any, TypeId};
use std::path::{Path, PathBuf};
use uom::si::thermodynamic_temperature::degree_celsius;

pub mod as_components;
mod convert;
mod fallback;

pub use convert::{image_format_from_cu, pointcloud_positions, transform_parts};
pub use fallback::{extract_scalars, flatten_json};

pub fn tov_to_secs(tov: &Tov) -> Option<f64> {
    match tov {
        Tov::Time(t) => Some(t.0 as f64 / 1_000_000_000.0),
        Tov::Range(r) => Some(r.start.0 as f64 / 1_000_000_000.0),
        Tov::None => None,
    }
}

pub fn apply_tov(rec: &RecordingStream, tov: &Tov) {
    if let Some(secs) = tov_to_secs(tov) {
        rec.set_duration_secs("tov", secs);
    } else {
        rec.reset_time();
    }
}

pub trait LogvizDataSet {
    fn logviz_emit(&self, rec: &RecordingStream) -> CuResult<()>;
}

fn log_scalar(rec: &RecordingStream, path: &str, value: f64) -> CuResult<()> {
    rec.log(path, &Scalars::new([value]))
        .map_err(|e| CuError::new_with_cause("Failed to log scalar", e))
}

pub fn log_image<A>(rec: &RecordingStream, path: &str, image: &CuImage<A>) -> CuResult<()>
where
    A: ArrayLike<Element = u8>,
{
    let rr_image = build_rerun_image(image);
    rec.log(path, &rr_image)
        .map_err(|e| CuError::new_with_cause("Failed to log image", e))
}

pub fn log_pointcloud<const N: usize>(
    rec: &RecordingStream,
    path: &str,
    pc: &PointCloudSoa<N>,
) -> CuResult<()> {
    let points = pointcloud_positions(pc);
    rec.log(path, &Points3D::new(points))
        .map_err(|e| CuError::new_with_cause("Failed to log point cloud", e))
}

pub fn log_transform<T: Copy + Into<f64> + std::fmt::Debug + Default + 'static>(
    rec: &RecordingStream,
    path: &str,
    transform: &CuTransform3D<T>,
) -> CuResult<()> {
    let rr_transform = build_rerun_transform(transform);
    rec.log(path, &rr_transform)
        .map_err(|e| CuError::new_with_cause("Failed to log transform", e))
}

pub fn log_imu(rec: &RecordingStream, base: &str, imu: &ImuPayload) -> CuResult<()> {
    log_scalar(rec, &format!("{}/accel_x", base), imu.accel_x.value as f64)?;
    log_scalar(rec, &format!("{}/accel_y", base), imu.accel_y.value as f64)?;
    log_scalar(rec, &format!("{}/accel_z", base), imu.accel_z.value as f64)?;
    log_scalar(rec, &format!("{}/gyro_x", base), imu.gyro_x.value as f64)?;
    log_scalar(rec, &format!("{}/gyro_y", base), imu.gyro_y.value as f64)?;
    log_scalar(rec, &format!("{}/gyro_z", base), imu.gyro_z.value as f64)?;
    log_scalar(
        rec,
        &format!("{}/temperature_c", base),
        imu.temperature.get::<degree_celsius>() as f64,
    )?;
    Ok(())
}

pub fn log_as_components<A: rerun::AsComponents>(
    rec: &RecordingStream,
    path: &str,
    value: &A,
) -> CuResult<()> {
    rec.log(path, value)
        .map_err(|e| CuError::new_with_cause("Failed to log rerun components", e))
}

pub fn log_payload_auto<T>(rec: &RecordingStream, path: &str, payload: &T) -> CuResult<()>
where
    T: Serialize + 'static,
{
    let any_payload = payload as &dyn Any;
    let payload_id = any_payload.type_id();

    if payload_id == TypeId::of::<CuImage<Vec<u8>>>() {
        let image = any_payload
            .downcast_ref::<CuImage<Vec<u8>>>()
            .expect("downcast must match TypeId");
        return log_as_components(rec, path, &as_components::LogvizImageVec::new(image));
    }

    if payload_id == TypeId::of::<PointCloud>() {
        let point = any_payload
            .downcast_ref::<PointCloud>()
            .expect("downcast must match TypeId");
        return log_as_components(rec, path, &as_components::LogvizPoint::new(point));
    }

    if payload_id == TypeId::of::<CuTransform3D<f32>>() {
        let transform = any_payload
            .downcast_ref::<CuTransform3D<f32>>()
            .expect("downcast must match TypeId");
        return log_as_components(rec, path, &as_components::LogvizTransform::new(transform));
    }

    if payload_id == TypeId::of::<CuTransform3D<f64>>() {
        let transform = any_payload
            .downcast_ref::<CuTransform3D<f64>>()
            .expect("downcast must match TypeId");
        return log_as_components(rec, path, &as_components::LogvizTransform::new(transform));
    }

    if payload_id == TypeId::of::<ImuPayload>() {
        let imu = any_payload
            .downcast_ref::<ImuPayload>()
            .expect("downcast must match TypeId");
        return log_imu(rec, path, imu);
    }

    if is_pointcloud_soa_type_name(core::any::type_name::<T>())
        && let Some(points) = extract_pointcloud_soa_positions(payload)?
    {
        return rec
            .log(path, &Points3D::new(points))
            .map_err(|e| CuError::new_with_cause("Failed to log point cloud", e));
    }

    log_fallback_payload(rec, path, payload)
}

fn is_pointcloud_soa_type_name(type_name: &str) -> bool {
    type_name.contains("PointCloudSoa<")
}

fn extract_pointcloud_soa_positions<T: Serialize>(
    payload: &T,
) -> CuResult<Option<Vec<rerun::Position3D>>> {
    let value = serde_json::to_value(payload)
        .map_err(|e| CuError::new_with_cause("Failed to serialize payload", e))?;
    let object = match value.as_object() {
        Some(object) => object,
        None => return Ok(None),
    };

    let xs = match object.get("x").and_then(|v| v.as_array()) {
        Some(values) => values,
        None => return Ok(None),
    };
    let ys = match object.get("y").and_then(|v| v.as_array()) {
        Some(values) => values,
        None => return Ok(None),
    };
    let zs = match object.get("z").and_then(|v| v.as_array()) {
        Some(values) => values,
        None => return Ok(None),
    };

    let default_len = xs.len().min(ys.len()).min(zs.len());
    let len = object
        .get("len")
        .and_then(|v| v.as_u64())
        .map_or(default_len, |value| value as usize)
        .min(default_len);

    let mut points = Vec::with_capacity(len);
    for i in 0..len {
        let Some(x) = xs[i].as_f64() else {
            return Ok(None);
        };
        let Some(y) = ys[i].as_f64() else {
            return Ok(None);
        };
        let Some(z) = zs[i].as_f64() else {
            return Ok(None);
        };
        points.push(rerun::Position3D::new(x as f32, y as f32, z as f32));
    }

    Ok(Some(points))
}

pub(crate) fn build_rerun_image<A>(image: &CuImage<A>) -> Image
where
    A: ArrayLike<Element = u8>,
{
    let (pixel_format, color_model, channel_datatype) = image_format_from_cu(image.format);
    let format = ImageFormat(RerunImageFormat {
        width: image.format.width,
        height: image.format.height,
        pixel_format,
        color_model,
        channel_datatype,
    });
    let blob = image
        .buffer_handle
        .with_inner(|inner| Blob::from(&inner[..]));
    let image_buffer = ImageBuffer::from(blob);
    Image::new(image_buffer, format)
}

pub(crate) fn build_rerun_transform<T>(transform: &CuTransform3D<T>) -> RerunTransform3D
where
    T: Copy + Into<f64> + std::fmt::Debug + Default + 'static,
{
    let (translation, mat3) = transform_parts(*transform);
    let mat_flat = [
        mat3[0][0], mat3[0][1], mat3[0][2], mat3[1][0], mat3[1][1], mat3[1][2], mat3[2][0],
        mat3[2][1], mat3[2][2],
    ];
    RerunTransform3D::new()
        .with_translation(translation)
        .with_mat3x3(TransformMat3x3::from(Mat3x3(mat_flat)))
}

pub fn log_fallback_payload<T: Serialize>(
    rec: &RecordingStream,
    base: &str,
    payload: &T,
) -> CuResult<()> {
    let value = serde_json::to_value(payload)
        .map_err(|e| CuError::new_with_cause("Failed to serialize payload", e))?;
    let flat = flatten_json(base, &value);
    for (path, value) in flat {
        if let Some(num) = value.as_f64() {
            log_scalar(rec, &path, num)?;
        } else if let Some(value_bool) = value.as_bool() {
            log_scalar(rec, &path, if value_bool { 1.0 } else { 0.0 })?;
        } else if let Some(text) = value.as_str() {
            rec.log(path.as_str(), &TextDocument::new(text))
                .map_err(|e| CuError::new_with_cause("Failed to log text", e))?;
        } else if !value.is_null() {
            rec.log(path.as_str(), &TextDocument::new(value.to_string()))
                .map_err(|e| CuError::new_with_cause("Failed to log text", e))?;
        }
    }
    Ok(())
}

#[derive(Debug, Parser)]
#[command(author, version, about)]
struct LogVizCli {
    /// Base path to the unified log (e.g. logs/my_log.copper)
    #[arg(value_name = "UNIFIEDLOG_BASE")]
    unifiedlog_base: PathBuf,

    #[command(flatten)]
    rerun: rerun::clap::RerunArgs,
}

fn build_read_logger(unifiedlog_base: &Path) -> CuResult<UnifiedLoggerRead> {
    let logger = UnifiedLoggerBuilder::new()
        .file_base_name(unifiedlog_base)
        .build()
        .map_err(|e| CuError::new_with_cause("Failed to create logger", e))?;
    match logger {
        UnifiedLogger::Read(dl) => Ok(dl),
        UnifiedLogger::Write(_) => Err(CuError::from(
            "Expected read-only unified logger in logviz CLI",
        )),
    }
}

pub fn logviz_emit_dataset<P: LogvizDataSet>(dataset: &P, rec: &RecordingStream) -> CuResult<()> {
    dataset.logviz_emit(rec)
}

pub fn run_cli<P>() -> CuResult<()>
where
    P: CopperListTuple + LogvizDataSet,
{
    let args = LogVizCli::parse();
    let (rec, _guard) = args
        .rerun
        .init("cu29-logviz")
        .map_err(|e| CuError::from(format!("Failed to init rerun: {e}")))?;
    let dl = build_read_logger(&args.unifiedlog_base)?;
    let mut reader = UnifiedLoggerIOReader::new(dl, UnifiedLogType::CopperList);
    for culist in copperlists_reader::<P>(&mut reader) {
        logviz_emit_dataset(&culist.msgs, &rec)?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use cu_sensor_payloads::{CuImageBufferFormat, Distance, PointCloud, PointCloudSoa};
    use cu_spatial_payloads::Transform3D;
    use cu29::clock::CuTime;
    use rerun::{ChannelDatatype, ColorModel, Position3D};
    use serde::Serialize;

    #[test]
    fn tov_time_to_secs() {
        let tov = Tov::Time(CuTime::from(1_500_000_000u64));
        assert_eq!(tov_to_secs(&tov), Some(1.5));
    }

    #[test]
    fn tov_none_returns_none() {
        assert_eq!(tov_to_secs(&Tov::None), None);
    }

    #[derive(Serialize)]
    struct Inner {
        c: f64,
    }

    #[derive(Serialize)]
    struct Outer {
        a: i32,
        b: Inner,
        arr: [u8; 2],
    }

    #[test]
    fn flatten_json_paths() {
        let value = serde_json::to_value(Outer {
            a: 1,
            b: Inner { c: 2.5 },
            arr: [3, 4],
        })
        .unwrap();
        let flat = flatten_json("root", &value);
        assert_eq!(flat["root/a"], serde_json::json!(1));
        assert_eq!(flat["root/b/c"], serde_json::json!(2.5));
        assert_eq!(flat["root/arr/0"], serde_json::json!(3));
        assert_eq!(flat["root/arr/1"], serde_json::json!(4));
    }

    #[test]
    fn pointcloud_positions_len() {
        let mut pc = PointCloudSoa::<3> {
            len: 3,
            ..Default::default()
        };
        pc.x[0] = Distance::from(1.0);
        pc.y[0] = Distance::from(2.0);
        pc.z[0] = Distance::from(3.0);
        let pts = pointcloud_positions(&pc);
        assert_eq!(pts.len(), 3);
        let _ = Position3D::new(0.0, 0.0, 0.0);
    }

    #[test]
    fn pointcloud_positions_respects_len() {
        let mut pc = PointCloudSoa::<3> {
            len: 1,
            ..Default::default()
        };
        pc.x[0] = Distance::from(1.0);
        pc.y[0] = Distance::from(2.0);
        pc.z[0] = Distance::from(3.0);
        pc.x[1] = Distance::from(9.0);
        pc.y[1] = Distance::from(9.0);
        pc.z[1] = Distance::from(9.0);
        let pts = pointcloud_positions(&pc);
        assert_eq!(pts.len(), 1);
        assert_eq!(pts[0], Position3D::new(1.0, 2.0, 3.0));
    }

    #[test]
    fn map_rgb3_to_color_model() {
        let fmt = CuImageBufferFormat {
            width: 2,
            height: 1,
            stride: 2,
            pixel_format: *b"RGB3",
        };
        let (pf, cm, cd) = image_format_from_cu(fmt);
        assert!(pf.is_none());
        assert_eq!(cm, Some(ColorModel::RGB));
        assert_eq!(cd, Some(ChannelDatatype::U8));
    }

    #[test]
    fn transform_translation_extracted() {
        let t = Transform3D::<f32>::from_matrix([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [1.0, 2.0, 3.0, 1.0],
        ]);
        let (translation, _mat3) = transform_parts(t);
        assert_eq!(translation, [1.0, 2.0, 3.0]);
    }

    #[test]
    fn pointcloud_soa_type_name_is_detected() {
        assert!(is_pointcloud_soa_type_name(core::any::type_name::<
            PointCloudSoa<64>,
        >()));
        assert!(!is_pointcloud_soa_type_name(core::any::type_name::<
            PointCloud,
        >()));
    }

    #[test]
    fn extract_pointcloud_soa_positions_uses_len_and_xyz_arrays() {
        let payload = serde_json::json!({
            "len": 2,
            "x": [1.0, 4.0, 9.0],
            "y": [2.0, 5.0, 9.0],
            "z": [3.0, 6.0, 9.0],
        });
        let points = extract_pointcloud_soa_positions(&payload)
            .expect("serialize")
            .expect("pointcloud payload");
        assert_eq!(points.len(), 2);
        assert_eq!(points[0], Position3D::new(1.0, 2.0, 3.0));
        assert_eq!(points[1], Position3D::new(4.0, 5.0, 6.0));
    }
}
