pub mod tasks;

use clap::Parser;
use cu_sensor_payloads::{CuImage, PointCloud};
use cu_spatial_payloads::Transform3D;
use cu29::prelude::{
    CuError, CuResult, UnifiedLogType, UnifiedLogger, UnifiedLoggerBuilder, UnifiedLoggerIOReader,
    UnifiedLoggerRead, gen_cumsgs,
};
use cu29_export::copperlists_reader;
use cu29_logviz::{apply_tov, log_as_components, log_imu, log_pointcloud};
use rerun::RecordingStream;
use rerun::components::ViewCoordinates as ViewCoordinatesComponent;
use std::path::{Path, PathBuf};

// This will create the CuStampedDataSet that is specific to this example.
// It is used to instruct logviz how to decode the logs.
gen_cumsgs!("copperconfig.ron");

#[derive(Clone, Copy, Debug)]
struct DemoPaths {
    map: &'static str,
    base_link: &'static str,
    lidar: &'static str,
    point: &'static str,
    camera: &'static str,
    imu: &'static str,
}

impl Default for DemoPaths {
    fn default() -> Self {
        Self {
            map: "map",
            base_link: "map/base_link",
            lidar: "map/base_link/lidar",
            point: "map/base_link/point",
            camera: "map/base_link/camera",
            imu: "map/base_link/imu",
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct DemoSensorTransforms {
    lidar: Transform3D<f32>,
    camera: Transform3D<f32>,
    imu: Transform3D<f32>,
}

fn demo_sensor_transforms() -> DemoSensorTransforms {
    DemoSensorTransforms {
        lidar: Transform3D::from_matrix([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.2, 0.0, 0.1, 1.0],
        ]),
        camera: Transform3D::from_matrix([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.1, 0.05, 0.2, 1.0],
        ]),
        imu: Transform3D::from_matrix([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.05, 1.0],
        ]),
    }
}

fn map_view_coordinates() -> ViewCoordinatesComponent {
    ViewCoordinatesComponent::FLU
}

fn build_camera_pinhole(width: u32, height: u32) -> rerun::Pinhole {
    let width = width as f32;
    let height = height as f32;
    rerun::Pinhole::from_focal_length_and_resolution([width, height], [width, height])
        .with_camera_xyz(ViewCoordinatesComponent::FLU)
}

fn log_axes(rec: &RecordingStream, frame: &str, axis_len: f32) -> CuResult<()> {
    let path = format!("{}/axes", frame);
    let arrows = rerun::Arrows3D::from_vectors([
        [axis_len, 0.0, 0.0],
        [0.0, axis_len, 0.0],
        [0.0, 0.0, axis_len],
    ])
    .with_colors([
        rerun::Color::from([255, 0, 0]),
        rerun::Color::from([0, 255, 0]),
        rerun::Color::from([0, 0, 255]),
    ]);
    rec.log_static(path, &arrows)
        .map_err(|e| CuError::new_with_cause("Failed to log axes", e))
}

fn log_image_vec_as_components(
    rec: &RecordingStream,
    path: &str,
    image: &CuImage<Vec<u8>>,
) -> CuResult<()> {
    log_as_components(rec, path, image)
}

fn log_transform_as_components(
    rec: &RecordingStream,
    path: &str,
    transform: &Transform3D<f32>,
) -> CuResult<()> {
    log_as_components(rec, path, transform)
}

fn log_point_as_components(rec: &RecordingStream, path: &str, point: &PointCloud) -> CuResult<()> {
    log_as_components(rec, path, point)
}

#[derive(Debug, Parser)]
#[command(author, version, about)]
struct LogVizCustomCli {
    /// Base path to the unified log (e.g. logs/logviz_demo.copper)
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

fn log_demo_frame(
    rec: &RecordingStream,
    culist: &CuStampedDataSet,
    paths: DemoPaths,
    pinhole_logged: &mut bool,
) -> CuResult<()> {
    let outputs = culist.get_demo_src_outputs();
    let image_msg = &outputs.0;
    let pointcloud_msg = &outputs.1;
    let point_msg = &outputs.2;
    let transform_msg = &outputs.3;
    let imu_msg = &outputs.4;
    let sensor_transforms = demo_sensor_transforms();

    if let Some(payload) = transform_msg.payload() {
        apply_tov(rec, &transform_msg.tov);
        log_transform_as_components(rec, paths.base_link, payload)?;
        log_transform_as_components(rec, paths.lidar, &sensor_transforms.lidar)?;
        log_transform_as_components(rec, paths.camera, &sensor_transforms.camera)?;
        log_transform_as_components(rec, paths.imu, &sensor_transforms.imu)?;
    }

    if let Some(payload) = pointcloud_msg.payload() {
        apply_tov(rec, &pointcloud_msg.tov);
        log_pointcloud(rec, paths.lidar, payload)?;
    }

    if let Some(payload) = point_msg.payload() {
        apply_tov(rec, &point_msg.tov);
        log_point_as_components(rec, paths.point, payload)?;
    }

    if let Some(payload) = image_msg.payload() {
        if !*pinhole_logged {
            let pinhole = build_camera_pinhole(payload.format.width, payload.format.height);
            rec.log_static(paths.camera, &pinhole)
                .map_err(|e| CuError::new_with_cause("Failed to log pinhole", e))?;
            *pinhole_logged = true;
        }
        apply_tov(rec, &image_msg.tov);
        let image_path = format!("{}/image", paths.camera);
        log_image_vec_as_components(rec, &image_path, payload)?;
    }

    if let Some(payload) = imu_msg.payload() {
        apply_tov(rec, &imu_msg.tov);
        log_imu(rec, paths.imu, payload)?;
    }

    Ok(())
}

fn main() {
    let args = LogVizCustomCli::parse();
    let (rec, _guard) = args
        .rerun
        .init("cu-logviz-demo")
        .map_err(|e| CuError::from(format!("Failed to init rerun: {e}")))
        .expect("Failed to init rerun");

    let paths = DemoPaths::default();
    rec.log_static(
        paths.map,
        &rerun::ViewCoordinates::new(map_view_coordinates()),
    )
    .expect("Failed to log view coordinates");
    log_axes(&rec, paths.lidar, 0.2).expect("Failed to log axes");

    let dl = build_read_logger(&args.unifiedlog_base).expect("Failed to open unified log");
    let mut reader = UnifiedLoggerIOReader::new(dl, UnifiedLogType::CopperList);

    let mut pinhole_logged = false;
    for culist in copperlists_reader::<CuStampedDataSet>(&mut reader) {
        log_demo_frame(&rec, &culist.msgs, paths, &mut pinhole_logged)
            .expect("Failed to log demo frame");
    }
}

#[cfg(test)]
mod tests {
    use super::{DemoPaths, PointCloud, demo_sensor_transforms};
    use rerun::components::{PinholeProjection, Resolution};

    #[test]
    fn demo_paths_match_expected() {
        let paths = DemoPaths::default();
        assert_eq!(paths.map, "map");
        assert_eq!(paths.base_link, "map/base_link");
        assert_eq!(paths.lidar, "map/base_link/lidar");
        assert_eq!(paths.point, "map/base_link/point");
        assert_eq!(paths.camera, "map/base_link/camera");
        assert_eq!(paths.imu, "map/base_link/imu");
    }

    #[test]
    fn demo_sensor_transforms_match_expected() {
        let transforms = demo_sensor_transforms();
        let lidar_matrix = transforms.lidar.to_matrix();
        let camera_matrix = transforms.camera.to_matrix();
        let imu_matrix = transforms.imu.to_matrix();

        assert_eq!(
            [lidar_matrix[3][0], lidar_matrix[3][1], lidar_matrix[3][2]],
            [0.2, 0.0, 0.1]
        );
        assert_eq!(
            [
                camera_matrix[3][0],
                camera_matrix[3][1],
                camera_matrix[3][2]
            ],
            [0.1, 0.05, 0.2]
        );
        assert_eq!(
            [imu_matrix[3][0], imu_matrix[3][1], imu_matrix[3][2]],
            [0.0, 0.0, 0.05]
        );
    }

    #[test]
    fn pinhole_uses_image_dimensions() {
        let pinhole = super::build_camera_pinhole(64, 48);
        let resolution = pinhole.resolution_from_arrow().expect("missing resolution");
        assert_eq!(resolution, Resolution::from([64.0, 48.0]));

        let projection = pinhole
            .image_from_camera_from_arrow()
            .expect("missing projection");
        let expected =
            PinholeProjection::from_focal_length_and_principal_point([64.0, 48.0], [32.0, 24.0]);
        assert_eq!(projection, expected);
    }

    #[test]
    fn log_image_vec_as_components_smoke() -> cu29::prelude::CuResult<()> {
        let fmt = cu_sensor_payloads::CuImageBufferFormat {
            width: 4,
            height: 2,
            stride: 12,
            pixel_format: *b"RGB3",
        };
        let img = cu_sensor_payloads::CuImage::new(
            fmt,
            cu29::prelude::CuHandle::new_detached(vec![0u8; 24]),
        );
        let rec = rerun::RecordingStream::disabled();
        super::log_image_vec_as_components(&rec, "image", &img)
    }

    #[test]
    fn log_transform_as_components_smoke() -> cu29::prelude::CuResult<()> {
        let t = cu_spatial_payloads::Transform3D::<f32>::from_matrix([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.1, 0.2, 0.3, 1.0],
        ]);
        let rec = rerun::RecordingStream::disabled();
        super::log_transform_as_components(&rec, "transform", &t)
    }

    #[test]
    fn log_point_as_components_smoke() -> cu29::prelude::CuResult<()> {
        let point = PointCloud::new(cu29::clock::CuTime::from(0u64), 1.0, 2.0, 3.0, 0.0, None);
        let rec = rerun::RecordingStream::disabled();
        super::log_point_as_components(&rec, "point", &point)
    }
}
