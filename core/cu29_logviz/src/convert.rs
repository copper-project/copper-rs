use cu_sensor_payloads::{CuImageBufferFormat, Distance, PointCloudSoa};
use cu_spatial_payloads::Transform3D;
use rerun::{ChannelDatatype, ColorModel, PixelFormat, Position3D};

pub fn pointcloud_positions<const N: usize>(pc: &PointCloudSoa<N>) -> Vec<Position3D> {
    let len = pc.len.min(N);
    (0..len)
        .map(|i| {
            let Distance(x) = pc.x[i];
            let Distance(y) = pc.y[i];
            let Distance(z) = pc.z[i];
            Position3D::new(x.value, y.value, z.value)
        })
        .collect()
}

pub fn image_format_from_cu(
    fmt: CuImageBufferFormat,
) -> (
    Option<PixelFormat>,
    Option<ColorModel>,
    Option<ChannelDatatype>,
) {
    let fourcc = fmt.pixel_format;
    match &fourcc {
        b"NV12" => (Some(PixelFormat::NV12), None, None),
        b"YUYV" | b"YUY2" => (Some(PixelFormat::YUY2), None, None),
        b"RGB3" => (None, Some(ColorModel::RGB), Some(ChannelDatatype::U8)),
        b"BGR3" => (None, Some(ColorModel::BGR), Some(ChannelDatatype::U8)),
        _ => (None, None, None),
    }
}

pub fn transform_parts<T: Copy + Into<f64> + std::fmt::Debug + Default + 'static>(
    t: Transform3D<T>,
) -> ([f32; 3], [[f32; 3]; 3]) {
    let m = t.to_matrix();
    let translation = [
        m[3][0].into() as f32,
        m[3][1].into() as f32,
        m[3][2].into() as f32,
    ];
    let mat3 = [
        [
            m[0][0].into() as f32,
            m[0][1].into() as f32,
            m[0][2].into() as f32,
        ],
        [
            m[1][0].into() as f32,
            m[1][1].into() as f32,
            m[1][2].into() as f32,
        ],
        [
            m[2][0].into() as f32,
            m[2][1].into() as f32,
            m[2][2].into() as f32,
        ],
    ];
    (translation, mat3)
}
