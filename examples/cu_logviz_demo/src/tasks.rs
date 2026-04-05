use cu_sensor_payloads::{
    CuImage, CuImageBufferFormat, Distance, ImuPayload, PointCloud, PointCloudSoa,
};
use cu_spatial_payloads::Transform3D;
use cu29::prelude::*;
use cu29::units::si::length::meter;
use std::f32::consts::PI;

const IMAGE_WIDTH: u32 = 64;
const IMAGE_HEIGHT: u32 = 48;
const IMAGE_STRIDE: u32 = IMAGE_WIDTH * 3;
const POINTS: usize = 64;

fn normalized_coord(index: usize, extent: u32) -> f32 {
    if extent <= 1 {
        return 0.0;
    }
    index as f32 / (extent - 1) as f32
}

fn unit_to_u8(value: f32) -> u8 {
    (value.clamp(0.0, 1.0) * 255.0).round() as u8
}

fn demo_pixel(x: usize, y: usize, phase: f32) -> [u8; 3] {
    let x_norm = normalized_coord(x, IMAGE_WIDTH);
    let y_norm = normalized_coord(y, IMAGE_HEIGHT);

    let tint = 0.5 + 0.5 * (phase * 0.35).sin();
    let sweep = 0.5 + 0.5 * (phase * 0.2 + x_norm * PI).cos();
    let lift = 0.5 + 0.5 * (phase * 0.5 + y_norm * PI).sin();

    [
        unit_to_u8(0.15 + 0.65 * x_norm + 0.20 * tint),
        unit_to_u8(0.10 + 0.60 * y_norm + 0.30 * sweep),
        unit_to_u8(0.20 + 0.35 * (1.0 - x_norm) + 0.45 * lift),
    ]
}

#[derive(Clone, Reflect)]
#[reflect(opaque, from_reflect = false)]
pub struct LogvizDemoSrc {
    tick: u64,
    image: CuImage<Vec<u8>>,
    pointcloud: PointCloudSoa<POINTS>,
    point: PointCloud,
}

impl Freezable for LogvizDemoSrc {}

impl CuSrcTask for LogvizDemoSrc {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(
        CuImage<Vec<u8>>,
        PointCloudSoa<POINTS>,
        PointCloud,
        Transform3D<f32>,
        ImuPayload
    );

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let format = CuImageBufferFormat {
            width: IMAGE_WIDTH,
            height: IMAGE_HEIGHT,
            stride: IMAGE_STRIDE,
            pixel_format: *b"RGB3",
        };
        let buffer = vec![0u8; (IMAGE_STRIDE * IMAGE_HEIGHT) as usize];
        let image = CuImage::new(format, CuHandle::new_detached(buffer));
        let pointcloud = PointCloudSoa {
            len: POINTS,
            ..Default::default()
        };
        let point = PointCloud::new(cu29::clock::CuTime::from(0u64), 0.0, 0.0, 0.0, 0.0, None);

        Ok(Self {
            tick: 0,
            image,
            pointcloud,
            point,
        })
    }

    fn process(&mut self, ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        self.tick = self.tick.wrapping_add(1);
        let phase = self.tick as f32 * 0.1;
        let now = Tov::Time(ctx.now());

        self.image.buffer_handle.with_inner_mut(|inner| {
            let data = &mut inner[..];
            for y in 0..IMAGE_HEIGHT as usize {
                let row = y * IMAGE_STRIDE as usize;
                for x in 0..IMAGE_WIDTH as usize {
                    let idx = row + x * 3;
                    let [r, g, b] = demo_pixel(x, y, phase);
                    data[idx] = r;
                    data[idx + 1] = g;
                    data[idx + 2] = b;
                }
            }
        });

        let step = 2.0 * PI / POINTS as f32;
        let offset_x = phase.cos() * 0.5;
        let offset_y = phase.sin() * 0.5;
        for i in 0..POINTS {
            let angle = phase + i as f32 * step;
            self.pointcloud.x[i] = Distance::new::<meter>(angle.cos() + offset_x);
            self.pointcloud.y[i] = Distance::new::<meter>(angle.sin() + offset_y);
            self.pointcloud.z[i] = Distance::new::<meter>(i as f32 * 0.02);
        }

        let transform = Transform3D::from_matrix([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [phase.cos(), phase.sin(), 0.2, 1.0],
        ]);

        let imu = ImuPayload::from_raw(
            [phase.sin(), phase.cos(), 0.1],
            [0.01, 0.02, 0.03],
            25.0 + phase.sin(),
        );

        self.point = PointCloud::new(
            ctx.now(),
            phase.cos(),
            phase.sin(),
            0.1 + phase.sin() * 0.05,
            0.0,
            None,
        );

        output.0.tov = now;
        output.0.set_payload(self.image.clone());
        output.1.tov = now;
        output.1.set_payload(self.pointcloud.clone());
        output.2.tov = now;
        output.2.set_payload(self.point.clone());
        output.3.tov = now;
        output.3.set_payload(transform);
        output.4.tov = now;
        output.4.set_payload(imu);

        Ok(())
    }
}

#[derive(Reflect)]
pub struct LogvizDemoSink;

impl Freezable for LogvizDemoSink {}

impl CuSinkTask for LogvizDemoSink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(
        'm,
        CuImage<Vec<u8>>,
        PointCloudSoa<POINTS>,
        PointCloud,
        Transform3D<f32>,
        ImuPayload
    );

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self)
    }

    fn process(&mut self, _ctx: &CuContext, _input: &Self::Input<'_>) -> CuResult<()> {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cu29::clock::CuDuration;

    fn centroid_x(pc: &PointCloudSoa<POINTS>) -> f32 {
        let len = pc.len.min(POINTS);
        if len == 0 {
            return 0.0;
        }
        let mut sum = 0.0;
        for i in 0..len {
            sum += pc.x[i].value;
        }
        sum / len as f32
    }

    #[test]
    fn demo_pointcloud_len_is_initialized() {
        let src = LogvizDemoSrc::new(None, ()).expect("failed to build demo source");
        assert_eq!(src.pointcloud.len, POINTS);
    }

    #[test]
    fn demo_image_stays_visually_continuous_at_end_of_run() {
        let phase = 120.0 * 0.1;
        let mut max_green_delta = 0u8;

        for y in 1..IMAGE_HEIGHT as usize {
            let prev = demo_pixel(0, y - 1, phase)[1];
            let next = demo_pixel(0, y, phase)[1];
            max_green_delta = max_green_delta.max(prev.abs_diff(next));
        }

        assert!(
            max_green_delta < 32,
            "unexpected vertical seam in generated image, max green delta was {max_green_delta}"
        );
    }

    #[test]
    fn demo_pointcloud_centroid_moves() {
        let mut src = LogvizDemoSrc::new(None, ()).expect("failed to build demo source");
        let (ctx, mock) = CuContext::new_mock_clock();
        let mut output: <LogvizDemoSrc as CuSrcTask>::Output<'_> = Default::default();

        src.process(&ctx, &mut output)
            .expect("failed to run demo process");
        let pc0 = output.1.payload().expect("missing pointcloud payload");
        let c0 = centroid_x(pc0);

        mock.increment(CuDuration(1_000_000));
        let mut output2: <LogvizDemoSrc as CuSrcTask>::Output<'_> = Default::default();
        src.process(&ctx, &mut output2)
            .expect("failed to run demo process");
        let pc1 = output2.1.payload().expect("missing pointcloud payload");
        let c1 = centroid_x(pc1);

        assert!(
            (c0 - c1).abs() > 1.0e-3,
            "expected centroid to move between frames (c0={c0}, c1={c1})"
        );
    }

    #[test]
    fn demo_point_payload_moves() {
        let mut src = LogvizDemoSrc::new(None, ()).expect("failed to build demo source");
        let (ctx, mock) = CuContext::new_mock_clock();
        let mut output: <LogvizDemoSrc as CuSrcTask>::Output<'_> = Default::default();

        src.process(&ctx, &mut output)
            .expect("failed to run demo process");
        let p0 = output.2.payload().expect("missing point payload");
        let x0 = p0.x;

        mock.increment(CuDuration(1_000_000));
        let mut output2: <LogvizDemoSrc as CuSrcTask>::Output<'_> = Default::default();
        src.process(&ctx, &mut output2)
            .expect("failed to run demo process");
        let p1 = output2.2.payload().expect("missing point payload");
        let x1 = p1.x;

        assert!(
            (x0.value - x1.value).abs() > 1.0e-3,
            "expected point x to move between frames (x0={}, x1={})",
            x0.value,
            x1.value
        );
    }
}
