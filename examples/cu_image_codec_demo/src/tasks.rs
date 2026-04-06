use cu_sensor_payloads::{CuImage, CuImageBufferFormat};
use cu29::prelude::*;
use std::f32::consts::PI;
use std::sync::Arc;

const IMAGE_WIDTH: u32 = 320;
const IMAGE_HEIGHT: u32 = 240;
const IMAGE_STRIDE: u32 = IMAGE_WIDTH * 3;
const IMAGE_POOL_ID: &str = "image_codec_demo_src";
const IMAGE_POOL_SLOTS: usize = 8;

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

    let wave = 0.5 + 0.5 * (phase * 0.5 + x_norm * PI * 3.0).sin();
    let sweep = 0.5 + 0.5 * (phase * 0.35 + y_norm * PI * 2.0).cos();
    let radial = 0.5 + 0.5 * ((x_norm - 0.5).hypot(y_norm - 0.5) * 12.0 - phase).cos();

    [
        unit_to_u8(0.10 + 0.55 * x_norm + 0.25 * wave),
        unit_to_u8(0.08 + 0.50 * y_norm + 0.32 * sweep),
        unit_to_u8(0.12 + 0.28 * (1.0 - x_norm) + 0.45 * radial),
    ]
}

fn same_format(lhs: CuImageBufferFormat, rhs: CuImageBufferFormat) -> bool {
    lhs.width == rhs.width
        && lhs.height == rhs.height
        && lhs.stride == rhs.stride
        && lhs.pixel_format == rhs.pixel_format
}

#[derive(Clone, Reflect)]
#[reflect(from_reflect = false)]
pub struct SyntheticImageSrc {
    #[reflect(ignore)]
    pool: Arc<CuHostMemoryPool<Vec<u8>>>,
    tick: u64,
    format: CuImageBufferFormat,
}

impl Freezable for SyntheticImageSrc {}

impl CuSrcTask for SyntheticImageSrc {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(CuImage<Vec<u8>>);

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
        let buffer_len = format.byte_size();
        let pool =
            CuHostMemoryPool::new(IMAGE_POOL_ID, IMAGE_POOL_SLOTS, || vec![0u8; buffer_len])?;

        Ok(Self {
            pool,
            tick: 0,
            format,
        })
    }

    fn process(&mut self, ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        self.tick = self.tick.wrapping_add(1);
        let phase = self.tick as f32 * 0.08;

        let handle = self.pool.acquire().ok_or_else(|| {
            CuError::from(format!(
                "SyntheticImageSrc pool exhausted (pool_id={IMAGE_POOL_ID}, slots={IMAGE_POOL_SLOTS})"
            ))
        })?;

        handle.with_inner_mut(|inner| {
            for y in 0..IMAGE_HEIGHT as usize {
                let row = y * IMAGE_STRIDE as usize;
                for x in 0..IMAGE_WIDTH as usize {
                    let idx = row + x * 3;
                    let [r, g, b] = demo_pixel(x, y, phase);
                    inner[idx] = r;
                    inner[idx + 1] = g;
                    inner[idx + 2] = b;
                }
            }
        });

        let mut image = CuImage::new(self.format, handle);
        image.seq = self.tick;

        output.tov = Tov::Time(ctx.now());
        output.set_payload(image);
        Ok(())
    }
}

#[derive(Clone, Reflect)]
pub struct ImagePassThrough;

impl Freezable for ImagePassThrough {}

impl CuTask for ImagePassThrough {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(CuImage<Vec<u8>>);
    type Output<'m> = output_msg!(CuImage<Vec<u8>>);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self)
    }

    fn process(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        let image = input
            .payload()
            .cloned()
            .ok_or_else(|| CuError::from("ImagePassThrough expected an image payload"))?;
        output.tov = input.tov;
        output.metadata = input.metadata.clone();
        output.set_payload(image);
        Ok(())
    }
}

#[derive(Reflect)]
pub struct ImageSink;

impl Freezable for ImageSink {}

impl CuSinkTask for ImageSink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!('m, CuImage<Vec<u8>>, CuImage<Vec<u8>>);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self)
    }

    fn process(&mut self, _ctx: &CuContext, input: &Self::Input<'_>) -> CuResult<()> {
        let png = input
            .0
            .payload()
            .ok_or_else(|| CuError::from("ImageSink expected PNG image payload"))?;
        let ffv1 = input
            .1
            .payload()
            .ok_or_else(|| CuError::from("ImageSink expected FFV1 image payload"))?;

        if png.seq != ffv1.seq {
            return Err(CuError::from(format!(
                "PNG/FFV1 image sequence mismatch: {} != {}",
                png.seq, ffv1.seq
            )));
        }
        if !same_format(png.format, ffv1.format) {
            return Err(CuError::from(format!(
                "PNG/FFV1 image format mismatch: {:?} != {:?}",
                png.format, ffv1.format
            )));
        }

        Ok(())
    }
}
