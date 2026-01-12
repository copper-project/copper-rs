use cu_aligner::define_task;
use cu_sensor_payloads::{CuImage, CuImageBufferFormat};
use cu29::payload::CuArray;
use cu29::prelude::*;
use std::ops::DerefMut;
use std::sync::Arc;

const BUFFER_CAP: usize = 16;
const OUTPUT_CAP: usize = 4;

define_task!(
    ImageAlignerTask,
    0 => { BUFFER_CAP, OUTPUT_CAP, CuImage<Vec<u8>> },
    1 => { BUFFER_CAP, OUTPUT_CAP, CuImage<Vec<u8>> }
);

pub struct ImageSrcTask {
    pool: Arc<CuHostMemoryPool<Vec<u8>>>,
    format: CuImageBufferFormat,
    seq: u64,
    base_value: u8,
    tov_offset: CuDuration,
}

impl Freezable for ImageSrcTask {}

impl CuSrcTask for ImageSrcTask {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(CuImage<Vec<u8>>);

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let config = config.ok_or_else(|| CuError::from("Missing config"))?;
        let width = config
            .get::<u32>("width")
            .ok_or_else(|| CuError::from("Missing width"))?;
        let height = config
            .get::<u32>("height")
            .ok_or_else(|| CuError::from("Missing height"))?;
        let base_value = config.get::<u32>("base_value").unwrap_or(0) as u8;
        let tov_offset_ms = config.get::<u32>("tov_offset_ms").unwrap_or(0);
        let pool_size = config
            .get::<u32>("pool_size")
            .unwrap_or((BUFFER_CAP + 1) as u32) as usize;
        let pool_id = config
            .get::<String>("pool_id")
            .unwrap_or_else(|| "image_src_pool".to_string());

        if pool_size <= BUFFER_CAP {
            return Err(CuError::from(format!(
                "pool_size ({pool_size}) must be greater than buffer capacity ({BUFFER_CAP})"
            )));
        }

        let format = CuImageBufferFormat {
            width,
            height,
            stride: width,
            pixel_format: *b"GRAY",
        };
        let buffer_len = format.byte_size();
        let pool = CuHostMemoryPool::new(&pool_id, pool_size, || vec![0u8; buffer_len])?;

        Ok(Self {
            pool,
            format,
            seq: 0,
            base_value,
            tov_offset: CuDuration::from_millis(tov_offset_ms as u64),
        })
    }

    fn process(&mut self, clock: &RobotClock, output: &mut Self::Output<'_>) -> CuResult<()> {
        let handle = self
            .pool
            .acquire()
            .ok_or(CuError::from("Failed to acquire buffer from pool"))?;
        {
            let mut guard = handle
                .lock()
                .map_err(|e| CuError::from("Failed to lock buffer").add_cause(&e.to_string()))?;
            let buffer = guard.deref_mut().deref_mut();
            let value = self.base_value.wrapping_add((self.seq % 64) as u8);
            buffer.fill(value);
        }

        let mut image = CuImage::new(self.format, handle);
        image.seq = self.seq;
        output.tov = Tov::Time(clock.now() + self.tov_offset);
        output.set_payload(image);
        self.seq = self.seq.wrapping_add(1);
        Ok(())
    }
}

pub struct AlignedImageSink {}

impl Freezable for AlignedImageSink {}

impl CuSinkTask for AlignedImageSink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!((
        CuArray<CuImage<Vec<u8>>, OUTPUT_CAP>,
        CuArray<CuImage<Vec<u8>>, OUTPUT_CAP>
    ));

    fn new(
        _config: Option<&ComponentConfig>,
        _resources: Self::Resources<'_>,
    ) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {})
    }

    fn process(&mut self, _clock: &RobotClock, input: &Self::Input<'_>) -> CuResult<()> {
        let payload = match input.payload() {
            Some(payload) => payload,
            None => return Ok(()),
        };
        let (left, right) = payload;
        let left_len = left.len();
        let right_len = right.len();
        let left_img = left.as_slice().first();
        let right_img = right.as_slice().first();

        if let (Some(left_img), Some(right_img)) = (left_img, right_img) {
            let left_sample = sample_pixel(left_img);
            let right_sample = sample_pixel(right_img);
            println!(
                "Aligned window: left_len={} right_len={} left_seq={} left_pixel0={} right_seq={} right_pixel0={}",
                left_len, right_len, left_img.seq, left_sample, right_img.seq, right_sample
            );
        } else {
            println!(
                "Aligned window: left_len={} right_len={}",
                left_len, right_len
            );
        }

        Ok(())
    }
}

fn sample_pixel(img: &CuImage<Vec<u8>>) -> u8 {
    img.buffer_handle
        .with_inner(|inner| inner.first().copied().unwrap_or(0))
}
