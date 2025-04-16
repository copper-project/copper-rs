use cu29::prelude::*;
use cu_gstreamer::CuGstBuffer;
use cu_sensor_payloads::{CuImage, CuImageBufferFormat};
use std::cmp::{max, min};
use std::ops::DerefMut;
use std::sync::Arc;

pub trait PixelReadAccess<U> {
    fn get_pixel(&self, x: usize, y: usize, width: usize) -> U;
}

pub trait PixelWriteAccess<U> {
    fn put_pixel(&mut self, x: usize, y: usize, width: usize, value: U);
}

impl<U: Copy, T: AsRef<[U]>> PixelReadAccess<U> for T {
    #[inline]
    fn get_pixel(&self, x: usize, y: usize, width: usize) -> U {
        unsafe {
            let slice = self.as_ref();
            *slice.get_unchecked(x + y * width)
        }
    }
}

impl<U: Copy, T: AsMut<[U]>> PixelWriteAccess<U> for T {
    #[inline]
    fn put_pixel(&mut self, x: usize, y: usize, width: usize, value: U) {
        unsafe {
            *self.as_mut().get_unchecked_mut(x + y * width) = value;
        }
    }
}

fn integral_image(src: &[u8], mut dst: &mut [u32], width: u32, height: u32) {
    let (width, height) = (width as usize, height as usize);
    let out_width = width + 1;

    for y in 0..height {
        let mut sum = 0;
        for x in 0..width {
            sum += src.get_pixel(x, y, width) as u32;

            let above = dst.get_pixel(x + 1, y, out_width);
            dst.put_pixel(x + 1, y + 1, out_width, above + sum);
        }
    }
}

fn sum_image_pixels(
    integral_image: &[u32],
    left: usize,
    top: usize,
    right: usize,
    bottom: usize,
    width: usize,
) -> u32 {
    let (a, b, c, d) = (
        integral_image.get_pixel(right + 1, bottom + 1, width) as i64,
        integral_image.get_pixel(left, top, width) as i64,
        integral_image.get_pixel(right + 1, top, width) as i64,
        integral_image.get_pixel(left, bottom + 1, width) as i64,
    );
    let sum = a + b - c - d;
    assert!(sum >= 0);
    sum as u32
}

fn adaptive_threshold(
    src: &[u8],
    integral: &[u32],
    mut dst: &mut [u8],
    block_radius: u32,
    width: usize,
    height: usize,
) {
    assert!(block_radius > 0);

    for y in 0..height {
        for x in 0..width {
            let current_pixel = src.get_pixel(x, y, width) as u32;

            // Traverse all neighbors in (2 * block_radius + 1) x (2 * block_radius + 1)
            let (y_low, y_high) = (
                max(0, y as i32 - block_radius as i32) as usize,
                min(height - 1, y + block_radius as usize),
            );
            let (x_low, x_high) = (
                max(0, x as i32 - block_radius as i32) as usize,
                min(width - 1, x + block_radius as usize),
            );

            // Number of pixels in the block, adjusted for edge cases.
            let w = ((y_high - y_low + 1) * (x_high - x_low + 1)) as u32;
            let sum = sum_image_pixels(integral, x_low, y_low, x_high, y_high, width + 1);

            if current_pixel * w > sum {
                dst.put_pixel(x, y, width, 255);
            } else {
                dst.put_pixel(x, y, width, 0);
            }
        }
    }
}

/// A task that computes a dynamic threshold for a grayscale image.
pub struct DynThreshold {
    integral_img: Vec<u32>,
    pool: Arc<CuHostMemoryPool<Vec<u8>>>,
    height: u32,
    width: u32,
    block_radius: u32,
}

impl Freezable for DynThreshold {}

impl<'cl> CuTask<'cl> for DynThreshold {
    type Input = input_msg!('cl, CuGstBuffer);
    type Output = output_msg!('cl, CuImage<Vec<u8>>);

    fn new(config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let config = config.expect("No config provided");
        let width = config.get::<u32>("width").expect("No width provided");
        let height = config.get::<u32>("height").expect("No height provided");
        let block_radius = config
            .get::<u32>("block_radius")
            .expect("No block_radius provided");

        let pool =
            CuHostMemoryPool::new("dynthreshold", 3, || vec![0u8; (width * height) as usize])?;
        Ok(DynThreshold {
            integral_img: vec![0; ((width + 1) * (height + 1)) as usize],
            pool,
            width,
            height,
            block_radius,
        })
    }

    fn process(
        &mut self,
        _clock: &RobotClock,
        input: Self::Input,
        output: Self::Output,
    ) -> CuResult<()> {
        if input.payload().is_none() {
            debug!("DynThreshold: No payload in input message, skipping.");
            return Ok(());
        }
        let buffer_hold = input.payload().ok_or(CuError::from("No payload"))?;
        let buffer_hold = buffer_hold
            .as_ref()
            .map_readable()
            .map_err(|e| CuError::new_with_cause("Could not map the gstreamer buffer", e))?;
        let src = buffer_hold.as_slice();

        if src.len() != (self.width * self.height) as usize {
            return Err(CuError::from(format!(
                "Input buffer size does not match the expected size {}, slice {}",
                self.width * self.height,
                src.len(),
            )));
        }

        let handle = self
            .pool
            .acquire()
            .ok_or(CuError::from("Failed to acquire buffer from pool"))?;
        {
            let mut dst = handle
                .lock()
                .map_err(|e| CuError::new_with_cause("Failed to lock buffer", e))?;
            let dst = dst.deref_mut().deref_mut();

            integral_image(src, &mut self.integral_img, self.width, self.height);
            adaptive_threshold(
                src,
                &self.integral_img,
                dst,
                self.block_radius,
                self.width as usize,
                self.height as usize,
            );
        }
        let image = CuImage::new(
            CuImageBufferFormat {
                width: self.width,
                height: self.height,
                stride: self.width,
                pixel_format: "GRAY"
                    .as_bytes()
                    .try_into()
                    .map_err(|_| CuError::from("Failed to convert pixel format to byte array"))?,
            },
            handle,
        );
        output.metadata.tov = input.metadata.tov;
        output.set_payload(image);
        Ok(())
    }
}

#[cfg(test)]
#[cfg(feature = "gst")]
mod tests {
    use crate::DynThreshold;
    use cu29::prelude::*;
    use cu_gstreamer::CuGstBuffer;
    use cu_sensor_payloads::CuImage;
    use gstreamer::Buffer;
    use std::ops::Deref;

    #[test]
    fn test_dynthreshold_bicolor() -> Result<(), Box<dyn std::error::Error>> {
        let width = 4;
        let height = 4;
        let block_radius = 2;
        gstreamer::init().unwrap();

        let mut config = ComponentConfig::default();
        config.set("width", width as u32);
        config.set("height", height as u32);
        config.set("block_radius", block_radius as u32);

        let mut dynthresh = DynThreshold::new(Some(&config))?;

        let input_data = vec![
            128, 128, 130, 130, // L1
            128, 128, 130, 130, // L2
            128, 128, 130, 130, // L3
            128, 128, 130, 130, // L4
        ];

        // Create a new GStreamer buffer and fill it with input data
        let gstreamer_buffer = Buffer::from_mut_slice(input_data.clone());
        let cu_gst_buffer = CuGstBuffer(gstreamer_buffer);
        let input_msg = CuMsg::new(Some(cu_gst_buffer));
        let mut output_image_msg = CuMsg::<CuImage<Vec<u8>>>::default();
        let output: <DynThreshold as CuTask>::Output = &mut output_image_msg;

        let clock = cu29::clock::RobotClock::new();
        let result = dynthresh.process(&clock, &input_msg, output);
        assert!(result.is_ok());

        let output_image = output_image_msg.payload().unwrap();
        let hold = output_image.buffer_handle.lock().unwrap();
        let output_data = hold.deref().deref();

        let expected_output = vec![
            0, 0, 255, 255, // L1
            0, 0, 255, 255, // L2
            0, 0, 255, 255, // L3
            0, 0, 255, 255, // L4
        ];

        assert_eq!(output_data, expected_output);
        Ok(())
    }
}
