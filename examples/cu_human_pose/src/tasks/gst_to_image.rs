//! Convert a GStreamer buffer into a CuImage for downstream tasks.

use cu_gstreamer::CuGstBuffer;
use cu_sensor_payloads::{CuImage, CuImageBufferFormat};
use cu29::prelude::*;
use std::ops::DerefMut;
use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};

static GST_LOG_COUNT: AtomicUsize = AtomicUsize::new(0);

pub struct GstToCuImage {
    width: u32,
    height: u32,
    stride: u32,
    pixel_format: [u8; 4],
    pool: Arc<CuHostMemoryPool<Vec<u8>>>,
    last_payload_ns: Option<u64>,
    last_warn_ns: u64,
}

impl Freezable for GstToCuImage {}

impl CuTask for GstToCuImage {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(CuGstBuffer);
    type Output<'m> = output_msg!(CuImage<Vec<u8>>);

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let config = config.ok_or_else(|| CuError::from("GstToCuImage requires configuration"))?;

        let width = config
            .get::<u32>("width")?
            .ok_or_else(|| CuError::from("GstToCuImage requires width"))?;
        let height = config
            .get::<u32>("height")?
            .ok_or_else(|| CuError::from("GstToCuImage requires height"))?;
        let fourcc = if let Some(fourcc) = config.get::<String>("fourcc")? {
            fourcc
        } else if let Some(pixel_format) = config.get::<String>("pixel_format")? {
            pixel_format
        } else {
            return Err(CuError::from(
                "GstToCuImage requires fourcc (or pixel_format)",
            ));
        };
        let pixel_format = fourcc_to_bytes(&fourcc)?;
        let stride = config
            .get::<u32>("stride")?
            .unwrap_or_else(|| default_stride(width, pixel_format));

        // Calculate buffer size based on pixel format
        let buffer_size = compute_buffer_size(height, stride, pixel_format);
        let pool_size = config.get::<u32>("pool_size")?.unwrap_or(4) as usize;
        let pool_id = config
            .get::<String>("pool_id")?
            .unwrap_or_else(|| "gst_image_pool".to_string());

        let pool = CuHostMemoryPool::new(&pool_id, pool_size, || vec![0u8; buffer_size])?;

        Ok(Self {
            width,
            height,
            stride,
            pixel_format,
            pool,
            last_payload_ns: None,
            last_warn_ns: 0,
        })
    }

    fn process(
        &mut self,
        clock: &RobotClock,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        let now_ns = clock.now().as_nanos();
        let Some(buffer) = input.payload() else {
            const WARN_AFTER_NS: u64 = 2_000_000_000;
            const WARN_EVERY_NS: u64 = 2_000_000_000;
            let last_payload_ns = self.last_payload_ns.unwrap_or(0);
            if now_ns.saturating_sub(last_payload_ns) >= WARN_AFTER_NS
                && now_ns.saturating_sub(self.last_warn_ns) >= WARN_EVERY_NS
            {
                warning!(
                    "gst_to_image: no frames received for {} ms",
                    now_ns.saturating_sub(last_payload_ns) / 1_000_000
                );
                self.last_warn_ns = now_ns;
            }
            output.clear_payload();
            return Ok(());
        };
        self.last_payload_ns = Some(now_ns);

        let mapped = buffer
            .as_ref()
            .map_readable()
            .map_err(|e| CuError::new_with_cause("Failed to map GStreamer buffer", e))?;
        let src = mapped.as_slice();
        let min_len = (self.stride * self.height) as usize;
        let log_idx = GST_LOG_COUNT.fetch_add(1, Ordering::Relaxed);
        if log_idx < 5 {
            info!(
                "gst_to_image: buf_len={} min_len={} width={} height={} stride={} fourcc={}",
                src.len(),
                min_len,
                self.width,
                self.height,
                self.stride,
                String::from_utf8_lossy(&self.pixel_format)
            );
        }
        if src.len() < min_len {
            return Err(CuError::from(format!(
                "GStreamer buffer too small: expected >= {}, got {} bytes",
                min_len,
                src.len()
            )));
        }

        // Acquire a buffer from the pool instead of allocating
        let handle = self
            .pool
            .acquire()
            .ok_or_else(|| CuError::from("Failed to acquire buffer from image pool"))?;

        // Copy the source data into the pooled buffer
        handle.with_inner_mut(|inner| {
            let dest = inner.deref_mut();
            let copy_len = src.len().min(dest.len());
            dest[..copy_len].copy_from_slice(&src[..copy_len]);
        });

        let image = CuImage::new(
            CuImageBufferFormat {
                width: self.width,
                height: self.height,
                stride: self.stride,
                pixel_format: self.pixel_format,
            },
            handle,
        );
        output.tov = input.tov;
        output.set_payload(image);
        Ok(())
    }
}

fn default_stride(width: u32, pixel_format: [u8; 4]) -> u32 {
    match &pixel_format {
        b"YUYV" => width * 2,
        b"BGR3" | b"RGB3" | b"BGR " | b"RGB " => width * 3,
        _ => width,
    }
}

fn fourcc_to_bytes(fourcc: &str) -> CuResult<[u8; 4]> {
    let bytes = fourcc.as_bytes();
    if bytes.len() != 4 {
        return Err(CuError::from("fourcc must be exactly 4 characters"));
    }
    let mut out = [0u8; 4];
    out.copy_from_slice(&bytes[0..4]);
    Ok(out)
}

/// Compute the buffer size in bytes for a given image format.
fn compute_buffer_size(height: u32, stride: u32, pixel_format: [u8; 4]) -> usize {
    match &pixel_format {
        // NV12/NV21: Y plane (stride * height) + UV plane (stride * height/2)
        b"NV12" | b"NV21" => (stride * height + stride * height / 2) as usize,
        // I420/YV12: Y plane + U plane (stride/2 * height/2) + V plane (stride/2 * height/2)
        b"I420" | b"YV12" => (stride * height + stride * height / 2) as usize,
        // YUYV/UYVY: 2 bytes per pixel, packed
        b"YUYV" | b"UYVY" => (stride * height) as usize,
        // RGB/BGR 24-bit
        b"BGR3" | b"RGB3" | b"BGR " | b"RGB " => (stride * height) as usize,
        // RGBA/BGRA 32-bit
        b"RGBA" | b"BGRA" => (stride * height) as usize,
        // Grayscale
        b"GRAY" | b"Y800" => (stride * height) as usize,
        // Default fallback: assume stride already accounts for bytes per row
        _ => (stride * height) as usize,
    }
}
