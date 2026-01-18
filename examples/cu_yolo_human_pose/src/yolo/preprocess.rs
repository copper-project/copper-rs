//! Image preprocessing for YOLOv8 inference
//!
//! Handles conversion from V4L camera formats (YUYV, NV12, etc.) to
//! the RGB tensor format expected by the model.

use candle_core::{DType, Device, Result, Tensor};
use cu_sensor_payloads::CuImage;

/// Target input size for the model (width and height)
pub const MODEL_INPUT_SIZE: usize = 640;

/// Preprocess a CuImage for YOLO inference
///
/// Steps:
/// 1. Convert from camera format (YUYV/NV12) to RGB
/// 2. Resize to model input size with letterboxing
/// 3. Normalize to [0, 1] range
/// 4. Convert to tensor with shape (1, 3, H, W)
///
/// Returns the tensor and the actual (width, height) used after scaling
pub fn preprocess_image(
    image: &CuImage<Vec<u8>>,
    device: &Device,
) -> Result<(Tensor, usize, usize)> {
    let width = image.format.width as usize;
    let height = image.format.height as usize;
    let pixel_format = &image.format.pixel_format;

    // Convert to RGB based on pixel format
    let rgb_data = image.buffer_handle.with_inner(|data| {
        match pixel_format {
            b"YUYV" => yuyv_to_rgb(data, width, height),
            b"NV12" => nv12_to_rgb(data, width, height),
            b"RGB3" | b"RGB " => data.to_vec(), // Already RGB
            b"BGR3" | b"BGR " => bgr_to_rgb(data, width, height),
            _ => {
                // Fallback: assume grayscale or try to interpret as RGB
                if data.len() == width * height * 3 {
                    data.to_vec()
                } else if data.len() == width * height {
                    // Grayscale to RGB
                    data.iter().flat_map(|&g| [g, g, g]).collect()
                } else {
                    // Unknown format, return zeros
                    vec![0u8; width * height * 3]
                }
            }
        }
    });

    // Calculate resize dimensions maintaining aspect ratio
    let (target_w, target_h) = calculate_resize_dims(width, height, MODEL_INPUT_SIZE);

    // Resize the image
    let resized = resize_rgb(&rgb_data, width, height, target_w, target_h);

    // Create tensor: (H, W, 3) -> (3, H, W) -> (1, 3, H, W)
    let tensor = Tensor::from_vec(resized, (target_h, target_w, 3), device)?
        .permute((2, 0, 1))?
        .unsqueeze(0)?
        .to_dtype(DType::F32)?;

    // Normalize to [0, 1]
    let tensor = (tensor * (1.0 / 255.0))?;

    Ok((tensor, target_w, target_h))
}

/// Calculate resize dimensions to fit within target size while maintaining aspect ratio
/// Dimensions are rounded down to multiples of 32 for YOLO compatibility
fn calculate_resize_dims(width: usize, height: usize, target: usize) -> (usize, usize) {
    if width > height {
        let new_h = (height * target) / width;
        (target, (new_h / 32) * 32)
    } else {
        let new_w = (width * target) / height;
        ((new_w / 32) * 32, target)
    }
}

/// Convert YUYV (YUV 4:2:2) to RGB
fn yuyv_to_rgb(data: &[u8], width: usize, height: usize) -> Vec<u8> {
    let mut rgb = Vec::with_capacity(width * height * 3);

    for chunk in data.chunks(4) {
        if chunk.len() < 4 {
            break;
        }
        let y0 = chunk[0] as f32;
        let u = chunk[1] as f32 - 128.0;
        let y1 = chunk[2] as f32;
        let v = chunk[3] as f32 - 128.0;

        // First pixel
        let r0 = (y0 + 1.402 * v).clamp(0.0, 255.0) as u8;
        let g0 = (y0 - 0.344 * u - 0.714 * v).clamp(0.0, 255.0) as u8;
        let b0 = (y0 + 1.772 * u).clamp(0.0, 255.0) as u8;
        rgb.extend_from_slice(&[r0, g0, b0]);

        // Second pixel
        let r1 = (y1 + 1.402 * v).clamp(0.0, 255.0) as u8;
        let g1 = (y1 - 0.344 * u - 0.714 * v).clamp(0.0, 255.0) as u8;
        let b1 = (y1 + 1.772 * u).clamp(0.0, 255.0) as u8;
        rgb.extend_from_slice(&[r1, g1, b1]);
    }

    rgb
}

/// Convert NV12 (YUV 4:2:0) to RGB
fn nv12_to_rgb(data: &[u8], width: usize, height: usize) -> Vec<u8> {
    let mut rgb = Vec::with_capacity(width * height * 3);
    let y_plane = &data[..width * height];
    let uv_plane = &data[width * height..];

    for y_idx in 0..height {
        for x_idx in 0..width {
            let y = y_plane[y_idx * width + x_idx] as f32;
            let uv_idx = (y_idx / 2) * width + (x_idx / 2) * 2;
            let u = uv_plane.get(uv_idx).copied().unwrap_or(128) as f32 - 128.0;
            let v = uv_plane.get(uv_idx + 1).copied().unwrap_or(128) as f32 - 128.0;

            let r = (y + 1.402 * v).clamp(0.0, 255.0) as u8;
            let g = (y - 0.344 * u - 0.714 * v).clamp(0.0, 255.0) as u8;
            let b = (y + 1.772 * u).clamp(0.0, 255.0) as u8;
            rgb.extend_from_slice(&[r, g, b]);
        }
    }

    rgb
}

/// Convert BGR to RGB
fn bgr_to_rgb(data: &[u8], _width: usize, _height: usize) -> Vec<u8> {
    data.chunks(3)
        .flat_map(|bgr| {
            if bgr.len() >= 3 {
                [bgr[2], bgr[1], bgr[0]]
            } else {
                [0, 0, 0]
            }
        })
        .collect()
}

/// Simple bilinear resize for RGB images
fn resize_rgb(data: &[u8], src_w: usize, src_h: usize, dst_w: usize, dst_h: usize) -> Vec<u8> {
    let mut result = Vec::with_capacity(dst_w * dst_h * 3);

    let x_ratio = src_w as f32 / dst_w as f32;
    let y_ratio = src_h as f32 / dst_h as f32;

    for y in 0..dst_h {
        for x in 0..dst_w {
            let src_x = x as f32 * x_ratio;
            let src_y = y as f32 * y_ratio;

            let x0 = src_x.floor() as usize;
            let y0 = src_y.floor() as usize;
            let x1 = (x0 + 1).min(src_w - 1);
            let y1 = (y0 + 1).min(src_h - 1);

            let x_frac = src_x - x0 as f32;
            let y_frac = src_y - y0 as f32;

            for c in 0..3 {
                let p00 = data[(y0 * src_w + x0) * 3 + c] as f32;
                let p01 = data[(y0 * src_w + x1) * 3 + c] as f32;
                let p10 = data[(y1 * src_w + x0) * 3 + c] as f32;
                let p11 = data[(y1 * src_w + x1) * 3 + c] as f32;

                let top = p00 * (1.0 - x_frac) + p01 * x_frac;
                let bottom = p10 * (1.0 - x_frac) + p11 * x_frac;
                let value = top * (1.0 - y_frac) + bottom * y_frac;

                result.push(value.clamp(0.0, 255.0) as u8);
            }
        }
    }

    result
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_resize_dims() {
        // Landscape image
        let (w, h) = calculate_resize_dims(1920, 1080, 640);
        assert_eq!(w, 640);
        assert!(h <= 640);
        assert_eq!(h % 32, 0);

        // Portrait image
        let (w, h) = calculate_resize_dims(1080, 1920, 640);
        assert!(w <= 640);
        assert_eq!(h, 640);
        assert_eq!(w % 32, 0);

        // Square image
        let (w, h) = calculate_resize_dims(640, 640, 640);
        assert_eq!(w, 640);
        assert_eq!(h, 640);
    }
}
