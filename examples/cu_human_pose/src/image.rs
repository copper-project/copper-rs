//! Shared image format conversion helpers.

/// Convert YUYV (YUV 4:2:2) to RGB.
pub fn yuyv_to_rgb(data: &[u8], width: usize, height: usize) -> Vec<u8> {
    let mut rgb = Vec::with_capacity(width * height * 3);

    for chunk in data.chunks(4) {
        if chunk.len() < 4 {
            break;
        }
        let y0 = chunk[0] as f32;
        let u = chunk[1] as f32 - 128.0;
        let y1 = chunk[2] as f32;
        let v = chunk[3] as f32 - 128.0;

        let r0 = (y0 + 1.402 * v).clamp(0.0, 255.0) as u8;
        let g0 = (y0 - 0.344 * u - 0.714 * v).clamp(0.0, 255.0) as u8;
        let b0 = (y0 + 1.772 * u).clamp(0.0, 255.0) as u8;
        rgb.extend_from_slice(&[r0, g0, b0]);

        let r1 = (y1 + 1.402 * v).clamp(0.0, 255.0) as u8;
        let g1 = (y1 - 0.344 * u - 0.714 * v).clamp(0.0, 255.0) as u8;
        let b1 = (y1 + 1.772 * u).clamp(0.0, 255.0) as u8;
        rgb.extend_from_slice(&[r1, g1, b1]);
    }

    rgb
}

/// Convert NV12 (YUV 4:2:0) to RGB.
pub fn nv12_to_rgb(data: &[u8], width: usize, height: usize) -> Vec<u8> {
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

/// Convert BGR to RGB.
pub fn bgr_to_rgb(data: &[u8], _width: usize, _height: usize) -> Vec<u8> {
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
