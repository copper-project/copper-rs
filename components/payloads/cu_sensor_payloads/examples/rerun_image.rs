//! Record a padded RGB image through Copper's debug representation.
use cu_sensor_payloads::{CuImage, CuImageBufferFormat};
use cu29::debug_representation::{DebugRepresentation, DebugRepresentationContext};
use cu29::prelude::*;
use cu29::reflect::PartialReflect;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let format = CuImageBufferFormat {
        width: 640,
        height: 360,
        stride: 1936,
        pixel_format: *b"RGB3",
    };
    let mut bytes = vec![255_u8; format.required_bytes()];
    for y in 0..format.height as usize {
        for x in 0..format.width as usize {
            let offset = y * format.stride as usize + x * 3;
            bytes[offset..offset + 3].copy_from_slice(&[
                (x * 255 / 639) as u8,
                (y * 255 / 359) as u8,
                if (x / 40 + y / 40) % 2 == 0 { 210 } else { 60 },
            ]);
        }
    }
    let image = CuImage::new(format, CuHandle::new_detached(bytes));
    let info = image
        .get_represented_type_info()
        .ok_or("missing type info")?;
    let representation =
        DebugRepresentation::for_type(info).ok_or("missing image representation")?;
    let data = (representation.encode)(
        &image,
        &DebugRepresentationContext {
            entity_path: "camera/image",
            culistid: 1,
            time_ns: 1_000_000_000,
        },
    )?;
    let output = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "image.rrd".into());
    std::fs::write(output, data)?;
    Ok(())
}
