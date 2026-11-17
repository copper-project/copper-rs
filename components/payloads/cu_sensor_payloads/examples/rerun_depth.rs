//! Record a compact metric depth image through Copper's debug representation.
use cu_sensor_payloads::{CuDepthInteger, CuDepthMap, CuDepthMapFormat, CuDepthMillimeter};
use cu29::debug_representation::{DebugRepresentation, DebugRepresentationContext};
use cu29::prelude::*;
use cu29::reflect::PartialReflect;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let format = CuDepthMapFormat {
        width: 640,
        height: 360,
        stride: 656,
    };
    let mut samples = vec![0_u16; format.required_elements()];
    for y in 0..format.height as usize {
        for x in 0..format.width as usize {
            let dx = x as f32 - 320.0;
            let dy = y as f32 - 180.0;
            let distance = (dx * dx + dy * dy).sqrt();
            samples[y * format.stride as usize + x] = if distance < 70.0 {
                800
            } else {
                1500 + (x * 4 + y * 3) as u16
            };
        }
    }
    let depth = CuDepthMap::<_, CuDepthInteger<u16, CuDepthMillimeter>>::from_integer(
        format,
        CuHandle::new_detached(samples),
    );
    let info = depth
        .get_represented_type_info()
        .ok_or("missing type info")?;
    let representation =
        DebugRepresentation::for_type(info).ok_or("missing depth representation")?;
    let data = (representation.encode)(
        &depth,
        &DebugRepresentationContext {
            entity_path: "camera/depth",
            culistid: 1,
            time_ns: 1_000_000_000,
        },
    )?;
    let output = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "depth.rrd".into());
    std::fs::write(output, data)?;
    Ok(())
}
