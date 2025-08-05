#![cfg_attr(target_arch = "spirv", no_std)]
#![allow(unused_attributes)]

use spirv_std::glam::{Vec2, UVec3};
use spirv_std::num_traits::Float;

pub const MAX_VERTS: usize = 64;

#[repr(C)]
pub struct Params {
    pub polygon_len: u32,
}

#[allow(clippy::needless_range_loop)]
#[spirv(compute(threads(64)))]
pub fn point_in_polygon_kernel(
    #[spirv(global_invocation_id)] id: UVec3,
    #[spirv(storage_buffer, descriptor_set = 0, binding = 0)] polygon: &[Vec2],
    #[spirv(storage_buffer, descriptor_set = 0, binding = 1)] points: &[Vec2],
    #[spirv(storage_buffer, descriptor_set = 0, binding = 2)] results: &mut [u32],
    #[spirv(push_constant)] params: &Params,
) {
    let idx = id.x as usize;
    if idx >= points.len() {
        return;
    }
    let p = points[idx];
    let len = params.polygon_len as usize;
    if len < 3 { return; }
    let mut prev = 0.0f32;
    let mut inside = true;
    let mut i = 0;
    while i < len {
        let a = polygon[i];
        let b = polygon[(i + 1) % len];
        let edge = b - a;
        let to_point = p - a;
        let cross = edge.x * to_point.y - edge.y * to_point.x;
        if i == 0 {
            prev = cross;
        } else if cross * prev < 0.0 {
            inside = false;
            break;
        }
        i += 1;
    }
    results[idx] = inside as u32;
}
