//! Rerun geometry and debugger views for standard coordinate types.
use crate::geometry::*;
use alloc::vec::Vec;
use core::any::TypeId;
use cu_rerun::views::{Spatial2DView, Spatial3DView};
use cu29::debug_representation::DebugRepresentation;
use cu29::units::si::f32::Length as Length32;
use cu29::units::si::f64::Length as Length64;
use cu29::units::si::length::meter;
use rerun::{AsComponents, SerializedComponentBatch};
impl AsComponents for Point2<Length32> {
    fn as_serialized_batches(&self) -> Vec<SerializedComponentBatch> {
        let convert = |v: Length32| v.get::<meter>();
        rerun::Points2D::new([[convert(self.x), convert(self.y)]]).as_serialized_batches()
    }
}
impl<const N: usize> AsComponents for Point2Soa<Length32, N> {
    fn as_serialized_batches(&self) -> Vec<SerializedComponentBatch> {
        let convert = |v: Length32| v.get::<meter>();
        rerun::Points2D::new((0..self.len.min(N)).map(|i| [convert(self.x[i]), convert(self.y[i])]))
            .as_serialized_batches()
    }
}
impl AsComponents for BBox<Point2<Length32>> {
    fn as_serialized_batches(&self) -> Vec<SerializedComponentBatch> {
        let convert = |v: Length32| v.get::<meter>();
        rerun::Boxes2D::from_centers_and_half_sizes(
            [[
                (convert(self.min.x) + convert(self.max.x)) * 0.5,
                (convert(self.min.y) + convert(self.max.y)) * 0.5,
            ]],
            [[
                (convert(self.max.x) - convert(self.min.x)) * 0.5,
                (convert(self.max.y) - convert(self.min.y)) * 0.5,
            ]],
        )
        .as_serialized_batches()
    }
}
impl AsComponents for Point3<Length32> {
    fn as_serialized_batches(&self) -> Vec<SerializedComponentBatch> {
        let convert = |v: Length32| v.get::<meter>();
        rerun::Points3D::new([[convert(self.x), convert(self.y), convert(self.z)]])
            .as_serialized_batches()
    }
}
impl<const N: usize> AsComponents for Point3Soa<Length32, N> {
    fn as_serialized_batches(&self) -> Vec<SerializedComponentBatch> {
        let convert = |v: Length32| v.get::<meter>();
        rerun::Points3D::new(
            (0..self.len.min(N))
                .map(|i| [convert(self.x[i]), convert(self.y[i]), convert(self.z[i])]),
        )
        .as_serialized_batches()
    }
}
impl AsComponents for BBox<Point3<Length32>> {
    fn as_serialized_batches(&self) -> Vec<SerializedComponentBatch> {
        let convert = |v: Length32| v.get::<meter>();
        rerun::Boxes3D::from_centers_and_half_sizes(
            [[
                (convert(self.min.x) + convert(self.max.x)) * 0.5,
                (convert(self.min.y) + convert(self.max.y)) * 0.5,
                (convert(self.min.z) + convert(self.max.z)) * 0.5,
            ]],
            [[
                (convert(self.max.x) - convert(self.min.x)) * 0.5,
                (convert(self.max.y) - convert(self.min.y)) * 0.5,
                (convert(self.max.z) - convert(self.min.z)) * 0.5,
            ]],
        )
        .as_serialized_batches()
    }
}
impl AsComponents for Point2<Length64> {
    fn as_serialized_batches(&self) -> Vec<SerializedComponentBatch> {
        let convert = |v: Length64| v.get::<meter>() as f32;
        rerun::Points2D::new([[convert(self.x), convert(self.y)]]).as_serialized_batches()
    }
}
impl<const N: usize> AsComponents for Point2Soa<Length64, N> {
    fn as_serialized_batches(&self) -> Vec<SerializedComponentBatch> {
        let convert = |v: Length64| v.get::<meter>() as f32;
        rerun::Points2D::new((0..self.len.min(N)).map(|i| [convert(self.x[i]), convert(self.y[i])]))
            .as_serialized_batches()
    }
}
impl AsComponents for BBox<Point2<Length64>> {
    fn as_serialized_batches(&self) -> Vec<SerializedComponentBatch> {
        let convert = |v: Length64| v.get::<meter>();
        rerun::Boxes2D::from_centers_and_half_sizes(
            [[
                ((convert(self.min.x) + convert(self.max.x)) * 0.5) as f32,
                ((convert(self.min.y) + convert(self.max.y)) * 0.5) as f32,
            ]],
            [[
                ((convert(self.max.x) - convert(self.min.x)) * 0.5) as f32,
                ((convert(self.max.y) - convert(self.min.y)) * 0.5) as f32,
            ]],
        )
        .as_serialized_batches()
    }
}
impl AsComponents for Point3<Length64> {
    fn as_serialized_batches(&self) -> Vec<SerializedComponentBatch> {
        let convert = |v: Length64| v.get::<meter>() as f32;
        rerun::Points3D::new([[convert(self.x), convert(self.y), convert(self.z)]])
            .as_serialized_batches()
    }
}
impl<const N: usize> AsComponents for Point3Soa<Length64, N> {
    fn as_serialized_batches(&self) -> Vec<SerializedComponentBatch> {
        let convert = |v: Length64| v.get::<meter>() as f32;
        rerun::Points3D::new(
            (0..self.len.min(N))
                .map(|i| [convert(self.x[i]), convert(self.y[i]), convert(self.z[i])]),
        )
        .as_serialized_batches()
    }
}
impl AsComponents for BBox<Point3<Length64>> {
    fn as_serialized_batches(&self) -> Vec<SerializedComponentBatch> {
        let convert = |v: Length64| v.get::<meter>();
        rerun::Boxes3D::from_centers_and_half_sizes(
            [[
                ((convert(self.min.x) + convert(self.max.x)) * 0.5) as f32,
                ((convert(self.min.y) + convert(self.max.y)) * 0.5) as f32,
                ((convert(self.min.z) + convert(self.max.z)) * 0.5) as f32,
            ]],
            [[
                ((convert(self.max.x) - convert(self.min.x)) * 0.5) as f32,
                ((convert(self.max.y) - convert(self.min.y)) * 0.5) as f32,
                ((convert(self.max.z) - convert(self.min.z)) * 0.5) as f32,
            ]],
        )
        .as_serialized_batches()
    }
}
impl AsComponents for Point2<u32> {
    fn as_serialized_batches(&self) -> Vec<SerializedComponentBatch> {
        let convert = |v: u32| v as f32;
        rerun::Points2D::new([[convert(self.x), convert(self.y)]]).as_serialized_batches()
    }
}
impl<const N: usize> AsComponents for Point2Soa<u32, N> {
    fn as_serialized_batches(&self) -> Vec<SerializedComponentBatch> {
        let convert = |v: u32| v as f32;
        rerun::Points2D::new((0..self.len.min(N)).map(|i| [convert(self.x[i]), convert(self.y[i])]))
            .as_serialized_batches()
    }
}
impl AsComponents for BBox<Point2<u32>> {
    fn as_serialized_batches(&self) -> Vec<SerializedComponentBatch> {
        let convert = |v: u32| v as f32;
        rerun::Boxes2D::from_centers_and_half_sizes(
            [[
                (convert(self.min.x) + convert(self.max.x)) * 0.5,
                (convert(self.min.y) + convert(self.max.y)) * 0.5,
            ]],
            [[
                (convert(self.max.x) - convert(self.min.x)) * 0.5,
                (convert(self.max.y) - convert(self.min.y)) * 0.5,
            ]],
        )
        .as_serialized_batches()
    }
}
impl AsComponents for Point2<i32> {
    fn as_serialized_batches(&self) -> Vec<SerializedComponentBatch> {
        let convert = |v: i32| v as f32;
        rerun::Points2D::new([[convert(self.x), convert(self.y)]]).as_serialized_batches()
    }
}
impl<const N: usize> AsComponents for Point2Soa<i32, N> {
    fn as_serialized_batches(&self) -> Vec<SerializedComponentBatch> {
        let convert = |v: i32| v as f32;
        rerun::Points2D::new((0..self.len.min(N)).map(|i| [convert(self.x[i]), convert(self.y[i])]))
            .as_serialized_batches()
    }
}
impl AsComponents for BBox<Point2<i32>> {
    fn as_serialized_batches(&self) -> Vec<SerializedComponentBatch> {
        let convert = |v: i32| v as f32;
        rerun::Boxes2D::from_centers_and_half_sizes(
            [[
                (convert(self.min.x) + convert(self.max.x)) * 0.5,
                (convert(self.min.y) + convert(self.max.y)) * 0.5,
            ]],
            [[
                (convert(self.max.x) - convert(self.min.x)) * 0.5,
                (convert(self.max.y) - convert(self.min.y)) * 0.5,
            ]],
        )
        .as_serialized_batches()
    }
}
pub(crate) fn point2_representation<L: 'static>() -> Option<DebugRepresentation> {
    if TypeId::of::<L>() == TypeId::of::<Length32>() {
        return Some(cu_rerun::representation::<Point2<Length32>, Spatial2DView>());
    }
    if TypeId::of::<L>() == TypeId::of::<Length64>() {
        return Some(cu_rerun::representation::<Point2<Length64>, Spatial2DView>());
    }
    if TypeId::of::<L>() == TypeId::of::<u32>() {
        return Some(cu_rerun::representation::<Point2<u32>, Spatial2DView>());
    }
    if TypeId::of::<L>() == TypeId::of::<i32>() {
        return Some(cu_rerun::representation::<Point2<i32>, Spatial2DView>());
    }
    None
}
pub(crate) fn point2_soa_representation<L: 'static, const N: usize>() -> Option<DebugRepresentation>
{
    if TypeId::of::<L>() == TypeId::of::<Length32>() {
        return Some(cu_rerun::representation::<
            Point2Soa<Length32, N>,
            Spatial2DView,
        >());
    }
    if TypeId::of::<L>() == TypeId::of::<Length64>() {
        return Some(cu_rerun::representation::<
            Point2Soa<Length64, N>,
            Spatial2DView,
        >());
    }
    if TypeId::of::<L>() == TypeId::of::<u32>() {
        return Some(cu_rerun::representation::<Point2Soa<u32, N>, Spatial2DView>());
    }
    if TypeId::of::<L>() == TypeId::of::<i32>() {
        return Some(cu_rerun::representation::<Point2Soa<i32, N>, Spatial2DView>());
    }
    None
}
pub(crate) fn point3_representation<L: 'static>() -> Option<DebugRepresentation> {
    if TypeId::of::<L>() == TypeId::of::<Length32>() {
        return Some(cu_rerun::representation::<Point3<Length32>, Spatial3DView>());
    }
    if TypeId::of::<L>() == TypeId::of::<Length64>() {
        return Some(cu_rerun::representation::<Point3<Length64>, Spatial3DView>());
    }
    None
}
pub(crate) fn point3_soa_representation<L: 'static, const N: usize>() -> Option<DebugRepresentation>
{
    if TypeId::of::<L>() == TypeId::of::<Length32>() {
        return Some(cu_rerun::representation::<
            Point3Soa<Length32, N>,
            Spatial3DView,
        >());
    }
    if TypeId::of::<L>() == TypeId::of::<Length64>() {
        return Some(cu_rerun::representation::<
            Point3Soa<Length64, N>,
            Spatial3DView,
        >());
    }
    None
}
pub(crate) fn bbox_representation<P: 'static>() -> Option<DebugRepresentation> {
    if TypeId::of::<P>() == TypeId::of::<Point2<Length32>>() {
        return Some(cu_rerun::representation::<
            BBox<Point2<Length32>>,
            Spatial2DView,
        >());
    }
    if TypeId::of::<P>() == TypeId::of::<Point2<Length64>>() {
        return Some(cu_rerun::representation::<
            BBox<Point2<Length64>>,
            Spatial2DView,
        >());
    }
    if TypeId::of::<P>() == TypeId::of::<Point2<u32>>() {
        return Some(cu_rerun::representation::<BBox<Point2<u32>>, Spatial2DView>());
    }
    if TypeId::of::<P>() == TypeId::of::<Point2<i32>>() {
        return Some(cu_rerun::representation::<BBox<Point2<i32>>, Spatial2DView>());
    }
    if TypeId::of::<P>() == TypeId::of::<Point3<Length32>>() {
        return Some(cu_rerun::representation::<
            BBox<Point3<Length32>>,
            Spatial3DView,
        >());
    }
    if TypeId::of::<P>() == TypeId::of::<Point3<Length64>>() {
        return Some(cu_rerun::representation::<
            BBox<Point3<Length64>>,
            Spatial3DView,
        >());
    }
    None
}
