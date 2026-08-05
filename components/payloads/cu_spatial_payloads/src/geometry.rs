//! The shared point vocabulary: unit-typed points and axis-aligned bounding
//! boxes spatial components compose over.

use bincode::{Decode, Encode};
use core::fmt::Debug;
use cu29::prelude::*;
use cu29::units::si::f32::Length as Length32;
use cu29::units::si::f64::Length as Length64;
use cu29::units::si::length::meter;
use serde::{Deserialize, Serialize};

/// A 2D point with unit-typed coordinates.
///
/// This is a scalar type; for large batches consider an SoA layout so bulk
/// operations vectorize (see `cu29_soa_derive`).
#[derive(
    Default, Debug, Clone, Copy, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect,
)]
pub struct Point2<L: Copy + Debug + 'static> {
    pub x: L,
    pub y: L,
}

/// A 3D point with unit-typed coordinates.
///
/// This is a scalar type; for large batches consider an SoA layout so bulk
/// operations vectorize (see `cu29_soa_derive`).
#[derive(
    Default, Debug, Clone, Copy, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect,
)]
pub struct Point3<L: Copy + Debug + 'static> {
    pub x: L,
    pub y: L,
    pub z: L,
}

pub type Point2f = Point2<Length32>;
pub type Point2d = Point2<Length64>;
pub type Point3f = Point3<Length32>;
pub type Point3d = Point3<Length64>;

/// A 2D point in pixel coordinates, for image-space geometry.
pub type Point2u = Point2<u32>;
/// A 2D point in signed pixel coordinates, for image-space offsets.
pub type Point2i = Point2<i32>;

impl<L: Copy + Debug + 'static> Point2<L> {
    pub const fn new(x: L, y: L) -> Self {
        Self { x, y }
    }

    /// The point lifted to 3D at height `z`.
    pub fn with_z(self, z: L) -> Point3<L> {
        Point3::new(self.x, self.y, z)
    }
}

impl<L: Copy + Debug + 'static> Point3<L> {
    pub const fn new(x: L, y: L, z: L) -> Self {
        Self { x, y, z }
    }

    /// The planar projection: z dropped.
    pub fn xy(self) -> Point2<L> {
        Point2::new(self.x, self.y)
    }
}

macro_rules! impl_point_metrics {
    ($len:ty, $scalar:ty, $sqrt:path) => {
        impl Point2<$len> {
            pub fn from_meters(x: $scalar, y: $scalar) -> Self {
                Self::new(<$len>::new::<meter>(x), <$len>::new::<meter>(y))
            }

            /// Euclidean distance to `other`.
            pub fn distance(self, other: Self) -> $len {
                let (dx, dy) = ((self.x - other.x).raw(), (self.y - other.y).raw());
                <$len>::new::<meter>($sqrt(dx * dx + dy * dy))
            }

            /// The point at `ratio` of the way toward `other`: 0 is `self`,
            /// 1 is `other`.
            pub fn lerp(self, other: Self, ratio: $scalar) -> Self {
                Self::new(
                    self.x + (other.x - self.x) * ratio,
                    self.y + (other.y - self.y) * ratio,
                )
            }
        }

        impl Point3<$len> {
            pub fn from_meters(x: $scalar, y: $scalar, z: $scalar) -> Self {
                Self::new(
                    <$len>::new::<meter>(x),
                    <$len>::new::<meter>(y),
                    <$len>::new::<meter>(z),
                )
            }

            /// Euclidean distance to `other`.
            pub fn distance(self, other: Self) -> $len {
                let (dx, dy, dz) = (
                    (self.x - other.x).raw(),
                    (self.y - other.y).raw(),
                    (self.z - other.z).raw(),
                );
                <$len>::new::<meter>($sqrt(dx * dx + dy * dy + dz * dz))
            }

            /// The point at `ratio` of the way toward `other`: 0 is `self`,
            /// 1 is `other`.
            pub fn lerp(self, other: Self, ratio: $scalar) -> Self {
                Self::new(
                    self.x + (other.x - self.x) * ratio,
                    self.y + (other.y - self.y) * ratio,
                    self.z + (other.z - self.z) * ratio,
                )
            }
        }
    };
}

impl_point_metrics!(Length32, f32, libm::sqrtf);
impl_point_metrics!(Length64, f64, libm::sqrt);

/// An axis-aligned bounding box; the point type carries the dimension.
///
/// This is a scalar type; for large batches (e.g. detection anchors) consider
/// an SoA layout so bulk operations vectorize (see `cu29_soa_derive`).
#[derive(
    Default, Debug, Clone, Copy, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect,
)]
pub struct BBox<P: Copy + Debug + 'static> {
    pub min: P,
    pub max: P,
}

pub type BBox2f = BBox<Point2f>;
pub type BBox2d = BBox<Point2d>;
pub type BBox3f = BBox<Point3f>;
pub type BBox3d = BBox<Point3d>;

/// A 2D bounding box in pixel coordinates, for image-space geometry.
pub type BBox2u = BBox<Point2u>;
/// A 2D bounding box in signed pixel coordinates.
pub type BBox2i = BBox<Point2i>;

impl<P: Copy + Debug + 'static> BBox<P> {
    pub const fn new(min: P, max: P) -> Self {
        Self { min, max }
    }
}

impl<L: Copy + Debug + PartialOrd + 'static> BBox<Point2<L>> {
    /// True when `p` lies inside the box, boundary included.
    pub fn contains(&self, p: Point2<L>) -> bool {
        self.min.x <= p.x && p.x <= self.max.x && self.min.y <= p.y && p.y <= self.max.y
    }
}

impl<L: Copy + Debug + PartialOrd + 'static> BBox<Point3<L>> {
    /// True when `p` lies inside the box, boundary included.
    pub fn contains(&self, p: Point3<L>) -> bool {
        self.min.x <= p.x
            && p.x <= self.max.x
            && self.min.y <= p.y
            && p.y <= self.max.y
            && self.min.z <= p.z
            && p.z <= self.max.z
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn point_distance_and_lerp() {
        let a = Point2f::from_meters(1.0, 2.0);
        let b = Point2f::from_meters(4.0, 6.0);
        assert_eq!(a.distance(b).raw(), 5.0);
        assert_eq!(a.lerp(b, 0.0), a);
        assert_eq!(a.lerp(b, 1.0), b);
        assert_eq!(a.lerp(b, 0.5), Point2f::from_meters(2.5, 4.0));

        let a = Point3d::from_meters(1.0, 2.0, 3.0);
        let b = Point3d::from_meters(3.0, 5.0, 9.0);
        assert_eq!(a.distance(b).raw(), 7.0);
        assert_eq!(a.lerp(b, 0.5), Point3d::from_meters(2.0, 3.5, 6.0));
    }

    #[test]
    fn point_dimension_conversions() {
        let p = Point3f::from_meters(1.0, 2.0, 3.0);
        assert_eq!(p.xy(), Point2f::from_meters(1.0, 2.0));
        assert_eq!(p.xy().with_z(p.z), p);
    }

    #[test]
    fn bbox_contains_boundary_included() {
        let b = BBox2f::new(
            Point2f::from_meters(0.0, 0.0),
            Point2f::from_meters(2.0, 2.0),
        );
        assert!(b.contains(Point2f::from_meters(1.0, 1.0)));
        assert!(b.contains(Point2f::from_meters(0.0, 2.0)));
        assert!(!b.contains(Point2f::from_meters(-0.1, 1.0)));
        assert!(!b.contains(Point2f::from_meters(1.0, 2.1)));

        let b = BBox3f::new(
            Point3f::from_meters(0.0, 0.0, 0.0),
            Point3f::from_meters(1.0, 1.0, 1.0),
        );
        assert!(b.contains(Point3f::from_meters(0.5, 0.5, 1.0)));
        assert!(!b.contains(Point3f::from_meters(0.5, 0.5, 1.1)));
    }

    #[test]
    fn pixel_bbox_contains() {
        let b = BBox2u::new(Point2u::new(10, 20), Point2u::new(110, 220));
        assert!(b.contains(Point2u::new(10, 220)));
        assert!(b.contains(Point2u::new(60, 120)));
        assert!(!b.contains(Point2u::new(9, 120)));
        assert!(!b.contains(Point2u::new(60, 221)));
    }
}
