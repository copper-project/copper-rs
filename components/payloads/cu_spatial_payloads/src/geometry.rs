//! The shared point vocabulary: unit-typed points and axis-aligned bounding
//! boxes spatial components compose over.

use bincode::{Decode, Encode};
use core::fmt::Debug;
use cu29::prelude::*;
use cu29::units::si::area::square_meter;
use cu29::units::si::f32::Area as Area32;
use cu29::units::si::f32::Length as Length32;
use cu29::units::si::f64::Area as Area64;
use cu29::units::si::f64::Length as Length64;
use cu29::units::si::length::meter;
use cu29_soa_derive::Soa;
use serde::{Deserialize, Serialize};

/// A 2D point with unit-typed coordinates.
///
/// This is a scalar type; for large batches use [`Point2Soa`] so bulk
/// operations vectorize.
#[derive(
    Default, Debug, Clone, Copy, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect, Soa,
)]
#[reflect(from_reflect = false)]
pub struct Point2<L: Copy + Debug + 'static> {
    pub x: L,
    pub y: L,
}

/// A 3D point with unit-typed coordinates.
///
/// This is a scalar type; for large batches use [`Point3Soa`] so bulk
/// operations vectorize.
#[derive(
    Default, Debug, Clone, Copy, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect, Soa,
)]
#[reflect(from_reflect = false)]
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

pub type Point2fSoa<const N: usize> = Point2Soa<Length32, N>;
pub type Point2dSoa<const N: usize> = Point2Soa<Length64, N>;
pub type Point3fSoa<const N: usize> = Point3Soa<Length32, N>;
pub type Point3dSoa<const N: usize> = Point3Soa<Length64, N>;
pub type Point2uSoa<const N: usize> = Point2Soa<u32, N>;
pub type Point2iSoa<const N: usize> = Point2Soa<i32, N>;

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
    ($len:ty, $area:ty, $scalar:ty, $sqrt:path) => {
        impl Point2<$len> {
            pub fn from_meters(x: $scalar, y: $scalar) -> Self {
                Self::new(<$len>::new::<meter>(x), <$len>::new::<meter>(y))
            }

            /// Euclidean distance to `other`.
            pub fn distance(self, other: Self) -> $len {
                let (dx, dy) = ((self.x - other.x).raw(), (self.y - other.y).raw());
                <$len>::new::<meter>($sqrt(dx * dx + dy * dy))
            }

            /// The point at `ratio` of the way toward `other`. Ratio 0
            /// returns `self` exactly; ratio 1 returns `other` up to rounding.
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

            /// The point at `ratio` of the way toward `other`. Ratio 0
            /// returns `self` exactly; ratio 1 returns `other` up to rounding.
            pub fn lerp(self, other: Self, ratio: $scalar) -> Self {
                Self::new(
                    self.x + (other.x - self.x) * ratio,
                    self.y + (other.y - self.y) * ratio,
                    self.z + (other.z - self.z) * ratio,
                )
            }
        }

        impl<const N: usize> Point2Soa<$len, N> {
            /// Squared distance from every point to `target`, written to
            /// `out[..len]`. Sqrt-free, so the loop vectorizes; enough for
            /// nearest-neighbor style comparisons. Accurate to 1 ulp.
            ///
            /// # Panics
            /// If `out` is shorter than `self.len()`.
            pub fn distances_squared(&self, target: Point2<$len>, out: &mut [$area]) {
                let n = self.len();
                let (xs, ys, out) = (&self.x[..n], &self.y[..n], &mut out[..n]);
                let (tx, ty) = (target.x.raw(), target.y.raw());
                for i in 0..n {
                    let (dx, dy) = (xs[i].raw() - tx, ys[i].raw() - ty);
                    out[i] = <$area>::new::<square_meter>(dx * dx + dy * dy);
                }
            }

            /// Distance from every point to `target`, written to `out[..len]`.
            ///
            /// # Panics
            /// If `out` is shorter than `self.len()`.
            pub fn distances(&self, target: Point2<$len>, out: &mut [$len]) {
                let n = self.len();
                let (xs, ys, out) = (&self.x[..n], &self.y[..n], &mut out[..n]);
                let (tx, ty) = (target.x.raw(), target.y.raw());
                for i in 0..n {
                    let (dx, dy) = (xs[i].raw() - tx, ys[i].raw() - ty);
                    out[i] = <$len>::new::<meter>($sqrt(dx * dx + dy * dy));
                }
            }

            /// Every point moved `ratio` of the way toward `target`, in place.
            pub fn lerp_toward(&mut self, target: Point2<$len>, ratio: $scalar) {
                let n = self.len();
                for i in 0..n {
                    self.x[i] = self.x[i] + (target.x - self.x[i]) * ratio;
                    self.y[i] = self.y[i] + (target.y - self.y[i]) * ratio;
                }
            }
        }

        impl<const N: usize> Point3Soa<$len, N> {
            /// Squared distance from every point to `target`, written to
            /// `out[..len]`. Sqrt-free, so the loop vectorizes; enough for
            /// nearest-neighbor style comparisons. Accurate to 1 ulp.
            ///
            /// # Panics
            /// If `out` is shorter than `self.len()`.
            pub fn distances_squared(&self, target: Point3<$len>, out: &mut [$area]) {
                let n = self.len();
                let (xs, ys, zs) = (&self.x[..n], &self.y[..n], &self.z[..n]);
                let out = &mut out[..n];
                let (tx, ty, tz) = (target.x.raw(), target.y.raw(), target.z.raw());
                for i in 0..n {
                    let (dx, dy, dz) = (xs[i].raw() - tx, ys[i].raw() - ty, zs[i].raw() - tz);
                    out[i] = <$area>::new::<square_meter>(dx * dx + dy * dy + dz * dz);
                }
            }

            /// Distance from every point to `target`, written to `out[..len]`.
            ///
            /// # Panics
            /// If `out` is shorter than `self.len()`.
            pub fn distances(&self, target: Point3<$len>, out: &mut [$len]) {
                let n = self.len();
                let (xs, ys, zs) = (&self.x[..n], &self.y[..n], &self.z[..n]);
                let out = &mut out[..n];
                let (tx, ty, tz) = (target.x.raw(), target.y.raw(), target.z.raw());
                for i in 0..n {
                    let (dx, dy, dz) = (xs[i].raw() - tx, ys[i].raw() - ty, zs[i].raw() - tz);
                    out[i] = <$len>::new::<meter>($sqrt(dx * dx + dy * dy + dz * dz));
                }
            }

            /// Every point moved `ratio` of the way toward `target`, in place.
            pub fn lerp_toward(&mut self, target: Point3<$len>, ratio: $scalar) {
                let n = self.len();
                for i in 0..n {
                    self.x[i] = self.x[i] + (target.x - self.x[i]) * ratio;
                    self.y[i] = self.y[i] + (target.y - self.y[i]) * ratio;
                    self.z[i] = self.z[i] + (target.z - self.z[i]) * ratio;
                }
            }
        }
    };
}

impl_point_metrics!(Length32, Area32, f32, libm::sqrtf);
impl_point_metrics!(Length64, Area64, f64, libm::sqrt);

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

    /// `contains` for every point in the set, written to `out[..len]`.
    ///
    /// # Panics
    /// If `out` is shorter than `points.len()`.
    pub fn contains_points<const N: usize>(&self, points: &Point2Soa<L, N>, out: &mut [bool]) {
        let n = points.len();
        let (xs, ys, out) = (&points.x[..n], &points.y[..n], &mut out[..n]);
        for i in 0..n {
            out[i] = (self.min.x <= xs[i])
                & (xs[i] <= self.max.x)
                & (self.min.y <= ys[i])
                & (ys[i] <= self.max.y);
        }
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

    /// `contains` for every point in the set, written to `out[..len]`.
    ///
    /// # Panics
    /// If `out` is shorter than `points.len()`.
    pub fn contains_points<const N: usize>(&self, points: &Point3Soa<L, N>, out: &mut [bool]) {
        let n = points.len();
        let (xs, ys, zs) = (&points.x[..n], &points.y[..n], &points.z[..n]);
        let out = &mut out[..n];
        for i in 0..n {
            out[i] = (self.min.x <= xs[i])
                & (xs[i] <= self.max.x)
                & (self.min.y <= ys[i])
                & (ys[i] <= self.max.y)
                & (self.min.z <= zs[i])
                & (zs[i] <= self.max.z);
        }
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
        // Ratio 0 is exact; ratio 1 is only exact up to rounding.
        assert_eq!(a.lerp(b, 0.0), a);
        assert!((a.lerp(b, 1.0).distance(b)).raw() <= 1e-6);
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

    #[test]
    fn soa_distances_match_scalar() {
        let mut set = Point2fSoa::<8>::default();
        let target = Point2f::from_meters(1.0, -2.0);
        let points = [(0.0, 0.0), (3.0, 2.0), (-1.5, 4.0)];
        for (x, y) in points {
            set.push(Point2f::from_meters(x, y));
        }

        let mut d = [Length32::default(); 3];
        let mut d2 = [Area32::default(); 3];
        set.distances(target, &mut d);
        set.distances_squared(target, &mut d2);
        for i in 0..set.len() {
            let scalar = set.get(i).distance(target);
            assert!((d[i] - scalar).raw().abs() < 1e-6);
            assert!((d2[i].raw() - scalar.raw() * scalar.raw()).abs() < 1e-5);
        }

        let mut set3 = Point3dSoa::<4>::default();
        set3.push(Point3d::from_meters(1.0, 2.0, 3.0));
        set3.push(Point3d::from_meters(-2.0, 0.5, 1.0));
        let target3 = Point3d::from_meters(0.0, 1.0, -1.0);
        let mut d3 = [Length64::default(); 2];
        set3.distances(target3, &mut d3);
        for (i, d) in d3.iter().enumerate() {
            assert!((*d - set3.get(i).distance(target3)).raw().abs() < 1e-12);
        }
    }

    #[test]
    fn soa_lerp_toward_matches_scalar() {
        let mut set = Point2fSoa::<4>::default();
        set.push(Point2f::from_meters(0.0, 0.0));
        set.push(Point2f::from_meters(4.0, -2.0));
        let target = Point2f::from_meters(2.0, 2.0);

        let expected: [Point2f; 2] = [set.get(0).lerp(target, 0.25), set.get(1).lerp(target, 0.25)];
        set.lerp_toward(target, 0.25);
        assert_eq!(set.get(0), expected[0]);
        assert_eq!(set.get(1), expected[1]);
    }

    #[test]
    fn bbox_contains_points_bulk() {
        let b = BBox2f::new(
            Point2f::from_meters(0.0, 0.0),
            Point2f::from_meters(2.0, 2.0),
        );
        let mut set = Point2fSoa::<4>::default();
        set.push(Point2f::from_meters(1.0, 1.0));
        set.push(Point2f::from_meters(0.0, 2.0));
        set.push(Point2f::from_meters(-0.1, 1.0));
        let mut out = [false; 3];
        b.contains_points(&set, &mut out);
        assert_eq!(out, [true, true, false]);

        let pix = BBox2u::new(Point2u::new(10, 20), Point2u::new(110, 220));
        let mut pixels = Point2uSoa::<4>::default();
        pixels.push(Point2u::new(60, 120));
        pixels.push(Point2u::new(9, 120));
        let mut out = [false; 2];
        pix.contains_points(&pixels, &mut out);
        assert_eq!(out, [true, false]);
    }

    #[test]
    fn bbox3_contains_points_bulk_matches_scalar() {
        let b = BBox3f::new(
            Point3f::from_meters(0.0, 0.0, 0.0),
            Point3f::from_meters(1.0, 1.0, 1.0),
        );
        let points = [
            Point3f::from_meters(0.5, 0.5, 1.0),  // on the z boundary
            Point3f::from_meters(0.5, 0.5, 1.1),  // outside in z
            Point3f::from_meters(0.0, 0.0, 0.0),  // corner
            Point3f::from_meters(-0.1, 0.5, 0.5), // outside in x
        ];
        let mut set = Point3fSoa::<8>::default();
        for p in points {
            set.push(p);
        }

        // `out` is longer than the set; the tail stays untouched.
        let mut out = [true; 6];
        b.contains_points(&set, &mut out);
        assert_eq!(out, [true, false, true, false, true, true]);
        for (i, p) in points.iter().enumerate() {
            assert_eq!(out[i], b.contains(*p));
        }
    }

    #[test]
    fn bulk_kernels_handle_an_empty_set() {
        let set = Point2fSoa::<4>::default();
        assert!(set.is_empty());
        set.distances(Point2f::from_meters(1.0, 1.0), &mut []);
        set.distances_squared(Point2f::from_meters(1.0, 1.0), &mut []);
        let b = BBox2f::new(
            Point2f::from_meters(0.0, 0.0),
            Point2f::from_meters(1.0, 1.0),
        );
        b.contains_points(&set, &mut []);
    }

    /// Replaying a log recorded at a larger capacity must error, not panic.
    #[test]
    fn decode_rejects_len_over_capacity() {
        let mut wide = Point2fSoa::<8>::default();
        for i in 0..8 {
            wide.push(Point2f::from_meters(i as f32, -(i as f32)));
        }
        let cfg = cu29::bincode::config::standard();
        let bytes = cu29::bincode::encode_to_vec(&wide, cfg).expect("encode");

        let narrow: Result<(Point2fSoa<4>, usize), _> =
            cu29::bincode::decode_from_slice(&bytes, cfg);
        assert!(narrow.is_err(), "expected a capacity error");

        let (same, _): (Point2fSoa<8>, _) =
            cu29::bincode::decode_from_slice(&bytes, cfg).expect("decode");
        assert_eq!(same.len(), 8);
        assert_eq!(same.get(7), wide.get(7));
    }
}
