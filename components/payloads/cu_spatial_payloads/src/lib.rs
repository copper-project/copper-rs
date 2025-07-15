use bincode::{Decode, Encode};
use core::fmt::Debug;
use serde::{Deserialize, Serialize};
use std::ops::Mul;
use uom::si::angle::radian;
use uom::si::f32::Angle as Angle32;
use uom::si::f32::Length as Length32;
use uom::si::f64::Angle as Angle64;
use uom::si::f64::Length as Length64;
use uom::si::length::meter;

use glam::DVec4;
#[cfg(feature = "glam")]
use glam::{Affine3A, DAffine3, DMat4, Mat4};

/// Transform3D represents a 3D transformation (rotation + translation)
/// When the glam feature is enabled, it uses glam's optimized types internally
#[derive(Debug, Clone, Copy)]
pub struct Transform3D<T: Copy + Debug + 'static> {
    #[cfg(feature = "glam")]
    inner: TransformInner<T>,
    #[cfg(not(feature = "glam"))]
    pub mat: [[T; 4]; 4],
}

#[cfg(feature = "glam")]
#[derive(Debug, Clone, Copy)]
enum TransformInner<T: Copy + Debug + 'static> {
    F32(Affine3A),
    F64(DAffine3),
    _Phantom(std::marker::PhantomData<T>),
}

pub type Pose<T> = Transform3D<T>;

// Manual implementations for serialization
impl<T: Copy + Debug + Default + 'static> Serialize for Transform3D<T>
where
    T: Serialize,
{
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        #[cfg(feature = "glam")]
        {
            let mat = self.to_matrix();
            mat.serialize(serializer)
        }
        #[cfg(not(feature = "glam"))]
        {
            self.mat.serialize(serializer)
        }
    }
}

impl<'de, T: Copy + Debug + 'static> Deserialize<'de> for Transform3D<T>
where
    T: Deserialize<'de> + Default,
{
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let mat: [[T; 4]; 4] = Deserialize::deserialize(deserializer)?;
        Ok(Self::from_matrix(mat))
    }
}

// Bincode implementations
impl<T: Copy + Debug + Default + 'static> Encode for Transform3D<T>
where
    T: Encode,
{
    fn encode<E: bincode::enc::Encoder>(
        &self,
        encoder: &mut E,
    ) -> Result<(), bincode::error::EncodeError> {
        #[cfg(feature = "glam")]
        {
            let mat = self.to_matrix();
            mat.encode(encoder)
        }
        #[cfg(not(feature = "glam"))]
        {
            self.mat.encode(encoder)
        }
    }
}

impl<T: Copy + Debug + 'static> Decode<()> for Transform3D<T>
where
    T: Decode<()> + Default,
{
    fn decode<D: bincode::de::Decoder<Context = ()>>(
        decoder: &mut D,
    ) -> Result<Self, bincode::error::DecodeError> {
        let mat: [[T; 4]; 4] = Decode::decode(decoder)?;
        Ok(Self::from_matrix(mat))
    }
}

impl<T: Copy + Debug + Default + 'static> Transform3D<T> {
    /// Create a transform from a 4x4 matrix
    pub fn from_matrix(mat: [[T; 4]; 4]) -> Self {
        #[cfg(feature = "glam")]
        {
            Self {
                inner: TransformInner::from_matrix(mat),
            }
        }
        #[cfg(not(feature = "glam"))]
        {
            Self { mat }
        }
    }

    /// Get the transform as a 4x4 matrix
    pub fn to_matrix(self) -> [[T; 4]; 4] {
        #[cfg(feature = "glam")]
        {
            self.inner.to_matrix()
        }
        #[cfg(not(feature = "glam"))]
        {
            self.mat
        }
    }

    /// Get a mutable reference to the matrix (for compatibility)
    #[cfg(not(feature = "glam"))]
    pub fn mat_mut(&mut self) -> &mut [[T; 4]; 4] {
        &mut self.mat
    }
}

#[cfg(feature = "glam")]
impl<T: Copy + Debug + Default + 'static> TransformInner<T> {
    fn from_matrix(mat: [[T; 4]; 4]) -> Self {
        use std::any::TypeId;

        // This is a bit hacky but necessary for type safety
        // In practice, T will be f32 or f64
        if TypeId::of::<T>() == TypeId::of::<f32>() {
            // Convert to f32 matrix
            let mat_f32: [[f32; 4]; 4] = unsafe { std::mem::transmute_copy(&mat) };
            let glam_mat = Mat4::from_cols_array_2d(&mat_f32);
            let affine = Affine3A::from_mat4(glam_mat);
            unsafe { std::mem::transmute_copy(&TransformInner::<T>::F32(affine)) }
        } else if TypeId::of::<T>() == TypeId::of::<f64>() {
            // Convert to f64 matrix
            let mat_f64: [[f64; 4]; 4] = unsafe { std::mem::transmute_copy(&mat) };
            let m = mat_f64;
            let glam_mat = DMat4::from_cols(
                DVec4::new(m[0][0], m[1][0], m[2][0], m[3][0]),
                DVec4::new(m[0][1], m[1][1], m[2][1], m[3][1]),
                DVec4::new(m[0][2], m[1][2], m[2][2], m[3][2]),
                DVec4::new(m[0][3], m[1][3], m[2][3], m[3][3]),
            );
            let affine = DAffine3::from_mat4(glam_mat);
            unsafe { std::mem::transmute_copy(&TransformInner::<T>::F64(affine)) };
        } else {
            panic!("Transform3D only supports f32 and f64 types when using glam feature");
        }
    }

    fn to_matrix(self) -> [[T; 4]; 4] {
        match self {
            TransformInner::F32(affine) => {
                let mat = Mat4::from(affine);
                let mat_array = mat.to_cols_array_2d();
                unsafe { std::mem::transmute_copy(&mat_array) }
            }
            TransformInner::F64(affine) => {
                let mat = DMat4::from(affine);
                let mat_array = mat.to_cols_array_2d();
                unsafe { std::mem::transmute_copy(&mat_array) }
            }
            TransformInner::_Phantom(_) => unreachable!(),
        }
    }
}

impl Transform3D<f64> {
    pub fn translation(&self) -> [Length64; 3] {
        let mat = self.to_matrix();
        [
            Length64::new::<meter>(mat[0][3]),
            Length64::new::<meter>(mat[1][3]),
            Length64::new::<meter>(mat[2][3]),
        ]
    }

    pub fn rotation(&self) -> [[Angle64; 3]; 3] {
        let mat = self.to_matrix();
        [
            [
                Angle64::new::<radian>(mat[0][0]),
                Angle64::new::<radian>(mat[0][1]),
                Angle64::new::<radian>(mat[0][2]),
            ],
            [
                Angle64::new::<radian>(mat[1][0]),
                Angle64::new::<radian>(mat[1][1]),
                Angle64::new::<radian>(mat[1][2]),
            ],
            [
                Angle64::new::<radian>(mat[2][0]),
                Angle64::new::<radian>(mat[2][1]),
                Angle64::new::<radian>(mat[2][2]),
            ],
        ]
    }
}

impl Transform3D<f32> {
    pub fn translation(&self) -> [Length32; 3] {
        let mat = self.to_matrix();
        [
            Length32::new::<meter>(mat[0][3]),
            Length32::new::<meter>(mat[1][3]),
            Length32::new::<meter>(mat[2][3]),
        ]
    }

    pub fn rotation(&self) -> [[Angle32; 3]; 3] {
        let mat = self.to_matrix();
        [
            [
                Angle32::new::<radian>(mat[0][0]),
                Angle32::new::<radian>(mat[0][1]),
                Angle32::new::<radian>(mat[0][2]),
            ],
            [
                Angle32::new::<radian>(mat[1][0]),
                Angle32::new::<radian>(mat[1][1]),
                Angle32::new::<radian>(mat[1][2]),
            ],
            [
                Angle32::new::<radian>(mat[2][0]),
                Angle32::new::<radian>(mat[2][1]),
                Angle32::new::<radian>(mat[2][2]),
            ],
        ]
    }
}

impl<T: Copy + Debug + Default + 'static> Default for Transform3D<T> {
    fn default() -> Self {
        Self::from_matrix([[T::default(); 4]; 4])
    }
}

/// Implementation for f32
impl Mul for Transform3D<f32> {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self::Output {
        #[cfg(feature = "glam")]
        {
            match (&self.inner, &rhs.inner) {
                (TransformInner::F32(a), TransformInner::F32(b)) => Self {
                    inner: TransformInner::F32(*a * *b),
                },
                _ => unreachable!(),
            }
        }
        #[cfg(not(feature = "glam"))]
        {
            let mut result = [[0.0f32; 4]; 4];
            for i in 0..4 {
                for j in 0..4 {
                    let mut sum = 0.0;
                    for k in 0..4 {
                        sum += self.mat[i][k] * rhs.mat[k][j];
                    }
                    result[i][j] = sum;
                }
            }
            Self { mat: result }
        }
    }
}

/// Implementation for f64
impl Mul for Transform3D<f64> {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self::Output {
        #[cfg(feature = "glam")]
        {
            match (&self.inner, &rhs.inner) {
                (TransformInner::F64(a), TransformInner::F64(b)) => Self {
                    inner: TransformInner::F64(*a * *b),
                },
                _ => unreachable!(),
            }
        }
        #[cfg(not(feature = "glam"))]
        {
            let mut result = [[0.0f64; 4]; 4];
            for i in 0..4 {
                for j in 0..4 {
                    let mut sum = 0.0;
                    for k in 0..4 {
                        sum += self.mat[i][k] * rhs.mat[k][j];
                    }
                    result[i][j] = sum;
                }
            }
            Self { mat: result }
        }
    }
}

/// Reference implementations for f32
impl Mul for &Transform3D<f32> {
    type Output = Transform3D<f32>;

    fn mul(self, rhs: Self) -> Self::Output {
        *self * *rhs
    }
}

impl Mul<Transform3D<f32>> for &Transform3D<f32> {
    type Output = Transform3D<f32>;

    fn mul(self, rhs: Transform3D<f32>) -> Self::Output {
        *self * rhs
    }
}

impl Mul<&Transform3D<f32>> for Transform3D<f32> {
    type Output = Transform3D<f32>;

    fn mul(self, rhs: &Transform3D<f32>) -> Self::Output {
        self * *rhs
    }
}

/// Reference implementations for f64
impl Mul for &Transform3D<f64> {
    type Output = Transform3D<f64>;

    fn mul(self, rhs: Self) -> Self::Output {
        *self * *rhs
    }
}

impl Mul<Transform3D<f64>> for &Transform3D<f64> {
    type Output = Transform3D<f64>;

    fn mul(self, rhs: Transform3D<f64>) -> Self::Output {
        *self * rhs
    }
}

impl Mul<&Transform3D<f64>> for Transform3D<f64> {
    type Output = Transform3D<f64>;

    fn mul(self, rhs: &Transform3D<f64>) -> Self::Output {
        self * *rhs
    }
}

impl Transform3D<f32> {
    /// Computes the inverse of this transformation matrix.
    pub fn inverse(&self) -> Self {
        #[cfg(feature = "glam")]
        {
            match &self.inner {
                TransformInner::F32(affine) => Self {
                    inner: TransformInner::F32(affine.inverse()),
                },
                _ => unreachable!(),
            }
        }
        #[cfg(not(feature = "glam"))]
        {
            let mat = self.mat;
            // Extract rotation matrix (top-left 3x3)
            let r = [
                [mat[0][0], mat[0][1], mat[0][2]],
                [mat[1][0], mat[1][1], mat[1][2]],
                [mat[2][0], mat[2][1], mat[2][2]],
            ];

            // Extract translation (top-right 3x1)
            let t = [mat[0][3], mat[1][3], mat[2][3]];

            // Compute transpose of rotation matrix (which is its inverse for orthogonal matrices)
            let r_inv = [
                [r[0][0], r[1][0], r[2][0]],
                [r[0][1], r[1][1], r[2][1]],
                [r[0][2], r[1][2], r[2][2]],
            ];

            // Compute -R^T * t
            let t_inv = [
                -(r_inv[0][0] * t[0] + r_inv[0][1] * t[1] + r_inv[0][2] * t[2]),
                -(r_inv[1][0] * t[0] + r_inv[1][1] * t[1] + r_inv[1][2] * t[2]),
                -(r_inv[2][0] * t[0] + r_inv[2][1] * t[1] + r_inv[2][2] * t[2]),
            ];

            // Construct the inverse transformation matrix
            let mut inv_mat = [[0.0f32; 4]; 4];

            // Copy rotation transpose
            for i in 0..3 {
                for j in 0..3 {
                    inv_mat[i][j] = r_inv[i][j];
                }
            }

            // Copy translation part
            inv_mat[0][3] = t_inv[0];
            inv_mat[1][3] = t_inv[1];
            inv_mat[2][3] = t_inv[2];

            // Keep the homogeneous coordinate the same
            inv_mat[3][3] = 1.0;

            Self { mat: inv_mat }
        }
    }
}

impl Transform3D<f64> {
    /// Computes the inverse of this transformation matrix.
    pub fn inverse(&self) -> Self {
        #[cfg(feature = "glam")]
        {
            match &self.inner {
                TransformInner::F64(affine) => Self {
                    inner: TransformInner::F64(affine.inverse()),
                },
                _ => unreachable!(),
            }
        }
        #[cfg(not(feature = "glam"))]
        {
            let mat = self.mat;
            // Extract rotation matrix (top-left 3x3)
            let r = [
                [mat[0][0], mat[0][1], mat[0][2]],
                [mat[1][0], mat[1][1], mat[1][2]],
                [mat[2][0], mat[2][1], mat[2][2]],
            ];

            // Extract translation (top-right 3x1)
            let t = [mat[0][3], mat[1][3], mat[2][3]];

            // Compute transpose of rotation matrix (which is its inverse for orthogonal matrices)
            let r_inv = [
                [r[0][0], r[1][0], r[2][0]],
                [r[0][1], r[1][1], r[2][1]],
                [r[0][2], r[1][2], r[2][2]],
            ];

            // Compute -R^T * t
            let t_inv = [
                -(r_inv[0][0] * t[0] + r_inv[0][1] * t[1] + r_inv[0][2] * t[2]),
                -(r_inv[1][0] * t[0] + r_inv[1][1] * t[1] + r_inv[1][2] * t[2]),
                -(r_inv[2][0] * t[0] + r_inv[2][1] * t[1] + r_inv[2][2] * t[2]),
            ];

            // Construct the inverse transformation matrix
            let mut inv_mat = [[0.0f64; 4]; 4];

            // Copy rotation transpose
            for i in 0..3 {
                for j in 0..3 {
                    inv_mat[i][j] = r_inv[i][j];
                }
            }

            // Copy translation part
            inv_mat[0][3] = t_inv[0];
            inv_mat[1][3] = t_inv[1];
            inv_mat[2][3] = t_inv[2];

            // Keep the homogeneous coordinate the same
            inv_mat[3][3] = 1.0;

            Self { mat: inv_mat }
        }
    }
}

#[cfg(feature = "faer")]
mod faer_integration {
    use super::Transform3D;
    use faer::prelude::*;

    impl From<&Transform3D<f64>> for Mat<f64> {
        fn from(p: &Transform3D<f64>) -> Self {
            let mat_array = p.to_matrix();
            let mut mat: Mat<f64> = Mat::zeros(4, 4);
            for (r, row) in mat_array.iter().enumerate() {
                for (c, item) in row.iter().enumerate() {
                    *mat.get_mut(r, c) = *item;
                }
            }
            mat
        }
    }

    impl From<Mat<f64>> for Transform3D<f64> {
        fn from(mat: Mat<f64>) -> Self {
            assert_eq!(mat.nrows(), 4);
            assert_eq!(mat.ncols(), 4);
            let mut transform = [[0.0; 4]; 4];
            for (r, row) in transform.iter_mut().enumerate() {
                for (c, val) in row.iter_mut().enumerate() {
                    *val = *mat.get(r, c);
                }
            }
            Self::from_matrix(transform)
        }
    }
}

// Optional Nalgebra integration
#[cfg(feature = "nalgebra")]
mod nalgebra_integration {
    use super::Transform3D;
    use nalgebra::{Isometry3, Matrix3, Matrix4, Rotation3, Translation3, Vector3};

    impl From<&Transform3D<f64>> for Isometry3<f64> {
        fn from(pose: &Transform3D<f64>) -> Self {
            let mat_array = pose.to_matrix();
            let flat_transform: [f64; 16] = std::array::from_fn(|i| mat_array[i / 4][i % 4]);
            let matrix = Matrix4::from_row_slice(&flat_transform);

            let rotation_matrix: Matrix3<f64> = matrix.fixed_view::<3, 3>(0, 0).into();
            let rotation = Rotation3::from_matrix_unchecked(rotation_matrix);

            let translation_vector: Vector3<f64> = matrix.fixed_view::<3, 1>(0, 3).into();
            let translation = Translation3::from(translation_vector);

            Isometry3::from_parts(translation, rotation.into())
        }
    }

    impl From<Isometry3<f64>> for Transform3D<f64> {
        fn from(iso: Isometry3<f64>) -> Self {
            let matrix = iso.to_homogeneous();
            let transform = std::array::from_fn(|r| std::array::from_fn(|c| matrix[(r, c)]));
            Transform3D::from_matrix(transform)
        }
    }
}

// Keep existing glam integration but update it
#[cfg(feature = "glam")]
mod glam_integration {
    use super::Transform3D;
    use glam::{Affine3A, DAffine3};

    impl From<Transform3D<f64>> for DAffine3 {
        fn from(p: Transform3D<f64>) -> Self {
            let mat = p.to_matrix();
            let mut aff = DAffine3::IDENTITY;
            aff.matrix3.x_axis.x = mat[0][0];
            aff.matrix3.x_axis.y = mat[0][1];
            aff.matrix3.x_axis.z = mat[0][2];

            aff.matrix3.y_axis.x = mat[1][0];
            aff.matrix3.y_axis.y = mat[1][1];
            aff.matrix3.y_axis.z = mat[1][2];

            aff.matrix3.z_axis.x = mat[2][0];
            aff.matrix3.z_axis.y = mat[2][1];
            aff.matrix3.z_axis.z = mat[2][2];

            aff.translation.x = mat[3][0];
            aff.translation.y = mat[3][1];
            aff.translation.z = mat[3][2];

            aff
        }
    }

    impl From<DAffine3> for Transform3D<f64> {
        fn from(aff: DAffine3) -> Self {
            let mut transform = [[0.0f64; 4]; 4];

            transform[0][0] = aff.matrix3.x_axis.x;
            transform[0][1] = aff.matrix3.x_axis.y;
            transform[0][2] = aff.matrix3.x_axis.z;

            transform[1][0] = aff.matrix3.y_axis.x;
            transform[1][1] = aff.matrix3.y_axis.y;
            transform[1][2] = aff.matrix3.y_axis.z;

            transform[2][0] = aff.matrix3.z_axis.x;
            transform[2][1] = aff.matrix3.z_axis.y;
            transform[2][2] = aff.matrix3.z_axis.z;

            transform[0][3] = aff.translation.x;
            transform[1][3] = aff.translation.y;
            transform[2][3] = aff.translation.z;
            transform[3][3] = 1.0;

            Transform3D::from_matrix(transform)
        }
    }

    impl From<Transform3D<f32>> for Affine3A {
        fn from(p: Transform3D<f32>) -> Self {
            let mat = p.to_matrix();
            let mut aff = Affine3A::IDENTITY;
            aff.matrix3.x_axis.x = mat[0][0];
            aff.matrix3.x_axis.y = mat[0][1];
            aff.matrix3.x_axis.z = mat[0][2];

            aff.matrix3.y_axis.x = mat[1][0];
            aff.matrix3.y_axis.y = mat[1][1];
            aff.matrix3.y_axis.z = mat[1][2];

            aff.matrix3.z_axis.x = mat[2][0];
            aff.matrix3.z_axis.y = mat[2][1];
            aff.matrix3.z_axis.z = mat[2][2];

            aff.translation.x = mat[0][3];
            aff.translation.y = mat[1][3];
            aff.translation.z = mat[2][3];

            aff
        }
    }

    impl From<Affine3A> for Transform3D<f32> {
        fn from(aff: Affine3A) -> Self {
            let mut transform = [[0.0f32; 4]; 4];

            transform[0][0] = aff.matrix3.x_axis.x;
            transform[0][1] = aff.matrix3.x_axis.y;
            transform[0][2] = aff.matrix3.x_axis.z;

            transform[1][0] = aff.matrix3.y_axis.x;
            transform[1][1] = aff.matrix3.y_axis.y;
            transform[1][2] = aff.matrix3.y_axis.z;

            transform[2][0] = aff.matrix3.z_axis.x;
            transform[2][1] = aff.matrix3.z_axis.y;
            transform[2][2] = aff.matrix3.z_axis.z;

            transform[0][3] = aff.translation.x;
            transform[1][3] = aff.translation.y;
            transform[2][3] = aff.translation.z;
            transform[3][3] = 1.0;

            Transform3D::from_matrix(transform)
        }
    }
}

#[cfg(feature = "nalgebra")]
#[allow(unused_imports)]
pub use nalgebra_integration::*;

#[cfg(feature = "faer")]
#[allow(unused_imports)]
pub use faer_integration::*;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pose_default() {
        let pose: Transform3D<f32> = Transform3D::default();
        let mat = pose.to_matrix();

        // With glam feature, the default is created from a zero matrix
        // but internally glam may adjust it to ensure valid transforms
        #[cfg(feature = "glam")]
        {
            // When we create from all zeros, glam will create a transform
            // that has zeros except for the homogeneous coordinate
            let expected = [
                [0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 1.0], // homogeneous coordinate
            ];
            assert_eq!(mat, expected, "Default pose with glam should have w=1");
        }

        #[cfg(not(feature = "glam"))]
        {
            assert_eq!(
                mat, [[0.0; 4]; 4],
                "Default pose without glam should be a zero matrix"
            );
        }
    }

    #[test]
    fn test_transform_inverse_f32() {
        // Create a test transform with rotation and translation
        let transform = Transform3D::<f32>::from_matrix([
            [1.0, 0.0, 0.0, 2.0], // x-axis with 2m translation
            [0.0, 1.0, 0.0, 3.0], // y-axis with 3m translation
            [0.0, 0.0, 1.0, 4.0], // z-axis with 4m translation
            [0.0, 0.0, 0.0, 1.0], // homogeneous coordinate
        ]);

        // Compute inverse
        let inverse = transform.inverse();

        // Expected inverse for this transform
        let expected_inverse = Transform3D::<f32>::from_matrix([
            [1.0, 0.0, 0.0, -2.0], // Negated translation
            [0.0, 1.0, 0.0, -3.0],
            [0.0, 0.0, 1.0, -4.0],
            [0.0, 0.0, 0.0, 1.0],
        ]);

        // Check each element with a small epsilon for floating-point comparison
        let epsilon = 1e-5;
        let inv_mat = inverse.to_matrix();
        let exp_mat = expected_inverse.to_matrix();
        for i in 0..4 {
            for j in 0..4 {
                assert!(
                    (inv_mat[i][j] - exp_mat[i][j]).abs() < epsilon,
                    "Element at [{},{}] differs: {} vs expected {}",
                    i,
                    j,
                    inv_mat[i][j],
                    exp_mat[i][j]
                );
            }
        }
    }

    #[test]
    fn test_transform_inverse_f64() {
        // Create a test transform with rotation and translation
        let transform = Transform3D::<f64>::from_matrix([
            [0.0, -1.0, 0.0, 5.0], // 90-degree rotation around z with translation
            [1.0, 0.0, 0.0, 6.0],
            [0.0, 0.0, 1.0, 7.0],
            [0.0, 0.0, 0.0, 1.0],
        ]);

        // Compute inverse
        let inverse = transform.inverse();

        // Expected inverse for this transform
        let expected_inverse = Transform3D::<f64>::from_matrix([
            [0.0, 1.0, 0.0, -6.0], // Transposed rotation and adjusted translation
            [-1.0, 0.0, 0.0, 5.0],
            [0.0, 0.0, 1.0, -7.0],
            [0.0, 0.0, 0.0, 1.0],
        ]);

        // Check each element with a small epsilon for floating-point comparison
        let epsilon = 1e-10;
        let inv_mat = inverse.to_matrix();
        let exp_mat = expected_inverse.to_matrix();
        for i in 0..4 {
            for j in 0..4 {
                assert!(
                    (inv_mat[i][j] - exp_mat[i][j]).abs() < epsilon,
                    "Element at [{},{}] differs: {} vs expected {}",
                    i,
                    j,
                    inv_mat[i][j],
                    exp_mat[i][j]
                );
            }
        }
    }

    #[test]
    fn test_transform_inverse_identity() {
        // Create identity transform
        let identity = Transform3D::<f32>::from_matrix([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]);

        // Inverse of identity should be identity
        let inverse = identity.inverse();

        // Check if inverse is also identity
        let epsilon = 1e-5;
        let inv_mat = inverse.to_matrix();
        let id_mat = identity.to_matrix();
        for i in 0..4 {
            for j in 0..4 {
                assert!(
                    (inv_mat[i][j] - id_mat[i][j]).abs() < epsilon,
                    "Element at [{},{}] differs: {} vs expected {}",
                    i,
                    j,
                    inv_mat[i][j],
                    id_mat[i][j]
                );
            }
        }
    }

    #[test]
    fn test_transform_multiplication_f32() {
        // Create two transforms to multiply
        let t1 = Transform3D::<f32>::from_matrix([
            [1.0, 0.0, 0.0, 2.0], // Identity rotation + translation (2,3,4)
            [0.0, 1.0, 0.0, 3.0],
            [0.0, 0.0, 1.0, 4.0],
            [0.0, 0.0, 0.0, 1.0],
        ]);

        let t2 = Transform3D::<f32>::from_matrix([
            [0.0, -1.0, 0.0, 5.0], // 90-degree rotation around z + translation (5,6,7)
            [1.0, 0.0, 0.0, 6.0],
            [0.0, 0.0, 1.0, 7.0],
            [0.0, 0.0, 0.0, 1.0],
        ]);

        // Compute t1 * t2
        let result = t1 * t2;

        // Expected result: t1 * t2 represents first rotating by t2, then translating by t1
        let expected = Transform3D::<f32>::from_matrix([
            [0.0, -1.0, 0.0, 7.0], // Rotation from t2 + combined translation
            [1.0, 0.0, 0.0, 9.0],
            [0.0, 0.0, 1.0, 11.0],
            [0.0, 0.0, 0.0, 1.0],
        ]);

        // Check results
        let epsilon = 1e-5;
        let res_mat = result.to_matrix();
        let exp_mat = expected.to_matrix();
        for i in 0..4 {
            for j in 0..4 {
                assert!(
                    (res_mat[i][j] - exp_mat[i][j]).abs() < epsilon,
                    "Element at [{},{}] differs: {} vs expected {}",
                    i,
                    j,
                    res_mat[i][j],
                    exp_mat[i][j]
                );
            }
        }
    }

    #[test]
    fn test_transform_multiplication_f64() {
        // Create two transforms to multiply
        let t1 = Transform3D::<f64>::from_matrix([
            [1.0, 0.0, 0.0, 2.0], // Identity rotation + translation (2,3,4)
            [0.0, 1.0, 0.0, 3.0],
            [0.0, 0.0, 1.0, 4.0],
            [0.0, 0.0, 0.0, 1.0],
        ]);

        let t2 = Transform3D::<f64>::from_matrix([
            [0.0, -1.0, 0.0, 5.0], // 90-degree rotation around z + translation (5,6,7)
            [1.0, 0.0, 0.0, 6.0],
            [0.0, 0.0, 1.0, 7.0],
            [0.0, 0.0, 0.0, 1.0],
        ]);

        // Compute t1 * t2
        let result = t1 * t2;

        // Expected result
        let expected = Transform3D::<f64>::from_matrix([
            [0.0, -1.0, 0.0, 7.0], // Rotation from t2 + combined translation
            [1.0, 0.0, 0.0, 9.0],
            [0.0, 0.0, 1.0, 11.0],
            [0.0, 0.0, 0.0, 1.0],
        ]);

        // Check results
        let epsilon = 1e-10;
        let res_mat = result.to_matrix();
        let exp_mat = expected.to_matrix();
        for i in 0..4 {
            for j in 0..4 {
                assert!(
                    (res_mat[i][j] - exp_mat[i][j]).abs() < epsilon,
                    "Element at [{},{}] differs: {} vs expected {}",
                    i,
                    j,
                    res_mat[i][j],
                    exp_mat[i][j]
                );
            }
        }
    }

    #[test]
    fn test_transform_reference_multiplication() {
        // Test multiplication on references
        let t1 = Transform3D::<f32>::from_matrix([
            [1.0, 0.0, 0.0, 2.0],
            [0.0, 1.0, 0.0, 3.0],
            [0.0, 0.0, 1.0, 4.0],
            [0.0, 0.0, 0.0, 1.0],
        ]);

        let t2 = Transform3D::<f32>::from_matrix([
            [0.0, -1.0, 0.0, 5.0],
            [1.0, 0.0, 0.0, 6.0],
            [0.0, 0.0, 1.0, 7.0],
            [0.0, 0.0, 0.0, 1.0],
        ]);

        // Compute &t1 * &t2
        let result = t1 * t2;

        // Expected result
        let expected = Transform3D::<f32>::from_matrix([
            [0.0, -1.0, 0.0, 7.0],
            [1.0, 0.0, 0.0, 9.0],
            [0.0, 0.0, 1.0, 11.0],
            [0.0, 0.0, 0.0, 1.0],
        ]);

        // Check results
        let epsilon = 1e-5;
        let res_mat = result.to_matrix();
        let exp_mat = expected.to_matrix();
        for i in 0..4 {
            for j in 0..4 {
                assert!(
                    (res_mat[i][j] - exp_mat[i][j]).abs() < epsilon,
                    "Element at [{},{}] differs: {} vs expected {}",
                    i,
                    j,
                    res_mat[i][j],
                    exp_mat[i][j]
                );
            }
        }
    }

    #[cfg(feature = "faer")]
    #[test]
    fn test_pose_faer_conversion() {
        use faer::prelude::*;

        let pose = Transform3D::from_matrix([
            [1.0, 2.0, 3.0, 4.0],
            [5.0, 6.0, 7.0, 8.0],
            [9.0, 10.0, 11.0, 12.0],
            [13.0, 14.0, 15.0, 16.0],
        ]);

        let mat: Mat<f64> = (&pose).into();
        let pose_from_mat = Transform3D::from(mat);

        assert_eq!(
            pose.to_matrix(),
            pose_from_mat.to_matrix(),
            "Faer conversion should be lossless"
        );
    }

    #[cfg(feature = "nalgebra")]
    #[test]
    fn test_pose_nalgebra_conversion() {
        use nalgebra::Isometry3;

        let pose = Transform3D::from_matrix([
            [1.0, 0.0, 0.0, 2.0],
            [0.0, 1.0, 0.0, 3.0],
            [0.0, 0.0, 1.0, 4.0],
            [0.0, 0.0, 0.0, 1.0],
        ]);

        let iso: Isometry3<f64> = (&pose.clone()).into();
        let pose_from_iso: Transform3D<f64> = iso.into();

        assert_eq!(
            pose.to_matrix(),
            pose_from_iso.to_matrix(),
            "Nalgebra conversion should be lossless"
        );
    }

    #[cfg(feature = "glam")]
    #[test]
    fn test_pose_glam_conversion() {
        use glam::DAffine3;

        let pose = Transform3D::from_matrix([
            [1.0, 0.0, 0.0, 5.0],
            [0.0, 1.0, 0.0, 6.0],
            [0.0, 0.0, 1.0, 7.0],
            [0.0, 0.0, 0.0, 1.0],
        ]);
        let aff: DAffine3 = pose.into();
        assert_eq!(aff.translation[0], 5.0);
        let pose_from_aff: Transform3D<f64> = aff.into();

        assert_eq!(
            pose.to_matrix(),
            pose_from_aff.to_matrix(),
            "Glam conversion should be lossless"
        );
    }

    #[cfg(feature = "glam")]
    #[test]
    fn test_matrix_format_issue() {
        use glam::Mat4;

        // Test case: row-major matrix with translation in last column
        let row_major = [
            [1.0, 0.0, 0.0, 5.0], // row 0: x-axis + x translation
            [0.0, 1.0, 0.0, 6.0], // row 1: y-axis + y translation
            [0.0, 0.0, 1.0, 7.0], // row 2: z-axis + z translation
            [0.0, 0.0, 0.0, 1.0], // row 3: homogeneous
        ];

        // What glam expects: column-major format
        // Each inner array is a COLUMN, not a row
        let col_major = [
            [1.0, 0.0, 0.0, 0.0], // column 0: x-axis
            [0.0, 1.0, 0.0, 0.0], // column 1: y-axis
            [0.0, 0.0, 1.0, 0.0], // column 2: z-axis
            [5.0, 6.0, 7.0, 1.0], // column 3: translation + w
        ];

        // Create matrices
        let mat_from_row = Mat4::from_cols_array_2d(&row_major);
        let mat_from_col = Mat4::from_cols_array_2d(&col_major);

        // When using row-major data directly, translation ends up in wrong place
        assert_ne!(mat_from_row.w_axis.x, 5.0); // Translation is NOT where we expect

        // When using column-major data, translation is correct
        assert_eq!(mat_from_col.w_axis.x, 5.0);
        assert_eq!(mat_from_col.w_axis.y, 6.0);
        assert_eq!(mat_from_col.w_axis.z, 7.0);

        // The fix: transpose the row-major matrix
        let mat_transposed = Mat4::from_cols_array_2d(&row_major).transpose();
        assert_eq!(mat_transposed.w_axis.x, 5.0);
        assert_eq!(mat_transposed.w_axis.y, 6.0);
        assert_eq!(mat_transposed.w_axis.z, 7.0);
    }
}
