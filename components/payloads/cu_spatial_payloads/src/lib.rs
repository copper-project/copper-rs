use bincode::{Decode, Encode};
use core::fmt::Debug;
use serde::{Deserialize, Serialize};
use std::ops::Mul;
use uom::si::angle::radian;
use uom::si::length::meter;
pub type Pose<T> = Transform3D<T>;
use uom::si::f32::Angle as Angle32;
use uom::si::f32::Length as Length32;
use uom::si::f64::Angle as Angle64;
use uom::si::f64::Length as Length64;

#[derive(Debug, Clone, Encode, Decode, Serialize, Deserialize)]
pub struct Transform3D<T: Copy + Debug + 'static> {
    pub mat: [[T; 4]; 4],
}

impl Transform3D<f64> {
    pub fn translation(&self) -> [Length64; 3] {
        [
            Length64::new::<meter>(self.mat[0][3]),
            Length64::new::<meter>(self.mat[1][3]),
            Length64::new::<meter>(self.mat[2][3]),
        ]
    }

    pub fn rotation(&self) -> [[Angle64; 3]; 3] {
        [
            [
                Angle64::new::<radian>(self.mat[0][0]),
                Angle64::new::<radian>(self.mat[0][1]),
                Angle64::new::<radian>(self.mat[0][2]),
            ],
            [
                Angle64::new::<radian>(self.mat[1][0]),
                Angle64::new::<radian>(self.mat[1][1]),
                Angle64::new::<radian>(self.mat[1][2]),
            ],
            [
                Angle64::new::<radian>(self.mat[2][0]),
                Angle64::new::<radian>(self.mat[2][1]),
                Angle64::new::<radian>(self.mat[2][2]),
            ],
        ]
    }
}

impl Transform3D<f32> {
    pub fn translation(&self) -> [Length32; 3] {
        [
            Length32::new::<meter>(self.mat[0][3]),
            Length32::new::<meter>(self.mat[1][3]),
            Length32::new::<meter>(self.mat[2][3]),
        ]
    }

    pub fn rotation(&self) -> [[Angle32; 3]; 3] {
        [
            [
                Angle32::new::<radian>(self.mat[0][0]),
                Angle32::new::<radian>(self.mat[0][1]),
                Angle32::new::<radian>(self.mat[0][2]),
            ],
            [
                Angle32::new::<radian>(self.mat[1][0]),
                Angle32::new::<radian>(self.mat[1][1]),
                Angle32::new::<radian>(self.mat[1][2]),
            ],
            [
                Angle32::new::<radian>(self.mat[2][0]),
                Angle32::new::<radian>(self.mat[2][1]),
                Angle32::new::<radian>(self.mat[2][2]),
            ],
        ]
    }
}

impl<T: Copy + Debug + Default> Default for Transform3D<T> {
    fn default() -> Self {
        Self {
            mat: [[T::default(); 4]; 4],
        }
    }
}

/// Generic implementation of matrix multiplication for transforms
impl<T> Mul for Transform3D<T>
where
    T: Copy + Debug + 'static + Default + std::ops::Add<Output = T> + std::ops::Mul<Output = T>,
{
    type Output = Self;

    fn mul(self, rhs: Self) -> Self::Output {
        let mut result = Transform3D::default();

        for i in 0..4 {
            for j in 0..4 {
                let mut sum = T::default();
                for k in 0..4 {
                    sum = sum + (self.mat[i][k] * rhs.mat[k][j]);
                }
                result.mat[i][j] = sum;
            }
        }

        result
    }
}

/// Generic implementation of matrix multiplication on references for transforms
impl<T> Mul for &Transform3D<T>
where
    T: Copy + Debug + 'static + Default + std::ops::Add<Output = T> + std::ops::Mul<Output = T>,
{
    type Output = Transform3D<T>;

    fn mul(self, rhs: Self) -> Self::Output {
        let mut result = Transform3D::default();

        for i in 0..4 {
            for j in 0..4 {
                let mut sum = T::default();
                for k in 0..4 {
                    sum = sum + (self.mat[i][k] * rhs.mat[k][j]);
                }
                result.mat[i][j] = sum;
            }
        }

        result
    }
}

/// Generic implementation of matrix multiplication: reference * owned for transforms
impl<'a, T> Mul<Transform3D<T>> for &'a Transform3D<T>
where
    T: Copy + Debug + 'static + Default + std::ops::Add<Output = T> + std::ops::Mul<Output = T>,
{
    type Output = Transform3D<T>;

    fn mul(self, rhs: Transform3D<T>) -> Self::Output {
        let mut result = Transform3D::default();

        for i in 0..4 {
            for j in 0..4 {
                let mut sum = T::default();
                for k in 0..4 {
                    sum = sum + (self.mat[i][k] * rhs.mat[k][j]);
                }
                result.mat[i][j] = sum;
            }
        }

        result
    }
}

/// Generic implementation of matrix multiplication: owned * reference for transforms
impl<'a, T> Mul<&'a Transform3D<T>> for Transform3D<T>
where
    T: Copy + Debug + 'static + Default + std::ops::Add<Output = T> + std::ops::Mul<Output = T>,
{
    type Output = Transform3D<T>;

    fn mul(self, rhs: &'a Transform3D<T>) -> Self::Output {
        let mut result = Transform3D::default();

        for i in 0..4 {
            for j in 0..4 {
                let mut sum = T::default();
                for k in 0..4 {
                    sum = sum + (self.mat[i][k] * rhs.mat[k][j]);
                }
                result.mat[i][j] = sum;
            }
        }

        result
    }
}

impl Transform3D<f32> {
    /// Computes the inverse of this transformation matrix.
    /// For a valid homogeneous transformation matrix, this efficiently
    /// computes the inverse by using the block structure of the matrix.
    pub fn inverse(&self) -> Self {
        // Extract rotation matrix (top-left 3x3)
        let r = [
            [self.mat[0][0], self.mat[0][1], self.mat[0][2]],
            [self.mat[1][0], self.mat[1][1], self.mat[1][2]],
            [self.mat[2][0], self.mat[2][1], self.mat[2][2]],
        ];

        // Extract translation (top-right 3x1)
        let t = [self.mat[0][3], self.mat[1][3], self.mat[2][3]];

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

impl Transform3D<f64> {
    /// Computes the inverse of this transformation matrix.
    /// For a valid homogeneous transformation matrix, this efficiently
    /// computes the inverse by using the block structure of the matrix.
    pub fn inverse(&self) -> Self {
        // Extract rotation matrix (top-left 3x3)
        let r = [
            [self.mat[0][0], self.mat[0][1], self.mat[0][2]],
            [self.mat[1][0], self.mat[1][1], self.mat[1][2]],
            [self.mat[2][0], self.mat[2][1], self.mat[2][2]],
        ];

        // Extract translation (top-right 3x1)
        let t = [self.mat[0][3], self.mat[1][3], self.mat[2][3]];

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

#[cfg(feature = "faer")]
mod faer_integration {
    use super::Transform3D;
    use faer::prelude::*;

    impl From<&Transform3D<f64>> for Mat<f64> {
        fn from(p: &Transform3D<f64>) -> Self {
            let mut mat: Mat<f64> = Mat::zeros(4, 4);
            for r in 0..4 {
                for c in 0..4 {
                    *mat.get_mut(r, c) = p.mat[r][c];
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
            Self { mat: transform }
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
            let flat_transform: [f64; 16] = std::array::from_fn(|i| pose.mat[i / 4][i % 4]);
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
            Transform3D { mat: transform }
        }
    }
}

#[cfg(feature = "glam")]
mod glam_integration {
    use super::Transform3D;
    use glam::DAffine3;

    impl From<Transform3D<f64>> for DAffine3 {
        fn from(p: Transform3D<f64>) -> Self {
            let mut aff = DAffine3::IDENTITY;
            aff.matrix3.x_axis.x = p.mat[0][0];
            aff.matrix3.x_axis.y = p.mat[0][1];
            aff.matrix3.x_axis.z = p.mat[0][2];

            aff.matrix3.y_axis.x = p.mat[1][0];
            aff.matrix3.y_axis.y = p.mat[1][1];
            aff.matrix3.y_axis.z = p.mat[1][2];

            aff.matrix3.z_axis.x = p.mat[2][0];
            aff.matrix3.z_axis.y = p.mat[2][1];
            aff.matrix3.z_axis.z = p.mat[2][2];

            aff.translation.x = p.mat[0][3];
            aff.translation.y = p.mat[1][3];
            aff.translation.z = p.mat[2][3];

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

            Transform3D { mat: transform }
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
        assert_eq!(
            pose.mat, [[0.0; 4]; 4],
            "Default pose should be a zero matrix"
        );
    }

    #[test]
    fn test_transform_inverse_f32() {
        // Create a test transform with rotation and translation
        let transform = Transform3D::<f32> {
            mat: [
                [1.0, 0.0, 0.0, 2.0], // x-axis with 2m translation
                [0.0, 1.0, 0.0, 3.0], // y-axis with 3m translation
                [0.0, 0.0, 1.0, 4.0], // z-axis with 4m translation
                [0.0, 0.0, 0.0, 1.0], // homogeneous coordinate
            ],
        };

        // Compute inverse
        let inverse = transform.inverse();

        // Expected inverse for this transform
        let expected_inverse = Transform3D::<f32> {
            mat: [
                [1.0, 0.0, 0.0, -2.0], // Negated translation
                [0.0, 1.0, 0.0, -3.0],
                [0.0, 0.0, 1.0, -4.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
        };

        // Check each element with a small epsilon for floating-point comparison
        let epsilon = 1e-5;
        for i in 0..4 {
            for j in 0..4 {
                assert!(
                    (inverse.mat[i][j] - expected_inverse.mat[i][j]).abs() < epsilon,
                    "Element at [{},{}] differs: {} vs expected {}",
                    i,
                    j,
                    inverse.mat[i][j],
                    expected_inverse.mat[i][j]
                );
            }
        }
    }

    #[test]
    fn test_transform_inverse_f64() {
        // Create a test transform with rotation and translation
        let transform = Transform3D::<f64> {
            mat: [
                [0.0, -1.0, 0.0, 5.0], // 90-degree rotation around z with translation
                [1.0, 0.0, 0.0, 6.0],
                [0.0, 0.0, 1.0, 7.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
        };

        // Compute inverse
        let inverse = transform.inverse();

        // Expected inverse for this transform
        let expected_inverse = Transform3D::<f64> {
            mat: [
                [0.0, 1.0, 0.0, -6.0], // Transposed rotation and adjusted translation
                [-1.0, 0.0, 0.0, 5.0],
                [0.0, 0.0, 1.0, -7.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
        };

        // Check each element with a small epsilon for floating-point comparison
        let epsilon = 1e-10;
        for i in 0..4 {
            for j in 0..4 {
                assert!(
                    (inverse.mat[i][j] - expected_inverse.mat[i][j]).abs() < epsilon,
                    "Element at [{},{}] differs: {} vs expected {}",
                    i,
                    j,
                    inverse.mat[i][j],
                    expected_inverse.mat[i][j]
                );
            }
        }
    }

    #[test]
    fn test_transform_inverse_identity() {
        // Create identity transform
        let identity = Transform3D::<f32> {
            mat: [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
        };

        // Inverse of identity should be identity
        let inverse = identity.inverse();

        // Check if inverse is also identity
        let epsilon = 1e-5;
        for i in 0..4 {
            for j in 0..4 {
                assert!(
                    (inverse.mat[i][j] - identity.mat[i][j]).abs() < epsilon,
                    "Element at [{},{}] differs: {} vs expected {}",
                    i,
                    j,
                    inverse.mat[i][j],
                    identity.mat[i][j]
                );
            }
        }
    }

    #[test]
    fn test_transform_multiplication_f32() {
        // Create two transforms to multiply
        let t1 = Transform3D::<f32> {
            mat: [
                [1.0, 0.0, 0.0, 2.0], // Identity rotation + translation (2,3,4)
                [0.0, 1.0, 0.0, 3.0],
                [0.0, 0.0, 1.0, 4.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
        };

        let t2 = Transform3D::<f32> {
            mat: [
                [0.0, -1.0, 0.0, 5.0], // 90-degree rotation around z + translation (5,6,7)
                [1.0, 0.0, 0.0, 6.0],
                [0.0, 0.0, 1.0, 7.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
        };

        // Compute t1 * t2
        let result = t1 * t2;

        // Expected result: t1 * t2 represents first rotating by t2, then translating by t1
        let expected = Transform3D::<f32> {
            mat: [
                [0.0, -1.0, 0.0, 7.0], // Rotation from t2 + combined translation
                [1.0, 0.0, 0.0, 9.0],
                [0.0, 0.0, 1.0, 11.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
        };

        // Check results
        let epsilon = 1e-5;
        for i in 0..4 {
            for j in 0..4 {
                assert!(
                    (result.mat[i][j] - expected.mat[i][j]).abs() < epsilon,
                    "Element at [{},{}] differs: {} vs expected {}",
                    i,
                    j,
                    result.mat[i][j],
                    expected.mat[i][j]
                );
            }
        }
    }

    #[test]
    fn test_transform_multiplication_f64() {
        // Create two transforms to multiply
        let t1 = Transform3D::<f64> {
            mat: [
                [1.0, 0.0, 0.0, 2.0], // Identity rotation + translation (2,3,4)
                [0.0, 1.0, 0.0, 3.0],
                [0.0, 0.0, 1.0, 4.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
        };

        let t2 = Transform3D::<f64> {
            mat: [
                [0.0, -1.0, 0.0, 5.0], // 90-degree rotation around z + translation (5,6,7)
                [1.0, 0.0, 0.0, 6.0],
                [0.0, 0.0, 1.0, 7.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
        };

        // Compute t1 * t2
        let result = t1 * t2;

        // Expected result
        let expected = Transform3D::<f64> {
            mat: [
                [0.0, -1.0, 0.0, 7.0], // Rotation from t2 + combined translation
                [1.0, 0.0, 0.0, 9.0],
                [0.0, 0.0, 1.0, 11.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
        };

        // Check results
        let epsilon = 1e-10;
        for i in 0..4 {
            for j in 0..4 {
                assert!(
                    (result.mat[i][j] - expected.mat[i][j]).abs() < epsilon,
                    "Element at [{},{}] differs: {} vs expected {}",
                    i,
                    j,
                    result.mat[i][j],
                    expected.mat[i][j]
                );
            }
        }
    }

    #[test]
    fn test_transform_reference_multiplication() {
        // Test multiplication on references
        let t1 = Transform3D::<f32> {
            mat: [
                [1.0, 0.0, 0.0, 2.0],
                [0.0, 1.0, 0.0, 3.0],
                [0.0, 0.0, 1.0, 4.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
        };

        let t2 = Transform3D::<f32> {
            mat: [
                [0.0, -1.0, 0.0, 5.0],
                [1.0, 0.0, 0.0, 6.0],
                [0.0, 0.0, 1.0, 7.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
        };

        // Compute &t1 * &t2
        let result = &t1 * &t2;

        // Expected result
        let expected = Transform3D::<f32> {
            mat: [
                [0.0, -1.0, 0.0, 7.0],
                [1.0, 0.0, 0.0, 9.0],
                [0.0, 0.0, 1.0, 11.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
        };

        // Check results
        let epsilon = 1e-5;
        for i in 0..4 {
            for j in 0..4 {
                assert!(
                    (result.mat[i][j] - expected.mat[i][j]).abs() < epsilon,
                    "Element at [{},{}] differs: {} vs expected {}",
                    i,
                    j,
                    result.mat[i][j],
                    expected.mat[i][j]
                );
            }
        }
    }

    #[cfg(feature = "faer")]
    #[test]
    fn test_pose_faer_conversion() {
        use faer::prelude::*;

        let pose = Transform3D {
            mat: [
                [1.0, 2.0, 3.0, 4.0],
                [5.0, 6.0, 7.0, 8.0],
                [9.0, 10.0, 11.0, 12.0],
                [13.0, 14.0, 15.0, 16.0],
            ],
        };

        let mat: Mat<f64> = (&pose).into();
        let pose_from_mat = Transform3D::from(mat);

        assert_eq!(
            pose.mat, pose_from_mat.mat,
            "Faer conversion should be lossless"
        );
    }

    #[cfg(feature = "nalgebra")]
    #[test]
    fn test_pose_nalgebra_conversion() {
        use nalgebra::Isometry3;

        let pose = Transform3D {
            mat: [
                [1.0, 0.0, 0.0, 2.0],
                [0.0, 1.0, 0.0, 3.0],
                [0.0, 0.0, 1.0, 4.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
        };

        let iso: Isometry3<f64> = (&pose.clone()).into();
        let pose_from_iso: Transform3D<f64> = iso.into();

        assert_eq!(
            pose.mat, pose_from_iso.mat,
            "Nalgebra conversion should be lossless"
        );
    }

    #[cfg(feature = "glam")]
    #[test]
    fn test_pose_glam_conversion() {
        use glam::DAffine3;

        let orig_pose = Transform3D {
            mat: [
                [1.0, 0.0, 0.0, 5.0],
                [0.0, 1.0, 0.0, 6.0],
                [0.0, 0.0, 1.0, 7.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
        };
        let pose = orig_pose.clone();
        assert_eq!(pose.mat, orig_pose.mat);
        let aff: DAffine3 = pose.into();
        assert_eq!(aff.translation[0], 5.0);
        let pose_from_aff: Transform3D<f64> = aff.into();

        assert_eq!(
            orig_pose.mat, pose_from_aff.mat,
            "Glam conversion should be lossless"
        );
    }
}
