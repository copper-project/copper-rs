use bincode::{Decode, Encode};
use core::fmt::Debug;
use serde::{Deserialize, Serialize};
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
