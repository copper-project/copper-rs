use core::fmt::Debug;

use rerun::components::TransformMat3x3;
use rerun::datatypes::Mat3x3;
use rerun::{AsComponents, SerializedComponentBatch, Transform3D as RerunTransform3D};

use crate::Transform3D;

fn transform_parts<T: Copy + Into<f64> + Debug + Default + 'static>(
    transform: &Transform3D<T>,
) -> ([f32; 3], [[f32; 3]; 3]) {
    let matrix = (*transform).to_matrix();
    let translation = [
        matrix[3][0].into() as f32,
        matrix[3][1].into() as f32,
        matrix[3][2].into() as f32,
    ];
    let mat3 = [
        [
            matrix[0][0].into() as f32,
            matrix[0][1].into() as f32,
            matrix[0][2].into() as f32,
        ],
        [
            matrix[1][0].into() as f32,
            matrix[1][1].into() as f32,
            matrix[1][2].into() as f32,
        ],
        [
            matrix[2][0].into() as f32,
            matrix[2][1].into() as f32,
            matrix[2][2].into() as f32,
        ],
    ];

    (translation, mat3)
}

fn build_rerun_transform<T: Copy + Into<f64> + Debug + Default + 'static>(
    transform: &Transform3D<T>,
) -> RerunTransform3D {
    let (translation, mat3) = transform_parts(transform);
    let mat_flat = [
        mat3[0][0], mat3[0][1], mat3[0][2], mat3[1][0], mat3[1][1], mat3[1][2], mat3[2][0],
        mat3[2][1], mat3[2][2],
    ];

    RerunTransform3D::new()
        .with_translation(translation)
        .with_mat3x3(TransformMat3x3::from(Mat3x3(mat_flat)))
}

impl AsComponents for Transform3D<f32> {
    fn as_serialized_batches(&self) -> Vec<SerializedComponentBatch> {
        build_rerun_transform(self).as_serialized_batches()
    }
}

impl AsComponents for Transform3D<f64> {
    fn as_serialized_batches(&self) -> Vec<SerializedComponentBatch> {
        build_rerun_transform(self).as_serialized_batches()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn transform_parts_uses_fourth_row_for_translation() {
        let transform = Transform3D::<f32>::from_matrix([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [10.0, 20.0, 30.0, 1.0],
        ]);

        let (translation, _mat3) = transform_parts(&transform);
        assert_eq!(translation, [10.0, 20.0, 30.0]);
    }

    #[test]
    fn transform3d_emits_rerun_components() {
        let transform = Transform3D::<f32>::from_matrix([
            [1.0, 0.0, 0.0, 0.1],
            [0.0, 1.0, 0.0, 0.2],
            [0.0, 0.0, 1.0, 0.3],
            [0.0, 0.0, 0.0, 1.0],
        ]);

        assert!(!transform.as_serialized_batches().is_empty());
    }
}
