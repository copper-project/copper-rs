/// Test utilities for creating and manipulating transforms
use cu_spatial_payloads::Transform3D;

/// Helper macro to create FrameIdString from string literal
#[macro_export]
macro_rules! frame_id {
    ($s:expr) => {
        $crate::FrameIdString::from($s).unwrap()
    };
}

/// Convert row-major matrix to column-major for glam compatibility
#[allow(dead_code)]
fn transpose_matrix<T: Copy>(mat: [[T; 4]; 4]) -> [[T; 4]; 4] {
    [
        [mat[0][0], mat[1][0], mat[2][0], mat[3][0]],
        [mat[0][1], mat[1][1], mat[2][1], mat[3][1]],
        [mat[0][2], mat[1][2], mat[2][2], mat[3][2]],
        [mat[0][3], mat[1][3], mat[2][3], mat[3][3]],
    ]
}

/// Create a translation transform
pub fn translation_transform<T>(x: T, y: T, z: T) -> Transform3D<T>
where
    T: Copy + std::fmt::Debug + Default + num_traits::Zero + num_traits::One + 'static,
{
    let zero = T::zero();
    let one = T::one();

    // Note: matrices are row-major, so translation is in the last column
    Transform3D::from_matrix([
        [one, zero, zero, x],
        [zero, one, zero, y],
        [zero, zero, one, z],
        [zero, zero, zero, one],
    ])
}

/// Create a 90-degree rotation around Z axis
#[allow(dead_code)]
pub fn rotation_z_90<T>() -> Transform3D<T>
where
    T: Copy
        + std::fmt::Debug
        + Default
        + num_traits::Zero
        + num_traits::One
        + std::ops::Neg<Output = T>
        + 'static,
{
    let zero = T::zero();
    let one = T::one();
    let neg_one = -one;

    // Note: matrices are row-major (R(z, 90°) = [[0, -1, 0], [1, 0, 0], [0, 0, 1]])
    Transform3D::from_matrix([
        [zero, neg_one, zero, zero],
        [one, zero, zero, zero],
        [zero, zero, one, zero],
        [zero, zero, zero, one],
    ])
}

/// Get translation components from a transform
pub fn get_translation<T>(transform: &Transform3D<T>) -> (T, T, T)
where
    T: Copy + std::fmt::Debug + Default + 'static,
{
    let mat = transform.to_matrix();
    // Note: matrices are row-major, so translation is in the last column
    (mat[0][3], mat[1][3], mat[2][3])
}

/// Set translation components on a transform
#[allow(dead_code)]
pub fn set_translation<T>(transform: &mut Transform3D<T>, x: T, y: T, z: T)
where
    T: Copy + std::fmt::Debug + Default + 'static,
{
    let mut mat = transform.to_matrix();
    // Note: matrices are row-major, so translation is in the last column
    mat[0][3] = x;
    mat[1][3] = y;
    mat[2][3] = z;
    *transform = Transform3D::from_matrix(mat);
}
