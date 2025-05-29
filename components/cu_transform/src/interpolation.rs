use crate::error::{TransformError, TransformResult};
use crate::transform::StampedTransform;
use cu29::clock::CuTime;
use cu_spatial_payloads::Transform3D;
use std::fmt::Debug;

/// Trait for numeric types that can be interpolated
pub trait Interpolate: Copy + Debug + Default + 'static {
    /// Convert to f64 for intermediate calculations
    fn to_f64(self) -> f64;
    /// Create from f64 after interpolation calculations
    fn from_f64(val: f64) -> Self;
}

// Implement for common numeric types
impl Interpolate for f32 {
    fn to_f64(self) -> f64 {
        self as f64
    }
    fn from_f64(val: f64) -> Self {
        val as f32
    }
}

impl Interpolate for f64 {
    fn to_f64(self) -> f64 {
        self
    }
    fn from_f64(val: f64) -> Self {
        val
    }
}

impl Interpolate for i32 {
    fn to_f64(self) -> f64 {
        self as f64
    }
    fn from_f64(val: f64) -> Self {
        val.round() as i32
    }
}

impl Interpolate for i64 {
    fn to_f64(self) -> f64 {
        self as f64
    }
    fn from_f64(val: f64) -> Self {
        val.round() as i64
    }
}

impl Interpolate for u32 {
    fn to_f64(self) -> f64 {
        self as f64
    }
    fn from_f64(val: f64) -> Self {
        val.round() as u32
    }
}

impl Interpolate for u64 {
    fn to_f64(self) -> f64 {
        self as f64
    }
    fn from_f64(val: f64) -> Self {
        val.round() as u64
    }
}

/// Interpolate between two transforms at a specific time point
///
/// This function performs linear interpolation between two transforms for the translation
/// components (last column of the transformation matrix) at the specified time point.
/// The rotation components are not interpolated and are copied from the first transform.
///
/// Works with any numeric type implementing the `Interpolate` trait, including both
/// floating-point (f32, f64) and integer types (i32, i64, u32, u64).
///
/// # Arguments
/// * `before` - The transform at an earlier time
/// * `after` - The transform at a later time
/// * `time` - The time at which to interpolate (must be between before.stamp and after.stamp)
///
/// # Returns
/// * A new interpolated transform at the specified time
/// * Error if frames don't match or time is outside the valid range
pub fn interpolate_transforms<T: Interpolate>(
    before: &StampedTransform<T>,
    after: &StampedTransform<T>,
    time: CuTime,
) -> TransformResult<Transform3D<T>> {
    if before.parent_frame != after.parent_frame || before.child_frame != after.child_frame {
        return Err(TransformError::InterpolationError(
            "Cannot interpolate between different frame pairs".to_string(),
        ));
    }

    if time < before.stamp || time > after.stamp {
        return Err(TransformError::InterpolationError(
            "Requested time is outside the range of the transforms".to_string(),
        ));
    }

    let before_nanos = before.stamp.as_nanos() as f64;
    let after_nanos = after.stamp.as_nanos() as f64;
    let time_nanos = time.as_nanos() as f64;

    let ratio = (time_nanos - before_nanos) / (after_nanos - before_nanos);

    let mut result = Transform3D::default();

    // Copy rotation matrix from before transform (no interpolation for rotation yet)
    for i in 0..3 {
        for j in 0..3 {
            result.mat[i][j] = before.transform.mat[i][j];
        }
    }

    // Interpolate translation (last column of matrix)
    for i in 0..3 {
        let before_val = before.transform.mat[i][3].to_f64();
        let after_val = after.transform.mat[i][3].to_f64();
        let interpolated = before_val + (after_val - before_val) * ratio;
        result.mat[i][3] = T::from_f64(interpolated);
    }

    // Copy homogeneous component
    result.mat[3][3] = before.transform.mat[3][3];

    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use cu29::clock::CuDuration;

    #[test]
    fn test_interpolate_transforms_f32() {
        let mut before: StampedTransform<f32> = StampedTransform {
            transform: Transform3D::default(),
            stamp: CuDuration(1000),
            parent_frame: "world".to_string(),
            child_frame: "robot".to_string(),
        };

        let mut after: StampedTransform<f32> = StampedTransform {
            transform: Transform3D::default(),
            stamp: CuDuration(3000),
            parent_frame: "world".to_string(),
            child_frame: "robot".to_string(),
        };

        before.transform.mat[0][3] = 0.0;
        after.transform.mat[0][3] = 10.0;

        let result = interpolate_transforms(&before, &after, CuDuration(2000));
        assert!(result.is_ok());

        let transform = result.unwrap();
        assert_relative_eq!(transform.mat[0][3], 5.0);

        let result = interpolate_transforms(&before, &after, CuDuration(1500));
        assert!(result.is_ok());

        let transform = result.unwrap();
        assert_relative_eq!(transform.mat[0][3], 2.5);

        let result = interpolate_transforms(&before, &after, CuDuration(2500));
        assert!(result.is_ok());

        let transform = result.unwrap();
        assert_relative_eq!(transform.mat[0][3], 7.5);
    }

    #[test]
    fn test_interpolate_transforms_f64() {
        let mut before: StampedTransform<f64> = StampedTransform {
            transform: Transform3D::default(),
            stamp: CuDuration(1000),
            parent_frame: "world".to_string(),
            child_frame: "robot".to_string(),
        };

        let mut after: StampedTransform<f64> = StampedTransform {
            transform: Transform3D::default(),
            stamp: CuDuration(3000),
            parent_frame: "world".to_string(),
            child_frame: "robot".to_string(),
        };

        before.transform.mat[0][3] = 0.0;
        after.transform.mat[0][3] = 10.0;

        // Test at midpoint
        let result = interpolate_transforms(&before, &after, CuDuration(2000));
        assert!(result.is_ok());
        let transform = result.unwrap();
        assert_relative_eq!(transform.mat[0][3], 5.0);
    }

    #[test]
    fn test_interpolate_transforms_integer() {
        // Test with i32
        let mut before: StampedTransform<i32> = StampedTransform {
            transform: Transform3D::default(),
            stamp: CuDuration(1000),
            parent_frame: "world".to_string(),
            child_frame: "robot".to_string(),
        };

        let mut after: StampedTransform<i32> = StampedTransform {
            transform: Transform3D::default(),
            stamp: CuDuration(3000),
            parent_frame: "world".to_string(),
            child_frame: "robot".to_string(),
        };

        before.transform.mat[0][3] = 0;
        after.transform.mat[0][3] = 10;

        // Test at midpoint
        let result = interpolate_transforms(&before, &after, CuDuration(2000));
        assert!(result.is_ok());
        let transform = result.unwrap();
        assert_eq!(transform.mat[0][3], 5);

        // Test with non-integer result (should round)
        let result = interpolate_transforms(&before, &after, CuDuration(1500));
        assert!(result.is_ok());
        let transform = result.unwrap();
        assert_eq!(transform.mat[0][3], 3); // 2.5 rounds to 3
    }

    #[test]
    fn test_interpolate_transforms_u64() {
        // Test with u64
        let mut before: StampedTransform<u64> = StampedTransform {
            transform: Transform3D::default(),
            stamp: CuDuration(1000),
            parent_frame: "world".to_string(),
            child_frame: "robot".to_string(),
        };

        let mut after: StampedTransform<u64> = StampedTransform {
            transform: Transform3D::default(),
            stamp: CuDuration(3000),
            parent_frame: "world".to_string(),
            child_frame: "robot".to_string(),
        };

        // Set large values to test u64 range
        before.transform.mat[0][3] = 1_000_000_000;
        after.transform.mat[0][3] = 2_000_000_000;

        // Test at 75% point
        let result = interpolate_transforms(&before, &after, CuDuration(2500));
        assert!(result.is_ok());
        let transform = result.unwrap();
        assert_eq!(transform.mat[0][3], 1_750_000_000);
    }

    #[test]
    fn test_interpolate_transforms_errors() {
        let before: StampedTransform<f32> = StampedTransform {
            transform: Transform3D::default(),
            stamp: CuDuration(1000),
            parent_frame: "world".to_string(),
            child_frame: "robot".to_string(),
        };

        let after: StampedTransform<f32> = StampedTransform {
            transform: Transform3D::default(),
            stamp: CuDuration(3000),
            parent_frame: "different".to_string(),
            child_frame: "robot".to_string(),
        };

        let result = interpolate_transforms(&before, &after, CuDuration(2000));
        assert!(result.is_err());

        let after: StampedTransform<f32> = StampedTransform {
            transform: Transform3D::default(),
            stamp: CuDuration(3000),
            parent_frame: "world".to_string(),
            child_frame: "different".to_string(),
        };

        let result = interpolate_transforms(&before, &after, CuDuration(2000));
        assert!(result.is_err());

        let after: StampedTransform<f32> = StampedTransform {
            transform: Transform3D::default(),
            stamp: CuDuration(3000),
            parent_frame: "world".to_string(),
            child_frame: "robot".to_string(),
        };

        let result = interpolate_transforms(&before, &after, CuDuration(500));
        assert!(result.is_err());

        let result = interpolate_transforms(&before, &after, CuDuration(3500));
        assert!(result.is_err());
    }
}
