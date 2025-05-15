use crate::error::{TransformError, TransformResult};
use crate::transform::StampedTransform;
use cu29::clock::CuTime;
use cu_spatial_payloads::Transform3D;
use std::fmt::Debug;

pub fn interpolate_transforms<T: Copy + Debug + Default + 'static + Into<f64>>(
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

    for i in 0..3 {
        for j in 0..3 {
            result.mat[i][j] = before.transform.mat[i][j];
        }
    }

    if std::any::type_name::<T>() == std::any::type_name::<f32>() {
        for i in 0..3 {
            let before_val: f64 = before.transform.mat[i][3].into();
            let after_val: f64 = after.transform.mat[i][3].into();
            let interpolated = before_val + (after_val - before_val) * ratio;

            let interpolated_f32 = interpolated as f32;
            let ptr = &interpolated_f32 as *const f32 as *const T;
            result.mat[i][3] = unsafe { *ptr };
        }
    } else {
        for i in 0..3 {
            result.mat[i][3] = before.transform.mat[i][3];
        }
    }

    result.mat[3][3] = before.transform.mat[3][3];

    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use cu29::clock::CuDuration;

    #[test]
    fn test_interpolate_transforms() {
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
