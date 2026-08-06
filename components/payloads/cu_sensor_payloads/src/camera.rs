use bincode::{Decode, Encode};
use core::fmt;
use cu29::prelude::*;
use serde::{Deserialize, Serialize};

/// Errors found while validating a camera model.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CuCameraModelError {
    ZeroImageDimension,
    NonFiniteIntrinsic,
    NonPositiveFocalLength,
    NonFiniteDistortionCoefficient { index: usize },
}

impl fmt::Display for CuCameraModelError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ZeroImageDimension => write!(f, "camera image dimensions must be non-zero"),
            Self::NonFiniteIntrinsic => write!(f, "camera intrinsics must be finite"),
            Self::NonPositiveFocalLength => {
                write!(f, "camera focal lengths must be positive")
            }
            Self::NonFiniteDistortionCoefficient { index } => {
                write!(f, "camera distortion coefficient {index} is not finite")
            }
        }
    }
}

impl core::error::Error for CuCameraModelError {}

/// Pinhole camera intrinsics in pixel units.
///
/// Pixel centers use the conventional integer coordinate system. Image bounds
/// therefore run from `-0.5` to `width - 0.5` horizontally and from `-0.5` to
/// `height - 0.5` vertically. Keeping this convention explicit avoids the
/// half-pixel field-of-view errors that otherwise appear when consumers use a
/// different interpretation of `cx` and `cy`. A principal point outside those
/// bounds is allowed because cropped sensors can legitimately place the optical
/// axis outside the delivered image.
#[derive(
    Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize, Encode, Decode, Reflect,
)]
pub struct CuCameraIntrinsics {
    pub width: u32,
    pub height: u32,
    pub fx: f32,
    pub fy: f32,
    pub cx: f32,
    pub cy: f32,
    pub skew: f32,
}

impl CuCameraIntrinsics {
    pub fn new(
        width: u32,
        height: u32,
        fx: f32,
        fy: f32,
        cx: f32,
        cy: f32,
        skew: f32,
    ) -> Result<Self, CuCameraModelError> {
        let intrinsics = Self {
            width,
            height,
            fx,
            fy,
            cx,
            cy,
            skew,
        };
        intrinsics.validate()?;
        Ok(intrinsics)
    }

    pub fn validate(&self) -> Result<(), CuCameraModelError> {
        if self.width == 0 || self.height == 0 {
            return Err(CuCameraModelError::ZeroImageDimension);
        }
        if ![self.fx, self.fy, self.cx, self.cy, self.skew]
            .iter()
            .all(|value| value.is_finite())
        {
            return Err(CuCameraModelError::NonFiniteIntrinsic);
        }
        if self.fx <= 0.0 || self.fy <= 0.0 {
            return Err(CuCameraModelError::NonPositiveFocalLength);
        }

        Ok(())
    }

    /// Horizontal field of view in radians, measured between the outer pixel edges.
    pub fn horizontal_fov_rad(&self) -> Result<f32, CuCameraModelError> {
        self.validate()?;
        let left_extent = self.cx + 0.5;
        let right_extent = self.width as f32 - 0.5 - self.cx;
        Ok(libm::atanf(left_extent / self.fx) + libm::atanf(right_extent / self.fx))
    }

    /// Vertical field of view in radians, measured between the outer pixel edges.
    pub fn vertical_fov_rad(&self) -> Result<f32, CuCameraModelError> {
        self.validate()?;
        let top_extent = self.cy + 0.5;
        let bottom_extent = self.height as f32 - 0.5 - self.cy;
        Ok(libm::atanf(top_extent / self.fy) + libm::atanf(bottom_extent / self.fy))
    }

    /// Convert a rectified pixel coordinate into an unnormalized camera-frame ray.
    ///
    /// The result is `[x, y, 1]`. Lens distortion is intentionally not applied;
    /// callers should rectify the pixel according to the model's distortion type first.
    pub fn rectified_pixel_ray(&self, pixel: [f32; 2]) -> Result<[f32; 3], CuCameraModelError> {
        self.validate()?;
        let y = (pixel[1] - self.cy) / self.fy;
        let x = (pixel[0] - self.cx - self.skew * y) / self.fx;
        Ok([x, y, 1.0])
    }
}

/// Compile-time lens-distortion model for a camera.
///
/// The concrete distortion type is part of [`CuCameraModel`]'s Rust type. A
/// running robot may update the coefficients of its chosen calibration model,
/// but it cannot silently switch from (for example) `plumb_bob` to `equidistant`.
pub trait CuCameraDistortion {
    fn coefficients(&self) -> &[f32];

    fn validate(&self) -> Result<(), CuCameraModelError> {
        for (index, coefficient) in self.coefficients().iter().enumerate() {
            if !coefficient.is_finite() {
                return Err(CuCameraModelError::NonFiniteDistortionCoefficient { index });
            }
        }
        Ok(())
    }
}

/// Camera without a lens-distortion correction model.
#[derive(
    Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, Encode, Decode, Reflect,
)]
pub struct CuNoDistortion;

impl CuCameraDistortion for CuNoDistortion {
    fn coefficients(&self) -> &[f32] {
        &[]
    }
}

macro_rules! define_camera_distortion {
    ($(#[$meta:meta])* $name:ident, $coefficient_count:literal) => {
        $(#[$meta])*
        ///
        /// The coefficient count is encoded in the constructor's array type, so
        /// an invalid count is rejected by the Rust compiler rather than at runtime.
        #[derive(
            Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize, Encode, Decode, Reflect,
        )]
        pub struct $name {
            coefficients: [f32; $coefficient_count],
        }

        impl $name {
            pub fn new(
                coefficients: [f32; $coefficient_count],
            ) -> Result<Self, CuCameraModelError> {
                let distortion = Self { coefficients };
                distortion.validate()?;
                Ok(distortion)
            }

            pub const fn coefficients(&self) -> &[f32; $coefficient_count] {
                &self.coefficients
            }
        }

        impl CuCameraDistortion for $name {
            fn coefficients(&self) -> &[f32] {
                &self.coefficients
            }
        }
    };
}

define_camera_distortion!(
    /// Brown-Conrady / ROS `plumb_bob`: `[k1, k2, t1, t2, k3]`.
    ///
    /// ```compile_fail
    /// use cu_sensor_payloads::CuPlumbBobDistortion;
    /// let _ = CuPlumbBobDistortion::new([0.0; 4]);
    /// ```
    CuPlumbBobDistortion,
    5
);

define_camera_distortion!(
    /// OpenCV rational polynomial: `[k1, k2, t1, t2, k3, k4, k5, k6]`.
    CuRationalPolynomialDistortion,
    8
);

define_camera_distortion!(
    /// OpenCV fisheye / ROS `equidistant`: `[k1, k2, k3, k4]`.
    CuEquidistantDistortion,
    4
);

/// Standard camera geometry shared by image and depth-map producers.
///
/// `D` fixes the distortion model at compile time. A source performing dynamic
/// calibration should publish this as [`CuCameraModelUpdate::Set`] when its
/// intrinsics or coefficients first become available or change, then publish
/// `NoChange` on later cycles. Consumers keep a [`CuCameraModelState`] cache.
/// This transmits the full calibration only on state transitions while keeping
/// every change in Copper's deterministic log and replay stream.
#[derive(
    Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize, Encode, Decode, Reflect,
)]
pub struct CuCameraModel<D = CuNoDistortion> {
    pub intrinsics: CuCameraIntrinsics,
    pub distortion: D,
}

impl<D> CuCameraModel<D>
where
    D: CuCameraDistortion,
{
    pub fn new(intrinsics: CuCameraIntrinsics, distortion: D) -> Result<Self, CuCameraModelError> {
        let model = Self {
            intrinsics,
            distortion,
        };
        model.validate()?;
        Ok(model)
    }

    pub fn validate(&self) -> Result<(), CuCameraModelError> {
        self.intrinsics.validate()?;
        self.distortion.validate()
    }
}

/// Producer-side dynamic calibration update carried on a dedicated Copper output.
pub type CuCameraModelUpdate<D = CuNoDistortion> = CuLatchedStateUpdate<CuCameraModel<D>>;

/// Consumer-side cache for the latest dynamic calibration.
pub type CuCameraModelState<D = CuNoDistortion> = CuLatchedState<CuCameraModel<D>>;

#[cfg(test)]
mod tests {
    use super::*;
    use bincode::config;
    use core::f32::consts::FRAC_PI_2;

    fn centered_intrinsics() -> CuCameraIntrinsics {
        CuCameraIntrinsics::new(640, 480, 320.0, 240.0, 319.5, 239.5, 0.0).unwrap()
    }

    #[test]
    fn centered_intrinsics_compute_ninety_degree_fov() {
        let intrinsics = centered_intrinsics();
        assert!((intrinsics.horizontal_fov_rad().unwrap() - FRAC_PI_2).abs() < 1.0e-6);
        assert!((intrinsics.vertical_fov_rad().unwrap() - FRAC_PI_2).abs() < 1.0e-6);
    }

    #[test]
    fn rectified_pixel_ray_accounts_for_skew() {
        let intrinsics =
            CuCameraIntrinsics::new(640, 480, 400.0, 200.0, 300.0, 200.0, 10.0).unwrap();
        let ray = intrinsics.rectified_pixel_ray([412.0, 240.0]).unwrap();
        assert_eq!(ray, [0.275, 0.2, 1.0]);
    }

    #[test]
    fn invalid_intrinsics_are_rejected() {
        assert_eq!(
            CuCameraIntrinsics::new(640, 480, 0.0, 240.0, 319.5, 239.5, 0.0),
            Err(CuCameraModelError::NonPositiveFocalLength)
        );
        assert_eq!(
            CuCameraIntrinsics::new(640, 480, 320.0, 240.0, f32::NAN, 239.5, 0.0),
            Err(CuCameraModelError::NonFiniteIntrinsic)
        );
    }

    #[test]
    fn cropped_camera_may_have_principal_point_outside_image() {
        let intrinsics = CuCameraIntrinsics::new(640, 480, 320.0, 240.0, -0.5, 239.5, 0.0).unwrap();
        let expected = libm::atanf(2.0);
        assert!((intrinsics.horizontal_fov_rad().unwrap() - expected).abs() < 1.0e-6);
    }

    #[test]
    fn distortion_models_have_compile_time_coefficient_counts() {
        let plumb_bob = CuPlumbBobDistortion::new([0.1, -0.02, 0.001, -0.001, 0.0]).unwrap();
        let rational = CuRationalPolynomialDistortion::new([0.0; 8]).unwrap();
        let equidistant = CuEquidistantDistortion::new([0.0; 4]).unwrap();

        assert_eq!(plumb_bob.coefficients().len(), 5);
        assert_eq!(rational.coefficients().len(), 8);
        assert_eq!(equidistant.coefficients().len(), 4);
        assert_eq!(
            CuEquidistantDistortion::new([0.0, f32::NAN, 0.0, 0.0]),
            Err(CuCameraModelError::NonFiniteDistortionCoefficient { index: 1 })
        );
    }

    #[test]
    fn camera_model_round_trips_through_bincode() {
        let model = CuCameraModel::new(centered_intrinsics(), CuNoDistortion).unwrap();
        let cfg = config::standard();
        let mut buffer = [0_u8; 256];
        let len = bincode::encode_into_slice(model, &mut buffer, cfg).unwrap();
        let (decoded, used) =
            bincode::decode_from_slice::<CuCameraModel<CuNoDistortion>, _>(&buffer[..len], cfg)
                .unwrap();
        assert_eq!(used, len);
        assert_eq!(decoded, model);
    }

    #[test]
    fn latched_updates_send_full_model_only_on_change() {
        let model = CuCameraModel::new(
            centered_intrinsics(),
            CuPlumbBobDistortion::new([0.1, -0.02, 0.001, -0.001, 0.0]).unwrap(),
        )
        .unwrap();
        let cfg = config::standard();
        let set = bincode::encode_to_vec(CuCameraModelUpdate::Set(model), cfg).unwrap();
        let no_change =
            bincode::encode_to_vec(CuCameraModelUpdate::<CuPlumbBobDistortion>::NoChange, cfg)
                .unwrap();
        assert!(no_change.len() < set.len());

        let mut state = CuCameraModelState::default();
        state.update_owned(CuCameraModelUpdate::Set(model));
        state.update_owned(CuCameraModelUpdate::NoChange);
        assert_eq!(state.get(), Some(&model));
        state.update_owned(CuCameraModelUpdate::Clear);
        assert!(state.is_unset());
    }
}
