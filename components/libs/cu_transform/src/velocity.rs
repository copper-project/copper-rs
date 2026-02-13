use bincode::{Decode, Encode};
use core::fmt::Debug;
use cu29::units::si::angular_velocity::radian_per_second;
use cu29::units::si::f32::AngularVelocity as AngularVelocity32;
use cu29::units::si::f32::Velocity as Velocity32;
use cu29::units::si::f64::AngularVelocity as AngularVelocity64;
use cu29::units::si::f64::Velocity as Velocity64;
use cu29::units::si::velocity::meter_per_second;
use serde::{Deserialize, Serialize};

/// A struct representing linear and angular velocity in 3D space
///
/// The `VelocityTransform` encapsulates both linear and angular velocity components
/// of a rigid body in 3D space. It complements the `Transform3D` struct which
/// represents position and orientation.
///
/// Linear velocity is represented as a 3D vector [vx, vy, vz] in meters per second.
/// Angular velocity is represented as a 3D vector [wx, wy, wz] in radians per second,
/// where the vector represents the axis of rotation and the magnitude is the rate.
///
/// # Examples
///
/// ```
/// use cu_transform::VelocityTransform;
///
/// // Create a velocity with 1 m/s forward and 0.5 rad/s rotation around Z axis
/// let velocity = VelocityTransform::<f32> {
///     linear: [1.0, 0.0, 0.0],
///     angular: [0.0, 0.0, 0.5],
/// };
/// ```
///
/// Velocities can be transformed between coordinate frames using the
/// `transform_velocity` function, and new velocities can be computed from time-stamped
/// transforms using `StampedTransform::compute_velocity`.
///
/// When using the TransformTree to look up velocities with `lookup_velocity`, results
/// are automatically cached for improved performance on repeated lookups. The cache is
/// invalidated when new transforms are added or when entries exceed their age limit.
#[derive(Debug, Clone, Encode, Decode, Serialize, Deserialize)]
pub struct VelocityTransform<T: Copy + Debug + 'static> {
    /// Linear velocity components [vx, vy, vz] in meters per second
    pub linear: [T; 3],
    /// Angular velocity components [wx, wy, wz] in radians per second
    pub angular: [T; 3],
}

impl<T: Copy + Debug + Default + num_traits::Zero + 'static> VelocityTransform<T> {
    /// Create a new velocity transform with zero velocities
    pub fn zero() -> Self {
        Self {
            linear: [T::zero(); 3],
            angular: [T::zero(); 3],
        }
    }
}

// Generic implementations for all numeric types
impl<T> VelocityTransform<T>
where
    T: Copy
        + Debug
        + Default
        + 'static
        + std::ops::Add<Output = T>
        + std::ops::Sub<Output = T>
        + std::ops::Mul<Output = T>
        + std::ops::Div<Output = T>
        + std::ops::Neg<Output = T>,
{
    /// Add two velocity transforms component-wise
    pub fn add(&self, other: &Self) -> Self {
        Self {
            linear: [
                self.linear[0] + other.linear[0],
                self.linear[1] + other.linear[1],
                self.linear[2] + other.linear[2],
            ],
            angular: [
                self.angular[0] + other.angular[0],
                self.angular[1] + other.angular[1],
                self.angular[2] + other.angular[2],
            ],
        }
    }

    /// Subtract two velocity transforms component-wise
    pub fn sub(&self, other: &Self) -> Self {
        Self {
            linear: [
                self.linear[0] - other.linear[0],
                self.linear[1] - other.linear[1],
                self.linear[2] - other.linear[2],
            ],
            angular: [
                self.angular[0] - other.angular[0],
                self.angular[1] - other.angular[1],
                self.angular[2] - other.angular[2],
            ],
        }
    }

    /// Scale a velocity transform by a scalar
    pub fn scale(&self, scalar: T) -> Self {
        Self {
            linear: [
                self.linear[0] * scalar,
                self.linear[1] * scalar,
                self.linear[2] * scalar,
            ],
            angular: [
                self.angular[0] * scalar,
                self.angular[1] * scalar,
                self.angular[2] * scalar,
            ],
        }
    }

    /// Negate a velocity transform
    pub fn negate(&self) -> Self {
        Self {
            linear: [-self.linear[0], -self.linear[1], -self.linear[2]],
            angular: [-self.angular[0], -self.angular[1], -self.angular[2]],
        }
    }
}

// Specific implementations for f32 and f64 that provide unit conversions
impl VelocityTransform<f64> {
    /// Get linear velocity components with units
    pub fn linear_velocity(&self) -> [Velocity64; 3] {
        [
            Velocity64::new::<meter_per_second>(self.linear[0]),
            Velocity64::new::<meter_per_second>(self.linear[1]),
            Velocity64::new::<meter_per_second>(self.linear[2]),
        ]
    }

    /// Get angular velocity components with units
    pub fn angular_velocity(&self) -> [AngularVelocity64; 3] {
        [
            AngularVelocity64::new::<radian_per_second>(self.angular[0]),
            AngularVelocity64::new::<radian_per_second>(self.angular[1]),
            AngularVelocity64::new::<radian_per_second>(self.angular[2]),
        ]
    }
}

impl VelocityTransform<f32> {
    /// Get linear velocity components with units
    pub fn linear_velocity(&self) -> [Velocity32; 3] {
        [
            Velocity32::new::<meter_per_second>(self.linear[0]),
            Velocity32::new::<meter_per_second>(self.linear[1]),
            Velocity32::new::<meter_per_second>(self.linear[2]),
        ]
    }

    /// Get angular velocity components with units
    pub fn angular_velocity(&self) -> [AngularVelocity32; 3] {
        [
            AngularVelocity32::new::<radian_per_second>(self.angular[0]),
            AngularVelocity32::new::<radian_per_second>(self.angular[1]),
            AngularVelocity32::new::<radian_per_second>(self.angular[2]),
        ]
    }
}

impl<T: Copy + Debug + Default> Default for VelocityTransform<T> {
    fn default() -> Self {
        Self {
            linear: [T::default(); 3],
            angular: [T::default(); 3],
        }
    }
}

// Implement Add trait for VelocityTransform
impl<T> std::ops::Add for VelocityTransform<T>
where
    T: Copy + Debug + Default + 'static + std::ops::Add<Output = T>,
{
    type Output = Self;

    fn add(self, other: Self) -> Self::Output {
        Self {
            linear: [
                self.linear[0] + other.linear[0],
                self.linear[1] + other.linear[1],
                self.linear[2] + other.linear[2],
            ],
            angular: [
                self.angular[0] + other.angular[0],
                self.angular[1] + other.angular[1],
                self.angular[2] + other.angular[2],
            ],
        }
    }
}

// Implement Sub trait for VelocityTransform
impl<T> std::ops::Sub for VelocityTransform<T>
where
    T: Copy + Debug + Default + 'static + std::ops::Sub<Output = T>,
{
    type Output = Self;

    fn sub(self, other: Self) -> Self::Output {
        Self {
            linear: [
                self.linear[0] - other.linear[0],
                self.linear[1] - other.linear[1],
                self.linear[2] - other.linear[2],
            ],
            angular: [
                self.angular[0] - other.angular[0],
                self.angular[1] - other.angular[1],
                self.angular[2] - other.angular[2],
            ],
        }
    }
}

// Implement Neg trait for VelocityTransform
impl<T> std::ops::Neg for VelocityTransform<T>
where
    T: Copy + Debug + Default + 'static + std::ops::Neg<Output = T>,
{
    type Output = Self;

    fn neg(self) -> Self::Output {
        Self {
            linear: [-self.linear[0], -self.linear[1], -self.linear[2]],
            angular: [-self.angular[0], -self.angular[1], -self.angular[2]],
        }
    }
}

/// Transform a velocity from one frame to another
///
/// Given a velocity in frame A and a transform from frame A to frame B,
/// computes the equivalent velocity in frame B.
///
/// For a point moving with velocity v_a in frame A, the velocity v_b in frame B is:
/// v_b = R * v_a + ω × (R * p_a)
/// where:
/// - R is the rotation matrix from A to B
/// - ω is the angular velocity of frame A relative to frame B
/// - p_a is the position of the point in frame A
/// - × represents the cross product
///
/// For transform chain conversion, we follow similar rules as spatial transforms.
pub fn transform_velocity<T>(
    velocity: &VelocityTransform<T>,
    transform: &crate::Transform3D<T>,
    position: &[T; 3],
) -> VelocityTransform<T>
where
    T: Copy
        + Debug
        + Default
        + 'static
        + std::ops::Add<Output = T>
        + std::ops::Mul<Output = T>
        + std::ops::Sub<Output = T>
        + num_traits::NumCast,
{
    // Extract rotation matrix from transform
    let mat = transform.to_matrix();
    let rot = [
        [mat[0][0], mat[0][1], mat[0][2]],
        [mat[1][0], mat[1][1], mat[1][2]],
        [mat[2][0], mat[2][1], mat[2][2]],
    ];

    // First, transform the angular velocity by applying rotation
    let mut transformed_angular = [T::default(); 3];

    // Angular velocity transforms as: ω_b = R * ω_a
    #[allow(clippy::needless_range_loop)]
    for i in 0..3 {
        for j in 0..3 {
            transformed_angular[i] = transformed_angular[i] + rot[i][j] * velocity.angular[j];
        }
    }

    // Next, transform the linear velocity
    // First part: v'_linear = R * v_linear
    let mut transformed_linear = [T::default(); 3];
    #[allow(clippy::needless_range_loop)]
    for i in 0..3 {
        for j in 0..3 {
            transformed_linear[i] = transformed_linear[i] + rot[i][j] * velocity.linear[j];
        }
    }

    // Second part: Add ω × (R * p)
    // First calculate R * p
    let mut rotated_position = [T::default(); 3];
    for i in 0..3 {
        for (j, pos) in position.iter().enumerate() {
            rotated_position[i] = rotated_position[i] + rot[i][j] * *pos;
        }
    }

    // Then calculate cross product ω × (R * p)
    // (ω_x, ω_y, ω_z) × (p_x, p_y, p_z) = (ω_y*p_z - ω_z*p_y, ω_z*p_x - ω_x*p_z, ω_x*p_y - ω_y*p_x)
    let cross_product = [
        transformed_angular[1] * rotated_position[2] - transformed_angular[2] * rotated_position[1],
        transformed_angular[2] * rotated_position[0] - transformed_angular[0] * rotated_position[2],
        transformed_angular[0] * rotated_position[1] - transformed_angular[1] * rotated_position[0],
    ];

    // Add the cross product to the linear velocity
    transformed_linear[0] = transformed_linear[0] + cross_product[0];
    transformed_linear[1] = transformed_linear[1] + cross_product[1];
    transformed_linear[2] = transformed_linear[2] + cross_product[2];

    VelocityTransform {
        linear: transformed_linear,
        angular: transformed_angular,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_velocity_transform_default() {
        let vel: VelocityTransform<f32> = VelocityTransform::default();
        assert_eq!(vel.linear, [0.0, 0.0, 0.0]);
        assert_eq!(vel.angular, [0.0, 0.0, 0.0]);
    }

    #[test]
    fn test_velocity_transform_zero() {
        let vel = VelocityTransform::<f64>::zero();
        assert_eq!(vel.linear, [0.0, 0.0, 0.0]);
        assert_eq!(vel.angular, [0.0, 0.0, 0.0]);

        // Test generic zero() with other types
        let vel_f32 = VelocityTransform::<f32>::zero();
        assert_eq!(vel_f32.linear, [0.0, 0.0, 0.0]);
        assert_eq!(vel_f32.angular, [0.0, 0.0, 0.0]);
    }

    #[test]
    fn test_velocity_with_units() {
        let vel = VelocityTransform::<f32> {
            linear: [1.0, 2.0, 3.0],
            angular: [0.1, 0.2, 0.3],
        };

        let linear_vel = vel.linear_velocity();
        let angular_vel = vel.angular_velocity();

        // Check that units are correctly applied
        assert_eq!(linear_vel[0].get::<meter_per_second>(), 1.0);
        assert_eq!(linear_vel[1].get::<meter_per_second>(), 2.0);
        assert_eq!(linear_vel[2].get::<meter_per_second>(), 3.0);

        assert_eq!(angular_vel[0].get::<radian_per_second>(), 0.1);
        assert_eq!(angular_vel[1].get::<radian_per_second>(), 0.2);
        assert_eq!(angular_vel[2].get::<radian_per_second>(), 0.3);
    }

    #[test]
    fn test_transform_velocity() {
        // Create a velocity in frame A
        let vel_a = VelocityTransform {
            linear: [1.0f32, 0.0, 0.0], // Moving along x-axis at 1 m/s
            angular: [0.0, 0.0, 1.0],   // Rotating around z-axis at 1 rad/s
        };

        // Create a transform from frame A to frame B
        // 90-degree rotation around z-axis (cos(π/2)=0, sin(π/2)=1)
        let transform_a_to_b = crate::Transform3D::from_matrix([
            [0.0, -1.0, 0.0, 2.0], // 90-degree rotation with translation
            [1.0, 0.0, 0.0, 3.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]);

        // The point's position in frame A
        let position_a = [0.0f32, 0.0, 0.0]; // Position at origin

        // Transform the velocity from frame A to frame B
        let vel_b = transform_velocity(&vel_a, &transform_a_to_b, &position_a);

        // After 90-degree rotation:
        // - Linear velocity along x-axis (1,0,0) becomes (0,1,0)
        // - Angular velocity around z-axis stays (0,0,1)
        let epsilon = 1e-5;
        assert!(
            (vel_b.linear[0] - 0.0).abs() < epsilon,
            "X velocity should be 0 after rotation"
        );
        assert!(
            (vel_b.linear[1] - 1.0).abs() < epsilon,
            "Y velocity should be 1 after rotation"
        );
        assert!(
            (vel_b.linear[2] - 0.0).abs() < epsilon,
            "Z velocity should remain 0"
        );

        assert!(
            (vel_b.angular[0] - 0.0).abs() < epsilon,
            "X angular velocity should be 0"
        );
        assert!(
            (vel_b.angular[1] - 0.0).abs() < epsilon,
            "Y angular velocity should be 0"
        );
        assert!(
            (vel_b.angular[2] - 1.0).abs() < epsilon,
            "Z angular velocity should remain 1"
        );
    }

    #[test]
    fn test_transform_velocity_with_offset_point() {
        // Create a velocity in frame A
        let vel_a = VelocityTransform {
            linear: [0.0f32, 0.0, 0.0], // No linear velocity
            angular: [0.0, 0.0, 1.0],   // Rotating around z-axis at 1 rad/s
        };

        // Create a transform from frame A to frame B (identity rotation)
        let transform_a_to_b = crate::Transform3D::from_matrix([
            [1.0, 0.0, 0.0, 0.0], // Identity rotation
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]);

        // The point's position in frame A - 1 meter along x-axis
        let position_a = [1.0f32, 0.0, 0.0];

        // Transform the velocity from frame A to frame B
        let vel_b = transform_velocity(&vel_a, &transform_a_to_b, &position_a);

        // The point is offset 1m in x direction, with angular velocity 1 rad/s around z
        // This should create a linear velocity in the y direction of 1 m/s
        // v = ω × r = (0,0,1) × (1,0,0) = (0,1,0)
        let epsilon = 1e-5;
        assert!(
            (vel_b.linear[0] - 0.0).abs() < epsilon,
            "X velocity should be 0"
        );
        assert!(
            (vel_b.linear[1] - 1.0).abs() < epsilon,
            "Y velocity should be 1 due to cross product"
        );
        assert!(
            (vel_b.linear[2] - 0.0).abs() < epsilon,
            "Z velocity should be 0"
        );

        // Angular velocity should remain unchanged (no rotation in transform)
        assert!(
            (vel_b.angular[0] - 0.0).abs() < epsilon,
            "X angular velocity should be 0"
        );
        assert!(
            (vel_b.angular[1] - 0.0).abs() < epsilon,
            "Y angular velocity should be 0"
        );
        assert!(
            (vel_b.angular[2] - 1.0).abs() < epsilon,
            "Z angular velocity should remain 1"
        );
    }
}
