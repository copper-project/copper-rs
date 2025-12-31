const GRAVITY_M_S2: f32 = 9.81;

/// Max acceleration in multiples of g at full power; tune to match sim stability.
pub const MOTOR_MAX_ACCEL_G: f32 = 10.0;
pub const MOTOR_MAX_ACCEL: f32 = MOTOR_MAX_ACCEL_G * GRAVITY_M_S2;

/// Mass estimate for resim logs (cart + rod). Keep in sync with sim geometry.
pub const ESTIMATED_TOTAL_MASS_KG: f32 = 0.44;

pub fn force_from_power(power: f32, mass_kg: f32) -> f32 {
    power * MOTOR_MAX_ACCEL * mass_kg
}
