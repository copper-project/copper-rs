use avian3d::prelude::{Collider, Mass};

const GRAVITY_M_S2: f32 = 9.81;

/// Max acceleration in multiples of g at full power; tune to match sim stability.
pub const MOTOR_MAX_ACCEL_G: f32 = 10.0;
pub const MOTOR_MAX_ACCEL: f32 = MOTOR_MAX_ACCEL_G * GRAVITY_M_S2;

// Geometry/material constants shared between sim and resim.
pub const CART_WIDTH: f32 = 0.040;
pub const CART_HEIGHT: f32 = 0.045;
pub const CART_DEPTH: f32 = 0.06;

pub const ROD_WIDTH: f32 = 0.007;
pub const ROD_HEIGHT: f32 = 0.50;
#[allow(dead_code)]
pub const ROD_DEPTH: f32 = ROD_WIDTH;

pub const STEEL_DENSITY: f32 = 7800.0; // kg/m^3
pub const ALUMINUM_DENSITY: f32 = 2700.0; // kg/m^3

pub fn total_mass_kg() -> f32 {
    let cart_mass = Mass::from_shape(
        &Collider::cuboid(CART_WIDTH, CART_HEIGHT, CART_DEPTH),
        ALUMINUM_DENSITY,
    )
    .0;
    let rod_mass = Mass::from_shape(
        &Collider::capsule(ROD_WIDTH / 2.0, ROD_HEIGHT),
        STEEL_DENSITY,
    )
    .0;
    cart_mass + rod_mass
}

pub fn force_from_power(power: f32, mass_kg: f32) -> f32 {
    power * MOTOR_MAX_ACCEL * mass_kg
}
