//! Copper-native SI quantity wrappers.
//!
//! Feature flags:
//! - `default` = `["std"]`
//! - `std`: enables `uom/std`
//! - `reflect`: enables `bevy_reflect` support on wrapper types
//! - `textlogs`: compatibility no-op for downstream feature forwarding
//!
#![cfg_attr(not(feature = "std"), no_std)]

extern crate alloc;

pub use uom;

macro_rules! define_storage_wrappers {
    ($storage_mod:ident, $storage_ty:ty, [$(($id:literal, $unit_mod:ident, $quantity:ident, $symbol:literal),)+]) => {
        pub mod $storage_mod {
            use core::marker::PhantomData;
            use serde::{Deserialize, Deserializer, Serialize, Serializer};

            #[cfg(feature = "reflect")]
            use bevy_reflect::Reflect;

            macro_rules! define_quantity {
                ($unit_mod_name:ident, $quantity_name:ident, $unit_symbol:literal) => {
                    #[repr(transparent)]
                    #[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
                    #[cfg_attr(feature = "reflect", derive(Reflect))]
                    #[cfg_attr(feature = "reflect", reflect(from_reflect = false))]
                    pub struct $quantity_name {
                        pub value: $storage_ty,
                    }

                    impl $quantity_name {
                        const STORAGE_UNIT: &'static str = $unit_symbol;

                        #[inline]
                        pub fn new<U>(value: $storage_ty) -> Self
                        where
                            U: uom::si::$unit_mod_name::Conversion<$storage_ty>,
                        {
                            Self::from_uom(uom::si::$storage_mod::$quantity_name::new::<U>(value))
                        }

                        #[inline]
                        pub fn get<U>(&self) -> $storage_ty
                        where
                            U: uom::si::$unit_mod_name::Conversion<$storage_ty>,
                        {
                            (*self).to_uom().get::<U>()
                        }

                        #[inline]
                        pub fn raw(&self) -> $storage_ty {
                            self.value
                        }

                        #[inline]
                        pub fn into_uom(self) -> uom::si::$storage_mod::$quantity_name {
                            self.to_uom()
                        }

                        #[inline]
                        pub fn as_uom(&self) -> uom::si::$storage_mod::$quantity_name {
                            (*self).to_uom()
                        }

                        #[inline]
                        pub fn from_uom(inner: uom::si::$storage_mod::$quantity_name) -> Self {
                            Self::from_base_value(inner.value)
                        }

                        #[inline]
                        fn to_uom(self) -> uom::si::$storage_mod::$quantity_name {
                            uom::si::$storage_mod::$quantity_name {
                                dimension: PhantomData,
                                units: PhantomData,
                                value: self.value,
                            }
                        }

                        #[inline]
                        fn from_base_value(value: $storage_ty) -> Self {
                            Self { value }
                        }
                    }

                    impl Default for $quantity_name {
                        fn default() -> Self {
                            Self::from_base_value(0.0 as $storage_ty)
                        }
                    }

                    impl From<uom::si::$storage_mod::$quantity_name> for $quantity_name {
                        fn from(value: uom::si::$storage_mod::$quantity_name) -> Self {
                            Self::from_uom(value)
                        }
                    }

                    impl From<$quantity_name> for uom::si::$storage_mod::$quantity_name {
                        fn from(value: $quantity_name) -> Self {
                            value.into_uom()
                        }
                    }

                    impl core::ops::Add for $quantity_name {
                        type Output = Self;

                        fn add(self, rhs: Self) -> Self::Output {
                            Self::from_base_value(self.raw() + rhs.raw())
                        }
                    }

                    impl core::ops::AddAssign for $quantity_name {
                        fn add_assign(&mut self, rhs: Self) {
                            *self = *self + rhs;
                        }
                    }

                    impl core::ops::Sub for $quantity_name {
                        type Output = Self;

                        fn sub(self, rhs: Self) -> Self::Output {
                            Self::from_base_value(self.raw() - rhs.raw())
                        }
                    }

                    impl core::ops::SubAssign for $quantity_name {
                        fn sub_assign(&mut self, rhs: Self) {
                            *self = *self - rhs;
                        }
                    }

                    impl core::ops::Mul<$storage_ty> for $quantity_name {
                        type Output = Self;

                        fn mul(self, rhs: $storage_ty) -> Self::Output {
                            Self::from_base_value(self.raw() * rhs)
                        }
                    }

                    impl core::ops::MulAssign<$storage_ty> for $quantity_name {
                        fn mul_assign(&mut self, rhs: $storage_ty) {
                            *self = *self * rhs;
                        }
                    }

                    impl core::ops::Div<$storage_ty> for $quantity_name {
                        type Output = Self;

                        fn div(self, rhs: $storage_ty) -> Self::Output {
                            Self::from_base_value(self.raw() / rhs)
                        }
                    }

                    impl core::ops::DivAssign<$storage_ty> for $quantity_name {
                        fn div_assign(&mut self, rhs: $storage_ty) {
                            *self = *self / rhs;
                        }
                    }

                    impl core::ops::Neg for $quantity_name {
                        type Output = Self;

                        fn neg(self) -> Self::Output {
                            Self::from_base_value(-self.raw())
                        }
                    }

                    impl Serialize for $quantity_name {
                        fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
                        where
                            S: Serializer,
                        {
                            self.raw().serialize(serializer)
                        }
                    }

                    impl<'de> Deserialize<'de> for $quantity_name {
                        fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
                        where
                            D: Deserializer<'de>,
                        {
                            let value = <$storage_ty>::deserialize(deserializer)?;
                            Ok(Self::from_base_value(value))
                        }
                    }

                    impl bincode::ValueDecode for $quantity_name {
                        const DECODE: &'static bincode::ValueDecodeSpec = <$storage_ty as bincode::ValueDecode>::DECODE;
                        const METADATA: &'static [bincode::value_decode::ValueMetadata] = &[
                            bincode::value_decode::ValueMetadata::Quantity(
                                cu29_value_types::QuantityMetadata::coherent(cu29_value_types::Quantity::$quantity_name),
                            ),
                        ];
                    }

                    impl cu29_traits::DebugScalarType for $quantity_name {
                        fn debug_scalar_registration() -> cu29_traits::DebugScalarRegistration {
                            cu29_traits::DebugScalarRegistration {
                                type_path: core::any::type_name::<Self>(),
                                scalar_kind: if stringify!($storage_ty) == "f32" {
                                    cu29_traits::DebugScalarKind::F32
                                } else {
                                    cu29_traits::DebugScalarKind::F64
                                },
                                semantics: cu29_traits::DebugFieldSemantics::Quantity {
                                    quantity_name: alloc::string::String::from(stringify!($quantity_name)),
                                    unit_symbol: alloc::string::String::from(Self::STORAGE_UNIT),
                                },
                            }
                        }
                    }

                    impl bincode::Encode for $quantity_name {
                        fn encode<E: bincode::enc::Encoder>(
                            &self,
                            encoder: &mut E,
                        ) -> Result<(), bincode::error::EncodeError> {
                            bincode::Encode::encode(&self.raw(), encoder)
                        }
                    }

                    impl<Context> bincode::Decode<Context> for $quantity_name {
                        fn decode<D: bincode::de::Decoder<Context = Context>>(
                            decoder: &mut D,
                        ) -> Result<Self, bincode::error::DecodeError> {
                            let value: $storage_ty = bincode::Decode::decode(decoder)?;
                            Ok(Self::from_base_value(value))
                        }
                    }

                    impl<'de, Context> bincode::BorrowDecode<'de, Context> for $quantity_name {
                        fn borrow_decode<D: bincode::de::BorrowDecoder<'de, Context = Context>>(
                            decoder: &mut D,
                        ) -> Result<Self, bincode::error::DecodeError> {
                            <Self as bincode::Decode<Context>>::decode(decoder)
                        }
                    }

                    #[cfg(feature = "reflect")]
                    impl bevy_reflect::FromReflect for $quantity_name {
                        fn from_reflect(
                            reflect: &dyn bevy_reflect::PartialReflect,
                        ) -> Option<Self> {
                            if let Some(existing) = reflect.try_downcast_ref::<Self>() {
                                return Some(*existing);
                            }

                            reflect
                                .try_downcast_ref::<$storage_ty>()
                                .map(|value| Self::from_base_value(*value))
                        }
                    }
                };
            }

            $(define_quantity!($unit_mod, $quantity, $symbol);)+

            pub(crate) fn debug_scalar_registrations() -> alloc::vec::Vec<cu29_traits::DebugScalarRegistration> {
                alloc::vec![$(<$quantity as cu29_traits::DebugScalarType>::debug_scalar_registration()),+]
            }

            #[cfg(all(test, feature = "reflect"))]
            #[test]
            fn test_quantity_symbols_and_scalar_encoding() {
                let registrations = debug_scalar_registrations();
                assert_eq!(registrations.len(), cu29_value_types::Quantity::ALL.len());
                $(
                    let quantity = cu29_value_types::QuantityMetadata::coherent(cu29_value_types::Quantity::$quantity);
                    let registration = registrations.iter().find(|entry| entry.type_path == core::any::type_name::<$quantity>()).unwrap();
                    let cu29_traits::DebugFieldSemantics::Quantity { unit_symbol, .. } = &registration.semantics else {
                        panic!("expected quantity semantics");
                    };
                    assert_eq!(unit_symbol, quantity.storage_unit().symbol());
                    assert_eq!(<$quantity as bincode::ValueDecode>::METADATA, &[bincode::value_decode::ValueMetadata::Quantity(quantity)]);
                    for raw in [0.0, -1.25, 1234.5] {
                        let value = $quantity::from_base_value(raw);
                        let config = bincode::config::standard();
                        let encoded = bincode::encode_to_vec(value, config).unwrap();
                        assert_eq!(encoded, bincode::encode_to_vec(raw, config).unwrap());
                        let (decoded, used): ($quantity, usize) = bincode::decode_from_slice(&encoded, config).unwrap();
                        assert_eq!(decoded.raw(), raw);
                        assert_eq!(used, encoded.len());
                    }
                )+
            }


        }
    };
}

pub mod si {
    pub use uom::si::{
        ISQ, SI, absement, acceleration, action, amount_of_substance, angle, angular_absement,
        angular_acceleration, angular_jerk, angular_momentum, angular_velocity, area,
        areal_density_of_states, areal_heat_capacity, areal_mass_density, areal_number_density,
        areal_number_rate, available_energy, capacitance, catalytic_activity,
        catalytic_activity_concentration, curvature, diffusion_coefficient, dynamic_viscosity,
        electric_charge, electric_charge_areal_density, electric_charge_linear_density,
        electric_charge_volumetric_density, electric_current, electric_current_density,
        electric_dipole_moment, electric_displacement_field, electric_field, electric_flux,
        electric_permittivity, electric_potential, electric_quadrupole_moment,
        electrical_conductance, electrical_conductivity, electrical_mobility,
        electrical_resistance, electrical_resistivity, energy, force, frequency, frequency_drift,
        heat_capacity, heat_flux_density, heat_transfer, inductance, information, information_rate,
        inverse_velocity, jerk, kinematic_viscosity, length, linear_density_of_states,
        linear_mass_density, linear_number_density, linear_number_rate, linear_power_density,
        luminance, luminous_intensity, magnetic_field_strength, magnetic_flux,
        magnetic_flux_density, magnetic_moment, magnetic_permeability, mass, mass_concentration,
        mass_density, mass_flux, mass_per_energy, mass_rate, molality, molar_concentration,
        molar_energy, molar_flux, molar_heat_capacity, molar_mass, molar_radioactivity,
        molar_volume, moment_of_inertia, momentum, power, power_rate, pressure, radiant_exposure,
        radioactivity, ratio, reciprocal_length, solid_angle, specific_area,
        specific_heat_capacity, specific_power, specific_radioactivity, specific_volume,
        surface_electric_current_density, surface_tension, temperature_coefficient,
        temperature_gradient, temperature_interval, thermal_conductance, thermal_conductivity,
        thermal_resistance, thermodynamic_temperature, time, torque, velocity, volume, volume_rate,
        volumetric_density_of_states, volumetric_heat_capacity, volumetric_number_density,
        volumetric_number_rate, volumetric_power_density,
    };

    cu29_value_types::__quantity_catalogue!(define_storage_wrappers, f32, f32);
    cu29_value_types::__quantity_catalogue!(define_storage_wrappers, f64, f64);
}

/// Metadata and compile-time normalization support for constants declared in Copper RON files.
#[doc(hidden)]
pub mod constant {
    /// Storage-independent metadata for a supported physical quantity.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct QuantityDefinition {
        pub quantity: Quantity,
        pub quantity_name: &'static str,
        pub rust_type_f32: &'static str,
        pub rust_type_f64: &'static str,
        pub coherent_unit: &'static str,
        pub preferred_display_unit: &'static str,
        pub compatible_units: &'static [&'static str],
    }

    macro_rules! define_constant_catalogue {
        ($(($unit_mod:ident, $quantity:ident, $coherent_unit:ident, $display_unit:ident,
            [$($alternative_unit:ident),* $(,)?])),+ $(,)?) => {
            /// Physical quantities accepted by the top-level `constants:` configuration.
            #[allow(non_camel_case_types)]
            #[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
            pub enum Quantity {
                $($unit_mod,)+
            }

            impl Quantity {
                pub const fn name(self) -> &'static str {
                    match self {
                        $(Self::$unit_mod => stringify!($unit_mod),)+
                    }
                }
            }

            /// Unit names accepted by the top-level `constants:` configuration.
            #[allow(non_camel_case_types)]
            #[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
            pub enum Unit {
                $($coherent_unit, $($alternative_unit,)*)+
            }

            impl Unit {
                pub const fn name(self) -> &'static str {
                    match self {
                        $(
                            Self::$coherent_unit => stringify!($coherent_unit),
                            $(Self::$alternative_unit => stringify!($alternative_unit),)*
                        )+
                    }
                }

                pub fn from_name(name: &str) -> Option<Self> {
                    match name {
                        $(
                            stringify!($coherent_unit) => Some(Self::$coherent_unit),
                            $(stringify!($alternative_unit) => Some(Self::$alternative_unit),)*
                        )+
                        _ => None,
                    }
                }
            }

            pub const DEFINITIONS: &[QuantityDefinition] = &[
                $(QuantityDefinition {
                    quantity: Quantity::$unit_mod,
                    quantity_name: stringify!($unit_mod),
                    rust_type_f32: concat!("cu29::units::si::f32::", stringify!($quantity)),
                    rust_type_f64: concat!("cu29::units::si::f64::", stringify!($quantity)),
                    coherent_unit: stringify!($coherent_unit),
                    preferred_display_unit: stringify!($display_unit),
                    compatible_units: &[
                        stringify!($coherent_unit),
                        $(stringify!($alternative_unit),)*
                    ],
                },)+
            ];

            pub fn definition(quantity: Quantity) -> Option<&'static QuantityDefinition> {
                DEFINITIONS
                    .iter()
                    .find(|definition| definition.quantity == quantity)
            }

            pub fn normalize_f32(quantity: Quantity, unit: Unit, value: f32) -> Option<f32> {
                match quantity {
                    $(Quantity::$unit_mod => match unit {
                        Unit::$coherent_unit => Some(
                            crate::si::f32::$quantity::new::<
                                crate::uom::si::$unit_mod::$coherent_unit
                            >(value).raw()
                        ),
                        $(Unit::$alternative_unit => Some(
                            crate::si::f32::$quantity::new::<
                                crate::uom::si::$unit_mod::$alternative_unit
                            >(value).raw()
                        ),)*
                        _ => None,
                    },)+
                }
            }

            pub fn normalize_f64(quantity: Quantity, unit: Unit, value: f64) -> Option<f64> {
                match quantity {
                    $(Quantity::$unit_mod => match unit {
                        Unit::$coherent_unit => Some(
                            crate::si::f64::$quantity::new::<
                                crate::uom::si::$unit_mod::$coherent_unit
                            >(value).raw()
                        ),
                        $(Unit::$alternative_unit => Some(
                            crate::si::f64::$quantity::new::<
                                crate::uom::si::$unit_mod::$alternative_unit
                            >(value).raw()
                        ),)*
                        _ => None,
                    },)+
                }
            }
        };

    }

    // Every coherent unit here maps one input unit to one unit in uom's underlying SI storage.
    define_constant_catalogue! {
        (length, Length, meter, meter,
            [millimeter, centimeter, kilometer, inch, foot]),
        (angle, Angle, radian, radian, [degree, revolution]),
        (mass, Mass, kilogram, gram, [gram, milligram, pound]),
        (time, Time, second, second, [millisecond, microsecond, minute, hour]),
        (thermodynamic_temperature, ThermodynamicTemperature, kelvin, kelvin,
            [degree_celsius, degree_fahrenheit, degree_rankine]),
        (velocity, Velocity, meter_per_second, meter_per_second,
            [kilometer_per_hour, foot_per_second, knot]),
        (acceleration, Acceleration, meter_per_second_squared, meter_per_second_squared,
            [standard_gravity, foot_per_second_squared]),
        (angular_velocity, AngularVelocity, radian_per_second, radian_per_second,
            [degree_per_second, revolution_per_minute]),
        (angular_acceleration, AngularAcceleration, radian_per_second_squared,
            radian_per_second_squared, [degree_per_second_squared]),
        (frequency, Frequency, hertz, hertz, [kilohertz, megahertz]),
        (force, Force, newton, newton, [kilonewton, pound_force]),
        (pressure, Pressure, pascal, pascal,
            [kilopascal, bar, pound_force_per_square_inch]),
        (energy, Energy, joule, joule, [kilojoule, watt_hour]),
        (power, Power, watt, watt, [kilowatt, horsepower]),
        (electric_potential, ElectricPotential, volt, volt, [millivolt]),
        (electric_current, ElectricCurrent, ampere, ampere, [milliampere]),
        (ratio, Ratio, ratio, ratio, [percent]),
        (area, Area, square_meter, square_meter,
            [square_millimeter, square_centimeter]),
        (volume, Volume, cubic_meter, cubic_meter, [liter, milliliter]),
    }
}

use alloc::vec::Vec;

/// Register every supported quantity in both scalar widths for the debugger.
pub fn debug_scalar_registrations() -> Vec<cu29_traits::DebugScalarRegistration> {
    let mut registrations = si::f32::debug_scalar_registrations();
    registrations.extend(si::f64::debug_scalar_registrations());
    registrations
}

#[cfg(all(test, feature = "reflect"))]
mod tests {
    use super::si::f32::Velocity;
    use super::si::velocity::{kilometer_per_hour, meter_per_second};
    use bevy_reflect::{PartialReflect, Reflect, ReflectRef};

    #[derive(Reflect)]
    #[reflect(from_reflect = false)]
    struct Msg {
        speed: Velocity,
    }

    #[test]
    fn reflect_velocity_exposes_value_field() {
        let msg = Msg {
            speed: Velocity::new::<kilometer_per_hour>(36.0),
        };

        assert!(matches!(
            msg.speed.as_partial_reflect().reflect_ref(),
            ReflectRef::Struct(_)
        ));
        assert_eq!(msg.speed.get::<meter_per_second>(), 10.0);

        let speed_reflected = match msg.as_partial_reflect().reflect_ref() {
            ReflectRef::Struct(s) => s.field("speed").expect("speed field should exist"),
            _ => panic!("expected struct reflection"),
        };

        let speed = speed_reflected
            .try_downcast_ref::<Velocity>()
            .expect("speed should downcast to cu29_units::si::f32::Velocity");
        assert_eq!(speed.raw(), 10.0);
        assert_eq!(speed.value, 10.0);
        assert_eq!(speed.get::<meter_per_second>(), 10.0);
    }
}

#[cfg(test)]
mod storage_unit_tests {
    use super::si::f32;
    use bincode::ValueDecode;

    fn symbol<T: ValueDecode>() -> &'static str {
        let [bincode::value_decode::ValueMetadata::Quantity(quantity)] = T::METADATA else {
            panic!("expected typed quantity metadata")
        };
        quantity.storage_unit().symbol()
    }

    #[test]
    fn test_conventional_si_storage_units() {
        assert_eq!(symbol::<f32::ElectricPotential>(), "V");
        assert_eq!(symbol::<f32::MagneticFluxDensity>(), "T");
        assert_eq!(symbol::<f32::Force>(), "N");
        assert_eq!(symbol::<f32::Pressure>(), "Pa");
        assert_eq!(symbol::<f32::Energy>(), "J");
        assert_eq!(symbol::<f32::Torque>(), "N·m");
        assert_eq!(symbol::<f32::Power>(), "W");
        assert_eq!(symbol::<f32::Frequency>(), "Hz");
        assert_eq!(symbol::<f32::AngularVelocity>(), "rad·s⁻¹");
        assert_eq!(symbol::<f32::ElectricalResistance>(), "Ω");
        assert_eq!(symbol::<f32::Absement>(), "m·s");
        assert_eq!(symbol::<f32::Velocity>(), "m·s⁻¹");
        assert_eq!(symbol::<f32::Acceleration>(), "m·s⁻²");
        assert_eq!(symbol::<f32::Area>(), "m²");
        assert_eq!(symbol::<f32::Volume>(), "m³");
        assert_eq!(symbol::<f32::Mass>(), "kg");
        assert_eq!(symbol::<f32::Angle>(), "rad");
        assert_eq!(symbol::<f32::SolidAngle>(), "sr");
        assert_eq!(symbol::<f32::Information>(), "bit");
        assert_eq!(symbol::<f32::InformationRate>(), "bit·s⁻¹");
        assert_eq!(symbol::<f32::Ratio>(), "1");
    }

    #[test]
    fn test_named_units_match_storage_scale() {
        assert_eq!(
            f32::ElectricPotential::new::<super::si::electric_potential::millivolt>(1000.0).raw(),
            1.0
        );
        assert_eq!(f32::Mass::new::<super::si::mass::gram>(1000.0).raw(), 1.0);
        assert_eq!(
            f32::Pressure::new::<super::si::pressure::kilopascal>(1.0).raw(),
            1000.0
        );
    }
}
