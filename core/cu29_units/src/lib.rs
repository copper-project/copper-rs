//! Copper-native SI quantity wrappers.
//!
//! Feature flags:
//! - `default` = `["std"]`
//! - `std`: enables `uom/std`
//! - `reflect`: enables `bevy_reflect` support on wrapper types
//! - `textlogs`: compatibility no-op for downstream feature forwarding
//!
//! # Const construction
//!
//! Common robotics motion quantities support a deliberately limited set of units during const
//! evaluation:
//!
//! ```
//! use cu29_units::si::f32::Length;
//! use cu29_units::si::length::meter;
//!
//! const SENSOR_OFFSET: Length = Length::new_const::<meter>(0.12);
//! ```
//!
//! Use [`Length::new`](si::f32::Length::new) and the other runtime constructors for the complete
//! `uom` unit surface.
//!
#![cfg_attr(not(feature = "std"), no_std)]

extern crate alloc;

pub use uom;

macro_rules! define_storage_wrappers {
    ($storage_mod:ident, $storage_ty:ty, [$(($unit_mod:ident, $quantity:ident),)+]) => {
        pub mod $storage_mod {
            use core::marker::PhantomData;
            use serde::{Deserialize, Deserializer, Serialize, Serializer};

            #[cfg(feature = "reflect")]
            use bevy_reflect::Reflect;

            macro_rules! define_quantity {
                ($unit_mod_name:ident, $quantity_name:ident) => {
                    #[repr(transparent)]
                    #[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
                    #[cfg_attr(feature = "reflect", derive(Reflect))]
                    #[cfg_attr(feature = "reflect", reflect(from_reflect = false))]
                    pub struct $quantity_name {
                        pub value: $storage_ty,
                    }

                    impl $quantity_name {
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
                        pub const fn raw(&self) -> $storage_ty {
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

            $(define_quantity!($unit_mod, $quantity);)+
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

    mod const_unit_private {
        pub trait Sealed {}
    }

    macro_rules! define_const_unit_family {
        ($trait_name:ident, $unit_mod:ident, [$(($unit:ident, $scale:expr),)+]) => {
            #[doc(hidden)]
            pub trait $trait_name: const_unit_private::Sealed {
                const SCALE: f64;
            }

            $(
                impl const_unit_private::Sealed for uom::si::$unit_mod::$unit {}

                impl $trait_name for uom::si::$unit_mod::$unit {
                    const SCALE: f64 = $scale;
                }
            )+
        };
    }

    define_const_unit_family! {
        ConstLengthUnit,
        length,
        [
            (meter, 1.0),
            (centimeter, 1.0e-2),
            (millimeter, 1.0e-3),
        ]
    }

    define_const_unit_family! {
        ConstAngleUnit,
        angle,
        [
            (radian, 1.0),
            (degree, 1.745_329_251_994_329_5e-2),
        ]
    }

    define_const_unit_family! {
        ConstTimeUnit,
        time,
        [
            (second, 1.0),
            (millisecond, 1.0e-3),
            (microsecond, 1.0e-6),
            (nanosecond, 1.0e-9),
        ]
    }

    define_const_unit_family! {
        ConstVelocityUnit,
        velocity,
        [
            (meter_per_second, 1.0),
            (kilometer_per_hour, 2.777_777_777_777_778e-1),
        ]
    }

    define_const_unit_family! {
        ConstAngularVelocityUnit,
        angular_velocity,
        [
            (radian_per_second, 1.0),
            (degree_per_second, 1.745_329_251_994_329_5e-2),
        ]
    }

    define_const_unit_family! {
        ConstAccelerationUnit,
        acceleration,
        [
            (meter_per_second_squared, 1.0),
            (standard_gravity, 9.806_65),
        ]
    }

    define_storage_wrappers! {
        f32,
        f32,
        [
            (absement, Absement),
            (acceleration, Acceleration),
            (action, Action),
            (amount_of_substance, AmountOfSubstance),
            (angle, Angle),
            (angular_absement, AngularAbsement),
            (angular_acceleration, AngularAcceleration),
            (angular_jerk, AngularJerk),
            (angular_momentum, AngularMomentum),
            (angular_velocity, AngularVelocity),
            (area, Area),
            (areal_density_of_states, ArealDensityOfStates),
            (areal_heat_capacity, ArealHeatCapacity),
            (areal_mass_density, ArealMassDensity),
            (areal_number_density, ArealNumberDensity),
            (areal_number_rate, ArealNumberRate),
            (available_energy, AvailableEnergy),
            (capacitance, Capacitance),
            (catalytic_activity, CatalyticActivity),
            (catalytic_activity_concentration, CatalyticActivityConcentration),
            (curvature, Curvature),
            (diffusion_coefficient, DiffusionCoefficient),
            (dynamic_viscosity, DynamicViscosity),
            (electric_charge, ElectricCharge),
            (electric_charge_areal_density, ElectricChargeArealDensity),
            (electric_charge_linear_density, ElectricChargeLinearDensity),
            (electric_charge_volumetric_density, ElectricChargeVolumetricDensity),
            (electric_current, ElectricCurrent),
            (electric_current_density, ElectricCurrentDensity),
            (electric_dipole_moment, ElectricDipoleMoment),
            (electric_displacement_field, ElectricDisplacementField),
            (electric_field, ElectricField),
            (electric_flux, ElectricFlux),
            (electric_permittivity, ElectricPermittivity),
            (electric_potential, ElectricPotential),
            (electric_quadrupole_moment, ElectricQuadrupoleMoment),
            (electrical_conductance, ElectricalConductance),
            (electrical_conductivity, ElectricalConductivity),
            (electrical_mobility, ElectricalMobility),
            (electrical_resistance, ElectricalResistance),
            (electrical_resistivity, ElectricalResistivity),
            (energy, Energy),
            (force, Force),
            (frequency, Frequency),
            (frequency_drift, FrequencyDrift),
            (heat_capacity, HeatCapacity),
            (heat_flux_density, HeatFluxDensity),
            (heat_transfer, HeatTransfer),
            (inductance, Inductance),
            (information, Information),
            (information_rate, InformationRate),
            (inverse_velocity, InverseVelocity),
            (jerk, Jerk),
            (kinematic_viscosity, KinematicViscosity),
            (length, Length),
            (linear_density_of_states, LinearDensityOfStates),
            (linear_mass_density, LinearMassDensity),
            (linear_number_density, LinearNumberDensity),
            (linear_number_rate, LinearNumberRate),
            (linear_power_density, LinearPowerDensity),
            (luminance, Luminance),
            (luminous_intensity, LuminousIntensity),
            (magnetic_field_strength, MagneticFieldStrength),
            (magnetic_flux, MagneticFlux),
            (magnetic_flux_density, MagneticFluxDensity),
            (magnetic_moment, MagneticMoment),
            (magnetic_permeability, MagneticPermeability),
            (mass, Mass),
            (mass_concentration, MassConcentration),
            (mass_density, MassDensity),
            (mass_flux, MassFlux),
            (mass_per_energy, MassPerEnergy),
            (mass_rate, MassRate),
            (molality, Molality),
            (molar_concentration, MolarConcentration),
            (molar_energy, MolarEnergy),
            (molar_flux, MolarFlux),
            (molar_heat_capacity, MolarHeatCapacity),
            (molar_mass, MolarMass),
            (molar_radioactivity, MolarRadioactivity),
            (molar_volume, MolarVolume),
            (moment_of_inertia, MomentOfInertia),
            (momentum, Momentum),
            (power, Power),
            (power_rate, PowerRate),
            (pressure, Pressure),
            (radiant_exposure, RadiantExposure),
            (radioactivity, Radioactivity),
            (ratio, Ratio),
            (reciprocal_length, ReciprocalLength),
            (solid_angle, SolidAngle),
            (specific_area, SpecificArea),
            (specific_heat_capacity, SpecificHeatCapacity),
            (specific_power, SpecificPower),
            (specific_radioactivity, SpecificRadioactivity),
            (specific_volume, SpecificVolume),
            (surface_electric_current_density, SurfaceElectricCurrentDensity),
            (surface_tension, SurfaceTension),
            (temperature_coefficient, TemperatureCoefficient),
            (temperature_gradient, TemperatureGradient),
            (temperature_interval, TemperatureInterval),
            (thermal_conductance, ThermalConductance),
            (thermal_conductivity, ThermalConductivity),
            (thermal_resistance, ThermalResistance),
            (thermodynamic_temperature, ThermodynamicTemperature),
            (time, Time),
            (torque, Torque),
            (velocity, Velocity),
            (volume, Volume),
            (volume_rate, VolumeRate),
            (volumetric_density_of_states, VolumetricDensityOfStates),
            (volumetric_heat_capacity, VolumetricHeatCapacity),
            (volumetric_number_density, VolumetricNumberDensity),
            (volumetric_number_rate, VolumetricNumberRate),
            (volumetric_power_density, VolumetricPowerDensity),
        ]
    }

    define_storage_wrappers! {
        f64,
        f64,
        [
            (absement, Absement),
            (acceleration, Acceleration),
            (action, Action),
            (amount_of_substance, AmountOfSubstance),
            (angle, Angle),
            (angular_absement, AngularAbsement),
            (angular_acceleration, AngularAcceleration),
            (angular_jerk, AngularJerk),
            (angular_momentum, AngularMomentum),
            (angular_velocity, AngularVelocity),
            (area, Area),
            (areal_density_of_states, ArealDensityOfStates),
            (areal_heat_capacity, ArealHeatCapacity),
            (areal_mass_density, ArealMassDensity),
            (areal_number_density, ArealNumberDensity),
            (areal_number_rate, ArealNumberRate),
            (available_energy, AvailableEnergy),
            (capacitance, Capacitance),
            (catalytic_activity, CatalyticActivity),
            (catalytic_activity_concentration, CatalyticActivityConcentration),
            (curvature, Curvature),
            (diffusion_coefficient, DiffusionCoefficient),
            (dynamic_viscosity, DynamicViscosity),
            (electric_charge, ElectricCharge),
            (electric_charge_areal_density, ElectricChargeArealDensity),
            (electric_charge_linear_density, ElectricChargeLinearDensity),
            (electric_charge_volumetric_density, ElectricChargeVolumetricDensity),
            (electric_current, ElectricCurrent),
            (electric_current_density, ElectricCurrentDensity),
            (electric_dipole_moment, ElectricDipoleMoment),
            (electric_displacement_field, ElectricDisplacementField),
            (electric_field, ElectricField),
            (electric_flux, ElectricFlux),
            (electric_permittivity, ElectricPermittivity),
            (electric_potential, ElectricPotential),
            (electric_quadrupole_moment, ElectricQuadrupoleMoment),
            (electrical_conductance, ElectricalConductance),
            (electrical_conductivity, ElectricalConductivity),
            (electrical_mobility, ElectricalMobility),
            (electrical_resistance, ElectricalResistance),
            (electrical_resistivity, ElectricalResistivity),
            (energy, Energy),
            (force, Force),
            (frequency, Frequency),
            (frequency_drift, FrequencyDrift),
            (heat_capacity, HeatCapacity),
            (heat_flux_density, HeatFluxDensity),
            (heat_transfer, HeatTransfer),
            (inductance, Inductance),
            (information, Information),
            (information_rate, InformationRate),
            (inverse_velocity, InverseVelocity),
            (jerk, Jerk),
            (kinematic_viscosity, KinematicViscosity),
            (length, Length),
            (linear_density_of_states, LinearDensityOfStates),
            (linear_mass_density, LinearMassDensity),
            (linear_number_density, LinearNumberDensity),
            (linear_number_rate, LinearNumberRate),
            (linear_power_density, LinearPowerDensity),
            (luminance, Luminance),
            (luminous_intensity, LuminousIntensity),
            (magnetic_field_strength, MagneticFieldStrength),
            (magnetic_flux, MagneticFlux),
            (magnetic_flux_density, MagneticFluxDensity),
            (magnetic_moment, MagneticMoment),
            (magnetic_permeability, MagneticPermeability),
            (mass, Mass),
            (mass_concentration, MassConcentration),
            (mass_density, MassDensity),
            (mass_flux, MassFlux),
            (mass_per_energy, MassPerEnergy),
            (mass_rate, MassRate),
            (molality, Molality),
            (molar_concentration, MolarConcentration),
            (molar_energy, MolarEnergy),
            (molar_flux, MolarFlux),
            (molar_heat_capacity, MolarHeatCapacity),
            (molar_mass, MolarMass),
            (molar_radioactivity, MolarRadioactivity),
            (molar_volume, MolarVolume),
            (moment_of_inertia, MomentOfInertia),
            (momentum, Momentum),
            (power, Power),
            (power_rate, PowerRate),
            (pressure, Pressure),
            (radiant_exposure, RadiantExposure),
            (radioactivity, Radioactivity),
            (ratio, Ratio),
            (reciprocal_length, ReciprocalLength),
            (solid_angle, SolidAngle),
            (specific_area, SpecificArea),
            (specific_heat_capacity, SpecificHeatCapacity),
            (specific_power, SpecificPower),
            (specific_radioactivity, SpecificRadioactivity),
            (specific_volume, SpecificVolume),
            (surface_electric_current_density, SurfaceElectricCurrentDensity),
            (surface_tension, SurfaceTension),
            (temperature_coefficient, TemperatureCoefficient),
            (temperature_gradient, TemperatureGradient),
            (temperature_interval, TemperatureInterval),
            (thermal_conductance, ThermalConductance),
            (thermal_conductivity, ThermalConductivity),
            (thermal_resistance, ThermalResistance),
            (thermodynamic_temperature, ThermodynamicTemperature),
            (time, Time),
            (torque, Torque),
            (velocity, Velocity),
            (volume, Volume),
            (volume_rate, VolumeRate),
            (volumetric_density_of_states, VolumetricDensityOfStates),
            (volumetric_heat_capacity, VolumetricHeatCapacity),
            (volumetric_number_density, VolumetricNumberDensity),
            (volumetric_number_rate, VolumetricNumberRate),
            (volumetric_power_density, VolumetricPowerDensity),
        ]
    }

    macro_rules! impl_const_constructor {
        ($storage_mod:ident, $storage_ty:ty, $quantity:ident, $unit_trait:ident, $units:literal) => {
            impl $storage_mod::$quantity {
                /// Creates a quantity in a supported unit during const evaluation.
                ///
                /// The deliberately limited set of supported units is:
                #[doc = $units]
                /// Runtime construction with [`Self::new`] remains available for every unit
                /// supported by `uom`.
                #[inline]
                pub const fn new_const<U>(value: $storage_ty) -> Self
                where
                    U: $unit_trait,
                {
                    Self {
                        value: value * U::SCALE as $storage_ty,
                    }
                }
            }
        };
    }

    macro_rules! impl_const_constructors {
        ($storage_mod:ident, $storage_ty:ty) => {
            impl_const_constructor!(
                $storage_mod,
                $storage_ty,
                Length,
                ConstLengthUnit,
                "`meter`, `centimeter`, and `millimeter`."
            );
            impl_const_constructor!(
                $storage_mod,
                $storage_ty,
                Angle,
                ConstAngleUnit,
                "`radian` and `degree`."
            );
            impl_const_constructor!(
                $storage_mod,
                $storage_ty,
                Time,
                ConstTimeUnit,
                "`second`, `millisecond`, `microsecond`, and `nanosecond`."
            );
            impl_const_constructor!(
                $storage_mod,
                $storage_ty,
                Velocity,
                ConstVelocityUnit,
                "`meter_per_second` and `kilometer_per_hour`."
            );
            impl_const_constructor!(
                $storage_mod,
                $storage_ty,
                AngularVelocity,
                ConstAngularVelocityUnit,
                "`radian_per_second` and `degree_per_second`."
            );
            impl_const_constructor!(
                $storage_mod,
                $storage_ty,
                Acceleration,
                ConstAccelerationUnit,
                "`meter_per_second_squared` and `standard_gravity`."
            );
        };
    }

    impl_const_constructors!(f32, f32);
    impl_const_constructors!(f64, f64);
}

use alloc::string::ToString;
use alloc::vec::Vec;
use cu29_traits::{DebugFieldSemantics, DebugScalarKind, DebugScalarRegistration, DebugScalarType};

macro_rules! impl_debug_scalar_units {
    ($(($unit_mod:ident, $quantity:ident, $base_unit:ident),)+) => {
        macro_rules! impl_storage_debug_scalar_units {
            ($storage_mod:ident, $storage_ty:ty) => {
                $(
                    impl DebugScalarType for si::$storage_mod::$quantity {
                        fn debug_scalar_registration() -> DebugScalarRegistration {
                            DebugScalarRegistration {
                                type_path: core::any::type_name::<Self>(),
                                scalar_kind: if stringify!($storage_ty) == "f32" {
                                    DebugScalarKind::F32
                                } else {
                                    DebugScalarKind::F64
                                },
                                semantics: DebugFieldSemantics::Quantity {
                                    quantity_name: stringify!($quantity).to_string(),
                                    unit_symbol: <uom::si::$unit_mod::$base_unit as uom::si::Unit>::abbreviation()
                                        .to_string(),
                                },
                            }
                        }
                    }
                )+
            };
        }

        impl_storage_debug_scalar_units!(f32, f32);
        impl_storage_debug_scalar_units!(f64, f64);

        pub fn debug_scalar_registrations() -> Vec<DebugScalarRegistration> {
            alloc::vec![
                $(
                    <si::f32::$quantity as DebugScalarType>::debug_scalar_registration(),
                    <si::f64::$quantity as DebugScalarType>::debug_scalar_registration(),
                )+
            ]
        }
    };
}

impl_debug_scalar_units! {
    (absement, Absement, meter_second),
    (acceleration, Acceleration, meter_per_second_squared),
    (action, Action, joule_second),
    (amount_of_substance, AmountOfSubstance, mole),
    (angle, Angle, radian),
    (angular_absement, AngularAbsement, radian_second),
    (angular_acceleration, AngularAcceleration, radian_per_second_squared),
    (angular_jerk, AngularJerk, radian_per_second_cubed),
    (angular_momentum, AngularMomentum, newton_meter_second),
    (angular_velocity, AngularVelocity, radian_per_second),
    (area, Area, square_meter),
    (areal_density_of_states, ArealDensityOfStates, state_per_square_meter_joule),
    (areal_heat_capacity, ArealHeatCapacity, joule_per_square_meter_kelvin),
    (areal_mass_density, ArealMassDensity, kilogram_per_square_meter),
    (areal_number_density, ArealNumberDensity, per_square_kilometer),
    (areal_number_rate, ArealNumberRate, per_square_meter_second),
    (available_energy, AvailableEnergy, joule_per_kilogram),
    (capacitance, Capacitance, farad),
    (catalytic_activity, CatalyticActivity, katal),
    (catalytic_activity_concentration, CatalyticActivityConcentration, katal_per_cubic_meter),
    (curvature, Curvature, radian_per_meter),
    (diffusion_coefficient, DiffusionCoefficient, square_meter_per_second),
    (dynamic_viscosity, DynamicViscosity, pascal_second),
    (electric_charge, ElectricCharge, coulomb),
    (electric_charge_areal_density, ElectricChargeArealDensity, coulomb_per_square_meter),
    (electric_charge_linear_density, ElectricChargeLinearDensity, coulomb_per_meter),
    (electric_charge_volumetric_density, ElectricChargeVolumetricDensity, coulomb_per_cubic_meter),
    (electric_current, ElectricCurrent, ampere),
    (electric_current_density, ElectricCurrentDensity, ampere_per_square_meter),
    (electric_dipole_moment, ElectricDipoleMoment, coulomb_meter),
    (electric_displacement_field, ElectricDisplacementField, coulomb_per_square_meter),
    (electric_field, ElectricField, volt_per_meter),
    (electric_flux, ElectricFlux, volt_meter),
    (electric_permittivity, ElectricPermittivity, farad_per_meter),
    (electric_potential, ElectricPotential, volt),
    (electric_quadrupole_moment, ElectricQuadrupoleMoment, coulomb_square_meter),
    (electrical_conductance, ElectricalConductance, siemens),
    (electrical_conductivity, ElectricalConductivity, siemens_per_meter),
    (electrical_mobility, ElectricalMobility, square_meter_per_volt_second),
    (electrical_resistance, ElectricalResistance, ohm),
    (electrical_resistivity, ElectricalResistivity, ohm_meter),
    (energy, Energy, joule),
    (force, Force, newton),
    (frequency, Frequency, hertz),
    (frequency_drift, FrequencyDrift, hertz_per_second),
    (heat_capacity, HeatCapacity, gram_square_meter_per_second_squared_kelvin),
    (heat_flux_density, HeatFluxDensity, watt_per_square_meter),
    (heat_transfer, HeatTransfer, gram_per_second_cubed_kelvin),
    (inductance, Inductance, henry),
    (information, Information, bit),
    (information_rate, InformationRate, bit_per_second),
    (inverse_velocity, InverseVelocity, second_per_meter),
    (jerk, Jerk, meter_per_second_cubed),
    (kinematic_viscosity, KinematicViscosity, square_meter_per_second),
    (length, Length, meter),
    (linear_density_of_states, LinearDensityOfStates, state_per_meter_joule),
    (linear_mass_density, LinearMassDensity, kilogram_per_meter),
    (linear_number_density, LinearNumberDensity, per_kilometer),
    (linear_number_rate, LinearNumberRate, per_kilometer_second),
    (linear_power_density, LinearPowerDensity, watt_per_meter),
    (luminance, Luminance, candela_per_square_meter),
    (luminous_intensity, LuminousIntensity, candela),
    (magnetic_field_strength, MagneticFieldStrength, ampere_per_meter),
    (magnetic_flux, MagneticFlux, weber),
    (magnetic_flux_density, MagneticFluxDensity, tesla),
    (magnetic_moment, MagneticMoment, ampere_square_meter),
    (magnetic_permeability, MagneticPermeability, henry_per_meter),
    (mass, Mass, gram),
    (mass_concentration, MassConcentration, gram_per_cubic_meter),
    (mass_density, MassDensity, gram_per_cubic_meter),
    (mass_flux, MassFlux, kilogram_per_square_meter_second),
    (mass_per_energy, MassPerEnergy, gram_per_joule),
    (mass_rate, MassRate, gram_per_second),
    (molality, Molality, mole_per_kilogram),
    (molar_concentration, MolarConcentration, mole_per_cubic_meter),
    (molar_energy, MolarEnergy, joule_per_mole),
    (molar_flux, MolarFlux, mole_per_square_meter_second),
    (molar_heat_capacity, MolarHeatCapacity, joule_per_kelvin_mole),
    (molar_mass, MolarMass, gram_per_mole),
    (molar_radioactivity, MolarRadioactivity, becquerel_per_mole),
    (molar_volume, MolarVolume, cubic_meter_per_mole),
    (moment_of_inertia, MomentOfInertia, kilogram_square_meter),
    (momentum, Momentum, gram_meter_per_second),
    (power, Power, watt),
    (power_rate, PowerRate, watt_per_second),
    (pressure, Pressure, pascal),
    (radiant_exposure, RadiantExposure, joule_per_square_meter),
    (radioactivity, Radioactivity, becquerel),
    (ratio, Ratio, ratio),
    (reciprocal_length, ReciprocalLength, reciprocal_kilometer),
    (solid_angle, SolidAngle, steradian),
    (specific_area, SpecificArea, square_meter_per_kilogram),
    (specific_heat_capacity, SpecificHeatCapacity, square_meter_per_second_squared_kelvin),
    (specific_power, SpecificPower, watt_per_kilogram),
    (specific_radioactivity, SpecificRadioactivity, becquerel_per_kilogram),
    (surface_electric_current_density, SurfaceElectricCurrentDensity, ampere_per_meter),
    (surface_tension, SurfaceTension, newton_per_meter),
    (temperature_coefficient, TemperatureCoefficient, per_kelvin),
    (temperature_gradient, TemperatureGradient, kelvin_per_kilometer),
    (temperature_interval, TemperatureInterval, kelvin),
    (thermal_conductance, ThermalConductance, gram_meter_squared_per_second_cubed_kelvin),
    (thermal_conductivity, ThermalConductivity, gram_meter_per_second_cubed_kelvin),
    (thermal_resistance, ThermalResistance, kelvin_per_yottawatt),
    (thermodynamic_temperature, ThermodynamicTemperature, kelvin),
    (time, Time, second),
    (torque, Torque, newton_meter),
    (velocity, Velocity, meter_per_second),
    (volume, Volume, cubic_meter),
    (volume_rate, VolumeRate, cubic_meter_per_second),
    (volumetric_density_of_states, VolumetricDensityOfStates, state_per_cubic_meter_joule),
    (volumetric_heat_capacity, VolumetricHeatCapacity, joule_per_cubic_meter_kelvin),
    (volumetric_number_density, VolumetricNumberDensity, per_cubic_kilometer),
    (volumetric_number_rate, VolumetricNumberRate, per_cubic_meter_second),
    (volumetric_power_density, VolumetricPowerDensity, watt_per_cubic_meter),
}

#[cfg(test)]
mod const_tests {
    use super::si::acceleration::{meter_per_second_squared, standard_gravity};
    use super::si::angle::{degree, radian};
    use super::si::angular_velocity::{degree_per_second, radian_per_second};
    use super::si::f32 as units_f32;
    use super::si::f64 as units_f64;
    use super::si::length::{centimeter, meter, millimeter};
    use super::si::time::{microsecond, millisecond, nanosecond, second};
    use super::si::velocity::{kilometer_per_hour, meter_per_second};

    const LENGTH_F32: [units_f32::Length; 3] = [
        units_f32::Length::new_const::<meter>(1.25),
        units_f32::Length::new_const::<centimeter>(1.25),
        units_f32::Length::new_const::<millimeter>(1.25),
    ];
    const ANGLE_F32: [units_f32::Angle; 2] = [
        units_f32::Angle::new_const::<radian>(1.25),
        units_f32::Angle::new_const::<degree>(1.25),
    ];
    const TIME_F32: [units_f32::Time; 4] = [
        units_f32::Time::new_const::<second>(1.25),
        units_f32::Time::new_const::<millisecond>(1.25),
        units_f32::Time::new_const::<microsecond>(1.25),
        units_f32::Time::new_const::<nanosecond>(1.25),
    ];
    const VELOCITY_F32: [units_f32::Velocity; 2] = [
        units_f32::Velocity::new_const::<meter_per_second>(1.25),
        units_f32::Velocity::new_const::<kilometer_per_hour>(1.25),
    ];
    const ANGULAR_VELOCITY_F32: [units_f32::AngularVelocity; 2] = [
        units_f32::AngularVelocity::new_const::<radian_per_second>(1.25),
        units_f32::AngularVelocity::new_const::<degree_per_second>(1.25),
    ];
    const ACCELERATION_F32: [units_f32::Acceleration; 2] = [
        units_f32::Acceleration::new_const::<meter_per_second_squared>(1.25),
        units_f32::Acceleration::new_const::<standard_gravity>(1.25),
    ];

    macro_rules! assert_const_matches_runtime {
        ($storage_mod:ident, $quantity:ident, [$(($unit:ident, $value:expr),)+]) => {
            $(
                assert_eq!(
                    $storage_mod::$quantity::new_const::<$unit>($value),
                    $storage_mod::$quantity::new::<$unit>($value),
                );
            )+
        };
    }

    #[test]
    fn f32_const_construction_matches_runtime_conversions() {
        assert_const_matches_runtime!(
            units_f32,
            Length,
            [(meter, 1.25), (centimeter, 1.25), (millimeter, 1.25),]
        );
        assert_const_matches_runtime!(units_f32, Angle, [(radian, 1.25), (degree, 1.25),]);
        assert_const_matches_runtime!(
            units_f32,
            Time,
            [
                (second, 1.25),
                (millisecond, 1.25),
                (microsecond, 1.25),
                (nanosecond, 1.25),
            ]
        );
        assert_const_matches_runtime!(
            units_f32,
            Velocity,
            [(meter_per_second, 1.25), (kilometer_per_hour, 1.25),]
        );
        assert_const_matches_runtime!(
            units_f32,
            AngularVelocity,
            [(radian_per_second, 1.25), (degree_per_second, 1.25),]
        );
        assert_const_matches_runtime!(
            units_f32,
            Acceleration,
            [(meter_per_second_squared, 1.25), (standard_gravity, 1.25),]
        );

        assert_eq!(LENGTH_F32[0].raw(), 1.25);
        assert_eq!(ANGLE_F32[0].raw(), 1.25);
        assert_eq!(TIME_F32[0].raw(), 1.25);
        assert_eq!(VELOCITY_F32[0].raw(), 1.25);
        assert_eq!(ANGULAR_VELOCITY_F32[0].raw(), 1.25);
        assert_eq!(ACCELERATION_F32[0].raw(), 1.25);
    }

    #[test]
    fn f64_const_construction_matches_runtime_conversions() {
        assert_const_matches_runtime!(
            units_f64,
            Length,
            [(meter, 1.25), (centimeter, 1.25), (millimeter, 1.25),]
        );
        assert_const_matches_runtime!(units_f64, Angle, [(radian, 1.25), (degree, 1.25),]);
        assert_const_matches_runtime!(
            units_f64,
            Time,
            [
                (second, 1.25),
                (millisecond, 1.25),
                (microsecond, 1.25),
                (nanosecond, 1.25),
            ]
        );
        assert_const_matches_runtime!(
            units_f64,
            Velocity,
            [(meter_per_second, 1.25), (kilometer_per_hour, 1.25),]
        );
        assert_const_matches_runtime!(
            units_f64,
            AngularVelocity,
            [(radian_per_second, 1.25), (degree_per_second, 1.25),]
        );
        assert_const_matches_runtime!(
            units_f64,
            Acceleration,
            [(meter_per_second_squared, 1.25), (standard_gravity, 1.25),]
        );
    }
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
