//! Copper-native SI quantity wrappers.
//!
//! Feature flags:
//! - `default` = `["std"]`
//! - `std`: enables `uom/std`
//! - `reflect`: enables `bevy_reflect` support on wrapper types
//! - `textlogs`: compatibility no-op for downstream feature forwarding
//!
#![cfg_attr(not(feature = "std"), no_std)]

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
                    #[cfg_attr(feature = "reflect", reflect(opaque, from_reflect = false))]
                    pub struct $quantity_name(pub(crate) uom::si::$storage_mod::$quantity_name);

                    impl $quantity_name {
                        #[inline]
                        pub fn new<U>(value: $storage_ty) -> Self
                        where
                            U: uom::si::$unit_mod_name::Conversion<$storage_ty>,
                        {
                            Self(uom::si::$storage_mod::$quantity_name::new::<U>(value))
                        }

                        #[inline]
                        pub fn get<U>(&self) -> $storage_ty
                        where
                            U: uom::si::$unit_mod_name::Conversion<$storage_ty>,
                        {
                            self.0.get::<U>()
                        }

                        #[inline]
                        pub fn raw(&self) -> $storage_ty {
                            self.0.value
                        }

                        #[inline]
                        pub fn into_uom(self) -> uom::si::$storage_mod::$quantity_name {
                            self.0
                        }

                        #[inline]
                        pub fn as_uom(&self) -> &uom::si::$storage_mod::$quantity_name {
                            &self.0
                        }

                        #[inline]
                        pub fn from_uom(inner: uom::si::$storage_mod::$quantity_name) -> Self {
                            Self(inner)
                        }

                        #[inline]
                        fn from_base_value(
                            value: $storage_ty,
                        ) -> uom::si::$storage_mod::$quantity_name {
                            uom::si::$storage_mod::$quantity_name {
                                dimension: PhantomData,
                                units: PhantomData,
                                value,
                            }
                        }
                    }

                    impl Default for $quantity_name {
                        fn default() -> Self {
                            Self(Self::from_base_value(0.0 as $storage_ty))
                        }
                    }

                    impl From<uom::si::$storage_mod::$quantity_name> for $quantity_name {
                        fn from(value: uom::si::$storage_mod::$quantity_name) -> Self {
                            Self(value)
                        }
                    }

                    impl From<$quantity_name> for uom::si::$storage_mod::$quantity_name {
                        fn from(value: $quantity_name) -> Self {
                            value.0
                        }
                    }

                    impl core::ops::Deref for $quantity_name {
                        type Target = uom::si::$storage_mod::$quantity_name;

                        fn deref(&self) -> &Self::Target {
                            &self.0
                        }
                    }

                    impl core::ops::Add for $quantity_name {
                        type Output = Self;

                        fn add(self, rhs: Self) -> Self::Output {
                            Self(Self::from_base_value(self.raw() + rhs.raw()))
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
                            Self(Self::from_base_value(self.raw() - rhs.raw()))
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
                            Self(Self::from_base_value(self.raw() * rhs))
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
                            Self(Self::from_base_value(self.raw() / rhs))
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
                            Self(Self::from_base_value(-self.raw()))
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
                            Ok(Self(Self::from_base_value(value)))
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
                            Ok(Self(Self::from_base_value(value)))
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
                                .map(|value| Self(Self::from_base_value(*value)))
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
    fn reflect_velocity_as_opaque_base_value() {
        let msg = Msg {
            speed: Velocity::new::<kilometer_per_hour>(36.0),
        };

        assert!(matches!(
            msg.speed.as_partial_reflect().reflect_ref(),
            ReflectRef::Opaque(_)
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
        assert_eq!(speed.get::<meter_per_second>(), 10.0);
    }
}
