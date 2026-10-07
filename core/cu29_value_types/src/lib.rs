//! Allocation-free vocabulary for Copper value descriptions and logical metadata.
//!
//! Copper owns the catalogue. Numeric IDs are permanent: additions receive new IDs,
//! and existing IDs must never be reused or renumbered.
#![no_std]

/// Scalar width and signedness. Integer encoding and endianness come from the codec configuration.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[repr(u32)]
pub enum Scalar {
    /// Native bool encoding.
    Bool = 0,
    /// Native u8 encoding.
    U8 = 1,
    /// Native u16 encoding.
    U16 = 2,
    /// Native u32 encoding.
    U32 = 3,
    /// Native u64 encoding.
    U64 = 4,
    /// Native u128 encoding.
    U128 = 5,
    /// Native i8 encoding.
    I8 = 6,
    /// Native i16 encoding.
    I16 = 7,
    /// Native i32 encoding.
    I32 = 8,
    /// Native i64 encoding.
    I64 = 9,
    /// Native i128 encoding.
    I128 = 10,
    /// Native f32 encoding.
    F32 = 11,
    /// Native f64 encoding.
    F64 = 12,
    /// Native char encoding.
    Char = 13,
}

/// Exported shape of an aggregate.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[repr(u32)]
pub enum RecordShape {
    /// Unit record.
    Unit = 0,
    /// Tuple record.
    Tuple = 1,
    /// One-field tuple struct or enum branch.
    Newtype = 2,
    /// Named record.
    Struct = 3,
}

/// Shared source of truth for supported quantity wrappers and storage metadata.
#[doc(hidden)]
#[macro_export]
macro_rules! __quantity_catalogue {
    ($callback:ident $(, $argument:ident)*) => {
        $callback! {
            $($argument,)*
            [
                (1, absement, Absement, "m s"),
                (2, acceleration, Acceleration, "m s^-2"),
                (3, action, Action, "m^2 kg s^-1"),
                (4, amount_of_substance, AmountOfSubstance, "mol"),
                (5, angle, Angle, "rad"),
                (6, angular_absement, AngularAbsement, "s"),
                (7, angular_acceleration, AngularAcceleration, "s^-2"),
                (8, angular_jerk, AngularJerk, "s^-3"),
                (9, angular_momentum, AngularMomentum, "m^2 kg s^-1"),
                (10, angular_velocity, AngularVelocity, "s^-1"),
                (11, area, Area, "m^2"),
                (12, areal_density_of_states, ArealDensityOfStates, "m^-4 kg^-1 s^2"),
                (13, areal_heat_capacity, ArealHeatCapacity, "kg s^-2 K^-1"),
                (14, areal_mass_density, ArealMassDensity, "m^-2 kg"),
                (15, areal_number_density, ArealNumberDensity, "m^-2"),
                (16, areal_number_rate, ArealNumberRate, "m^-2 s^-1"),
                (17, available_energy, AvailableEnergy, "m^2 s^-2"),
                (18, capacitance, Capacitance, "m^-2 kg^-1 s^4 A^2"),
                (19, catalytic_activity, CatalyticActivity, "s^-1 mol"),
                (20, catalytic_activity_concentration, CatalyticActivityConcentration, "m^-3 s^-1 mol"),
                (21, curvature, Curvature, "m^-1"),
                (22, diffusion_coefficient, DiffusionCoefficient, "m^2 s^-1"),
                (23, dynamic_viscosity, DynamicViscosity, "m^-1 kg s^-1"),
                (24, electric_charge, ElectricCharge, "s A"),
                (25, electric_charge_areal_density, ElectricChargeArealDensity, "m^-2 s A"),
                (26, electric_charge_linear_density, ElectricChargeLinearDensity, "m^-1 s A"),
                (27, electric_charge_volumetric_density, ElectricChargeVolumetricDensity, "m^-3 s A"),
                (28, electric_current, ElectricCurrent, "A"),
                (29, electric_current_density, ElectricCurrentDensity, "m^-2 A"),
                (30, electric_dipole_moment, ElectricDipoleMoment, "m s A"),
                (31, electric_displacement_field, ElectricDisplacementField, "m^-2 s A"),
                (32, electric_field, ElectricField, "m kg s^-3 A^-1"),
                (33, electric_flux, ElectricFlux, "m^3 kg s^-3 A^-1"),
                (34, electric_permittivity, ElectricPermittivity, "m^-3 kg^-1 s^4 A^2"),
                (35, electric_potential, ElectricPotential, "m^2 kg s^-3 A^-1"),
                (36, electric_quadrupole_moment, ElectricQuadrupoleMoment, "m^2 s A"),
                (37, electrical_conductance, ElectricalConductance, "m^-2 kg^-1 s^3 A^2"),
                (38, electrical_conductivity, ElectricalConductivity, "m^-3 kg^-1 s^3 A^2"),
                (39, electrical_mobility, ElectricalMobility, "kg^-1 s^2 A"),
                (40, electrical_resistance, ElectricalResistance, "m^2 kg s^-3 A^-2"),
                (41, electrical_resistivity, ElectricalResistivity, "m^3 kg s^-3 A^-2"),
                (42, energy, Energy, "m^2 kg s^-2"),
                (43, force, Force, "m kg s^-2"),
                (44, frequency, Frequency, "s^-1"),
                (45, frequency_drift, FrequencyDrift, "s^-2"),
                (46, heat_capacity, HeatCapacity, "m^2 kg s^-2 K^-1"),
                (47, heat_flux_density, HeatFluxDensity, "kg s^-3"),
                (48, heat_transfer, HeatTransfer, "kg s^-3 K^-1"),
                (49, inductance, Inductance, "m^2 kg s^-2 A^-2"),
                (50, information, Information, "bit"),
                (51, information_rate, InformationRate, "bit s^-1"),
                (52, inverse_velocity, InverseVelocity, "m^-1 s"),
                (53, jerk, Jerk, "m s^-3"),
                (54, kinematic_viscosity, KinematicViscosity, "m^2 s^-1"),
                (55, length, Length, "m"),
                (56, linear_density_of_states, LinearDensityOfStates, "m^-3 kg^-1 s^2"),
                (57, linear_mass_density, LinearMassDensity, "m^-1 kg"),
                (58, linear_number_density, LinearNumberDensity, "m^-1"),
                (59, linear_number_rate, LinearNumberRate, "m^-1 s^-1"),
                (60, linear_power_density, LinearPowerDensity, "m kg s^-3"),
                (61, luminance, Luminance, "m^-2 cd"),
                (62, luminous_intensity, LuminousIntensity, "cd"),
                (63, magnetic_field_strength, MagneticFieldStrength, "m^-1 A"),
                (64, magnetic_flux, MagneticFlux, "m^2 kg s^-2 A^-1"),
                (65, magnetic_flux_density, MagneticFluxDensity, "kg s^-2 A^-1"),
                (66, magnetic_moment, MagneticMoment, "m^2 A"),
                (67, magnetic_permeability, MagneticPermeability, "m kg s^-2 A^-2"),
                (68, mass, Mass, "kg"),
                (69, mass_concentration, MassConcentration, "m^-3 kg"),
                (70, mass_density, MassDensity, "m^-3 kg"),
                (71, mass_flux, MassFlux, "m^-2 kg s^-1"),
                (72, mass_per_energy, MassPerEnergy, "m^-2 s^2"),
                (73, mass_rate, MassRate, "kg s^-1"),
                (74, molality, Molality, "kg^-1 mol"),
                (75, molar_concentration, MolarConcentration, "m^-3 mol"),
                (76, molar_energy, MolarEnergy, "m^2 kg s^-2 mol^-1"),
                (77, molar_flux, MolarFlux, "m^-2 s^-1 mol"),
                (78, molar_heat_capacity, MolarHeatCapacity, "m^2 kg s^-2 K^-1 mol^-1"),
                (79, molar_mass, MolarMass, "kg mol^-1"),
                (80, molar_radioactivity, MolarRadioactivity, "s^-1 mol^-1"),
                (81, molar_volume, MolarVolume, "m^3 mol^-1"),
                (82, moment_of_inertia, MomentOfInertia, "m^2 kg"),
                (83, momentum, Momentum, "m kg s^-1"),
                (84, power, Power, "m^2 kg s^-3"),
                (85, power_rate, PowerRate, "m^2 kg s^-4"),
                (86, pressure, Pressure, "m^-1 kg s^-2"),
                (87, radiant_exposure, RadiantExposure, "kg s^-2"),
                (88, radioactivity, Radioactivity, "s^-1"),
                (89, ratio, Ratio, "1"),
                (90, reciprocal_length, ReciprocalLength, "m^-1"),
                (91, solid_angle, SolidAngle, "sr"),
                (92, specific_area, SpecificArea, "m^2 kg^-1"),
                (93, specific_heat_capacity, SpecificHeatCapacity, "m^2 s^-2 K^-1"),
                (94, specific_power, SpecificPower, "m^2 s^-3"),
                (95, specific_radioactivity, SpecificRadioactivity, "kg^-1 s^-1"),
                (96, specific_volume, SpecificVolume, "m^3 kg^-1"),
                (97, surface_electric_current_density, SurfaceElectricCurrentDensity, "m^-1 A"),
                (98, surface_tension, SurfaceTension, "kg s^-2"),
                (99, temperature_coefficient, TemperatureCoefficient, "K^-1"),
                (100, temperature_gradient, TemperatureGradient, "m^-1 K"),
                (101, temperature_interval, TemperatureInterval, "K"),
                (102, thermal_conductance, ThermalConductance, "m^2 kg s^-3 K^-1"),
                (103, thermal_conductivity, ThermalConductivity, "m kg s^-3 K^-1"),
                (104, thermal_resistance, ThermalResistance, "m^-2 kg^-1 s^3 K"),
                (105, thermodynamic_temperature, ThermodynamicTemperature, "K"),
                (106, time, Time, "s"),
                (107, torque, Torque, "m^2 kg s^-2"),
                (108, velocity, Velocity, "m s^-1"),
                (109, volume, Volume, "m^3"),
                (110, volume_rate, VolumeRate, "m^3 s^-1"),
                (111, volumetric_density_of_states, VolumetricDensityOfStates, "m^-5 kg^-1 s^2"),
                (112, volumetric_heat_capacity, VolumetricHeatCapacity, "m^-1 kg s^-2 K^-1"),
                (113, volumetric_number_density, VolumetricNumberDensity, "m^-3"),
                (114, volumetric_number_rate, VolumetricNumberRate, "m^-3 s^-1"),
                (115, volumetric_power_density, VolumetricPowerDensity, "m^-1 kg s^-3"),
            ]
        }
    };
}

macro_rules! define_quantities {
    ([$(($id:literal, $module:ident, $variant:ident, $unit:literal),)+]) => {
        /// Copper's supported physical quantities, independent of scalar width.
        #[derive(Clone, Copy, Debug, PartialEq, Eq)]
        #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
        #[repr(u32)]
        pub enum Quantity {
            $(#[doc = concat!("Physical quantity `", stringify!($module), "`.")]
            $variant = $id,)+
        }

        impl Quantity {
            /// Every supported quantity, in catalogue order.
            pub const ALL: &'static [Self] = &[$(Self::$variant,)+];

            /// Permanent identity used in portable metadata.
            pub const fn id(self) -> u32 { self as u32 }

            /// Resolve a permanent identity supported by this Copper version.
            pub const fn from_id(id: u32) -> Option<Self> {
                match id { $($id => Some(Self::$variant),)+ _ => None }
            }

            /// Canonical quantity name for presentation.
            pub const fn name(self) -> &'static str {
                match self { $(Self::$variant => stringify!($module),)+ }
            }

            const fn coherent_unit_symbol(self) -> &'static str {
                match self { $(Self::$variant => $unit,)+ }
            }
        }
    };
}
__quantity_catalogue!(define_quantities);

/// Physical storage unit, with coherent units identified by their quantity.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum StorageUnit {
    /// The quantity's coherent storage unit, including named dimensionless units.
    Coherent(Quantity),
    /// One billionth of a second.
    Nanosecond,
}

impl StorageUnit {
    /// Permanent storage alternative ID, independent of presentation symbols.
    pub const fn id(self) -> u32 {
        match self {
            Self::Coherent(_) => 1,
            Self::Nanosecond => 2,
        }
    }

    /// Presentation symbol, such as `m s^-1`, `rad`, or `ns`.
    pub const fn symbol(self) -> &'static str {
        match self {
            Self::Coherent(quantity) => quantity.coherent_unit_symbol(),
            Self::Nanosecond => "ns",
        }
    }
}

/// Storage choices for Copper time quantities.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum TimeStorageUnit {
    /// Coherent SI time storage.
    Second,
    /// Copper clock storage.
    Nanosecond,
}

/// A physical quantity associated with a compatible storage unit.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum QuantityMetadata {
    /// Coherent storage for a supported quantity.
    Coherent(Quantity),
    /// Copper's nanosecond time storage.
    NanosecondTime,
}

impl QuantityMetadata {
    /// Describe the quantity's coherent storage, including named dimensionless units.
    pub const fn coherent(quantity: Quantity) -> Self {
        Self::Coherent(quantity)
    }

    /// Describe time with an explicitly selected storage unit.
    pub const fn time(unit: TimeStorageUnit) -> Self {
        match unit {
            TimeStorageUnit::Second => Self::Coherent(Quantity::Time),
            TimeStorageUnit::Nanosecond => Self::NanosecondTime,
        }
    }

    /// Physical quantity identity.
    pub const fn quantity(self) -> Quantity {
        match self {
            Self::Coherent(quantity) => quantity,
            Self::NanosecondTime => Quantity::Time,
        }
    }

    /// Compatible storage unit.
    pub const fn storage_unit(self) -> StorageUnit {
        match self {
            Self::Coherent(quantity) => StorageUnit::Coherent(quantity),
            Self::NanosecondTime => StorageUnit::Nanosecond,
        }
    }
}

/// Copper-owned logical metadata attached to a value's schema.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum ValueMetadata {
    /// Physical quantity and its actual encoded storage unit.
    Quantity(QuantityMetadata),
}

impl ValueMetadata {
    /// Permanent metadata kind ID used in length-delimited portable entries.
    pub const fn kind_id(self) -> u32 {
        match self {
            Self::Quantity(_) => 1,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_catalogue_ids_are_unique_and_resolve() {
        for (index, quantity) in Quantity::ALL.iter().enumerate() {
            assert_eq!(Quantity::from_id(quantity.id()), Some(*quantity));
            assert!(
                !Quantity::ALL[..index]
                    .iter()
                    .any(|previous| previous.id() == quantity.id())
            );
        }
        assert_eq!(Quantity::from_id(0), None);
        assert_eq!(Quantity::from_id(u32::MAX), None);
    }

    #[test]
    fn test_quantity_storage_and_presentation() {
        for (quantity, symbol) in [
            (Quantity::Length, "m"),
            (Quantity::Mass, "kg"),
            (Quantity::Velocity, "m s^-1"),
            (Quantity::Angle, "rad"),
            (Quantity::SolidAngle, "sr"),
            (Quantity::Information, "bit"),
            (Quantity::InformationRate, "bit s^-1"),
            (Quantity::Ratio, "1"),
        ] {
            let metadata = QuantityMetadata::coherent(quantity);
            assert_eq!(metadata.quantity(), quantity);
            assert_eq!(metadata.storage_unit(), StorageUnit::Coherent(quantity));
            assert_eq!(metadata.storage_unit().symbol(), symbol);
        }
        assert_eq!(
            QuantityMetadata::time(TimeStorageUnit::Second),
            QuantityMetadata::coherent(Quantity::Time)
        );
        let nanos = QuantityMetadata::time(TimeStorageUnit::Nanosecond);
        assert_eq!(nanos.quantity(), Quantity::Time);
        assert_eq!(nanos.storage_unit(), StorageUnit::Nanosecond);
        assert_eq!(nanos.storage_unit().symbol(), "ns");
        assert_eq!(nanos.storage_unit().id(), 2);
        assert_eq!(ValueMetadata::Quantity(nanos).kind_id(), 1);
    }
}
