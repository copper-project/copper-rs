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
                (1, absement, Absement, "m·s", "m s"),
                (2, acceleration, Acceleration, "m·s⁻²", "m s^-2"),
                (3, action, Action, "J·s", "m^2 kg s^-1"),
                (4, amount_of_substance, AmountOfSubstance, "mol", "mol"),
                (5, angle, Angle, "rad", "rad"),
                (6, angular_absement, AngularAbsement, "rad·s", "s"),
                (7, angular_acceleration, AngularAcceleration, "rad·s⁻²", "s^-2"),
                (8, angular_jerk, AngularJerk, "rad·s⁻³", "s^-3"),
                (9, angular_momentum, AngularMomentum, "N·m·s", "m^2 kg s^-1"),
                (10, angular_velocity, AngularVelocity, "rad·s⁻¹", "s^-1"),
                (11, area, Area, "m²", "m^2"),
                (12, areal_density_of_states, ArealDensityOfStates, "m⁻²·J⁻¹", "m^-4 kg^-1 s^2"),
                (13, areal_heat_capacity, ArealHeatCapacity, "J·m⁻²·K⁻¹", "kg s^-2 K^-1"),
                (14, areal_mass_density, ArealMassDensity, "kg·m⁻²", "m^-2 kg"),
                (15, areal_number_density, ArealNumberDensity, "m⁻²", "m^-2"),
                (16, areal_number_rate, ArealNumberRate, "m⁻²·s⁻¹", "m^-2 s^-1"),
                (17, available_energy, AvailableEnergy, "J·kg⁻¹", "m^2 s^-2"),
                (18, capacitance, Capacitance, "F", "m^-2 kg^-1 s^4 A^2"),
                (19, catalytic_activity, CatalyticActivity, "kat", "s^-1 mol"),
                (20, catalytic_activity_concentration, CatalyticActivityConcentration, "kat·m⁻³", "m^-3 s^-1 mol"),
                (21, curvature, Curvature, "rad·m⁻¹", "m^-1"),
                (22, diffusion_coefficient, DiffusionCoefficient, "m²·s⁻¹", "m^2 s^-1"),
                (23, dynamic_viscosity, DynamicViscosity, "Pa·s", "m^-1 kg s^-1"),
                (24, electric_charge, ElectricCharge, "C", "s A"),
                (25, electric_charge_areal_density, ElectricChargeArealDensity, "C·m⁻²", "m^-2 s A"),
                (26, electric_charge_linear_density, ElectricChargeLinearDensity, "C·m⁻¹", "m^-1 s A"),
                (27, electric_charge_volumetric_density, ElectricChargeVolumetricDensity, "C·m⁻³", "m^-3 s A"),
                (28, electric_current, ElectricCurrent, "A", "A"),
                (29, electric_current_density, ElectricCurrentDensity, "A·m⁻²", "m^-2 A"),
                (30, electric_dipole_moment, ElectricDipoleMoment, "C·m", "m s A"),
                (31, electric_displacement_field, ElectricDisplacementField, "C·m⁻²", "m^-2 s A"),
                (32, electric_field, ElectricField, "V·m⁻¹", "m kg s^-3 A^-1"),
                (33, electric_flux, ElectricFlux, "V·m", "m^3 kg s^-3 A^-1"),
                (34, electric_permittivity, ElectricPermittivity, "F·m⁻¹", "m^-3 kg^-1 s^4 A^2"),
                (35, electric_potential, ElectricPotential, "V", "m^2 kg s^-3 A^-1"),
                (36, electric_quadrupole_moment, ElectricQuadrupoleMoment, "C·m²", "m^2 s A"),
                (37, electrical_conductance, ElectricalConductance, "S", "m^-2 kg^-1 s^3 A^2"),
                (38, electrical_conductivity, ElectricalConductivity, "S·m⁻¹", "m^-3 kg^-1 s^3 A^2"),
                (39, electrical_mobility, ElectricalMobility, "m²·V⁻¹·s⁻¹", "kg^-1 s^2 A"),
                (40, electrical_resistance, ElectricalResistance, "Ω", "m^2 kg s^-3 A^-2"),
                (41, electrical_resistivity, ElectricalResistivity, "Ω·m", "m^3 kg s^-3 A^-2"),
                (42, energy, Energy, "J", "m^2 kg s^-2"),
                (43, force, Force, "N", "m kg s^-2"),
                (44, frequency, Frequency, "Hz", "s^-1"),
                (45, frequency_drift, FrequencyDrift, "Hz·s⁻¹", "s^-2"),
                (46, heat_capacity, HeatCapacity, "J·K⁻¹", "m^2 kg s^-2 K^-1"),
                (47, heat_flux_density, HeatFluxDensity, "W·m⁻²", "kg s^-3"),
                (48, heat_transfer, HeatTransfer, "W·m⁻²·K⁻¹", "kg s^-3 K^-1"),
                (49, inductance, Inductance, "H", "m^2 kg s^-2 A^-2"),
                (50, information, Information, "bit", "bit"),
                (51, information_rate, InformationRate, "bit·s⁻¹", "bit s^-1"),
                (52, inverse_velocity, InverseVelocity, "s·m⁻¹", "m^-1 s"),
                (53, jerk, Jerk, "m·s⁻³", "m s^-3"),
                (54, kinematic_viscosity, KinematicViscosity, "m²·s⁻¹", "m^2 s^-1"),
                (55, length, Length, "m", "m"),
                (56, linear_density_of_states, LinearDensityOfStates, "m⁻¹·J⁻¹", "m^-3 kg^-1 s^2"),
                (57, linear_mass_density, LinearMassDensity, "kg·m⁻¹", "m^-1 kg"),
                (58, linear_number_density, LinearNumberDensity, "m⁻¹", "m^-1"),
                (59, linear_number_rate, LinearNumberRate, "m⁻¹·s⁻¹", "m^-1 s^-1"),
                (60, linear_power_density, LinearPowerDensity, "W·m⁻¹", "m kg s^-3"),
                (61, luminance, Luminance, "cd·m⁻²", "m^-2 cd"),
                (62, luminous_intensity, LuminousIntensity, "cd", "cd"),
                (63, magnetic_field_strength, MagneticFieldStrength, "A·m⁻¹", "m^-1 A"),
                (64, magnetic_flux, MagneticFlux, "Wb", "m^2 kg s^-2 A^-1"),
                (65, magnetic_flux_density, MagneticFluxDensity, "T", "kg s^-2 A^-1"),
                (66, magnetic_moment, MagneticMoment, "A·m²", "m^2 A"),
                (67, magnetic_permeability, MagneticPermeability, "H·m⁻¹", "m kg s^-2 A^-2"),
                (68, mass, Mass, "kg", "kg"),
                (69, mass_concentration, MassConcentration, "kg·m⁻³", "m^-3 kg"),
                (70, mass_density, MassDensity, "kg·m⁻³", "m^-3 kg"),
                (71, mass_flux, MassFlux, "kg·m⁻²·s⁻¹", "m^-2 kg s^-1"),
                (72, mass_per_energy, MassPerEnergy, "kg·J⁻¹", "m^-2 s^2"),
                (73, mass_rate, MassRate, "kg·s⁻¹", "kg s^-1"),
                (74, molality, Molality, "mol·kg⁻¹", "kg^-1 mol"),
                (75, molar_concentration, MolarConcentration, "mol·m⁻³", "m^-3 mol"),
                (76, molar_energy, MolarEnergy, "J·mol⁻¹", "m^2 kg s^-2 mol^-1"),
                (77, molar_flux, MolarFlux, "mol·m⁻²·s⁻¹", "m^-2 s^-1 mol"),
                (78, molar_heat_capacity, MolarHeatCapacity, "J·K⁻¹·mol⁻¹", "m^2 kg s^-2 K^-1 mol^-1"),
                (79, molar_mass, MolarMass, "kg·mol⁻¹", "kg mol^-1"),
                (80, molar_radioactivity, MolarRadioactivity, "Bq·mol⁻¹", "s^-1 mol^-1"),
                (81, molar_volume, MolarVolume, "m³·mol⁻¹", "m^3 mol^-1"),
                (82, moment_of_inertia, MomentOfInertia, "kg·m²", "m^2 kg"),
                (83, momentum, Momentum, "kg·m·s⁻¹", "m kg s^-1"),
                (84, power, Power, "W", "m^2 kg s^-3"),
                (85, power_rate, PowerRate, "W·s⁻¹", "m^2 kg s^-4"),
                (86, pressure, Pressure, "Pa", "m^-1 kg s^-2"),
                (87, radiant_exposure, RadiantExposure, "J·m⁻²", "kg s^-2"),
                (88, radioactivity, Radioactivity, "Bq", "s^-1"),
                (89, ratio, Ratio, "1", "1"),
                (90, reciprocal_length, ReciprocalLength, "m⁻¹", "m^-1"),
                (91, solid_angle, SolidAngle, "sr", "sr"),
                (92, specific_area, SpecificArea, "m²·kg⁻¹", "m^2 kg^-1"),
                (93, specific_heat_capacity, SpecificHeatCapacity, "J·kg⁻¹·K⁻¹", "m^2 s^-2 K^-1"),
                (94, specific_power, SpecificPower, "W·kg⁻¹", "m^2 s^-3"),
                (95, specific_radioactivity, SpecificRadioactivity, "Bq·kg⁻¹", "kg^-1 s^-1"),
                (96, specific_volume, SpecificVolume, "m³·kg⁻¹", "m^3 kg^-1"),
                (97, surface_electric_current_density, SurfaceElectricCurrentDensity, "A·m⁻¹", "m^-1 A"),
                (98, surface_tension, SurfaceTension, "N·m⁻¹", "kg s^-2"),
                (99, temperature_coefficient, TemperatureCoefficient, "K⁻¹", "K^-1"),
                (100, temperature_gradient, TemperatureGradient, "K·m⁻¹", "m^-1 K"),
                (101, temperature_interval, TemperatureInterval, "K", "K"),
                (102, thermal_conductance, ThermalConductance, "W·K⁻¹", "m^2 kg s^-3 K^-1"),
                (103, thermal_conductivity, ThermalConductivity, "W·m⁻¹·K⁻¹", "m kg s^-3 K^-1"),
                (104, thermal_resistance, ThermalResistance, "K·W⁻¹", "m^-2 kg^-1 s^3 K"),
                (105, thermodynamic_temperature, ThermodynamicTemperature, "K", "K"),
                (106, time, Time, "s", "s"),
                (107, torque, Torque, "N·m", "m^2 kg s^-2"),
                (108, velocity, Velocity, "m·s⁻¹", "m s^-1"),
                (109, volume, Volume, "m³", "m^3"),
                (110, volume_rate, VolumeRate, "m³·s⁻¹", "m^3 s^-1"),
                (111, volumetric_density_of_states, VolumetricDensityOfStates, "m⁻³·J⁻¹", "m^-5 kg^-1 s^2"),
                (112, volumetric_heat_capacity, VolumetricHeatCapacity, "J·m⁻³·K⁻¹", "m^-1 kg s^-2 K^-1"),
                (113, volumetric_number_density, VolumetricNumberDensity, "m⁻³", "m^-3"),
                (114, volumetric_number_rate, VolumetricNumberRate, "m⁻³·s⁻¹", "m^-3 s^-1"),
                (115, volumetric_power_density, VolumetricPowerDensity, "W·m⁻³", "m^-1 kg s^-3"),
            ]
        }
    };
}

macro_rules! define_quantities {
    ([$(($id:literal, $module:ident, $variant:ident, $unit:literal, $legacy:literal),)+]) => {
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

            /// ASCII base-dimension spelling used in V1/V2 catalogs.
            #[doc(hidden)]
            pub const fn legacy_coherent_unit_symbol(self) -> &'static str {
                match self { $(Self::$variant => $legacy,)+ }
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

    /// Presentation symbol, such as `m·s⁻¹`, `rad`, or `ns`.
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
            (Quantity::Velocity, "m·s⁻¹"),
            (Quantity::Angle, "rad"),
            (Quantity::SolidAngle, "sr"),
            (Quantity::Information, "bit"),
            (Quantity::InformationRate, "bit·s⁻¹"),
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
