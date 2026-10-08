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
                (1, absement, Absement, "m·s"),
                (2, acceleration, Acceleration, "m·s⁻²"),
                (3, action, Action, "J·s"),
                (4, amount_of_substance, AmountOfSubstance, "mol"),
                (5, angle, Angle, "rad"),
                (6, angular_absement, AngularAbsement, "rad·s"),
                (7, angular_acceleration, AngularAcceleration, "rad·s⁻²"),
                (8, angular_jerk, AngularJerk, "rad·s⁻³"),
                (9, angular_momentum, AngularMomentum, "N·m·s"),
                (10, angular_velocity, AngularVelocity, "rad·s⁻¹"),
                (11, area, Area, "m²"),
                (12, areal_density_of_states, ArealDensityOfStates, "m⁻²·J⁻¹"),
                (13, areal_heat_capacity, ArealHeatCapacity, "J·m⁻²·K⁻¹"),
                (14, areal_mass_density, ArealMassDensity, "kg·m⁻²"),
                (15, areal_number_density, ArealNumberDensity, "m⁻²"),
                (16, areal_number_rate, ArealNumberRate, "m⁻²·s⁻¹"),
                (17, available_energy, AvailableEnergy, "J·kg⁻¹"),
                (18, capacitance, Capacitance, "F"),
                (19, catalytic_activity, CatalyticActivity, "kat"),
                (20, catalytic_activity_concentration, CatalyticActivityConcentration, "kat·m⁻³"),
                (21, curvature, Curvature, "rad·m⁻¹"),
                (22, diffusion_coefficient, DiffusionCoefficient, "m²·s⁻¹"),
                (23, dynamic_viscosity, DynamicViscosity, "Pa·s"),
                (24, electric_charge, ElectricCharge, "C"),
                (25, electric_charge_areal_density, ElectricChargeArealDensity, "C·m⁻²"),
                (26, electric_charge_linear_density, ElectricChargeLinearDensity, "C·m⁻¹"),
                (27, electric_charge_volumetric_density, ElectricChargeVolumetricDensity, "C·m⁻³"),
                (28, electric_current, ElectricCurrent, "A"),
                (29, electric_current_density, ElectricCurrentDensity, "A·m⁻²"),
                (30, electric_dipole_moment, ElectricDipoleMoment, "C·m"),
                (31, electric_displacement_field, ElectricDisplacementField, "C·m⁻²"),
                (32, electric_field, ElectricField, "V·m⁻¹"),
                (33, electric_flux, ElectricFlux, "V·m"),
                (34, electric_permittivity, ElectricPermittivity, "F·m⁻¹"),
                (35, electric_potential, ElectricPotential, "V"),
                (36, electric_quadrupole_moment, ElectricQuadrupoleMoment, "C·m²"),
                (37, electrical_conductance, ElectricalConductance, "S"),
                (38, electrical_conductivity, ElectricalConductivity, "S·m⁻¹"),
                (39, electrical_mobility, ElectricalMobility, "m²·V⁻¹·s⁻¹"),
                (40, electrical_resistance, ElectricalResistance, "Ω"),
                (41, electrical_resistivity, ElectricalResistivity, "Ω·m"),
                (42, energy, Energy, "J"),
                (43, force, Force, "N"),
                (44, frequency, Frequency, "Hz"),
                (45, frequency_drift, FrequencyDrift, "Hz·s⁻¹"),
                (46, heat_capacity, HeatCapacity, "J·K⁻¹"),
                (47, heat_flux_density, HeatFluxDensity, "W·m⁻²"),
                (48, heat_transfer, HeatTransfer, "W·m⁻²·K⁻¹"),
                (49, inductance, Inductance, "H"),
                (50, information, Information, "bit"),
                (51, information_rate, InformationRate, "bit·s⁻¹"),
                (52, inverse_velocity, InverseVelocity, "s·m⁻¹"),
                (53, jerk, Jerk, "m·s⁻³"),
                (54, kinematic_viscosity, KinematicViscosity, "m²·s⁻¹"),
                (55, length, Length, "m"),
                (56, linear_density_of_states, LinearDensityOfStates, "m⁻¹·J⁻¹"),
                (57, linear_mass_density, LinearMassDensity, "kg·m⁻¹"),
                (58, linear_number_density, LinearNumberDensity, "m⁻¹"),
                (59, linear_number_rate, LinearNumberRate, "m⁻¹·s⁻¹"),
                (60, linear_power_density, LinearPowerDensity, "W·m⁻¹"),
                (61, luminance, Luminance, "cd·m⁻²"),
                (62, luminous_intensity, LuminousIntensity, "cd"),
                (63, magnetic_field_strength, MagneticFieldStrength, "A·m⁻¹"),
                (64, magnetic_flux, MagneticFlux, "Wb"),
                (65, magnetic_flux_density, MagneticFluxDensity, "T"),
                (66, magnetic_moment, MagneticMoment, "A·m²"),
                (67, magnetic_permeability, MagneticPermeability, "H·m⁻¹"),
                (68, mass, Mass, "kg"),
                (69, mass_concentration, MassConcentration, "kg·m⁻³"),
                (70, mass_density, MassDensity, "kg·m⁻³"),
                (71, mass_flux, MassFlux, "kg·m⁻²·s⁻¹"),
                (72, mass_per_energy, MassPerEnergy, "kg·J⁻¹"),
                (73, mass_rate, MassRate, "kg·s⁻¹"),
                (74, molality, Molality, "mol·kg⁻¹"),
                (75, molar_concentration, MolarConcentration, "mol·m⁻³"),
                (76, molar_energy, MolarEnergy, "J·mol⁻¹"),
                (77, molar_flux, MolarFlux, "mol·m⁻²·s⁻¹"),
                (78, molar_heat_capacity, MolarHeatCapacity, "J·K⁻¹·mol⁻¹"),
                (79, molar_mass, MolarMass, "kg·mol⁻¹"),
                (80, molar_radioactivity, MolarRadioactivity, "Bq·mol⁻¹"),
                (81, molar_volume, MolarVolume, "m³·mol⁻¹"),
                (82, moment_of_inertia, MomentOfInertia, "kg·m²"),
                (83, momentum, Momentum, "kg·m·s⁻¹"),
                (84, power, Power, "W"),
                (85, power_rate, PowerRate, "W·s⁻¹"),
                (86, pressure, Pressure, "Pa"),
                (87, radiant_exposure, RadiantExposure, "J·m⁻²"),
                (88, radioactivity, Radioactivity, "Bq"),
                (89, ratio, Ratio, "1"),
                (90, reciprocal_length, ReciprocalLength, "m⁻¹"),
                (91, solid_angle, SolidAngle, "sr"),
                (92, specific_area, SpecificArea, "m²·kg⁻¹"),
                (93, specific_heat_capacity, SpecificHeatCapacity, "J·kg⁻¹·K⁻¹"),
                (94, specific_power, SpecificPower, "W·kg⁻¹"),
                (95, specific_radioactivity, SpecificRadioactivity, "Bq·kg⁻¹"),
                (96, specific_volume, SpecificVolume, "m³·kg⁻¹"),
                (97, surface_electric_current_density, SurfaceElectricCurrentDensity, "A·m⁻¹"),
                (98, surface_tension, SurfaceTension, "N·m⁻¹"),
                (99, temperature_coefficient, TemperatureCoefficient, "K⁻¹"),
                (100, temperature_gradient, TemperatureGradient, "K·m⁻¹"),
                (101, temperature_interval, TemperatureInterval, "K"),
                (102, thermal_conductance, ThermalConductance, "W·K⁻¹"),
                (103, thermal_conductivity, ThermalConductivity, "W·m⁻¹·K⁻¹"),
                (104, thermal_resistance, ThermalResistance, "K·W⁻¹"),
                (105, thermodynamic_temperature, ThermodynamicTemperature, "K"),
                (106, time, Time, "s"),
                (107, torque, Torque, "N·m"),
                (108, velocity, Velocity, "m·s⁻¹"),
                (109, volume, Volume, "m³"),
                (110, volume_rate, VolumeRate, "m³·s⁻¹"),
                (111, volumetric_density_of_states, VolumetricDensityOfStates, "m⁻³·J⁻¹"),
                (112, volumetric_heat_capacity, VolumetricHeatCapacity, "J·m⁻³·K⁻¹"),
                (113, volumetric_number_density, VolumetricNumberDensity, "m⁻³"),
                (114, volumetric_number_rate, VolumetricNumberRate, "m⁻³·s⁻¹"),
                (115, volumetric_power_density, VolumetricPowerDensity, "W·m⁻³"),
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
