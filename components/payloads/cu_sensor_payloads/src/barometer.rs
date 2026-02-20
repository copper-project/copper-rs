use bincode::{Decode, Encode};
use cu29::prelude::*;
use cu29::units::si::f32::{Pressure, ThermodynamicTemperature};
use cu29::units::si::pressure::pascal;
use cu29::units::si::thermodynamic_temperature::degree_celsius;
use serde::{Deserialize, Serialize};

/// Standardized barometer payload carrying pressure and temperature.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, Encode, Decode, Reflect)]
pub struct BarometerPayload {
    pub pressure: Pressure,
    pub temperature: ThermodynamicTemperature,
}

impl Default for BarometerPayload {
    fn default() -> Self {
        Self {
            pressure: Pressure::new::<pascal>(0.0),
            temperature: ThermodynamicTemperature::new::<degree_celsius>(0.0),
        }
    }
}

impl BarometerPayload {
    /// Build a barometer payload from scalar values.
    ///
    /// * `pressure_pa` - pressure in Pascal.
    /// * `temperature_c` - temperature in °C.
    pub fn from_raw(pressure_pa: f32, temperature_c: f32) -> Self {
        Self {
            pressure: Pressure::new::<pascal>(pressure_pa),
            temperature: ThermodynamicTemperature::new::<degree_celsius>(temperature_c),
        }
    }

    /// Build a barometer payload from unit-carrying types.
    pub fn from_units(pressure: Pressure, temperature: ThermodynamicTemperature) -> Self {
        Self {
            pressure,
            temperature,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bincode::config;

    #[test]
    fn round_trip_encode_decode() {
        let payload = BarometerPayload::from_raw(101_325.0, 23.4);

        let cfg = config::standard();
        let mut buffer = [0u8; 128];
        let len = bincode::encode_into_slice(payload, &mut buffer, cfg).unwrap();
        let (decoded, used) =
            bincode::decode_from_slice::<BarometerPayload, _>(&buffer[..len], cfg).unwrap();

        assert_eq!(used, len);
        assert_eq!(
            decoded.pressure.get::<pascal>(),
            payload.pressure.get::<pascal>()
        );
        assert_eq!(
            decoded.temperature.get::<degree_celsius>(),
            payload.temperature.get::<degree_celsius>()
        );
    }
}
