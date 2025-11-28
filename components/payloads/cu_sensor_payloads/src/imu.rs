use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{Decode, Encode};
use serde::Serialize;
use uom::si::acceleration::meter_per_second_squared;
use uom::si::angular_velocity::radian_per_second;
use uom::si::f32::{Acceleration, AngularVelocity, MagneticFluxDensity, ThermodynamicTemperature};
use uom::si::magnetic_flux_density::microtesla;
use uom::si::magnetic_flux_density::tesla;
use uom::si::thermodynamic_temperature::degree_celsius;

/// Standardized IMU payload carrying acceleration, angular velocity, and optional magnetometer data.
#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct ImuPayload {
    pub accel: [Acceleration; 3],
    pub gyro: [AngularVelocity; 3],
    pub mag: Option<[MagneticFluxDensity; 3]>,
    pub temperature: ThermodynamicTemperature,
}

impl Default for ImuPayload {
    fn default() -> Self {
        Self {
            accel: [Acceleration::new::<meter_per_second_squared>(0.0); 3],
            gyro: [AngularVelocity::new::<radian_per_second>(0.0); 3],
            mag: None,
            temperature: ThermodynamicTemperature::new::<degree_celsius>(0.0),
        }
    }
}

impl ImuPayload {
    /// Build an IMU payload from plain scalar values.
    ///
    /// * `accel_mps2` - acceleration in m/s².
    /// * `gyro_rad` - angular velocity in rad/s.
    /// * `mag_ut` - magnetometer in microtesla.
    /// * `temperature_c` - temperature in °C.
    pub fn from_raw(
        accel_mps2: [f32; 3],
        gyro_rad: [f32; 3],
        mag_ut: Option<[f32; 3]>,
        temperature_c: f32,
    ) -> Self {
        let accel = accel_mps2.map(|v| Acceleration::new::<meter_per_second_squared>(v));
        let gyro = gyro_rad.map(|v| AngularVelocity::new::<radian_per_second>(v));
        let mag = mag_ut.map(|values| values.map(|v| MagneticFluxDensity::new::<microtesla>(v)));
        let temperature = ThermodynamicTemperature::new::<degree_celsius>(temperature_c);

        Self {
            accel,
            gyro,
            mag,
            temperature,
        }
    }

    /// Build an IMU payload from unit-carrying types.
    pub fn from_uom(
        accel: [Acceleration; 3],
        gyro: [AngularVelocity; 3],
        mag: Option<[MagneticFluxDensity; 3]>,
        temperature: ThermodynamicTemperature,
    ) -> Self {
        Self {
            accel,
            gyro,
            mag,
            temperature,
        }
    }
}

impl Encode for ImuPayload {
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        for axis in &self.accel {
            Encode::encode(&axis.value, encoder)?;
        }
        for axis in &self.gyro {
            Encode::encode(&axis.value, encoder)?;
        }

        Encode::encode(&self.mag.is_some(), encoder)?;
        if let Some(mag) = self.mag {
            for axis in &mag {
                Encode::encode(&axis.value, encoder)?;
            }
        }

        Encode::encode(&self.temperature.get::<degree_celsius>(), encoder)?;
        Ok(())
    }
}

impl Decode<()> for ImuPayload {
    fn decode<D: Decoder<Context = ()>>(decoder: &mut D) -> Result<Self, DecodeError> {
        let accel = [
            Acceleration::new::<meter_per_second_squared>(Decode::decode(decoder)?),
            Acceleration::new::<meter_per_second_squared>(Decode::decode(decoder)?),
            Acceleration::new::<meter_per_second_squared>(Decode::decode(decoder)?),
        ];

        let gyro = [
            AngularVelocity::new::<radian_per_second>(Decode::decode(decoder)?),
            AngularVelocity::new::<radian_per_second>(Decode::decode(decoder)?),
            AngularVelocity::new::<radian_per_second>(Decode::decode(decoder)?),
        ];

        let has_mag: bool = Decode::decode(decoder)?;
        let mag = if has_mag {
            Some([
                MagneticFluxDensity::new::<tesla>(Decode::decode(decoder)?),
                MagneticFluxDensity::new::<tesla>(Decode::decode(decoder)?),
                MagneticFluxDensity::new::<tesla>(Decode::decode(decoder)?),
            ])
        } else {
            None
        };

        let temperature = ThermodynamicTemperature::new::<degree_celsius>(Decode::decode(decoder)?);

        Ok(Self {
            accel,
            gyro,
            mag,
            temperature,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bincode::config;

    #[test]
    fn round_trip_encode_decode() {
        let payload = ImuPayload::from_raw(
            [9.8, -2.0, 0.5],
            [0.01, -0.02, 0.5],
            Some([42.0, -13.0, 8.0]),
            36.5,
        );

        let cfg = config::standard();
        let mut buffer = [0u8; 128];
        let len = bincode::encode_into_slice(payload, &mut buffer, cfg).unwrap();
        let (decoded, used) =
            bincode::decode_from_slice::<ImuPayload, _>(&buffer[..len], cfg).unwrap();

        assert_eq!(used, len);
        assert_eq!(decoded.accel[0].value, payload.accel[0].value);
        assert_eq!(decoded.gyro[1].value, payload.gyro[1].value);
        assert_eq!(decoded.mag.unwrap()[2].value, payload.mag.unwrap()[2].value);
        assert_eq!(
            decoded.temperature.get::<degree_celsius>(),
            payload.temperature.get::<degree_celsius>()
        );
    }

    #[test]
    fn builds_from_units() {
        let accel = Acceleration::new::<meter_per_second_squared>(9.81);
        let gyro = AngularVelocity::new::<radian_per_second>(0.25);
        let mag = MagneticFluxDensity::new::<microtesla>(12.0);
        let temp = ThermodynamicTemperature::new::<degree_celsius>(20.0);

        let payload = ImuPayload::from_uom([accel; 3], [gyro; 3], Some([mag; 3]), temp);

        assert_eq!(payload.accel[0].value, accel.value);
        assert!(payload.mag.is_some());
    }
}
