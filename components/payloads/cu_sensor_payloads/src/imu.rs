use bincode::{Decode, Encode};
use cu29::prelude::*;
use cu29::units::si::acceleration::meter_per_second_squared;
use cu29::units::si::angular_velocity::radian_per_second;
use cu29::units::si::f32::{
    Acceleration, AngularVelocity, MagneticFluxDensity, ThermodynamicTemperature,
};
use cu29::units::si::magnetic_flux_density::microtesla;
use cu29::units::si::thermodynamic_temperature::degree_celsius;
use serde::{Deserialize, Serialize};

/// Standardized IMU payload carrying acceleration, angular velocity, and optional magnetometer data.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, Encode, Decode, Reflect)]
pub struct ImuPayload {
    pub accel_x: Acceleration,
    pub accel_y: Acceleration,
    pub accel_z: Acceleration,
    pub gyro_x: AngularVelocity,
    pub gyro_y: AngularVelocity,
    pub gyro_z: AngularVelocity,
    pub temperature: ThermodynamicTemperature,
}

impl Default for ImuPayload {
    fn default() -> Self {
        Self {
            accel_x: Acceleration::new::<meter_per_second_squared>(0.0),
            accel_y: Acceleration::new::<meter_per_second_squared>(0.0),
            accel_z: Acceleration::new::<meter_per_second_squared>(0.0),
            gyro_x: AngularVelocity::new::<radian_per_second>(0.0),
            gyro_y: AngularVelocity::new::<radian_per_second>(0.0),
            gyro_z: AngularVelocity::new::<radian_per_second>(0.0),
            temperature: ThermodynamicTemperature::new::<degree_celsius>(0.0),
        }
    }
}

impl ImuPayload {
    /// Build an IMU payload from plain scalar values.
    ///
    /// * `accel_mps2` - acceleration in m/s².
    /// * `gyro_rad` - angular velocity in rad/s.
    /// * `temperature_c` - temperature in °C.
    pub fn from_raw(accel_mps2: [f32; 3], gyro_rad: [f32; 3], temperature_c: f32) -> Self {
        let [accel_x, accel_y, accel_z] =
            accel_mps2.map(Acceleration::new::<meter_per_second_squared>);
        let [gyro_x, gyro_y, gyro_z] = gyro_rad.map(AngularVelocity::new::<radian_per_second>);
        let temperature = ThermodynamicTemperature::new::<degree_celsius>(temperature_c);

        Self {
            accel_x,
            accel_y,
            accel_z,
            gyro_x,
            gyro_y,
            gyro_z,
            temperature,
        }
    }

    /// Build an IMU payload from unit-carrying types.
    pub fn from_units(
        accel_x: Acceleration,
        accel_y: Acceleration,
        accel_z: Acceleration,
        gyro_x: AngularVelocity,
        gyro_y: AngularVelocity,
        gyro_z: AngularVelocity,
        temperature: ThermodynamicTemperature,
    ) -> Self {
        Self {
            accel_x,
            accel_y,
            accel_z,
            gyro_x,
            gyro_y,
            gyro_z,
            temperature,
        }
    }
}

/// Magnetometer payload split from the main IMU data for composition.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, Encode, Decode)]
pub struct MagnetometerPayload {
    pub mag_x: MagneticFluxDensity,
    pub mag_y: MagneticFluxDensity,
    pub mag_z: MagneticFluxDensity,
}

impl Default for MagnetometerPayload {
    fn default() -> Self {
        Self {
            mag_x: MagneticFluxDensity::new::<microtesla>(0.0),
            mag_y: MagneticFluxDensity::new::<microtesla>(0.0),
            mag_z: MagneticFluxDensity::new::<microtesla>(0.0),
        }
    }
}

impl MagnetometerPayload {
    /// Build a magnetometer payload from raw microtesla values.
    pub fn from_raw(mag_ut: [f32; 3]) -> Self {
        let [mag_x, mag_y, mag_z] = mag_ut.map(MagneticFluxDensity::new::<microtesla>);
        Self {
            mag_x,
            mag_y,
            mag_z,
        }
    }

    /// Build a magnetometer payload from unit-carrying types.
    pub fn from_units(
        mag_x: MagneticFluxDensity,
        mag_y: MagneticFluxDensity,
        mag_z: MagneticFluxDensity,
    ) -> Self {
        Self {
            mag_x,
            mag_y,
            mag_z,
        }
    }
}

/// Combined payload allowing optional magnetometer data.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, Encode, Decode)]
pub struct ImuWithMagPayload {
    pub imu: ImuPayload,
    pub mag: Option<MagnetometerPayload>,
}

impl ImuWithMagPayload {
    pub fn new(imu: ImuPayload, mag: Option<MagnetometerPayload>) -> Self {
        Self { imu, mag }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bincode::config;

    #[test]
    fn round_trip_encode_decode() {
        let payload = ImuPayload::from_raw([9.8, -2.0, 0.5], [0.01, -0.02, 0.5], 36.5);

        let cfg = config::standard();
        let mut buffer = [0u8; 128];
        let len = bincode::encode_into_slice(payload, &mut buffer, cfg).unwrap();
        let (decoded, used) =
            bincode::decode_from_slice::<ImuPayload, _>(&buffer[..len], cfg).unwrap();

        assert_eq!(used, len);
        assert_eq!(decoded.accel_x.value, payload.accel_x.value);
        assert_eq!(decoded.gyro_y.value, payload.gyro_y.value);
        assert_eq!(
            decoded.temperature.get::<degree_celsius>(),
            payload.temperature.get::<degree_celsius>()
        );
    }

    #[test]
    fn builds_from_units() {
        let accel = Acceleration::new::<meter_per_second_squared>(9.81);
        let gyro = AngularVelocity::new::<radian_per_second>(0.25);
        let temp = ThermodynamicTemperature::new::<degree_celsius>(20.0);

        let payload = ImuPayload::from_units(accel, accel, accel, gyro, gyro, gyro, temp);

        assert_eq!(payload.accel_x.value, accel.value);
        assert_eq!(payload.gyro_z.value, gyro.value);
    }

    #[test]
    fn magnetometer_round_trip() {
        let mag_payload = MagnetometerPayload::from_raw([42.0, -13.0, 8.0]);
        let cfg = config::standard();
        let mut buffer = [0u8; 128];
        let len = bincode::encode_into_slice(mag_payload, &mut buffer, cfg).unwrap();
        let (decoded, used) =
            bincode::decode_from_slice::<MagnetometerPayload, _>(&buffer[..len], cfg).unwrap();

        assert_eq!(used, len);
        assert_eq!(decoded.mag_x.value, mag_payload.mag_x.value);
        assert_eq!(decoded.mag_z.value, mag_payload.mag_z.value);
    }

    #[test]
    fn combined_payload_handles_optional_mag() {
        let imu = ImuPayload::from_raw([1.0, 2.0, 3.0], [4.0, 5.0, 6.0], 22.0);
        let mag = MagnetometerPayload::from_raw([7.0, 8.0, 9.0]);
        let combined = ImuWithMagPayload::new(imu, Some(mag));

        let cfg = config::standard();
        let mut buffer = [0u8; 256];
        let len = bincode::encode_into_slice(combined, &mut buffer, cfg).unwrap();
        let (decoded, used) =
            bincode::decode_from_slice::<ImuWithMagPayload, _>(&buffer[..len], cfg).unwrap();

        assert_eq!(used, len);
        assert_eq!(decoded.imu.accel_y.value, imu.accel_y.value);
        assert_eq!(decoded.mag.unwrap().mag_y.value, mag.mag_y.value);
    }
}
