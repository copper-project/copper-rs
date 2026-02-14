use cu_sensor_payloads::{ImuPayload, ImuWithMagPayload, MagnetometerPayload};
use cu29::units::si::acceleration::meter_per_second_squared;
use cu29::units::si::angular_velocity::radian_per_second;
use cu29::units::si::magnetic_flux_density::microtesla;
use cu29::units::si::thermodynamic_temperature::degree_celsius;
use serde::{Deserialize, Serialize};

use crate::RosMsgAdapter;

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct CopperImu {
    pub accel_x_mps2: f32,
    pub accel_y_mps2: f32,
    pub accel_z_mps2: f32,
    pub gyro_x_radps: f32,
    pub gyro_y_radps: f32,
    pub gyro_z_radps: f32,
    pub temperature_c: f32,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct CopperMagnetometer {
    pub mag_x_ut: f32,
    pub mag_y_ut: f32,
    pub mag_z_ut: f32,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct CopperImuWithMag {
    pub imu: CopperImu,
    pub has_mag: bool,
    pub mag: CopperMagnetometer,
}

impl RosMsgAdapter<'static> for ImuPayload {
    type Output = CopperImu;

    fn namespace() -> &'static str {
        "copper_msgs"
    }

    fn type_name() -> &'static str {
        "ImuPayload"
    }

    fn type_hash() -> &'static str {
        "RIHS01_2e8439e84fbc5fb26a38b6f852f8c3c9f3d52274aebf277f4fc5d145f6ddbb91"
    }
}

impl From<&ImuPayload> for CopperImu {
    fn from(value: &ImuPayload) -> Self {
        Self {
            accel_x_mps2: value.accel_x.get::<meter_per_second_squared>(),
            accel_y_mps2: value.accel_y.get::<meter_per_second_squared>(),
            accel_z_mps2: value.accel_z.get::<meter_per_second_squared>(),
            gyro_x_radps: value.gyro_x.get::<radian_per_second>(),
            gyro_y_radps: value.gyro_y.get::<radian_per_second>(),
            gyro_z_radps: value.gyro_z.get::<radian_per_second>(),
            temperature_c: value.temperature.get::<degree_celsius>(),
        }
    }
}

impl TryFrom<CopperImu> for ImuPayload {
    type Error = &'static str;

    fn try_from(value: CopperImu) -> Result<Self, Self::Error> {
        Ok(ImuPayload::from_raw(
            [value.accel_x_mps2, value.accel_y_mps2, value.accel_z_mps2],
            [value.gyro_x_radps, value.gyro_y_radps, value.gyro_z_radps],
            value.temperature_c,
        ))
    }
}

impl RosMsgAdapter<'static> for MagnetometerPayload {
    type Output = CopperMagnetometer;

    fn namespace() -> &'static str {
        "copper_msgs"
    }

    fn type_name() -> &'static str {
        "MagnetometerPayload"
    }

    fn type_hash() -> &'static str {
        "RIHS01_93533684be87103b385e62f14dbb4f43f5f8b8b2eba246d4d89be4f723caf9fd"
    }
}

impl From<&MagnetometerPayload> for CopperMagnetometer {
    fn from(value: &MagnetometerPayload) -> Self {
        Self {
            mag_x_ut: value.mag_x.get::<microtesla>(),
            mag_y_ut: value.mag_y.get::<microtesla>(),
            mag_z_ut: value.mag_z.get::<microtesla>(),
        }
    }
}

impl TryFrom<CopperMagnetometer> for MagnetometerPayload {
    type Error = &'static str;

    fn try_from(value: CopperMagnetometer) -> Result<Self, Self::Error> {
        Ok(MagnetometerPayload::from_raw([
            value.mag_x_ut,
            value.mag_y_ut,
            value.mag_z_ut,
        ]))
    }
}

impl RosMsgAdapter<'static> for ImuWithMagPayload {
    type Output = CopperImuWithMag;

    fn namespace() -> &'static str {
        "copper_msgs"
    }

    fn type_name() -> &'static str {
        "ImuWithMagPayload"
    }

    fn type_hash() -> &'static str {
        "RIHS01_483398e2a26c66dfceb8f516e7d0f6234e456d31ca63cb01736c50d3d19f9cb9"
    }
}

impl From<&ImuWithMagPayload> for CopperImuWithMag {
    fn from(value: &ImuWithMagPayload) -> Self {
        let (has_mag, mag) = match value.mag.as_ref() {
            Some(mag) => (true, CopperMagnetometer::from(mag)),
            None => (
                false,
                CopperMagnetometer {
                    mag_x_ut: 0.0,
                    mag_y_ut: 0.0,
                    mag_z_ut: 0.0,
                },
            ),
        };
        Self {
            imu: CopperImu::from(&value.imu),
            has_mag,
            mag,
        }
    }
}

impl TryFrom<CopperImuWithMag> for ImuWithMagPayload {
    type Error = String;

    fn try_from(value: CopperImuWithMag) -> Result<Self, Self::Error> {
        let imu = ImuPayload::try_from(value.imu).map_err(|e| e.to_string())?;
        let mag = if value.has_mag {
            Some(MagnetometerPayload::try_from(value.mag).map_err(|e| e.to_string())?)
        } else {
            None
        };
        Ok(ImuWithMagPayload::new(imu, mag))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::RosBridgeAdapter;

    fn roundtrip<T>(value: T)
    where
        T: RosBridgeAdapter + PartialEq + core::fmt::Debug,
    {
        let ros_value = value.to_ros_message();
        let bytes = cdr::serialize::<_, _, cdr::CdrBe>(&ros_value, cdr::Infinite)
            .expect("cdr encode should succeed");
        let decoded_ros: <T as RosBridgeAdapter>::RosMessage =
            cdr::deserialize(bytes.as_slice()).expect("cdr decode should succeed");
        let recovered = T::from_ros_message(decoded_ros).expect("adapter decode should succeed");
        assert_eq!(value, recovered);
    }

    #[test]
    fn copper_imu_roundtrips() {
        roundtrip(ImuPayload::from_raw(
            [9.8, -0.2, 0.5],
            [0.1, -0.2, 1.5],
            36.5,
        ));
        roundtrip(MagnetometerPayload::from_raw([42.0, -13.0, 8.0]));
        roundtrip(ImuWithMagPayload::new(
            ImuPayload::from_raw([1.0, 2.0, 3.0], [0.01, 0.02, 0.03], 25.0),
            Some(MagnetometerPayload::from_raw([4.0, 5.0, 6.0])),
        ));
        roundtrip(ImuWithMagPayload::new(
            ImuPayload::from_raw([7.0, 8.0, 9.0], [0.1, 0.2, 0.3], 20.0),
            None,
        ));
    }
}
