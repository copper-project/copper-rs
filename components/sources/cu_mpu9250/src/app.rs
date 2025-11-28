use cu29::prelude::*;
use cu_sensor_payloads::ImuPayload;
use uom::si::acceleration::meter_per_second_squared;
use uom::si::angular_velocity::radian_per_second;
use uom::si::magnetic_flux_density::microtesla;
use uom::si::thermodynamic_temperature::degree_celsius;

pub struct ImuLogSink;

impl Freezable for ImuLogSink {}

impl CuSinkTask for ImuLogSink {
    type Input<'m> = input_msg!(ImuPayload);

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self> {
        Ok(Self {})
    }

    fn process<'i>(&mut self, _clock: &RobotClock, input: &Self::Input<'i>) -> CuResult<()> {
        let Some(payload) = input.payload() else {
            debug!("mpu9250 sink received empty payload");
            return Ok(());
        };

        let mag_txt = payload.mag.map(|mag| {
            format!(
                "{}, {}, {}",
                mag[0].get::<microtesla>(),
                mag[1].get::<microtesla>(),
                mag[2].get::<microtesla>()
            )
        });

        debug!(
            "IMU acc: {:.3} {:.3} {:.3} m/s2 gyro: {:.3} {:.3} {:.3} rad/s temp: {:.2} °C{}",
            payload.accel[0].get::<meter_per_second_squared>(),
            payload.accel[1].get::<meter_per_second_squared>(),
            payload.accel[2].get::<meter_per_second_squared>(),
            payload.gyro[0].get::<radian_per_second>(),
            payload.gyro[1].get::<radian_per_second>(),
            payload.gyro[2].get::<radian_per_second>(),
            payload.temperature.get::<degree_celsius>(),
            mag_txt
                .as_deref()
                .map(|txt| format!(" mag: {}", txt))
                .unwrap_or_else(|| " mag: n/a".to_string())
        );
        Ok(())
    }
}
