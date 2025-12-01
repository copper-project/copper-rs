use cu29::prelude::*;
use cu_sensor_payloads::ImuPayload;
use uom::si::acceleration::meter_per_second_squared;
use uom::si::angular_velocity::radian_per_second;
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

        debug!(
            "IMU acc: {:.3} {:.3} {:.3} m/s2 gyro: {:.3} {:.3} {:.3} rad/s temp: {:.2} °C",
            payload.accel_x.get::<meter_per_second_squared>(),
            payload.accel_y.get::<meter_per_second_squared>(),
            payload.accel_z.get::<meter_per_second_squared>(),
            payload.gyro_x.get::<radian_per_second>(),
            payload.gyro_y.get::<radian_per_second>(),
            payload.gyro_z.get::<radian_per_second>(),
            payload.temperature.get::<degree_celsius>()
        );
        Ok(())
    }
}
